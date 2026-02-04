/**
 * @file zb_topology.c
 * @brief Zigbee Network Topology Visualization Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_topology.h"
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "core/compat_stubs.h"
#include "utils/json_utils.h"
#include "gateway_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ZB_TOPOLOGY";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** Topology data */
static zb_topology_t s_topology = {0};

/** Module initialized flag */
static bool s_initialized = false;

/** Topology mutex for thread safety */
static SemaphoreHandle_t s_mutex = NULL;

/** Scan completion callback */
static zb_topology_scan_cb_t s_scan_callback = NULL;

/** Scan timer handle */
static esp_timer_handle_t s_scan_timer = NULL;

/** Pending LQI request tracking */
static struct {
    uint16_t short_addr;
    uint8_t next_index;
    bool waiting;
} s_pending_lqi[ZB_TOPOLOGY_MAX_NODES];
static uint8_t s_pending_lqi_count = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void topology_scan_timer_callback(void *arg);
static esp_err_t topology_scan_next_device(void);
static void topology_scan_complete(bool success);
static zb_topo_node_t* topology_find_node(uint16_t short_addr);
static zb_topo_node_t* topology_find_node_by_ieee(const esp_zb_ieee_addr_t ieee_addr);
static esp_err_t topology_add_link(const esp_zb_ieee_addr_t source_ieee,
                                   const esp_zb_ieee_addr_t target_ieee,
                                   uint16_t source_addr,
                                   uint16_t target_addr,
                                   uint8_t lqi,
                                   zb_topo_relationship_t relationship);
static void topology_build_links(void);
static uint32_t topology_get_timestamp(void);

/**
 * @brief Generic ZDO LQI response callback
 *
 * Logs the result of LQI (neighbor table) ZDO requests.
 *
 * @param rsp Response data from the stack
 * @param user_ctx User context (unused)
 */
static void zdo_lqi_callback(const esp_zb_zdo_mgmt_lqi_rsp_t *rsp, void *user_ctx)
{
    (void)user_ctx;
    if (rsp == NULL) {
        ESP_LOGW(TAG, "ZDO LQI response: NULL response");
        return;
    }
    if (rsp->status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGD(TAG, "ZDO LQI request succeeded");
    } else {
        ESP_LOGW(TAG, "ZDO LQI request failed: 0x%02x", rsp->status);
    }
}

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

esp_err_t zb_topology_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Topology module already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing topology module...");

    /* Create mutex */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize topology structure */
    memset(&s_topology, 0, sizeof(s_topology));
    s_topology.scan_state = ZB_TOPO_SCAN_STATE_IDLE;

    /* Initialize pending LQI tracking */
    memset(s_pending_lqi, 0, sizeof(s_pending_lqi));
    s_pending_lqi_count = 0;

    /* Create scan timeout timer */
    const esp_timer_create_args_t timer_args = {
        .callback = topology_scan_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "topo_scan_timer"
    };

    esp_err_t ret = esp_timer_create(&timer_args, &s_scan_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create scan timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    /* Try to load cached topology from NVS */
    ret = zb_topology_load_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded cached topology with %d nodes", s_topology.node_count);
    } else {
        ESP_LOGI(TAG, "No cached topology found, starting fresh");
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Topology module initialized successfully");
    return ESP_OK;
}

esp_err_t zb_topology_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing topology module...");

    /* Stop any ongoing scan */
    zb_topology_scan_stop();

    /* Save topology to NVS */
    zb_topology_save_nvs();

    /* Delete timer */
    if (s_scan_timer != NULL) {
        esp_timer_delete(s_scan_timer);
        s_scan_timer = NULL;
    }

    /* Delete mutex */
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Topology module deinitialized");
    return ESP_OK;
}

/* ============================================================================
 * Scan Functions
 * ============================================================================ */

esp_err_t zb_topology_scan(zb_topology_scan_cb_t callback)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Topology module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (s_topology.scan_state == ZB_TOPO_SCAN_STATE_SCANNING) {
        ESP_LOGW(TAG, "Scan already in progress");
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting topology scan...");

    /* Store callback */
    s_scan_callback = callback;

    /* Reset scan state */
    s_topology.scan_state = ZB_TOPO_SCAN_STATE_SCANNING;
    s_topology.pending_requests = 0;

    /* Clear pending LQI tracking */
    memset(s_pending_lqi, 0, sizeof(s_pending_lqi));
    s_pending_lqi_count = 0;

    /* Add coordinator as first node if not present */
    esp_zb_ieee_addr_t coord_ieee;
    esp_zb_get_long_address(coord_ieee);
    zb_topology_add_node(coord_ieee, 0x0000, ZB_TOPO_DEVICE_TYPE_COORDINATOR);

    /* Add coordinator to scan list */
    s_pending_lqi[0].short_addr = 0x0000;
    s_pending_lqi[0].next_index = 0;
    s_pending_lqi[0].waiting = false;
    s_pending_lqi_count = 1;

    /* Add routers to scan list (iterate by index to avoid stack allocation) */
    size_t device_count = zb_device_get_count();
    for (size_t i = 0; i < device_count && s_pending_lqi_count < ZB_TOPOLOGY_MAX_NODES; i++) {
        zb_device_t *dev = zb_device_get_by_index(i);
        if (dev == NULL) {
            continue;
        }
        /* Check if device is a router (has routing capability) */
        zb_topo_node_t *node = topology_find_node(dev->short_addr);
        if (node != NULL && node->device_type == ZB_TOPO_DEVICE_TYPE_ROUTER) {
            s_pending_lqi[s_pending_lqi_count].short_addr = dev->short_addr;
            s_pending_lqi[s_pending_lqi_count].next_index = 0;
            s_pending_lqi[s_pending_lqi_count].waiting = false;
            s_pending_lqi_count++;
        }
    }

    ESP_LOGI(TAG, "Scanning %d devices for neighbors", s_pending_lqi_count);

    /* Start scan timeout timer */
    esp_timer_start_once(s_scan_timer, ZB_TOPOLOGY_SCAN_TIMEOUT_MS * 1000);

    xSemaphoreGive(s_mutex);

    /* Start scanning first device */
    return topology_scan_next_device();
}

esp_err_t zb_topology_scan_stop(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_topology.scan_state == ZB_TOPO_SCAN_STATE_SCANNING) {
        ESP_LOGI(TAG, "Stopping topology scan");
        esp_timer_stop(s_scan_timer);
        s_topology.scan_state = ZB_TOPO_SCAN_STATE_IDLE;
        s_scan_callback = NULL;
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

zb_topo_scan_state_t zb_topology_get_scan_state(void)
{
    return s_topology.scan_state;
}

esp_err_t zb_topology_request_neighbors(uint16_t short_addr, uint8_t start_index)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Requesting neighbors from 0x%04X, start_index=%d", short_addr, start_index);

    /* Send Mgmt_Lqi_req ZDO command */
    esp_zb_zdo_mgmt_lqi_req_param_t lqi_req = {
        .dst_addr = short_addr,
        .start_index = start_index
    };

    /* Send LQI request with response logger callback */
    esp_zb_zdo_mgmt_lqi_req(&lqi_req, zdo_lqi_callback, NULL);
    ESP_LOGD(TAG, "Sent Mgmt_Lqi_req to 0x%04X", short_addr);

    return ESP_OK;
}

esp_err_t zb_topology_request_routes(uint16_t short_addr, uint8_t start_index)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Requesting routes from 0x%04X, start_index=%d", short_addr, start_index);

    /* Note: esp_zb_zdo_mgmt_rtg_req is not available in ESP-Zigbee-SDK v1.6.x
     * Routing table queries are not implemented at this time.
     * TODO: Implement routing table queries when SDK support is available.
     */
    ESP_LOGW(TAG, "Routing table queries not supported in this SDK version");

    return ESP_OK;
}

/* ============================================================================
 * Response Processing Functions
 * ============================================================================ */

esp_err_t zb_topology_process_mgmt_lqi_rsp(uint16_t src_addr,
                                           uint8_t status,
                                           uint8_t neighbor_table_entries,
                                           uint8_t start_index,
                                           uint8_t neighbor_table_list_count,
                                           const void *neighbor_table)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Processing LQI response from 0x%04X: status=%d, entries=%d, start=%d, count=%d",
             src_addr, status, neighbor_table_entries, start_index, neighbor_table_list_count);

    if (status != 0) {
        ESP_LOGW(TAG, "LQI request to 0x%04X failed with status %d", src_addr, status);
        /* Continue with scan even if one device fails */
        if (s_topology.scan_state == ZB_TOPO_SCAN_STATE_SCANNING) {
            topology_scan_next_device();
        }
        return ESP_OK;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Find the source node */
    zb_topo_node_t *src_node = topology_find_node(src_addr);
    if (src_node == NULL) {
        /* Create node if it doesn't exist */
        esp_zb_ieee_addr_t dummy_ieee = {0};
        zb_topology_add_node(dummy_ieee, src_addr, ZB_TOPO_DEVICE_TYPE_ROUTER);
        src_node = topology_find_node(src_addr);
    }

    if (src_node == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Parse neighbor table entries */
    /* Note: The actual structure depends on the ESP-Zigbee-SDK version */
    /* This is a simplified representation */
    typedef struct {
        esp_zb_ieee_addr_t ieee_addr;
        uint16_t nwk_addr;
        uint8_t device_type_rx_on_relationship; /* Packed: deviceType(2) + rxOnWhenIdle(2) + relationship(3) */
        uint8_t permit_join_reserved;           /* Packed: permitJoin(2) + reserved(6) */
        uint8_t depth;
        uint8_t lqi;
    } __attribute__((packed)) neighbor_entry_t;

    const neighbor_entry_t *entries = (const neighbor_entry_t *)neighbor_table;

    for (uint8_t i = 0; i < neighbor_table_list_count; i++) {
        if (src_node->neighbor_count >= ZB_TOPOLOGY_MAX_NEIGHBORS) {
            ESP_LOGW(TAG, "Neighbor table full for node 0x%04X", src_addr);
            break;
        }

        const neighbor_entry_t *entry = &entries[i];
        zb_topo_neighbor_entry_t *neighbor = &src_node->neighbors[src_node->neighbor_count];

        /* Copy IEEE address */
        memcpy(neighbor->ieee_addr, entry->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        neighbor->short_addr = entry->nwk_addr;
        neighbor->lqi = entry->lqi;
        neighbor->depth = entry->depth;

        /* Unpack device type and relationship */
        neighbor->device_type = (zb_topo_device_type_t)(entry->device_type_rx_on_relationship & 0x03);
        neighbor->rx_on_when_idle = (entry->device_type_rx_on_relationship >> 2) & 0x03;
        neighbor->relationship = (zb_topo_relationship_t)((entry->device_type_rx_on_relationship >> 4) & 0x07);
        neighbor->permit_joining = (entry->permit_join_reserved & 0x03) == 1;

        /* Add/update the neighbor as a node in the topology */
        zb_topology_add_node(neighbor->ieee_addr, neighbor->short_addr, neighbor->device_type);

        /* Update node info */
        zb_topo_node_t *neighbor_node = topology_find_node(neighbor->short_addr);
        if (neighbor_node != NULL) {
            neighbor_node->depth = neighbor->depth;
            neighbor_node->lqi = neighbor->lqi;
            if (neighbor->relationship == ZB_TOPO_RELATIONSHIP_CHILD) {
                neighbor_node->parent_addr = src_addr;
            }
        }

        src_node->neighbor_count++;

        ESP_LOGD(TAG, "  Neighbor: 0x%04X, LQI=%d, depth=%d, type=%d",
                 neighbor->short_addr, neighbor->lqi, neighbor->depth, neighbor->device_type);
    }

    src_node->last_scan = topology_get_timestamp();

    /* Check if we need to request more entries */
    uint8_t total_received = start_index + neighbor_table_list_count;
    if (total_received < neighbor_table_entries) {
        /* More entries available, request next batch */
        for (uint8_t i = 0; i < s_pending_lqi_count; i++) {
            if (s_pending_lqi[i].short_addr == src_addr) {
                s_pending_lqi[i].next_index = total_received;
                xSemaphoreGive(s_mutex);
                return zb_topology_request_neighbors(src_addr, total_received);
            }
        }
    }

    xSemaphoreGive(s_mutex);

    /* Continue with next device in scan */
    if (s_topology.scan_state == ZB_TOPO_SCAN_STATE_SCANNING) {
        topology_scan_next_device();
    }

    return ESP_OK;
}

esp_err_t zb_topology_process_mgmt_rtg_rsp(uint16_t src_addr,
                                           uint8_t status,
                                           uint8_t routing_table_entries,
                                           uint8_t start_index,
                                           uint8_t routing_table_list_count,
                                           const void *routing_table)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Processing routing response from 0x%04X: status=%d, entries=%d",
             src_addr, status, routing_table_entries);

    if (status != 0) {
        ESP_LOGW(TAG, "Routing request to 0x%04X failed with status %d", src_addr, status);
        return ESP_OK;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    zb_topo_node_t *src_node = topology_find_node(src_addr);
    if (src_node == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Parse routing table entries */
    typedef struct {
        uint16_t dest_addr;
        uint8_t status_flags;       /* Packed: status(3) + manyToOne(1) + routeRecordReq(1) + groupIdFlag(1) */
        uint16_t next_hop_addr;
    } __attribute__((packed)) routing_entry_t;

    const routing_entry_t *entries = (const routing_entry_t *)routing_table;

    for (uint8_t i = 0; i < routing_table_list_count; i++) {
        if (src_node->route_count >= ZB_TOPOLOGY_MAX_ROUTES) {
            ESP_LOGW(TAG, "Route table full for node 0x%04X", src_addr);
            break;
        }

        const routing_entry_t *entry = &entries[i];
        zb_topo_route_entry_t *route = &src_node->routes[src_node->route_count];

        route->dest_addr = entry->dest_addr;
        route->next_hop_addr = entry->next_hop_addr;
        route->status = (zb_topo_route_status_t)(entry->status_flags & 0x07);
        route->many_to_one = (entry->status_flags >> 3) & 0x01;
        route->route_record_required = (entry->status_flags >> 4) & 0x01;
        route->group_id_flag = (entry->status_flags >> 5) & 0x01;

        src_node->route_count++;

        ESP_LOGD(TAG, "  Route: dest=0x%04X, next_hop=0x%04X, status=%d",
                 route->dest_addr, route->next_hop_addr, route->status);
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* ============================================================================
 * Data Access Functions
 * ============================================================================ */

const zb_topology_t* zb_topology_get(void)
{
    if (!s_initialized) {
        return NULL;
    }
    return &s_topology;
}

int zb_topology_get_neighbors(uint16_t short_addr,
                              zb_topo_neighbor_entry_t *neighbors,
                              size_t max_count)
{
    if (!s_initialized || neighbors == NULL) {
        return -1;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return -1;
    }

    const zb_topo_node_t *node = topology_find_node(short_addr);
    if (node == NULL) {
        xSemaphoreGive(s_mutex);
        return -1;
    }

    size_t count = (node->neighbor_count < max_count) ? node->neighbor_count : max_count;
    memcpy(neighbors, node->neighbors, count * sizeof(zb_topo_neighbor_entry_t));

    xSemaphoreGive(s_mutex);
    return (int)count;
}

int zb_topology_get_routes(uint16_t short_addr,
                           zb_topo_route_entry_t *routes,
                           size_t max_count)
{
    if (!s_initialized || routes == NULL) {
        return -1;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return -1;
    }

    const zb_topo_node_t *node = topology_find_node(short_addr);
    if (node == NULL) {
        xSemaphoreGive(s_mutex);
        return -1;
    }

    size_t count = (node->route_count < max_count) ? node->route_count : max_count;
    memcpy(routes, node->routes, count * sizeof(zb_topo_route_entry_t));

    xSemaphoreGive(s_mutex);
    return (int)count;
}

const zb_topo_node_t* zb_topology_get_node(uint16_t short_addr)
{
    if (!s_initialized) {
        return NULL;
    }
    return topology_find_node(short_addr);
}

const zb_topo_node_t* zb_topology_get_node_by_ieee(const esp_zb_ieee_addr_t ieee_addr)
{
    if (!s_initialized) {
        return NULL;
    }
    return topology_find_node_by_ieee(ieee_addr);
}

/* ============================================================================
 * JSON Export Functions
 * ============================================================================ */

char* zb_topology_to_json(void)
{
    return zb_topology_to_json_ext(false, false);
}

char* zb_topology_to_json_ext(bool include_routes, bool include_offline)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Topology module not initialized");
        return NULL;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for JSON export");
        return NULL;
    }

    /* Build links from neighbor information */
    topology_build_links();

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    /* Create nodes array */
    cJSON *nodes_array = cJSON_CreateArray();
    if (nodes_array == NULL) {
        cJSON_Delete(root);
        xSemaphoreGive(s_mutex);
        return NULL;
    }
    cJSON_AddItemToObject(root, "nodes", nodes_array);

    char ieee_str[24];

    for (uint8_t i = 0; i < s_topology.node_count; i++) {
        const zb_topo_node_t *node = &s_topology.nodes[i];

        /* Skip offline nodes if not requested */
        if (!include_offline && !node->online) {
            continue;
        }

        cJSON *node_obj = cJSON_CreateObject();
        if (node_obj == NULL) {
            continue;
        }

        /* Format IEEE address as hex string */
        snprintf(ieee_str, sizeof(ieee_str), "0x%02X%02X%02X%02X%02X%02X%02X%02X",
                 node->ieee_addr[7], node->ieee_addr[6], node->ieee_addr[5], node->ieee_addr[4],
                 node->ieee_addr[3], node->ieee_addr[2], node->ieee_addr[1], node->ieee_addr[0]);

        cJSON_AddStringToObject(node_obj, "ieee", ieee_str);
        cJSON_AddStringToObject(node_obj, "type", zb_topology_device_type_str(node->device_type));
        cJSON_AddNumberToObject(node_obj, "nwk", node->short_addr);

        /* Add parent if not coordinator */
        if (node->device_type != ZB_TOPO_DEVICE_TYPE_COORDINATOR && node->parent_addr != 0xFFFF) {
            cJSON_AddNumberToObject(node_obj, "parent", node->parent_addr);
        }

        /* Add LQI if available */
        if (node->lqi > 0) {
            cJSON_AddNumberToObject(node_obj, "lqi", node->lqi);
        }

        /* Add depth */
        cJSON_AddNumberToObject(node_obj, "depth", node->depth);

        /* Add routing info if requested */
        if (include_routes && node->route_count > 0) {
            cJSON *routes_array = cJSON_CreateArray();
            if (routes_array != NULL) {
                for (uint8_t r = 0; r < node->route_count; r++) {
                    cJSON *route_obj = cJSON_CreateObject();
                    if (route_obj != NULL) {
                        cJSON_AddNumberToObject(route_obj, "dest", node->routes[r].dest_addr);
                        cJSON_AddNumberToObject(route_obj, "nextHop", node->routes[r].next_hop_addr);
                        cJSON_AddNumberToObject(route_obj, "status", node->routes[r].status);
                        cJSON_AddItemToArray(routes_array, route_obj);
                    }
                }
                cJSON_AddItemToObject(node_obj, "routes", routes_array);
            }
        }

        cJSON_AddItemToArray(nodes_array, node_obj);
    }

    /* Create links array */
    cJSON *links_array = cJSON_CreateArray();
    if (links_array == NULL) {
        cJSON_Delete(root);
        xSemaphoreGive(s_mutex);
        return NULL;
    }
    cJSON_AddItemToObject(root, "links", links_array);

    for (uint8_t i = 0; i < s_topology.link_count; i++) {
        const zb_topo_link_t *link = &s_topology.links[i];

        cJSON *link_obj = cJSON_CreateObject();
        if (link_obj == NULL) {
            continue;
        }

        /* Format source IEEE */
        snprintf(ieee_str, sizeof(ieee_str), "0x%02X%02X%02X%02X%02X%02X%02X%02X",
                 link->source_ieee[7], link->source_ieee[6], link->source_ieee[5], link->source_ieee[4],
                 link->source_ieee[3], link->source_ieee[2], link->source_ieee[1], link->source_ieee[0]);
        cJSON_AddStringToObject(link_obj, "source", ieee_str);

        /* Format target IEEE */
        snprintf(ieee_str, sizeof(ieee_str), "0x%02X%02X%02X%02X%02X%02X%02X%02X",
                 link->target_ieee[7], link->target_ieee[6], link->target_ieee[5], link->target_ieee[4],
                 link->target_ieee[3], link->target_ieee[2], link->target_ieee[1], link->target_ieee[0]);
        cJSON_AddStringToObject(link_obj, "target", ieee_str);

        cJSON_AddNumberToObject(link_obj, "lqi", link->lqi);

        /* Add relationship type for debugging */
        const char *rel_str = "unknown";
        switch (link->relationship) {
            case ZB_TOPO_RELATIONSHIP_PARENT: rel_str = "parent"; break;
            case ZB_TOPO_RELATIONSHIP_CHILD: rel_str = "child"; break;
            case ZB_TOPO_RELATIONSHIP_SIBLING: rel_str = "sibling"; break;
            default: break;
        }
        cJSON_AddStringToObject(link_obj, "relationship", rel_str);

        cJSON_AddItemToArray(links_array, link_obj);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to serialize topology JSON");
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    xSemaphoreGive(s_mutex);

    return json_str;
}

/* ============================================================================
 * MQTT Functions
 * ============================================================================ */

esp_err_t zb_topology_publish_mqtt(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char *json_str = zb_topology_to_json();
    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to create topology JSON");
        return ESP_FAIL;
    }

    esp_err_t ret = mqtt_client_publish(ZB_TOPOLOGY_MQTT_TOPIC_RESPONSE,
                                        json_str,
                                        strlen(json_str),
                                        0,
                                        false);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Published network topology (%d bytes)", strlen(json_str));
    } else {
        ESP_LOGE(TAG, "Failed to publish topology: %s", esp_err_to_name(ret));
    }

    free(json_str);
    return ret;
}

esp_err_t zb_topology_handle_mqtt_request(const char *payload, size_t len)
{
    if (payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Handling networkmap request: %.*s", (int)len, payload);

    /* Check request type */
    if (strncmp(payload, "raw", len) == 0 ||
        strncmp(payload, "graphviz", len) == 0 ||
        strncmp(payload, "plantuml", len) == 0) {

        /* Start scan and publish when complete */
        return zb_topology_scan(NULL);
    }

    /* Default: just publish current topology */
    return zb_topology_publish_mqtt();
}

/* ============================================================================
 * Persistence Functions
 * ============================================================================ */

esp_err_t zb_topology_save_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(ZB_TOPOLOGY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        nvs_close(handle);
        return ESP_ERR_TIMEOUT;
    }

    /* Save only essential topology data */
    /* For memory efficiency, we save a simplified structure */
    typedef struct {
        uint8_t node_count;
        struct {
            esp_zb_ieee_addr_t ieee_addr;
            uint16_t short_addr;
            uint8_t device_type;
            uint8_t depth;
            uint16_t parent_addr;
            uint8_t lqi;
        } nodes[ZB_TOPOLOGY_MAX_NODES];
    } topo_cache_t;

    topo_cache_t *cache = malloc(sizeof(topo_cache_t));
    if (cache == NULL) {
        xSemaphoreGive(s_mutex);
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    cache->node_count = s_topology.node_count;
    for (uint8_t i = 0; i < s_topology.node_count && i < ZB_TOPOLOGY_MAX_NODES; i++) {
        memcpy(cache->nodes[i].ieee_addr, s_topology.nodes[i].ieee_addr, sizeof(esp_zb_ieee_addr_t));
        cache->nodes[i].short_addr = s_topology.nodes[i].short_addr;
        cache->nodes[i].device_type = (uint8_t)s_topology.nodes[i].device_type;
        cache->nodes[i].depth = s_topology.nodes[i].depth;
        cache->nodes[i].parent_addr = s_topology.nodes[i].parent_addr;
        cache->nodes[i].lqi = s_topology.nodes[i].lqi;
    }

    ret = nvs_set_blob(handle, ZB_TOPOLOGY_NVS_KEY, cache, sizeof(topo_cache_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Saved topology cache with %d nodes", cache->node_count);
        }
    }

    free(cache);
    xSemaphoreGive(s_mutex);
    nvs_close(handle);

    return ret;
}

esp_err_t zb_topology_load_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(ZB_TOPOLOGY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    typedef struct {
        uint8_t node_count;
        struct {
            esp_zb_ieee_addr_t ieee_addr;
            uint16_t short_addr;
            uint8_t device_type;
            uint8_t depth;
            uint16_t parent_addr;
            uint8_t lqi;
        } nodes[ZB_TOPOLOGY_MAX_NODES];
    } topo_cache_t;

    size_t size = sizeof(topo_cache_t);
    topo_cache_t *cache = malloc(size);
    if (cache == NULL) {
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    ret = nvs_get_blob(handle, ZB_TOPOLOGY_NVS_KEY, cache, &size);
    if (ret != ESP_OK) {
        free(cache);
        nvs_close(handle);
        return ret;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        free(cache);
        nvs_close(handle);
        return ESP_ERR_TIMEOUT;
    }

    /* Restore topology from cache */
    s_topology.node_count = 0;
    for (uint8_t i = 0; i < cache->node_count && i < ZB_TOPOLOGY_MAX_NODES; i++) {
        zb_topo_node_t *node = &s_topology.nodes[s_topology.node_count];
        memcpy(node->ieee_addr, cache->nodes[i].ieee_addr, sizeof(esp_zb_ieee_addr_t));
        node->short_addr = cache->nodes[i].short_addr;
        node->device_type = (zb_topo_device_type_t)cache->nodes[i].device_type;
        node->depth = cache->nodes[i].depth;
        node->parent_addr = cache->nodes[i].parent_addr;
        node->lqi = cache->nodes[i].lqi;
        node->online = false;  /* Will be updated on next scan */
        node->neighbor_count = 0;
        node->route_count = 0;
        s_topology.node_count++;
    }

    xSemaphoreGive(s_mutex);
    free(cache);
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded topology cache with %d nodes", s_topology.node_count);
    return ESP_OK;
}

esp_err_t zb_topology_clear_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(ZB_TOPOLOGY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_key(handle, ZB_TOPOLOGY_NVS_KEY);
    if (ret == ESP_OK || ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_commit(handle);
        ret = ESP_OK;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Cleared topology cache from NVS");
    return ret;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

esp_err_t zb_topology_add_node(const esp_zb_ieee_addr_t ieee_addr,
                               uint16_t short_addr,
                               zb_topo_device_type_t device_type)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if node already exists */
    zb_topo_node_t *existing = topology_find_node(short_addr);
    if (existing != NULL) {
        /* Update existing node */
        if (memcmp(existing->ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) != 0) {
            memcpy(existing->ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
        }
        existing->device_type = device_type;
        existing->online = true;
        existing->last_seen = topology_get_timestamp();
        return ESP_OK;
    }

    /* Add new node */
    if (s_topology.node_count >= ZB_TOPOLOGY_MAX_NODES) {
        ESP_LOGE(TAG, "Topology full, cannot add node 0x%04X", short_addr);
        return ESP_ERR_NO_MEM;
    }

    zb_topo_node_t *node = &s_topology.nodes[s_topology.node_count];
    memset(node, 0, sizeof(zb_topo_node_t));
    memcpy(node->ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    node->short_addr = short_addr;
    node->device_type = device_type;
    node->parent_addr = 0xFFFF;
    node->online = true;
    node->last_seen = topology_get_timestamp();

    s_topology.node_count++;

    ESP_LOGD(TAG, "Added node 0x%04X to topology (total: %d)", short_addr, s_topology.node_count);
    return ESP_OK;
}

esp_err_t zb_topology_remove_node(uint16_t short_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < s_topology.node_count; i++) {
        if (s_topology.nodes[i].short_addr == short_addr) {
            /* Shift remaining nodes */
            if (i < s_topology.node_count - 1) {
                memmove(&s_topology.nodes[i],
                        &s_topology.nodes[i + 1],
                        (s_topology.node_count - i - 1) * sizeof(zb_topo_node_t));
            }
            s_topology.node_count--;
            xSemaphoreGive(s_mutex);
            ESP_LOGD(TAG, "Removed node 0x%04X from topology", short_addr);
            return ESP_OK;
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_topology_update_lqi(uint16_t short_addr, uint8_t lqi)
{
    zb_topo_node_t *node = topology_find_node(short_addr);
    if (node == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    node->lqi = lqi;
    node->last_seen = topology_get_timestamp();
    return ESP_OK;
}

esp_err_t zb_topology_set_node_online(uint16_t short_addr, bool online)
{
    zb_topo_node_t *node = topology_find_node(short_addr);
    if (node == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    node->online = online;
    if (online) {
        node->last_seen = topology_get_timestamp();
    }
    return ESP_OK;
}

const char* zb_topology_device_type_str(zb_topo_device_type_t device_type)
{
    switch (device_type) {
        case ZB_TOPO_DEVICE_TYPE_COORDINATOR:
            return "Coordinator";
        case ZB_TOPO_DEVICE_TYPE_ROUTER:
            return "Router";
        case ZB_TOPO_DEVICE_TYPE_END_DEVICE:
            return "EndDevice";
        default:
            return "Unknown";
    }
}

SemaphoreHandle_t zb_topology_get_mutex(void)
{
    return s_mutex;
}

/* ============================================================================
 * Static Helper Functions
 * ============================================================================ */

static void topology_scan_timer_callback(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "Topology scan timeout");
    topology_scan_complete(false);
}

static esp_err_t topology_scan_next_device(void)
{
    /* Find next device to scan */
    for (uint8_t i = 0; i < s_pending_lqi_count; i++) {
        if (!s_pending_lqi[i].waiting) {
            s_pending_lqi[i].waiting = true;
            s_topology.pending_requests++;

            ESP_LOGD(TAG, "Requesting neighbors from 0x%04X", s_pending_lqi[i].short_addr);
            return zb_topology_request_neighbors(s_pending_lqi[i].short_addr,
                                                  s_pending_lqi[i].next_index);
        }
    }

    /* All devices scanned */
    ESP_LOGI(TAG, "All devices scanned");
    topology_scan_complete(true);
    return ESP_OK;
}

static void topology_scan_complete(bool success)
{
    esp_timer_stop(s_scan_timer);

    if (xSemaphoreTake(s_mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) == pdTRUE) {
        s_topology.scan_state = success ? ZB_TOPO_SCAN_STATE_COMPLETE : ZB_TOPO_SCAN_STATE_ERROR;
        s_topology.last_full_scan = topology_get_timestamp();
        s_topology.pending_requests = 0;

        /* Build links from neighbor data */
        topology_build_links();

        xSemaphoreGive(s_mutex);
    }

    ESP_LOGI(TAG, "Topology scan %s: %d nodes, %d links",
             success ? "completed" : "failed",
             s_topology.node_count,
             s_topology.link_count);

    /* Notify callback */
    if (s_scan_callback != NULL) {
        s_scan_callback(&s_topology, success);
    }

    /* Publish to MQTT */
    zb_topology_publish_mqtt();

    /* Save to NVS */
    zb_topology_save_nvs();
}

static zb_topo_node_t* topology_find_node(uint16_t short_addr)
{
    for (uint8_t i = 0; i < s_topology.node_count; i++) {
        if (s_topology.nodes[i].short_addr == short_addr) {
            return &s_topology.nodes[i];
        }
    }
    return NULL;
}

static zb_topo_node_t* topology_find_node_by_ieee(const esp_zb_ieee_addr_t ieee_addr)
{
    for (uint8_t i = 0; i < s_topology.node_count; i++) {
        if (memcmp(s_topology.nodes[i].ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t)) == 0) {
            return &s_topology.nodes[i];
        }
    }
    return NULL;
}

static esp_err_t topology_add_link(const esp_zb_ieee_addr_t source_ieee,
                                   const esp_zb_ieee_addr_t target_ieee,
                                   uint16_t source_addr,
                                   uint16_t target_addr,
                                   uint8_t lqi,
                                   zb_topo_relationship_t relationship)
{
    /* Check for duplicate link */
    for (uint8_t i = 0; i < s_topology.link_count; i++) {
        if (memcmp(s_topology.links[i].source_ieee, source_ieee, sizeof(esp_zb_ieee_addr_t)) == 0 &&
            memcmp(s_topology.links[i].target_ieee, target_ieee, sizeof(esp_zb_ieee_addr_t)) == 0) {
            /* Update existing link */
            s_topology.links[i].lqi = lqi;
            return ESP_OK;
        }
    }

    if (s_topology.link_count >= ZB_TOPOLOGY_MAX_LINKS) {
        return ESP_ERR_NO_MEM;
    }

    zb_topo_link_t *link = &s_topology.links[s_topology.link_count];
    memcpy(link->source_ieee, source_ieee, sizeof(esp_zb_ieee_addr_t));
    memcpy(link->target_ieee, target_ieee, sizeof(esp_zb_ieee_addr_t));
    link->source_addr = source_addr;
    link->target_addr = target_addr;
    link->lqi = lqi;
    link->relationship = relationship;

    s_topology.link_count++;
    return ESP_OK;
}

static void topology_build_links(void)
{
    /* Clear existing links */
    s_topology.link_count = 0;

    /* Build links from neighbor tables */
    for (uint8_t i = 0; i < s_topology.node_count; i++) {
        zb_topo_node_t *node = &s_topology.nodes[i];

        for (uint8_t j = 0; j < node->neighbor_count; j++) {
            zb_topo_neighbor_entry_t *neighbor = &node->neighbors[j];

            /* Add link from this node to neighbor */
            topology_add_link(node->ieee_addr,
                              neighbor->ieee_addr,
                              node->short_addr,
                              neighbor->short_addr,
                              neighbor->lqi,
                              neighbor->relationship);
        }

        /* Add parent-child link if we have a parent */
        if (node->parent_addr != 0xFFFF && node->parent_addr != 0x0000 &&
            node->device_type != ZB_TOPO_DEVICE_TYPE_COORDINATOR) {
            zb_topo_node_t *parent = topology_find_node(node->parent_addr);
            if (parent != NULL) {
                topology_add_link(parent->ieee_addr,
                                  node->ieee_addr,
                                  parent->short_addr,
                                  node->short_addr,
                                  node->lqi,
                                  ZB_TOPO_RELATIONSHIP_PARENT);
            }
        }
    }
}

static uint32_t topology_get_timestamp(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);  /* Seconds since boot */
}

/* ============================================================================
 * Test Functions
 * ============================================================================ */

esp_err_t zb_topology_test(void)
{
    ESP_LOGI(TAG, "Running topology self-test...");

    /* Test 1: Initialization (should already be done) */
    if (!s_initialized) {
        esp_err_t ret = zb_topology_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Test FAILED: Initialization failed");
            return ESP_FAIL;
        }
    }

    /* Test 2: Add test nodes */
    esp_zb_ieee_addr_t test_ieee1 = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    esp_zb_ieee_addr_t test_ieee2 = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};

    esp_err_t ret = zb_topology_add_node(test_ieee1, 0x1234, ZB_TOPO_DEVICE_TYPE_ROUTER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test FAILED: Failed to add test node 1");
        return ESP_FAIL;
    }

    ret = zb_topology_add_node(test_ieee2, 0x5678, ZB_TOPO_DEVICE_TYPE_END_DEVICE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test FAILED: Failed to add test node 2");
        return ESP_FAIL;
    }

    /* Test 3: Find node */
    const zb_topo_node_t *node = zb_topology_get_node(0x1234);
    if (node == NULL) {
        ESP_LOGE(TAG, "Test FAILED: Failed to find test node");
        return ESP_FAIL;
    }

    if (node->device_type != ZB_TOPO_DEVICE_TYPE_ROUTER) {
        ESP_LOGE(TAG, "Test FAILED: Node type mismatch");
        return ESP_FAIL;
    }

    /* Test 4: JSON export */
    char *json = zb_topology_to_json();
    if (json == NULL) {
        ESP_LOGE(TAG, "Test FAILED: Failed to create JSON");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Test JSON: %s", json);
    free(json);

    /* Test 5: Device type string */
    const char *type_str = zb_topology_device_type_str(ZB_TOPO_DEVICE_TYPE_COORDINATOR);
    if (strcmp(type_str, "Coordinator") != 0) {
        ESP_LOGE(TAG, "Test FAILED: Device type string mismatch");
        return ESP_FAIL;
    }

    /* Test 6: Remove test nodes */
    ret = zb_topology_remove_node(0x1234);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test FAILED: Failed to remove test node");
        return ESP_FAIL;
    }

    ret = zb_topology_remove_node(0x5678);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test FAILED: Failed to remove test node 2");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Topology self-test PASSED");
    return ESP_OK;
}
