/**
 * @file zb_topology.h
 * @brief Zigbee Network Topology Visualization API
 *
 * This module provides network topology discovery and visualization capabilities
 * for the ESP32-C5 Zigbee2MQTT Gateway. It scans neighbor tables, collects routing
 * information, and generates JSON output compatible with zigbee2mqtt-frontend.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_TOPOLOGY_H
#define ZB_TOPOLOGY_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ============================================================================ */

/**
 * @brief Maximum number of neighbors per device
 */
#define ZB_TOPOLOGY_MAX_NEIGHBORS       32

/**
 * @brief Maximum number of routes per device
 */
#define ZB_TOPOLOGY_MAX_ROUTES          32

/**
 * @brief Maximum number of network nodes
 */
#define ZB_TOPOLOGY_MAX_NODES           64

/**
 * @brief Maximum number of links in topology
 */
#define ZB_TOPOLOGY_MAX_LINKS           128

/**
 * @brief NVS namespace for topology cache
 */
#define ZB_TOPOLOGY_NVS_NAMESPACE       "zb_topology"

/**
 * @brief NVS key for topology data
 */
#define ZB_TOPOLOGY_NVS_KEY             "topo_cache"

/**
 * @brief Topology scan timeout in milliseconds
 */
#define ZB_TOPOLOGY_SCAN_TIMEOUT_MS     30000

/**
 * @brief LQI request timeout in milliseconds
 */
#define ZB_TOPOLOGY_LQI_TIMEOUT_MS      5000

/**
 * @brief MQTT topic for network map
 */
#define ZB_TOPOLOGY_MQTT_TOPIC          "zigbee2mqtt/bridge/networkmap"

/**
 * @brief MQTT topic for topology response
 */
#define ZB_TOPOLOGY_MQTT_TOPIC_RESPONSE "zigbee2mqtt/bridge/response/networkmap"

/* ============================================================================
 * Enumerations
 * ============================================================================ */

/**
 * @brief Zigbee device type for topology
 */
typedef enum {
    ZB_TOPO_DEVICE_TYPE_COORDINATOR = 0,    /**< Coordinator device */
    ZB_TOPO_DEVICE_TYPE_ROUTER = 1,         /**< Router device */
    ZB_TOPO_DEVICE_TYPE_END_DEVICE = 2,     /**< End device */
    ZB_TOPO_DEVICE_TYPE_UNKNOWN = 3         /**< Unknown device type */
} zb_topo_device_type_t;

/**
 * @brief Neighbor relationship type
 */
typedef enum {
    ZB_TOPO_RELATIONSHIP_PARENT = 0,        /**< Parent node */
    ZB_TOPO_RELATIONSHIP_CHILD = 1,         /**< Child node */
    ZB_TOPO_RELATIONSHIP_SIBLING = 2,       /**< Sibling node */
    ZB_TOPO_RELATIONSHIP_NONE = 3,          /**< No relationship */
    ZB_TOPO_RELATIONSHIP_PREV_CHILD = 4,    /**< Previous child */
    ZB_TOPO_RELATIONSHIP_UNAUTHENTICATED = 5 /**< Unauthenticated child */
} zb_topo_relationship_t;

/**
 * @brief Route status
 */
typedef enum {
    ZB_TOPO_ROUTE_STATUS_ACTIVE = 0,        /**< Route is active */
    ZB_TOPO_ROUTE_STATUS_DISCOVERY_UNDERWAY = 1, /**< Discovery in progress */
    ZB_TOPO_ROUTE_STATUS_DISCOVERY_FAILED = 2,   /**< Discovery failed */
    ZB_TOPO_ROUTE_STATUS_INACTIVE = 3,      /**< Route is inactive */
    ZB_TOPO_ROUTE_STATUS_VALIDATION_UNDERWAY = 4 /**< Validation in progress */
} zb_topo_route_status_t;

/**
 * @brief Topology scan state
 */
typedef enum {
    ZB_TOPO_SCAN_STATE_IDLE = 0,            /**< No scan in progress */
    ZB_TOPO_SCAN_STATE_SCANNING = 1,        /**< Scan in progress */
    ZB_TOPO_SCAN_STATE_COMPLETE = 2,        /**< Scan completed */
    ZB_TOPO_SCAN_STATE_ERROR = 3            /**< Scan failed */
} zb_topo_scan_state_t;

/* ============================================================================
 * Structures
 * ============================================================================ */

/**
 * @brief Neighbor table entry structure
 *
 * Contains information about a neighboring device in the Zigbee network.
 */
typedef struct {
    uint16_t short_addr;                    /**< Network short address */
    esp_zb_ieee_addr_t ieee_addr;           /**< IEEE 64-bit address */
    uint8_t lqi;                            /**< Link Quality Indicator (0-255) */
    uint8_t depth;                          /**< Network depth from coordinator */
    zb_topo_device_type_t device_type;      /**< Device type */
    zb_topo_relationship_t relationship;    /**< Relationship to local device */
    uint16_t parent_addr;                   /**< Parent short address */
    bool permit_joining;                    /**< Device permits joining */
    bool rx_on_when_idle;                   /**< Receiver on when idle */
} zb_topo_neighbor_entry_t;

/**
 * @brief Route table entry structure
 *
 * Contains routing information for reaching destination devices.
 */
typedef struct {
    uint16_t dest_addr;                     /**< Destination short address */
    uint16_t next_hop_addr;                 /**< Next hop short address */
    zb_topo_route_status_t status;          /**< Route status */
    bool many_to_one;                       /**< Many-to-one route */
    bool route_record_required;             /**< Route record required */
    bool group_id_flag;                     /**< Group ID flag */
} zb_topo_route_entry_t;

/**
 * @brief Network node structure
 *
 * Represents a node in the network topology with its neighbors and routes.
 */
typedef struct {
    esp_zb_ieee_addr_t ieee_addr;           /**< IEEE 64-bit address */
    uint16_t short_addr;                    /**< Network short address */
    zb_topo_device_type_t device_type;      /**< Device type */
    uint8_t depth;                          /**< Network depth */
    uint16_t parent_addr;                   /**< Parent short address */
    uint8_t lqi;                            /**< Average LQI to this node */
    bool online;                            /**< Node is reachable */

    /* Neighbor information */
    zb_topo_neighbor_entry_t neighbors[ZB_TOPOLOGY_MAX_NEIGHBORS];
    uint8_t neighbor_count;                 /**< Number of neighbors */

    /* Route information */
    zb_topo_route_entry_t routes[ZB_TOPOLOGY_MAX_ROUTES];
    uint8_t route_count;                    /**< Number of routes */

    /* Timestamps */
    uint32_t last_seen;                     /**< Last seen timestamp (seconds since boot) */
    uint32_t last_scan;                     /**< Last topology scan timestamp */
} zb_topo_node_t;

/**
 * @brief Topology link structure
 *
 * Represents a link between two nodes for visualization.
 */
typedef struct {
    esp_zb_ieee_addr_t source_ieee;         /**< Source IEEE address */
    esp_zb_ieee_addr_t target_ieee;         /**< Target IEEE address */
    uint16_t source_addr;                   /**< Source short address */
    uint16_t target_addr;                   /**< Target short address */
    uint8_t lqi;                            /**< Link quality indicator */
    zb_topo_relationship_t relationship;    /**< Link relationship type */
} zb_topo_link_t;

/**
 * @brief Complete topology structure
 *
 * Contains all nodes and links in the network topology.
 */
typedef struct {
    zb_topo_node_t nodes[ZB_TOPOLOGY_MAX_NODES];
    uint8_t node_count;                     /**< Number of nodes */

    zb_topo_link_t links[ZB_TOPOLOGY_MAX_LINKS];
    uint8_t link_count;                     /**< Number of links */

    zb_topo_scan_state_t scan_state;        /**< Current scan state */
    uint32_t last_full_scan;                /**< Last full scan timestamp */
    uint8_t pending_requests;               /**< Number of pending LQI requests */
} zb_topology_t;

/**
 * @brief Topology scan callback type
 *
 * Called when a topology scan completes.
 *
 * @param[in] topology Pointer to the topology data
 * @param[in] success true if scan completed successfully
 */
typedef void (*zb_topology_scan_cb_t)(const zb_topology_t *topology, bool success);

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

/**
 * @brief Initialize topology module
 *
 * Initializes the topology module including mutex, NVS, and data structures.
 * Must be called before any other topology functions.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_topology_init(void);

/**
 * @brief Deinitialize topology module
 *
 * Frees all resources and saves topology cache to NVS.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_topology_deinit(void);

/* ============================================================================
 * Scan Functions
 * ============================================================================ */

/**
 * @brief Start topology scan
 *
 * Initiates a full network topology scan by sending Mgmt_Lqi_req to all
 * routers and the coordinator. The scan is asynchronous; use the callback
 * or poll zb_topology_get_scan_state() for completion.
 *
 * @param[in] callback Optional callback called when scan completes (can be NULL)
 * @return ESP_OK if scan started successfully
 * @return ESP_ERR_INVALID_STATE if scan already in progress
 * @return ESP_FAIL on error
 */
esp_err_t zb_topology_scan(zb_topology_scan_cb_t callback);

/**
 * @brief Stop ongoing topology scan
 *
 * Cancels any ongoing topology scan.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_topology_scan_stop(void);

/**
 * @brief Get current scan state
 *
 * @return Current scan state
 */
zb_topo_scan_state_t zb_topology_get_scan_state(void);

/**
 * @brief Request neighbor table from specific device
 *
 * Sends Mgmt_Lqi_req to a specific device to get its neighbor table.
 *
 * @param[in] short_addr Target device short address
 * @param[in] start_index Starting index in neighbor table
 * @return ESP_OK if request sent successfully
 * @return ESP_FAIL on error
 */
esp_err_t zb_topology_request_neighbors(uint16_t short_addr, uint8_t start_index);

/**
 * @brief Request routing table from specific device
 *
 * Sends Mgmt_Rtg_req to a specific device to get its routing table.
 *
 * @param[in] short_addr Target device short address
 * @param[in] start_index Starting index in routing table
 * @return ESP_OK if request sent successfully
 * @return ESP_FAIL on error
 */
esp_err_t zb_topology_request_routes(uint16_t short_addr, uint8_t start_index);

/* ============================================================================
 * Response Processing Functions
 * ============================================================================ */

/**
 * @brief Process Mgmt_Lqi_rsp (LQI/Neighbor table response)
 *
 * Called by the Zigbee callback handler when a Mgmt_Lqi_rsp is received.
 *
 * @param[in] src_addr Source device short address
 * @param[in] status Response status (ZDP status code)
 * @param[in] neighbor_table_entries Total entries in neighbor table
 * @param[in] start_index Starting index of this response
 * @param[in] neighbor_table_list_count Number of entries in this response
 * @param[in] neighbor_table Pointer to neighbor table data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_topology_process_mgmt_lqi_rsp(uint16_t src_addr,
                                           uint8_t status,
                                           uint8_t neighbor_table_entries,
                                           uint8_t start_index,
                                           uint8_t neighbor_table_list_count,
                                           const void *neighbor_table);

/**
 * @brief Process Mgmt_Rtg_rsp (Routing table response)
 *
 * Called by the Zigbee callback handler when a Mgmt_Rtg_rsp is received.
 *
 * @param[in] src_addr Source device short address
 * @param[in] status Response status (ZDP status code)
 * @param[in] routing_table_entries Total entries in routing table
 * @param[in] start_index Starting index of this response
 * @param[in] routing_table_list_count Number of entries in this response
 * @param[in] routing_table Pointer to routing table data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_topology_process_mgmt_rtg_rsp(uint16_t src_addr,
                                           uint8_t status,
                                           uint8_t routing_table_entries,
                                           uint8_t start_index,
                                           uint8_t routing_table_list_count,
                                           const void *routing_table);

/* ============================================================================
 * Data Access Functions
 * ============================================================================ */

/**
 * @brief Get current topology
 *
 * Returns pointer to the current topology data. Thread-safe.
 * The returned pointer is valid until the next topology scan.
 *
 * @return Pointer to topology structure or NULL if not initialized
 */
const zb_topology_t* zb_topology_get(void);

/**
 * @brief Get neighbors for a specific node
 *
 * Retrieves the neighbor table entries for a specific device.
 *
 * @param[in] short_addr Device short address
 * @param[out] neighbors Array to store neighbor entries
 * @param[in] max_count Maximum number of entries to return
 * @return Number of neighbors copied, or -1 on error
 */
int zb_topology_get_neighbors(uint16_t short_addr,
                              zb_topo_neighbor_entry_t *neighbors,
                              size_t max_count);

/**
 * @brief Get routes for a specific node
 *
 * Retrieves the routing table entries for a specific device.
 *
 * @param[in] short_addr Device short address
 * @param[out] routes Array to store route entries
 * @param[in] max_count Maximum number of entries to return
 * @return Number of routes copied, or -1 on error
 */
int zb_topology_get_routes(uint16_t short_addr,
                           zb_topo_route_entry_t *routes,
                           size_t max_count);

/**
 * @brief Get node by short address
 *
 * @param[in] short_addr Device short address
 * @return Pointer to node or NULL if not found
 */
const zb_topo_node_t* zb_topology_get_node(uint16_t short_addr);

/**
 * @brief Get node by IEEE address
 *
 * @param[in] ieee_addr IEEE 64-bit address
 * @return Pointer to node or NULL if not found
 */
const zb_topo_node_t* zb_topology_get_node_by_ieee(const esp_zb_ieee_addr_t ieee_addr);

/* ============================================================================
 * JSON Export Functions
 * ============================================================================ */

/**
 * @brief Convert topology to JSON
 *
 * Creates a JSON string representing the network topology in a format
 * compatible with zigbee2mqtt-frontend visualization.
 *
 * JSON format:
 * {
 *   "nodes": [
 *     {"ieee": "0x00124B001234ABCD", "type": "Coordinator", "nwk": 0},
 *     {"ieee": "0x00124B00ABCD1234", "type": "Router", "nwk": 1234, "parent": 0, "lqi": 255}
 *   ],
 *   "links": [
 *     {"source": "0x00124B001234ABCD", "target": "0x00124B00ABCD1234", "lqi": 255}
 *   ]
 * }
 *
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* zb_topology_to_json(void);

/**
 * @brief Convert topology to JSON with options
 *
 * Creates a JSON string with additional options for format control.
 *
 * @param[in] include_routes Include routing information
 * @param[in] include_offline Include offline nodes
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* zb_topology_to_json_ext(bool include_routes, bool include_offline);

/* ============================================================================
 * MQTT Functions
 * ============================================================================ */

/**
 * @brief Publish topology to MQTT
 *
 * Publishes the current network topology to the zigbee2mqtt/bridge/networkmap
 * topic for visualization in the frontend.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on error
 */
esp_err_t zb_topology_publish_mqtt(void);

/**
 * @brief Handle MQTT networkmap request
 *
 * Handles incoming MQTT requests for network map.
 * If payload is "graphviz", "plantuml", or "raw", triggers scan and publishes.
 *
 * @param[in] payload Request payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 */
esp_err_t zb_topology_handle_mqtt_request(const char *payload, size_t len);

/* ============================================================================
 * Persistence Functions
 * ============================================================================ */

/**
 * @brief Save topology to NVS
 *
 * Persists the current topology data to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_topology_save_nvs(void);

/**
 * @brief Load topology from NVS
 *
 * Loads previously saved topology data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no saved data exists
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_topology_load_nvs(void);

/**
 * @brief Clear topology cache from NVS
 *
 * Erases stored topology data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on error
 */
esp_err_t zb_topology_clear_nvs(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Add or update a node in the topology
 *
 * Adds a new node or updates an existing node with the given information.
 *
 * @param[in] ieee_addr IEEE 64-bit address
 * @param[in] short_addr Network short address
 * @param[in] device_type Device type
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if topology is full
 */
esp_err_t zb_topology_add_node(const esp_zb_ieee_addr_t ieee_addr,
                               uint16_t short_addr,
                               zb_topo_device_type_t device_type);

/**
 * @brief Remove a node from the topology
 *
 * @param[in] short_addr Network short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if node not found
 */
esp_err_t zb_topology_remove_node(uint16_t short_addr);

/**
 * @brief Update node LQI
 *
 * Updates the average LQI for a node.
 *
 * @param[in] short_addr Network short address
 * @param[in] lqi New LQI value
 * @return ESP_OK on success
 */
esp_err_t zb_topology_update_lqi(uint16_t short_addr, uint8_t lqi);

/**
 * @brief Mark node as online/offline
 *
 * @param[in] short_addr Network short address
 * @param[in] online Online status
 * @return ESP_OK on success
 */
esp_err_t zb_topology_set_node_online(uint16_t short_addr, bool online);

/**
 * @brief Get device type string
 *
 * Returns a human-readable string for the device type.
 *
 * @param[in] device_type Device type enum
 * @return Device type string (e.g., "Coordinator", "Router", "EndDevice")
 */
const char* zb_topology_device_type_str(zb_topo_device_type_t device_type);

/**
 * @brief Get topology mutex
 *
 * Returns the mutex for external thread-safe access.
 *
 * @return Mutex handle or NULL if not initialized
 */
SemaphoreHandle_t zb_topology_get_mutex(void);

/* ============================================================================
 * Test Functions
 * ============================================================================ */

/**
 * @brief Self-test function for topology module
 *
 * Tests topology initialization, JSON generation, and NVS operations.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_topology_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_TOPOLOGY_H */
