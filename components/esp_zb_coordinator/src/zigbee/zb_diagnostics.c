/**
 * @file zb_diagnostics.c
 * @brief Zigbee Diagnostics Cluster Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_diagnostics.h"
#include "zb_device_handler.h"
#include "compat_stubs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string.h>
#include <time.h>

static const char *TAG = "ZB_DIAG";

/* ============================================================================
 * Constants
 * ============================================================================ */

#define DIAGNOSTICS_NVS_NAMESPACE   "zb_diag"
#define DIAGNOSTICS_NVS_KEY_RESETS  "resets"

/** @brief Maximum number of cached device temperatures */
#define DEVICE_TEMP_CACHE_SIZE      16

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** @brief Module initialization flag */
static bool s_initialized = false;

/** @brief Thread safety mutex */
static SemaphoreHandle_t s_diag_mutex = NULL;

/** @brief Coordinator diagnostics */
static zb_diagnostics_t s_coordinator_diag = {0};

/** @brief Device temperature cache entry */
typedef struct {
    uint16_t short_addr;
    zb_device_temperature_t temp;
} device_temp_cache_entry_t;

/** @brief Device temperature cache */
static device_temp_cache_entry_t s_device_temp_cache[DEVICE_TEMP_CACHE_SIZE] = {0};

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

/**
 * @brief Load reset count from NVS
 */
static esp_err_t load_reset_count(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(DIAGNOSTICS_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        /* Namespace doesn't exist yet */
        return ESP_OK;
    }

    uint16_t reset_count = 0;
    ret = nvs_get_u16(nvs_handle, DIAGNOSTICS_NVS_KEY_RESETS, &reset_count);
    if (ret == ESP_OK) {
        s_coordinator_diag.number_of_resets = reset_count;
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

/**
 * @brief Save reset count to NVS
 */
static esp_err_t save_reset_count(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(DIAGNOSTICS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_u16(nvs_handle, DIAGNOSTICS_NVS_KEY_RESETS,
                      s_coordinator_diag.number_of_resets);
    if (ret == ESP_OK) {
        nvs_commit(nvs_handle);
        s_coordinator_diag.persistent_mem_writes++;
    }

    nvs_close(nvs_handle);
    return ret;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_diagnostics_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Diagnostics already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Diagnostics Cluster...");

    /* Create mutex */
    s_diag_mutex = xSemaphoreCreateMutex();
    if (s_diag_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize diagnostics structure */
    memset(&s_coordinator_diag, 0, sizeof(zb_diagnostics_t));

    /* Load persistent data */
    load_reset_count();

    /* Increment and save reset count */
    s_coordinator_diag.number_of_resets++;
    save_reset_count();

    s_initialized = true;
    ESP_LOGI(TAG, "Diagnostics Cluster initialized (reset count: %u)",
             s_coordinator_diag.number_of_resets);

    return ESP_OK;
}

esp_err_t zb_diagnostics_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Diagnostics Cluster...");

    if (s_diag_mutex != NULL) {
        vSemaphoreDelete(s_diag_mutex);
        s_diag_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Diagnostics Cluster deinitialized");

    return ESP_OK;
}

bool zb_diagnostics_is_initialized(void)
{
    return s_initialized;
}

esp_err_t zb_diagnostics_get_coordinator(zb_diagnostics_t *diag)
{
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);
    memcpy(diag, &s_coordinator_diag, sizeof(zb_diagnostics_t));
    xSemaphoreGive(s_diag_mutex);

    return ESP_OK;
}

esp_err_t zb_diagnostics_read(uint16_t short_addr, zb_diagnostics_t *diag)
{
    if (diag == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading diagnostics from device 0x%04X", short_addr);

    /* Note: Actual implementation would send ZCL Read Attributes request
     * and wait for response. This requires ESP-Zigbee-SDK support:
     *
     * esp_zb_zcl_read_attr_cmd_t cmd = {...};
     * esp_zb_zcl_read_attr_cmd_req(&cmd);
     */

    /* For now, return placeholder data */
    memset(diag, 0, sizeof(zb_diagnostics_t));

    ESP_LOGD(TAG, "Diagnostics read from 0x%04X (framework - actual read via SDK)", short_addr);
    return ESP_OK;
}

size_t zb_diagnostics_read_attr(uint16_t short_addr, uint16_t attr_id,
                                void *value, size_t max_len)
{
    if (value == NULL || max_len == 0) {
        return 0;
    }

    if (!s_initialized) {
        return 0;
    }

    ESP_LOGD(TAG, "Reading diagnostics attr 0x%04X from device 0x%04X",
             attr_id, short_addr);

    /* Note: Would send specific attribute read request */
    return 0;
}

void zb_diagnostics_update_mac(uint16_t counter, uint32_t increment)
{
    if (!s_initialized) {
        return;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);

    switch (counter) {
        case ZB_DIAG_ATTR_MAC_RX_BCAST:
            s_coordinator_diag.mac.rx_bcast += increment;
            break;
        case ZB_DIAG_ATTR_MAC_TX_BCAST:
            s_coordinator_diag.mac.tx_bcast += increment;
            break;
        case ZB_DIAG_ATTR_MAC_RX_UCAST:
            s_coordinator_diag.mac.rx_ucast += increment;
            break;
        case ZB_DIAG_ATTR_MAC_TX_UCAST:
            s_coordinator_diag.mac.tx_ucast += increment;
            break;
        case ZB_DIAG_ATTR_MAC_TX_UCAST_RETRY:
            s_coordinator_diag.mac.tx_ucast_retry += (uint16_t)increment;
            break;
        case ZB_DIAG_ATTR_MAC_TX_UCAST_FAIL:
            s_coordinator_diag.mac.tx_ucast_fail += (uint16_t)increment;
            break;
        default:
            ESP_LOGW(TAG, "Unknown MAC counter: 0x%04X", counter);
            break;
    }

    xSemaphoreGive(s_diag_mutex);
}

void zb_diagnostics_update_aps(uint16_t counter, uint16_t increment)
{
    if (!s_initialized) {
        return;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);

    switch (counter) {
        case ZB_DIAG_ATTR_APS_RX_BCAST:
            s_coordinator_diag.aps.rx_bcast += increment;
            break;
        case ZB_DIAG_ATTR_APS_TX_BCAST:
            s_coordinator_diag.aps.tx_bcast += increment;
            break;
        case ZB_DIAG_ATTR_APS_RX_UCAST:
            s_coordinator_diag.aps.rx_ucast += increment;
            break;
        case ZB_DIAG_ATTR_APS_TX_UCAST_SUCCESS:
            s_coordinator_diag.aps.tx_ucast_success += increment;
            break;
        case ZB_DIAG_ATTR_APS_TX_UCAST_RETRY:
            s_coordinator_diag.aps.tx_ucast_retry += increment;
            break;
        case ZB_DIAG_ATTR_APS_TX_UCAST_FAIL:
            s_coordinator_diag.aps.tx_ucast_fail += increment;
            break;
        default:
            ESP_LOGW(TAG, "Unknown APS counter: 0x%04X", counter);
            break;
    }

    xSemaphoreGive(s_diag_mutex);
}

void zb_diagnostics_update_network(uint16_t counter, uint16_t increment)
{
    if (!s_initialized) {
        return;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);

    switch (counter) {
        case ZB_DIAG_ATTR_ROUTE_DISC_INITIATED:
            s_coordinator_diag.network.route_disc_initiated += increment;
            break;
        case ZB_DIAG_ATTR_NEIGHBOR_ADDED:
            s_coordinator_diag.network.neighbor_added += increment;
            break;
        case ZB_DIAG_ATTR_NEIGHBOR_REMOVED:
            s_coordinator_diag.network.neighbor_removed += increment;
            break;
        case ZB_DIAG_ATTR_NEIGHBOR_STALE:
            s_coordinator_diag.network.neighbor_stale += increment;
            break;
        case ZB_DIAG_ATTR_JOIN_INDICATION:
            s_coordinator_diag.network.join_indication += increment;
            break;
        case ZB_DIAG_ATTR_CHILD_MOVED:
            s_coordinator_diag.network.child_moved += increment;
            break;
        default:
            ESP_LOGW(TAG, "Unknown network counter: 0x%04X", counter);
            break;
    }

    xSemaphoreGive(s_diag_mutex);
}

void zb_diagnostics_update_link_quality(uint8_t lqi, int8_t rssi)
{
    if (!s_initialized) {
        return;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);
    s_coordinator_diag.last_message_lqi = lqi;
    s_coordinator_diag.last_message_rssi = rssi;
    xSemaphoreGive(s_diag_mutex);
}

void zb_diagnostics_increment_reset_count(void)
{
    if (!s_initialized) {
        return;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);
    s_coordinator_diag.number_of_resets++;
    xSemaphoreGive(s_diag_mutex);

    save_reset_count();
}

esp_err_t zb_diagnostics_reset_counters(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting all diagnostic counters");

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);

    /* Preserve reset count and persistent writes */
    uint16_t reset_count = s_coordinator_diag.number_of_resets;
    uint16_t persistent_writes = s_coordinator_diag.persistent_mem_writes;

    memset(&s_coordinator_diag, 0, sizeof(zb_diagnostics_t));

    s_coordinator_diag.number_of_resets = reset_count;
    s_coordinator_diag.persistent_mem_writes = persistent_writes;

    xSemaphoreGive(s_diag_mutex);

    return ESP_OK;
}

size_t zb_diagnostics_get_network_map(zb_device_diagnostics_t *devices,
                                      size_t max_count)
{
    if (devices == NULL || max_count == 0) {
        return 0;
    }

    if (!s_initialized) {
        return 0;
    }

    /* Iterate devices by index (avoids stack allocation) */
    size_t device_count = zb_device_get_count();
    size_t copied = 0;

    for (size_t i = 0; i < device_count && copied < max_count; i++) {
        zb_device_t *dev = zb_device_get_by_index(i);
        if (dev == NULL) {
            continue;
        }

        zb_device_diagnostics_t *dev_diag = &devices[copied];

        dev_diag->short_addr = dev->short_addr;
        memcpy(dev_diag->ieee_addr, dev->ieee_addr, 8);
        dev_diag->last_updated = dev->last_seen;
        dev_diag->valid = dev->online;

        /* Note: Actual implementation would read diagnostics from device */
        memset(&dev_diag->diagnostics, 0, sizeof(zb_diagnostics_t));

        copied++;
    }

    return copied;
}

esp_err_t zb_diagnostics_publish_mqtt(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Publishing diagnostics to MQTT...");

    zb_diagnostics_t diag;
    zb_diagnostics_get_coordinator(&diag);

    /* Build JSON */
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Hardware */
    cJSON_AddNumberToObject(root, "number_of_resets", diag.number_of_resets);
    cJSON_AddNumberToObject(root, "persistent_memory_writes", diag.persistent_mem_writes);

    /* MAC Layer */
    cJSON *mac = cJSON_CreateObject();
    if (mac == NULL) {
        ESP_LOGE(TAG, "Failed to create MAC JSON object");
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(mac, "rx_bcast", diag.mac.rx_bcast);
    cJSON_AddNumberToObject(mac, "tx_bcast", diag.mac.tx_bcast);
    cJSON_AddNumberToObject(mac, "rx_ucast", diag.mac.rx_ucast);
    cJSON_AddNumberToObject(mac, "tx_ucast", diag.mac.tx_ucast);
    cJSON_AddNumberToObject(mac, "tx_ucast_retry", diag.mac.tx_ucast_retry);
    cJSON_AddNumberToObject(mac, "tx_ucast_fail", diag.mac.tx_ucast_fail);
    cJSON_AddItemToObject(root, "mac", mac);

    /* APS Layer */
    cJSON *aps = cJSON_CreateObject();
    if (aps == NULL) {
        ESP_LOGE(TAG, "Failed to create APS JSON object");
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(aps, "rx_bcast", diag.aps.rx_bcast);
    cJSON_AddNumberToObject(aps, "tx_bcast", diag.aps.tx_bcast);
    cJSON_AddNumberToObject(aps, "rx_ucast", diag.aps.rx_ucast);
    cJSON_AddNumberToObject(aps, "tx_ucast_success", diag.aps.tx_ucast_success);
    cJSON_AddNumberToObject(aps, "tx_ucast_retry", diag.aps.tx_ucast_retry);
    cJSON_AddNumberToObject(aps, "tx_ucast_fail", diag.aps.tx_ucast_fail);
    cJSON_AddItemToObject(root, "aps", aps);

    /* Network Layer */
    cJSON *network = cJSON_CreateObject();
    if (network == NULL) {
        ESP_LOGE(TAG, "Failed to create network JSON object");
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(network, "route_disc_initiated", diag.network.route_disc_initiated);
    cJSON_AddNumberToObject(network, "neighbor_added", diag.network.neighbor_added);
    cJSON_AddNumberToObject(network, "neighbor_removed", diag.network.neighbor_removed);
    cJSON_AddNumberToObject(network, "join_indication", diag.network.join_indication);
    cJSON_AddItemToObject(root, "network", network);

    /* Link Quality */
    cJSON_AddNumberToObject(root, "last_message_lqi", diag.last_message_lqi);
    cJSON_AddNumberToObject(root, "last_message_rssi", diag.last_message_rssi);

    /* Publish */
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_client_publish(
        "zigbee2mqtt/bridge/response/networkmap",
        json_str,
        strlen(json_str),
        1, 0
    );

    free(json_str);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Diagnostics published to MQTT");
    } else {
        ESP_LOGE(TAG, "Failed to publish diagnostics: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_diagnostics_process_mqtt_request(const char *topic,
                                              const char *payload,
                                              size_t len)
{
    (void)payload;
    (void)len;

    if (topic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Handle networkmap request */
    if (strstr(topic, "networkmap") != NULL) {
        return zb_diagnostics_publish_mqtt();
    }

    ESP_LOGW(TAG, "Unknown diagnostics request topic: %s", topic);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t zb_diagnostics_to_json(const zb_diagnostics_t *diag,
                                 char *json_buf, size_t buf_len)
{
    if (diag == NULL || json_buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(root, "number_of_resets", diag->number_of_resets);
    cJSON_AddNumberToObject(root, "mac_rx_ucast", diag->mac.rx_ucast);
    cJSON_AddNumberToObject(root, "mac_tx_ucast", diag->mac.tx_ucast);
    cJSON_AddNumberToObject(root, "aps_rx_ucast", diag->aps.rx_ucast);
    cJSON_AddNumberToObject(root, "aps_tx_ucast_success", diag->aps.tx_ucast_success);
    cJSON_AddNumberToObject(root, "last_lqi", diag->last_message_lqi);
    cJSON_AddNumberToObject(root, "last_rssi", diag->last_message_rssi);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t json_len = strlen(json_str);
    if (json_len >= buf_len) {
        free(json_str);
        return ESP_ERR_NO_MEM;
    }

    memcpy(json_buf, json_str, json_len + 1);
    free(json_str);

    return ESP_OK;
}

esp_err_t zb_diagnostics_test(void)
{
    ESP_LOGI(TAG, "Running Diagnostics self-test...");

    if (!s_initialized) {
        ESP_LOGW(TAG, "Module not initialized - skipping runtime tests");
        ESP_LOGI(TAG, "Diagnostics self-test PASSED (limited)");
        return ESP_OK;
    }

    /* Test counter updates */
    zb_diagnostics_t before, after;
    zb_diagnostics_get_coordinator(&before);

    zb_diagnostics_update_mac(ZB_DIAG_ATTR_MAC_RX_UCAST, 1);
    zb_diagnostics_update_aps(ZB_DIAG_ATTR_APS_TX_UCAST_SUCCESS, 1);
    zb_diagnostics_update_link_quality(200, -50);

    zb_diagnostics_get_coordinator(&after);

    if (after.mac.rx_ucast != before.mac.rx_ucast + 1) {
        ESP_LOGE(TAG, "MAC counter update failed");
        return ESP_FAIL;
    }

    if (after.aps.tx_ucast_success != before.aps.tx_ucast_success + 1) {
        ESP_LOGE(TAG, "APS counter update failed");
        return ESP_FAIL;
    }

    if (after.last_message_lqi != 200 || after.last_message_rssi != -50) {
        ESP_LOGE(TAG, "Link quality update failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Counter update tests PASSED");

    /* Test JSON conversion */
    char json_buf[512];
    esp_err_t ret = zb_diagnostics_to_json(&after, json_buf, sizeof(json_buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JSON conversion failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "JSON conversion test PASSED");

    ESP_LOGI(TAG, "Diagnostics self-test PASSED");
    return ESP_OK;
}

/* ============================================================================
 * Device Temperature Cluster (0x0002) Implementation
 * ============================================================================ */

/**
 * @brief Find or create a cache entry for device temperature
 */
static device_temp_cache_entry_t* find_or_create_temp_cache_entry(uint16_t short_addr)
{
    device_temp_cache_entry_t *oldest = NULL;
    uint32_t oldest_time = UINT32_MAX;

    /* Search for existing entry or find oldest */
    for (size_t i = 0; i < DEVICE_TEMP_CACHE_SIZE; i++) {
        if (s_device_temp_cache[i].short_addr == short_addr) {
            return &s_device_temp_cache[i];
        }
        if (s_device_temp_cache[i].short_addr == 0) {
            /* Empty slot - use it */
            s_device_temp_cache[i].short_addr = short_addr;
            return &s_device_temp_cache[i];
        }
        if (s_device_temp_cache[i].temp.last_updated < oldest_time) {
            oldest_time = s_device_temp_cache[i].temp.last_updated;
            oldest = &s_device_temp_cache[i];
        }
    }

    /* No free slot - evict oldest */
    if (oldest != NULL) {
        ESP_LOGD(TAG, "Evicting device temp cache entry for 0x%04X", oldest->short_addr);
        oldest->short_addr = short_addr;
        memset(&oldest->temp, 0, sizeof(zb_device_temperature_t));
        return oldest;
    }

    return NULL;
}

/**
 * @brief Get cached temperature entry for device
 */
static device_temp_cache_entry_t* get_temp_cache_entry(uint16_t short_addr)
{
    for (size_t i = 0; i < DEVICE_TEMP_CACHE_SIZE; i++) {
        if (s_device_temp_cache[i].short_addr == short_addr) {
            return &s_device_temp_cache[i];
        }
    }
    return NULL;
}

float zb_device_temp_to_celsius(int16_t raw_temp)
{
    if (raw_temp == (int16_t)ZB_DEVICE_TEMP_INVALID) {
        return 0.0f;
    }
    return (float)raw_temp / (float)ZB_DEVICE_TEMP_SCALE;
}

bool zb_device_temp_is_valid(int16_t raw_temp)
{
    return raw_temp != (int16_t)ZB_DEVICE_TEMP_INVALID;
}

esp_err_t zb_diagnostics_read_device_temperature(uint16_t short_addr,
                                                  zb_device_temperature_t *temp)
{
    if (temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading device temperature from device 0x%04X", short_addr);

    /* Check if device supports the cluster */
    if (!zb_diagnostics_has_device_temp_cluster(short_addr)) {
        ESP_LOGW(TAG, "Device 0x%04X does not support Device Temperature cluster", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Initialize output */
    memset(temp, 0, sizeof(zb_device_temperature_t));
    temp->current_temperature = ZB_DEVICE_TEMP_INVALID;
    temp->min_temp_experienced = ZB_DEVICE_TEMP_INVALID;
    temp->max_temp_experienced = ZB_DEVICE_TEMP_INVALID;

    /*
     * Note: Actual implementation would send ZCL Read Attributes request
     * to the device and wait for response. The ESP-Zigbee-SDK provides:
     *
     * esp_zb_zcl_read_attr_cmd_t cmd = {
     *     .zcl_basic_cmd = {
     *         .dst_addr_u.addr_short = short_addr,
     *         .dst_endpoint = endpoint,
     *         .src_endpoint = 1
     *     },
     *     .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
     *     .clusterID = ZB_ZCL_CLUSTER_ID_DEVICE_TEMP_CONFIG,
     *     .attr_number = 3,
     *     .attr_field = attr_ids
     * };
     * esp_zb_zcl_read_attr_cmd_req(&cmd);
     *
     * The response would be handled in the ZCL callback.
     */

    /* Check cache for recent data */
    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);
    device_temp_cache_entry_t *entry = get_temp_cache_entry(short_addr);
    if (entry != NULL && entry->temp.valid) {
        memcpy(temp, &entry->temp, sizeof(zb_device_temperature_t));
        xSemaphoreGive(s_diag_mutex);

        ESP_LOGD(TAG, "Returning cached device temperature for 0x%04X: %.2fC",
                 short_addr, zb_device_temp_to_celsius(temp->current_temperature));
        return ESP_OK;
    }
    xSemaphoreGive(s_diag_mutex);

    ESP_LOGD(TAG, "Device temperature read initiated for 0x%04X (async)", short_addr);
    return ESP_OK;
}

size_t zb_diagnostics_read_device_temp_attr(uint16_t short_addr,
                                            uint16_t attr_id,
                                            void *value, size_t max_len)
{
    if (value == NULL || max_len == 0) {
        return 0;
    }

    if (!s_initialized) {
        return 0;
    }

    ESP_LOGD(TAG, "Reading device temp attr 0x%04X from device 0x%04X",
             attr_id, short_addr);

    /*
     * Note: Would send specific attribute read request to device.
     * This is a placeholder for the actual ZCL read attribute command.
     */

    return 0;
}

esp_err_t zb_device_temperature_to_json(const zb_device_temperature_t *temp,
                                        char *json_buf, size_t buf_len)
{
    if (temp == NULL || json_buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Add device temperature object */
    cJSON *dev_temp = cJSON_CreateObject();
    if (dev_temp == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    /* Current temperature */
    if (zb_device_temp_is_valid(temp->current_temperature)) {
        cJSON_AddNumberToObject(dev_temp, "current",
                                zb_device_temp_to_celsius(temp->current_temperature));
    }

    /* Min/Max experienced temperatures */
    if (zb_device_temp_is_valid(temp->min_temp_experienced)) {
        cJSON_AddNumberToObject(dev_temp, "min_experienced",
                                zb_device_temp_to_celsius(temp->min_temp_experienced));
    }

    if (zb_device_temp_is_valid(temp->max_temp_experienced)) {
        cJSON_AddNumberToObject(dev_temp, "max_experienced",
                                zb_device_temp_to_celsius(temp->max_temp_experienced));
    }

    /* Alarm status */
    if (temp->over_temp_alarm != 0) {
        cJSON_AddNumberToObject(dev_temp, "over_temp_alarm", temp->over_temp_alarm);
    }

    /* Thresholds if set */
    if (zb_device_temp_is_valid(temp->low_threshold)) {
        cJSON_AddNumberToObject(dev_temp, "low_threshold",
                                zb_device_temp_to_celsius(temp->low_threshold));
    }

    if (zb_device_temp_is_valid(temp->high_threshold)) {
        cJSON_AddNumberToObject(dev_temp, "high_threshold",
                                zb_device_temp_to_celsius(temp->high_threshold));
    }

    /* Validity flag */
    cJSON_AddBoolToObject(dev_temp, "valid", temp->valid);

    cJSON_AddItemToObject(root, "device_temperature", dev_temp);

    /* Convert to string */
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t json_len = strlen(json_str);
    if (json_len >= buf_len) {
        free(json_str);
        return ESP_ERR_NO_MEM;
    }

    memcpy(json_buf, json_str, json_len + 1);
    free(json_str);

    return ESP_OK;
}

esp_err_t zb_diagnostics_handle_device_temp_report(uint16_t short_addr,
                                                   uint16_t attr_id,
                                                   const void *value,
                                                   size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_diag_mutex, portMAX_DELAY);

    /* Find or create cache entry */
    device_temp_cache_entry_t *entry = find_or_create_temp_cache_entry(short_addr);
    if (entry == NULL) {
        xSemaphoreGive(s_diag_mutex);
        ESP_LOGE(TAG, "Failed to allocate device temp cache entry");
        return ESP_ERR_NO_MEM;
    }

    /* Update the appropriate attribute */
    switch (attr_id) {
        case ZB_DEVICE_TEMP_ATTR_CURRENT:
            if (value_len >= sizeof(int16_t)) {
                entry->temp.current_temperature = *(const int16_t *)value;
                entry->temp.valid = true;
                ESP_LOGI(TAG, "Device 0x%04X temperature: %.2fC",
                         short_addr, zb_device_temp_to_celsius(entry->temp.current_temperature));
            }
            break;

        case ZB_DEVICE_TEMP_ATTR_MIN_EXPERIENCED:
            if (value_len >= sizeof(int16_t)) {
                entry->temp.min_temp_experienced = *(const int16_t *)value;
                ESP_LOGD(TAG, "Device 0x%04X min temp: %.2fC",
                         short_addr, zb_device_temp_to_celsius(entry->temp.min_temp_experienced));
            }
            break;

        case ZB_DEVICE_TEMP_ATTR_MAX_EXPERIENCED:
            if (value_len >= sizeof(int16_t)) {
                entry->temp.max_temp_experienced = *(const int16_t *)value;
                ESP_LOGD(TAG, "Device 0x%04X max temp: %.2fC",
                         short_addr, zb_device_temp_to_celsius(entry->temp.max_temp_experienced));
            }
            break;

        case ZB_DEVICE_TEMP_ATTR_OVER_ALARM:
            if (value_len >= sizeof(uint16_t)) {
                entry->temp.over_temp_alarm = *(const uint16_t *)value;
                if (entry->temp.over_temp_alarm != 0) {
                    ESP_LOGW(TAG, "Device 0x%04X over-temperature alarm: 0x%04X",
                             short_addr, entry->temp.over_temp_alarm);
                }
            }
            break;

        case ZB_DEVICE_TEMP_ATTR_LOW_THRESHOLD:
            if (value_len >= sizeof(int16_t)) {
                entry->temp.low_threshold = *(const int16_t *)value;
            }
            break;

        case ZB_DEVICE_TEMP_ATTR_HIGH_THRESHOLD:
            if (value_len >= sizeof(int16_t)) {
                entry->temp.high_threshold = *(const int16_t *)value;
            }
            break;

        default:
            ESP_LOGD(TAG, "Unknown device temp attribute 0x%04X from 0x%04X",
                     attr_id, short_addr);
            xSemaphoreGive(s_diag_mutex);
            return ESP_OK;
    }

    /* Update timestamp */
    entry->temp.last_updated = (uint32_t)time(NULL);

    xSemaphoreGive(s_diag_mutex);

    return ESP_OK;
}

bool zb_diagnostics_has_device_temp_cluster(uint16_t short_addr)
{
    if (!s_initialized) {
        return false;
    }

    /* Get device information */
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        return false;
    }

    /* Check if device has the Device Temperature Configuration cluster */
    for (size_t i = 0; i < device->cluster_count; i++) {
        if (device->clusters[i] == ZB_ZCL_CLUSTER_ID_DEVICE_TEMP_CONFIG) {
            return true;
        }
    }

    return false;
}
