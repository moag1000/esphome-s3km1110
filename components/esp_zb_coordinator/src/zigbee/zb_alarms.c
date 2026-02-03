/**
 * @file zb_alarms.c
 * @brief Zigbee Alarms Cluster (0x0009) Implementation
 *
 * Implements the ZCL Alarms Cluster for ESP32-C5 Zigbee2MQTT Gateway.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_alarms.h"
#include "zb_device_handler.h"
#include "zb_coordinator.h"
#include "compat_stubs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <time.h>
#include <inttypes.h>

static const char *TAG = "ZB_ALARMS";

/* ============================================================================
 * Module State
 * ============================================================================ */

static bool s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

/* Alarm table storage - allocated in PSRAM to save internal RAM */
static zb_device_alarms_t *s_device_alarms = NULL;
static size_t s_device_count = 0;

/* Callbacks */
static zb_alarms_notification_cb_t s_notification_cb = NULL;
static zb_alarms_cleared_cb_t s_cleared_cb = NULL;

/* Statistics */
static uint32_t s_total_alarms_received = 0;
static uint32_t s_total_alarms_cleared = 0;

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Find device alarms entry by short address
 *
 * @param[in] short_addr Device short address
 * @return Pointer to device alarms or NULL if not found
 */
static zb_device_alarms_t* find_device_alarms(uint16_t short_addr)
{
    for (size_t i = 0; i < s_device_count; i++) {
        if (s_device_alarms[i].short_addr == short_addr) {
            return &s_device_alarms[i];
        }
    }
    return NULL;
}

/**
 * @brief Get or create device alarms entry
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return Pointer to device alarms or NULL if table full
 */
static zb_device_alarms_t* get_or_create_device_alarms(uint16_t short_addr, uint8_t endpoint)
{
    /* Check if device already has an entry */
    zb_device_alarms_t *entry = find_device_alarms(short_addr);
    if (entry != NULL) {
        return entry;
    }

    /* Create new entry */
    if (s_device_count >= ZB_ALARMS_MAX_DEVICES) {
        ESP_LOGW(TAG, "Alarm device table full, cannot track alarms for 0x%04X", short_addr);
        return NULL;
    }

    entry = &s_device_alarms[s_device_count];
    memset(entry, 0, sizeof(zb_device_alarms_t));
    entry->short_addr = short_addr;
    entry->endpoint = endpoint;
    s_device_count++;

    ESP_LOGI(TAG, "Created alarm entry for device 0x%04X", short_addr);
    return entry;
}

/**
 * @brief Find alarm entry in device's alarm table
 *
 * @param[in] device Device alarms structure
 * @param[in] alarm_code Alarm code to find
 * @param[in] cluster_id Cluster ID to match
 * @return Index of alarm or -1 if not found
 */
static int find_alarm_in_device(const zb_device_alarms_t *device, uint8_t alarm_code, uint16_t cluster_id)
{
    for (uint8_t i = 0; i < device->alarm_count; i++) {
        if (device->alarms[i].valid &&
            device->alarms[i].alarm_code == alarm_code &&
            device->alarms[i].cluster_id == cluster_id) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Get current timestamp
 *
 * @return Current time in seconds since epoch
 */
static uint32_t get_current_timestamp(void)
{
    return (uint32_t)time(NULL);
}

/**
 * @brief Publish alarm event to MQTT
 *
 * @param[in] short_addr Device short address
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster ID
 * @param[in] active true if alarm is active, false if cleared
 */
static void publish_alarm_mqtt(uint16_t short_addr, uint8_t alarm_code,
                               uint16_t cluster_id, bool active)
{
    REQUIRE_MQTT_CONNECTED_VOID();

    /* Get device friendly name */
    zb_device_t *device = zb_device_get(short_addr);
    const char *friendly_name = device ? device->friendly_name : "unknown";

    /* Build topic: zigbee2mqtt/<device>/alarm */
    char topic[128];
    snprintf(topic, sizeof(topic), "zigbee2mqtt/%s/alarm", friendly_name);

    /* Build JSON payload */
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"alarm_code\":%u,\"cluster_id\":\"0x%04X\",\"cluster_name\":\"%s\","
             "\"alarm_description\":\"%s\",\"active\":%s,\"timestamp\":%" PRIu32 "}",
             alarm_code, cluster_id,
             zb_alarms_cluster_name(cluster_id),
             zb_alarms_code_to_string(alarm_code, cluster_id),
             active ? "true" : "false",
             get_current_timestamp());

    esp_err_t ret = mqtt_client_publish(topic, payload, 0, 0, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish alarm event: %s", esp_err_to_name(ret));
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_alarms_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Alarms module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create mutex */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate alarm storage in PSRAM (saves ~2KB internal RAM) */
    if (s_device_alarms == NULL) {
        s_device_alarms = heap_caps_calloc(ZB_ALARMS_MAX_DEVICES, sizeof(zb_device_alarms_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_device_alarms) {
            ESP_LOGW(TAG, "PSRAM alloc failed, falling back to internal RAM");
            s_device_alarms = calloc(ZB_ALARMS_MAX_DEVICES, sizeof(zb_device_alarms_t));
        }
        if (!s_device_alarms) {
            ESP_LOGE(TAG, "Failed to allocate alarm storage");
            vSemaphoreDelete(s_mutex);
            s_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    } else {
        memset(s_device_alarms, 0, ZB_ALARMS_MAX_DEVICES * sizeof(zb_device_alarms_t));
    }
    s_device_count = 0;
    s_notification_cb = NULL;
    s_cleared_cb = NULL;
    s_total_alarms_received = 0;
    s_total_alarms_cleared = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "Alarms module initialized (max %d devices, %d alarms/device)",
             ZB_ALARMS_MAX_DEVICES, ZB_ALARMS_MAX_PER_DEVICE);
    return ESP_OK;
}

esp_err_t zb_alarms_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_initialized = false;
    s_device_count = 0;
    s_notification_cb = NULL;
    s_cleared_cb = NULL;
    xSemaphoreGive(s_mutex);

    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;

    ESP_LOGI(TAG, "Alarms module deinitialized (total received: %" PRIu32 ", cleared: %" PRIu32 ")",
             s_total_alarms_received, s_total_alarms_cleared);
    return ESP_OK;
}

esp_err_t zb_alarms_register_notification_cb(zb_alarms_notification_cb_t callback)
{
    s_notification_cb = callback;
    ESP_LOGI(TAG, "Alarm notification callback %s",
             callback ? "registered" : "unregistered");
    return ESP_OK;
}

esp_err_t zb_alarms_register_cleared_cb(zb_alarms_cleared_cb_t callback)
{
    s_cleared_cb = callback;
    ESP_LOGI(TAG, "Alarm cleared callback %s",
             callback ? "registered" : "unregistered");
    return ESP_OK;
}

/* ============================================================================
 * Alarm Table Management
 * ============================================================================ */

esp_err_t zb_alarms_get_device_alarms(uint16_t short_addr, zb_alarm_entry_t *alarms, uint8_t *count)
{
    if (alarms == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_device_alarms_t *device = find_device_alarms(short_addr);
    if (device == NULL || device->alarm_count == 0) {
        xSemaphoreGive(s_mutex);
        *count = 0;
        return ESP_ERR_NOT_FOUND;
    }

    /* Copy valid alarms */
    uint8_t copied = 0;
    for (uint8_t i = 0; i < device->alarm_count && copied < ZB_ALARMS_MAX_PER_DEVICE; i++) {
        if (device->alarms[i].valid) {
            memcpy(&alarms[copied], &device->alarms[i], sizeof(zb_alarm_entry_t));
            copied++;
        }
    }

    *count = copied;
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

uint8_t zb_alarms_get_device_alarm_count(uint16_t short_addr)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_device_alarms_t *device = find_device_alarms(short_addr);
    uint8_t count = device ? device->alarm_count : 0;

    xSemaphoreGive(s_mutex);
    return count;
}

uint32_t zb_alarms_get_total_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uint32_t total = 0;
    for (size_t i = 0; i < s_device_count; i++) {
        total += s_device_alarms[i].alarm_count;
    }

    xSemaphoreGive(s_mutex);
    return total;
}

bool zb_alarms_device_has_alarms(uint16_t short_addr)
{
    return zb_alarms_get_device_alarm_count(short_addr) > 0;
}

esp_err_t zb_alarms_get_latest(uint16_t short_addr, zb_alarm_entry_t *alarm)
{
    if (alarm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_device_alarms_t *device = find_device_alarms(short_addr);
    if (device == NULL || device->alarm_count == 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Find most recent alarm by timestamp */
    zb_alarm_entry_t *latest = NULL;
    uint32_t latest_time = 0;

    for (uint8_t i = 0; i < device->alarm_count; i++) {
        if (device->alarms[i].valid && device->alarms[i].timestamp >= latest_time) {
            latest = &device->alarms[i];
            latest_time = device->alarms[i].timestamp;
        }
    }

    if (latest == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(alarm, latest, sizeof(zb_alarm_entry_t));
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

/* ============================================================================
 * Alarm Commands (Client to Server)
 * ============================================================================ */

esp_err_t zb_alarms_cmd_reset_alarm(uint16_t short_addr, uint8_t endpoint,
                                     uint8_t alarm_code, uint16_t cluster_id)
{
    ESP_LOGI(TAG, "Sending Reset Alarm to 0x%04X EP%d: code=%d cluster=0x%04X",
             short_addr, endpoint, alarm_code, cluster_id);

    /* Build Reset Alarm command payload: alarm_code (uint8) + cluster_id (uint16) */
    uint8_t payload[3];
    payload[0] = alarm_code;
    payload[1] = (uint8_t)(cluster_id & 0xFF);
    payload[2] = (uint8_t)((cluster_id >> 8) & 0xFF);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_ALARMS,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_ALARMS_RESET_ALARM_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_SET,
            .size = sizeof(payload),
            .value = payload,
        },
    };

    /* Acquire Zigbee lock for thread safety */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Reset Alarm command: %s", esp_err_to_name(ret));
    } else {
        /* Remove from local table */
        zb_alarms_remove_from_table(short_addr, alarm_code, cluster_id);
    }

    return ret;
}

esp_err_t zb_alarms_cmd_reset_all(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Reset All Alarms to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_ALARMS,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_ALARMS_RESET_ALL_ALARMS_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = 0,
            .value = NULL,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Reset All Alarms command: %s", esp_err_to_name(ret));
    } else {
        /* Clear from local table */
        zb_alarms_clear_device(short_addr);

        /* Invoke cleared callback */
        if (s_cleared_cb != NULL) {
            s_cleared_cb(short_addr, endpoint, ZB_ALARM_CODE_INVALID, 0xFFFF);
        }
    }

    return ret;
}

esp_err_t zb_alarms_cmd_get_alarm(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Get Alarm to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_ALARMS,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_ALARMS_GET_ALARM_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = 0,
            .value = NULL,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Get Alarm command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_alarms_cmd_reset_alarm_log(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Reset Alarm Log to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_ALARMS,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_ALARMS_RESET_ALARM_LOG_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = 0,
            .value = NULL,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Reset Alarm Log command: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* ============================================================================
 * Alarm Notifications Handling (Server to Client)
 * ============================================================================ */

esp_err_t zb_alarms_handle_notification(uint16_t short_addr, uint8_t endpoint,
                                         uint8_t alarm_code, uint16_t cluster_id)
{
    ESP_LOGI(TAG, "Alarm notification from 0x%04X EP%d: code=%d cluster=0x%04X",
             short_addr, endpoint, alarm_code, cluster_id);

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t timestamp = get_current_timestamp();

    /* Add to local alarm table */
    esp_err_t ret = zb_alarms_add_to_table(short_addr, endpoint, alarm_code, cluster_id, timestamp);
    if (ret != ESP_OK && ret != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "Failed to add alarm to table: %s", esp_err_to_name(ret));
    }

    s_total_alarms_received++;

    /* Invoke notification callback */
    if (s_notification_cb != NULL) {
        s_notification_cb(short_addr, endpoint, alarm_code, cluster_id, timestamp);
    }

    /* Publish to MQTT */
    publish_alarm_mqtt(short_addr, alarm_code, cluster_id, true);

    /* Log alarm details */
    ESP_LOGW(TAG, "ALARM: Device 0x%04X, Cluster %s (0x%04X), Code %d: %s",
             short_addr,
             zb_alarms_cluster_name(cluster_id),
             cluster_id,
             alarm_code,
             zb_alarms_code_to_string(alarm_code, cluster_id));

    return ESP_OK;
}

esp_err_t zb_alarms_handle_get_response(uint16_t short_addr, uint8_t endpoint,
                                         uint8_t status, uint8_t alarm_code,
                                         uint16_t cluster_id, uint32_t timestamp)
{
    ESP_LOGI(TAG, "Get Alarm Response from 0x%04X EP%d: status=%d",
             short_addr, endpoint, status);

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == 0x00) {
        /* Success - alarm data is valid */
        ESP_LOGI(TAG, "  Alarm: code=%d cluster=0x%04X timestamp=%" PRIu32,
                 alarm_code, cluster_id, timestamp);

        /* Add to local table if not already present */
        zb_alarms_add_to_table(short_addr, endpoint, alarm_code, cluster_id, timestamp);

        /* Invoke notification callback */
        if (s_notification_cb != NULL) {
            s_notification_cb(short_addr, endpoint, alarm_code, cluster_id, timestamp);
        }
    } else {
        /* No alarms present */
        ESP_LOGI(TAG, "  No alarms on device");
    }

    return ESP_OK;
}

/* ============================================================================
 * Alarm Table Local Management
 * ============================================================================ */

esp_err_t zb_alarms_add_to_table(uint16_t short_addr, uint8_t endpoint,
                                  uint8_t alarm_code, uint16_t cluster_id,
                                  uint32_t timestamp)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Get or create device entry */
    zb_device_alarms_t *device = get_or_create_device_alarms(short_addr, endpoint);
    if (device == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Check if alarm already exists */
    int existing_idx = find_alarm_in_device(device, alarm_code, cluster_id);
    if (existing_idx >= 0) {
        /* Update timestamp of existing alarm */
        device->alarms[existing_idx].timestamp = timestamp ? timestamp : get_current_timestamp();
        device->last_alarm_time = device->alarms[existing_idx].timestamp;
        xSemaphoreGive(s_mutex);
        ESP_LOGD(TAG, "Updated existing alarm for 0x%04X", short_addr);
        return ESP_OK;
    }

    /* Find free slot */
    if (device->alarm_count >= ZB_ALARMS_MAX_PER_DEVICE) {
        /* Overwrite oldest alarm (FIFO) */
        uint32_t oldest_time = UINT32_MAX;
        int oldest_idx = 0;
        for (uint8_t i = 0; i < device->alarm_count; i++) {
            if (device->alarms[i].valid && device->alarms[i].timestamp < oldest_time) {
                oldest_time = device->alarms[i].timestamp;
                oldest_idx = i;
            }
        }
        ESP_LOGW(TAG, "Alarm table full for 0x%04X, overwriting oldest entry", short_addr);

        device->alarms[oldest_idx].alarm_code = alarm_code;
        device->alarms[oldest_idx].cluster_id = cluster_id;
        device->alarms[oldest_idx].timestamp = timestamp ? timestamp : get_current_timestamp();
        device->alarms[oldest_idx].valid = true;
        device->last_alarm_time = device->alarms[oldest_idx].timestamp;
    } else {
        /* Add new entry */
        uint8_t idx = device->alarm_count;
        device->alarms[idx].alarm_code = alarm_code;
        device->alarms[idx].cluster_id = cluster_id;
        device->alarms[idx].timestamp = timestamp ? timestamp : get_current_timestamp();
        device->alarms[idx].valid = true;
        device->alarm_count++;
        device->last_alarm_time = device->alarms[idx].timestamp;

        ESP_LOGI(TAG, "Added alarm to table for 0x%04X (count: %d)",
                 short_addr, device->alarm_count);
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t zb_alarms_remove_from_table(uint16_t short_addr, uint8_t alarm_code,
                                       uint16_t cluster_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_device_alarms_t *device = find_device_alarms(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    int idx = find_alarm_in_device(device, alarm_code, cluster_id);
    if (idx < 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Mark as invalid and compact array */
    device->alarms[idx].valid = false;

    /* Shift remaining entries */
    for (uint8_t i = idx; i < device->alarm_count - 1; i++) {
        memcpy(&device->alarms[i], &device->alarms[i + 1], sizeof(zb_alarm_entry_t));
    }
    device->alarm_count--;

    s_total_alarms_cleared++;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Removed alarm from table for 0x%04X (remaining: %d)",
             short_addr, device->alarm_count);

    /* Publish cleared event */
    publish_alarm_mqtt(short_addr, alarm_code, cluster_id, false);

    /* Invoke cleared callback */
    if (s_cleared_cb != NULL) {
        s_cleared_cb(short_addr, device->endpoint, alarm_code, cluster_id);
    }

    return ESP_OK;
}

esp_err_t zb_alarms_clear_device(uint16_t short_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_device_alarms_t *device = find_device_alarms(short_addr);
    if (device != NULL) {
        uint8_t cleared_count = device->alarm_count;
        device->alarm_count = 0;
        memset(device->alarms, 0, sizeof(device->alarms));
        s_total_alarms_cleared += cleared_count;

        ESP_LOGI(TAG, "Cleared %d alarms for device 0x%04X", cleared_count, short_addr);
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t zb_alarms_clear_all(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uint32_t total_cleared = 0;
    for (size_t i = 0; i < s_device_count; i++) {
        total_cleared += s_device_alarms[i].alarm_count;
        s_device_alarms[i].alarm_count = 0;
        memset(s_device_alarms[i].alarms, 0, sizeof(s_device_alarms[i].alarms));
    }

    s_total_alarms_cleared += total_cleared;

    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Cleared all alarms (total: %" PRIu32 ")", total_cleared);
    return ESP_OK;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

const char* zb_alarms_code_to_string(uint8_t alarm_code, uint16_t cluster_id)
{
    /* Generic alarm codes (used by multiple clusters) */
    switch (alarm_code) {
        case 0x00: return "General alarm";
        case 0x01: return "Hardware fault";
        case 0x02: return "Software fault";
        default: break;
    }

    /* Cluster-specific alarm codes */
    switch (cluster_id) {
        case 0x0201:  /* Thermostat */
            switch (alarm_code) {
                case 0x00: return "Initialization failure";
                case 0x01: return "Hardware failure";
                case 0x02: return "Self-calibration failure";
                default: break;
            }
            break;

        case 0x0702:  /* Metering */
            switch (alarm_code) {
                case 0x00: return "Check meter";
                case 0x01: return "Low battery";
                case 0x02: return "Tamper detect";
                case 0x03: return "Power failure";
                case 0x04: return "Pipeline empty";
                case 0x05: return "Temperature sensor error";
                case 0x06: return "Burst detect";
                default: break;
            }
            break;

        case 0x0B04:  /* Electrical Measurement */
            switch (alarm_code) {
                case 0x00: return "Voltage too high";
                case 0x01: return "Voltage too low";
                case 0x02: return "Current too high";
                case 0x03: return "Power overload";
                default: break;
            }
            break;

        case 0x0500:  /* IAS Zone */
            switch (alarm_code) {
                case 0x00: return "Intrusion";
                case 0x01: return "Tamper";
                case 0x02: return "Low battery";
                case 0x03: return "Supervision check";
                default: break;
            }
            break;

        default:
            break;
    }

    /* Unknown alarm code */
    static char unknown_buf[24];
    snprintf(unknown_buf, sizeof(unknown_buf), "Unknown alarm (0x%02X)", alarm_code);
    return unknown_buf;
}

const char* zb_alarms_cluster_name(uint16_t cluster_id)
{
    switch (cluster_id) {
        case 0x0000: return "Basic";
        case 0x0001: return "Power Configuration";
        case 0x0002: return "Device Temperature";
        case 0x0003: return "Identify";
        case 0x0006: return "On/Off";
        case 0x0008: return "Level Control";
        case 0x0009: return "Alarms";
        case 0x0101: return "Door Lock";
        case 0x0102: return "Window Covering";
        case 0x0201: return "Thermostat";
        case 0x0202: return "Fan Control";
        case 0x0300: return "Color Control";
        case 0x0400: return "Illuminance Measurement";
        case 0x0402: return "Temperature Measurement";
        case 0x0403: return "Pressure Measurement";
        case 0x0405: return "Relative Humidity";
        case 0x0406: return "Occupancy Sensing";
        case 0x0500: return "IAS Zone";
        case 0x0501: return "IAS ACE";
        case 0x0502: return "IAS WD";
        case 0x0702: return "Metering";
        case 0x0B04: return "Electrical Measurement";
        default: {
            static char cluster_buf[16];
            snprintf(cluster_buf, sizeof(cluster_buf), "0x%04X", cluster_id);
            return cluster_buf;
        }
    }
}

esp_err_t zb_alarms_test(void)
{
    ESP_LOGI(TAG, "Running alarms module self-test...");

    /* Test add alarm */
    esp_err_t ret = zb_alarms_add_to_table(0x1234, 1, 0x01, 0x0201, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add test alarm");
        return ESP_FAIL;
    }

    /* Test get alarm count */
    uint8_t count = zb_alarms_get_device_alarm_count(0x1234);
    if (count != 1) {
        ESP_LOGE(TAG, "Alarm count mismatch: expected 1, got %d", count);
        return ESP_FAIL;
    }

    /* Test has alarms */
    if (!zb_alarms_device_has_alarms(0x1234)) {
        ESP_LOGE(TAG, "Device should have alarms");
        return ESP_FAIL;
    }

    /* Test get latest */
    zb_alarm_entry_t latest;
    ret = zb_alarms_get_latest(0x1234, &latest);
    if (ret != ESP_OK || latest.alarm_code != 0x01) {
        ESP_LOGE(TAG, "Failed to get latest alarm");
        return ESP_FAIL;
    }

    /* Test remove alarm */
    ret = zb_alarms_remove_from_table(0x1234, 0x01, 0x0201);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove test alarm");
        return ESP_FAIL;
    }

    /* Verify removal */
    count = zb_alarms_get_device_alarm_count(0x1234);
    if (count != 0) {
        ESP_LOGE(TAG, "Alarm should be removed");
        return ESP_FAIL;
    }

    /* Test utility functions */
    const char *str = zb_alarms_code_to_string(0x01, 0x0201);
    if (str == NULL) {
        ESP_LOGE(TAG, "Code to string returned NULL");
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Alarm code string: %s", str);

    const char *cluster_name = zb_alarms_cluster_name(0x0201);
    if (strcmp(cluster_name, "Thermostat") != 0) {
        ESP_LOGE(TAG, "Cluster name mismatch: %s", cluster_name);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Alarms module self-test PASSED");
    return ESP_OK;
}
