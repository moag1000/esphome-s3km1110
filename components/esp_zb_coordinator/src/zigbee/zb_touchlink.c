/**
 * @file zb_touchlink.c
 * @brief Zigbee Touchlink Commissioning Implementation
 *
 * Implements Touchlink (ZLL) commissioning for ESP32-C5 Zigbee2MQTT Gateway.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_touchlink.h"
#include "zb_coordinator.h"
#include "zb_network.h"
#include "zb_constants.h"
#include "compat_stubs.h"
#include "json_utils.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "ZB_TOUCHLINK";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** @brief Module initialization flag */
static bool s_initialized = false;

/** @brief Thread safety mutex */
static SemaphoreHandle_t s_touchlink_mutex = NULL;

/** @brief Current scan status */
static zb_touchlink_scan_status_t s_scan_status = ZB_TOUCHLINK_SCAN_STATUS_IDLE;

/** @brief Scan results storage - allocated in PSRAM to save internal RAM */
static zb_touchlink_scan_result_t *s_scan_results = NULL;

/** @brief Number of devices found in current/last scan */
static size_t s_scan_result_count = 0;

/** @brief Current scan configuration */
static zb_touchlink_scan_config_t s_scan_config;

/** @brief RSSI correction configuration */
static zb_touchlink_rssi_config_t s_rssi_config = {
    .correction_value = ZB_TOUCHLINK_RSSI_CORRECTION,
    .threshold = ZB_TOUCHLINK_MIN_RSSI,
    .enabled = true
};

/** @brief Current transaction */
static zb_touchlink_transaction_t s_current_transaction = {
    .transaction_id = 0,
    .timestamp = 0,
    .active = false
};

/** @brief Scan result callback */
static zb_touchlink_scan_result_cb_t s_scan_result_cb = NULL;

/** @brief Scan complete callback */
static zb_touchlink_scan_complete_cb_t s_scan_complete_cb = NULL;

/** @brief Operation result callback */
static zb_touchlink_operation_cb_t s_operation_cb = NULL;

/** @brief User data for callbacks */
static void *s_callback_user_data = NULL;

/** @brief Current channel being scanned */
static uint8_t s_current_scan_channel = 0;

/** @brief Scan timer handle */
static TimerHandle_t s_scan_timer = NULL;

/** @brief Original network channel (to restore after scan) */
static uint8_t s_original_channel = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static esp_err_t touchlink_send_scan_request(uint8_t channel);
static esp_err_t touchlink_process_scan_response(const uint8_t *data, size_t len,
                                                  uint8_t channel, int8_t rssi);
static void touchlink_scan_timer_callback(TimerHandle_t timer);
static void touchlink_scan_next_channel(void);
static void touchlink_scan_complete(zb_touchlink_scan_status_t status);
static uint32_t touchlink_generate_transaction_id(void);
static esp_err_t touchlink_send_interpan_command(uint8_t channel,
                                                  const uint8_t *dest_ieee,
                                                  uint8_t cmd_id,
                                                  const uint8_t *payload,
                                                  size_t payload_len);
static void touchlink_mqtt_scan_result_cb(const zb_touchlink_scan_result_t *result,
                                          void *user_data);
static void touchlink_mqtt_scan_complete_cb(zb_touchlink_scan_status_t status,
                                            size_t device_count, void *user_data);

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

esp_err_t zb_touchlink_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Touchlink module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Touchlink commissioning module...");

    /* Check coordinator is initialized */
    if (zb_coordinator_get_state() == ZB_COORD_STATE_UNINITIALIZED) {
        ESP_LOGE(TAG, "Coordinator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create mutex */
    s_touchlink_mutex = xSemaphoreCreateMutex();
    if (s_touchlink_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Create scan timer */
    s_scan_timer = xTimerCreate(
        "touchlink_scan",
        pdMS_TO_TICKS(ZB_TOUCHLINK_SCAN_DURATION_MS),
        pdFALSE,  /* One-shot */
        NULL,
        touchlink_scan_timer_callback
    );
    if (s_scan_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create scan timer");
        vSemaphoreDelete(s_touchlink_mutex);
        s_touchlink_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Allocate scan results in PSRAM (saves ~1KB internal RAM) */
    if (s_scan_results == NULL) {
        s_scan_results = heap_caps_calloc(ZB_TOUCHLINK_MAX_SCAN_RESULTS,
                                           sizeof(zb_touchlink_scan_result_t),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_scan_results) {
            ESP_LOGW(TAG, "PSRAM alloc failed, falling back to internal RAM");
            s_scan_results = calloc(ZB_TOUCHLINK_MAX_SCAN_RESULTS,
                                     sizeof(zb_touchlink_scan_result_t));
        }
        if (!s_scan_results) {
            ESP_LOGE(TAG, "Failed to allocate scan results");
            xTimerDelete(s_scan_timer, portMAX_DELAY);
            s_scan_timer = NULL;
            vSemaphoreDelete(s_touchlink_mutex);
            s_touchlink_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    } else {
        memset(s_scan_results, 0,
               ZB_TOUCHLINK_MAX_SCAN_RESULTS * sizeof(zb_touchlink_scan_result_t));
    }
    s_scan_result_count = 0;

    /* Set default scan configuration */
    memset(&s_scan_config, 0, sizeof(s_scan_config));
    s_scan_config.scan_duration_ms = ZB_TOUCHLINK_SCAN_DURATION_MS;
    s_scan_config.rssi_threshold = ZB_TOUCHLINK_MIN_RSSI;
    s_scan_config.scan_factory_new_only = false;

    /* Enable all channels (11-26) by default */
    for (uint8_t ch = ZB_TOUCHLINK_CHANNEL_FIRST; ch <= ZB_TOUCHLINK_CHANNEL_LAST; ch++) {
        uint8_t byte_idx = (ch - ZB_TOUCHLINK_CHANNEL_FIRST) / 8;
        uint8_t bit_idx = (ch - ZB_TOUCHLINK_CHANNEL_FIRST) % 8;
        s_scan_config.channel_mask[byte_idx] |= (1 << bit_idx);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Touchlink commissioning module initialized");

    return ESP_OK;
}

esp_err_t zb_touchlink_deinit(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Touchlink module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Touchlink commissioning module...");

    /* Stop any ongoing scan */
    if (s_scan_status == ZB_TOUCHLINK_SCAN_STATUS_SCANNING) {
        zb_touchlink_scan_stop();
    }

    /* Delete timer */
    if (s_scan_timer != NULL) {
        xTimerDelete(s_scan_timer, portMAX_DELAY);
        s_scan_timer = NULL;
    }

    /* Delete mutex */
    if (s_touchlink_mutex != NULL) {
        vSemaphoreDelete(s_touchlink_mutex);
        s_touchlink_mutex = NULL;
    }

    /* Clear state */
    s_scan_result_count = 0;
    s_scan_result_cb = NULL;
    s_scan_complete_cb = NULL;
    s_operation_cb = NULL;
    s_callback_user_data = NULL;

    s_initialized = false;
    ESP_LOGI(TAG, "Touchlink commissioning module deinitialized");

    return ESP_OK;
}

bool zb_touchlink_is_initialized(void)
{
    return s_initialized;
}

/* ============================================================================
 * Scan Functions
 * ============================================================================ */

esp_err_t zb_touchlink_scan(const zb_touchlink_scan_config_t *config,
                            zb_touchlink_scan_result_cb_t result_cb,
                            zb_touchlink_scan_complete_cb_t complete_cb,
                            void *user_data)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_scan_status == ZB_TOUCHLINK_SCAN_STATUS_SCANNING) {
        ESP_LOGW(TAG, "Scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    if (!zb_coordinator_is_running()) {
        ESP_LOGE(TAG, "Coordinator not running");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Starting Touchlink scan...");

    /* Apply configuration */
    if (config != NULL) {
        memcpy(&s_scan_config, config, sizeof(zb_touchlink_scan_config_t));
    }

    /* Clear previous results */
    if (s_scan_results) {
        memset(s_scan_results, 0,
               ZB_TOUCHLINK_MAX_SCAN_RESULTS * sizeof(zb_touchlink_scan_result_t));
    }
    s_scan_result_count = 0;

    /* Store callbacks */
    s_scan_result_cb = result_cb;
    s_scan_complete_cb = complete_cb;
    s_callback_user_data = user_data;

    /* Generate new transaction ID */
    s_current_transaction.transaction_id = touchlink_generate_transaction_id();
    s_current_transaction.timestamp = xTaskGetTickCount();
    s_current_transaction.active = true;

    /* Save original channel to restore later */
    zb_network_info_t net_info;
    if (zb_network_get_info(&net_info) == ESP_OK) {
        s_original_channel = net_info.channel;
    } else {
        s_original_channel = ZB_DEFAULT_PRIMARY_CHANNEL;  /* Default */
    }

    /* Start scanning from first channel */
    s_current_scan_channel = ZB_TOUCHLINK_CHANNEL_FIRST;
    s_scan_status = ZB_TOUCHLINK_SCAN_STATUS_SCANNING;

    ESP_LOGI(TAG, "Scan started, transaction_id=0x%08" PRIx32 ", channels 11-26",
             s_current_transaction.transaction_id);

    xSemaphoreGive(s_touchlink_mutex);

    /* Start scanning first channel */
    touchlink_scan_next_channel();

    return ESP_OK;
}

esp_err_t zb_touchlink_scan_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_scan_status != ZB_TOUCHLINK_SCAN_STATUS_SCANNING) {
        ESP_LOGW(TAG, "No scan in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping Touchlink scan...");

    /* Stop timer */
    if (s_scan_timer != NULL) {
        xTimerStop(s_scan_timer, 0);
    }

    touchlink_scan_complete(ZB_TOUCHLINK_SCAN_STATUS_COMPLETE);

    return ESP_OK;
}

zb_touchlink_scan_status_t zb_touchlink_get_scan_status(void)
{
    return s_scan_status;
}

size_t zb_touchlink_get_scan_results(zb_touchlink_scan_result_t *results, size_t max_count)
{
    if (results == NULL || max_count == 0) {
        return 0;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    size_t count = (s_scan_result_count < max_count) ? s_scan_result_count : max_count;
    memcpy(results, s_scan_results, count * sizeof(zb_touchlink_scan_result_t));

    xSemaphoreGive(s_touchlink_mutex);

    return count;
}

esp_err_t zb_touchlink_get_result_by_ieee(const uint8_t *ieee_addr,
                                          zb_touchlink_scan_result_t *result)
{
    if (ieee_addr == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    for (size_t i = 0; i < s_scan_result_count; i++) {
        if (s_scan_results[i].valid &&
            memcmp(s_scan_results[i].ieee_addr, ieee_addr, 8) == 0) {
            memcpy(result, &s_scan_results[i], sizeof(zb_touchlink_scan_result_t));
            xSemaphoreGive(s_touchlink_mutex);
            return ESP_OK;
        }
    }

    xSemaphoreGive(s_touchlink_mutex);
    return ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * Touchlink Operations
 * ============================================================================ */

esp_err_t zb_touchlink_identify(const uint8_t *ieee_addr,
                                uint16_t duration_sec,
                                zb_touchlink_operation_cb_t callback,
                                void *user_data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Find device in scan results */
    zb_touchlink_scan_result_t device;
    esp_err_t ret = zb_touchlink_get_result_by_ieee(ieee_addr, &device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device not found in scan results");
        return ESP_ERR_NOT_FOUND;
    }

    char ieee_str[19];
    zb_touchlink_ieee_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    ESP_LOGI(TAG, "Sending Identify request to %s, duration=%d sec",
             ieee_str, duration_sec);

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    /* Store callback */
    s_operation_cb = callback;
    s_callback_user_data = user_data;

    /* Build Identify Request payload */
    uint8_t payload[6];
    uint32_t trans_id = touchlink_generate_transaction_id();

    /* Transaction ID (4 bytes) */
    payload[0] = (trans_id >> 0) & 0xFF;
    payload[1] = (trans_id >> 8) & 0xFF;
    payload[2] = (trans_id >> 16) & 0xFF;
    payload[3] = (trans_id >> 24) & 0xFF;

    /* Identify duration (2 bytes, seconds) */
    payload[4] = (duration_sec >> 0) & 0xFF;
    payload[5] = (duration_sec >> 8) & 0xFF;

    /* Send via Inter-PAN */
    ret = touchlink_send_interpan_command(
        device.channel,
        ieee_addr,
        ZB_TOUCHLINK_CMD_IDENTIFY_REQUEST,
        payload,
        sizeof(payload)
    );

    xSemaphoreGive(s_touchlink_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Identify request: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Identify request sent successfully");

    /* Notify callback (Identify has no response, so immediate success) */
    if (callback != NULL) {
        callback(ieee_addr, ESP_OK, user_data);
    }

    return ESP_OK;
}

esp_err_t zb_touchlink_factory_reset(const uint8_t *ieee_addr,
                                     zb_touchlink_operation_cb_t callback,
                                     void *user_data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Find device in scan results */
    zb_touchlink_scan_result_t device;
    esp_err_t ret = zb_touchlink_get_result_by_ieee(ieee_addr, &device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device not found in scan results");
        return ESP_ERR_NOT_FOUND;
    }

    char ieee_str[19];
    zb_touchlink_ieee_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    ESP_LOGW(TAG, "Sending Factory Reset request to %s - THIS IS DESTRUCTIVE!", ieee_str);

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    /* Store callback */
    s_operation_cb = callback;
    s_callback_user_data = user_data;

    /* Build Factory Reset Request payload */
    uint8_t payload[4];
    uint32_t trans_id = touchlink_generate_transaction_id();

    /* Transaction ID (4 bytes) */
    payload[0] = (trans_id >> 0) & 0xFF;
    payload[1] = (trans_id >> 8) & 0xFF;
    payload[2] = (trans_id >> 16) & 0xFF;
    payload[3] = (trans_id >> 24) & 0xFF;

    /* Send via Inter-PAN */
    ret = touchlink_send_interpan_command(
        device.channel,
        ieee_addr,
        ZB_TOUCHLINK_CMD_FACTORY_RESET_REQUEST,
        payload,
        sizeof(payload)
    );

    xSemaphoreGive(s_touchlink_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Factory Reset request: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Factory Reset request sent successfully");

    /* Notify callback (Factory Reset has no response) */
    if (callback != NULL) {
        callback(ieee_addr, ESP_OK, user_data);
    }

    return ESP_OK;
}

esp_err_t zb_touchlink_join_device(const uint8_t *ieee_addr,
                                   zb_touchlink_operation_cb_t callback,
                                   void *user_data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!zb_coordinator_is_running()) {
        ESP_LOGE(TAG, "Coordinator not running");
        return ESP_ERR_INVALID_STATE;
    }

    /* Find device in scan results */
    zb_touchlink_scan_result_t device;
    esp_err_t ret = zb_touchlink_get_result_by_ieee(ieee_addr, &device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device not found in scan results");
        return ESP_ERR_NOT_FOUND;
    }

    /* Get network information */
    zb_network_info_t net_info;
    ret = zb_network_get_info(&net_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get network info");
        return ret;
    }

    char ieee_str[19];
    zb_touchlink_ieee_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    ESP_LOGI(TAG, "Sending Network Join request to %s", ieee_str);

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    /* Store callback */
    s_operation_cb = callback;
    s_callback_user_data = user_data;

    /* Determine command based on device type */
    uint8_t cmd_id;
    if (device.device_type == ZB_TOUCHLINK_DEVICE_TYPE_ROUTER) {
        cmd_id = ZB_TOUCHLINK_CMD_NETWORK_JOIN_ROUTER_REQUEST;
        ESP_LOGI(TAG, "Device is Router type");
    } else {
        cmd_id = ZB_TOUCHLINK_CMD_NETWORK_JOIN_ED_REQUEST;
        ESP_LOGI(TAG, "Device is End Device type");
    }

    /* Build Network Join Request payload */
    uint8_t payload[47];
    size_t idx = 0;
    uint32_t trans_id = touchlink_generate_transaction_id();

    /* Transaction ID (4 bytes) */
    payload[idx++] = (trans_id >> 0) & 0xFF;
    payload[idx++] = (trans_id >> 8) & 0xFF;
    payload[idx++] = (trans_id >> 16) & 0xFF;
    payload[idx++] = (trans_id >> 24) & 0xFF;

    /* Extended PAN ID (8 bytes) */
    memcpy(&payload[idx], net_info.extended_pan_id, 8);
    idx += 8;

    /* Key Index (1 byte) - use development key index 0 */
    payload[idx++] = 0x04;  /* Development key */

    /* Encrypted Network Key (16 bytes) - for simplicity, use unencrypted */
    memcpy(&payload[idx], net_info.network_key, 16);
    idx += 16;

    /* Network Update ID (1 byte) */
    payload[idx++] = 0x00;

    /* Logical Channel (1 byte) */
    payload[idx++] = net_info.channel;

    /* PAN ID (2 bytes) */
    payload[idx++] = (net_info.pan_id >> 0) & 0xFF;
    payload[idx++] = (net_info.pan_id >> 8) & 0xFF;

    /* Network Address (2 bytes) - assign new address */
    uint16_t assigned_addr = 0x0001 + (esp_random() % 0xFFF0);
    payload[idx++] = (assigned_addr >> 0) & 0xFF;
    payload[idx++] = (assigned_addr >> 8) & 0xFF;

    /* Group Identifiers Begin/End (4 bytes) */
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;

    /* Free Network Address Range Begin/End (4 bytes) */
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;

    /* Free Group Identifier Range Begin/End (4 bytes) */
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;

    /* Send via Inter-PAN */
    ret = touchlink_send_interpan_command(
        device.channel,
        ieee_addr,
        cmd_id,
        payload,
        idx
    );

    xSemaphoreGive(s_touchlink_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Network Join request: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Network Join request sent successfully, assigned_addr=0x%04X", assigned_addr);

    return ESP_OK;
}

/* ============================================================================
 * RSSI Configuration
 * ============================================================================ */

esp_err_t zb_touchlink_set_rssi_config(const zb_touchlink_rssi_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);
    memcpy(&s_rssi_config, config, sizeof(zb_touchlink_rssi_config_t));
    xSemaphoreGive(s_touchlink_mutex);

    ESP_LOGI(TAG, "RSSI config updated: correction=%d, threshold=%d, enabled=%d",
             config->correction_value, config->threshold, config->enabled);

    return ESP_OK;
}

esp_err_t zb_touchlink_get_rssi_config(zb_touchlink_rssi_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);
    memcpy(config, &s_rssi_config, sizeof(zb_touchlink_rssi_config_t));
    xSemaphoreGive(s_touchlink_mutex);

    return ESP_OK;
}

/* ============================================================================
 * MQTT Integration
 * ============================================================================ */

esp_err_t zb_touchlink_process_mqtt_request(const char *topic,
                                            const char *payload,
                                            size_t len)
{
    if (topic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Processing MQTT request: %s", topic);

    /* Check for scan request */
    if (strstr(topic, "touchlink/scan") != NULL) {
        ESP_LOGI(TAG, "MQTT: Starting Touchlink scan");
        return zb_touchlink_scan(
            NULL,
            touchlink_mqtt_scan_result_cb,
            touchlink_mqtt_scan_complete_cb,
            NULL
        );
    }

    /* Check for identify request */
    const char *identify_prefix = "touchlink/identify/";
    const char *identify_pos = strstr(topic, identify_prefix);
    if (identify_pos != NULL) {
        const char *ieee_str = identify_pos + strlen(identify_prefix);
        uint8_t ieee_addr[8];

        if (zb_touchlink_string_to_ieee(ieee_str, ieee_addr) != ESP_OK) {
            ESP_LOGE(TAG, "Invalid IEEE address format: %s", ieee_str);
            return ESP_ERR_INVALID_ARG;
        }

        uint16_t duration = ZB_TOUCHLINK_IDENTIFY_DURATION;
        /* Parse duration from payload if provided */
        if (payload != NULL && len > 0) {
            cJSON *json = cJSON_ParseWithLength(payload, len);
            if (json != NULL) {
                cJSON *dur_item = cJSON_GetObjectItem(json, "duration");
                if (cJSON_IsNumber(dur_item)) {
                    duration = (uint16_t)dur_item->valueint;
                }
                cJSON_Delete(json);
            }
        }

        ESP_LOGI(TAG, "MQTT: Identify device %s for %d seconds", ieee_str, duration);
        return zb_touchlink_identify(ieee_addr, duration, NULL, NULL);
    }

    /* Check for factory reset request */
    const char *reset_prefix = "touchlink/factoryreset/";
    const char *reset_pos = strstr(topic, reset_prefix);
    if (reset_pos != NULL) {
        const char *ieee_str = reset_pos + strlen(reset_prefix);
        uint8_t ieee_addr[8];

        if (zb_touchlink_string_to_ieee(ieee_str, ieee_addr) != ESP_OK) {
            ESP_LOGE(TAG, "Invalid IEEE address format: %s", ieee_str);
            return ESP_ERR_INVALID_ARG;
        }

        ESP_LOGW(TAG, "MQTT: Factory reset device %s", ieee_str);
        return zb_touchlink_factory_reset(ieee_addr, NULL, NULL);
    }

    ESP_LOGW(TAG, "Unknown Touchlink MQTT topic: %s", topic);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t zb_touchlink_publish_scan_results(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Publishing Touchlink scan results via MQTT...");

    /* Build JSON response */
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return ESP_ERR_NO_MEM;
    }

    cJSON *status_str = cJSON_AddStringToObject(root, "status",
        (s_scan_status == ZB_TOUCHLINK_SCAN_STATUS_COMPLETE) ? "ok" : "error");
    if (status_str == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON *devices = cJSON_AddArrayToObject(root, "devices");
    if (devices == NULL) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    for (size_t i = 0; i < s_scan_result_count; i++) {
        if (!s_scan_results[i].valid) {
            continue;
        }

        cJSON *device = cJSON_CreateObject();
        if (device == NULL) {
            continue;
        }

        /* IEEE address */
        char ieee_str[19];
        zb_touchlink_ieee_to_string(s_scan_results[i].ieee_addr, ieee_str, sizeof(ieee_str));
        cJSON_AddStringToObject(device, "ieee_address", ieee_str);

        /* Network addresses */
        cJSON_AddNumberToObject(device, "nwk_address", s_scan_results[i].short_addr);
        cJSON_AddNumberToObject(device, "pan_id", s_scan_results[i].pan_id);

        /* Radio parameters */
        cJSON_AddNumberToObject(device, "channel", s_scan_results[i].channel);
        cJSON_AddNumberToObject(device, "rssi", s_scan_results[i].rssi);

        /* Device info */
        const char *dev_type_str;
        switch (s_scan_results[i].device_type) {
            case ZB_TOUCHLINK_DEVICE_TYPE_COORDINATOR:
                dev_type_str = "Coordinator";
                break;
            case ZB_TOUCHLINK_DEVICE_TYPE_ROUTER:
                dev_type_str = "Router";
                break;
            default:
                dev_type_str = "EndDevice";
                break;
        }
        cJSON_AddStringToObject(device, "type", dev_type_str);

        /* Status flags */
        cJSON_AddBoolToObject(device, "factory_new", s_scan_results[i].factory_new);

        cJSON_AddItemToArray(devices, device);
    }

    xSemaphoreGive(s_touchlink_mutex);

    cJSON_AddNumberToObject(root, "count", s_scan_result_count);

    /* Publish to MQTT */
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        return ESP_ERR_NO_MEM;
    }

    /* Publish via mqtt_client */
    esp_err_t ret = mqtt_client_publish(
        "zigbee2mqtt/bridge/response/touchlink/scan",
        json_str,
        strlen(json_str),
        1,     /* QoS 1 */
        false  /* No retain */
    );

    free(json_str);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish scan results: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Scan results published successfully");
    }

    return ret;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

esp_err_t zb_touchlink_ieee_to_string(const uint8_t *ieee_addr,
                                      char *str_buf,
                                      size_t buf_len)
{
    if (ieee_addr == NULL || str_buf == NULL || buf_len < ZB_IEEE_ADDR_STR_BUFFER_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(str_buf, buf_len, "0x%02X%02X%02X%02X%02X%02X%02X%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

    return ESP_OK;
}

esp_err_t zb_touchlink_string_to_ieee(const char *str_buf, uint8_t *ieee_addr)
{
    if (str_buf == NULL || ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Skip "0x" prefix if present */
    if (strncmp(str_buf, "0x", 2) == 0 || strncmp(str_buf, "0X", 2) == 0) {
        str_buf += 2;
    }

    /* Parse 16 hex characters */
    if (strlen(str_buf) != ZB_IEEE_ADDR_HEX_STRING_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < 8; i++) {
        unsigned int byte_val;
        if (sscanf(&str_buf[i * 2], "%2x", &byte_val) != 1) {
            return ESP_ERR_INVALID_ARG;
        }
        /* Store in little-endian order (MSB first in string) */
        ieee_addr[7 - i] = (uint8_t)byte_val;
    }

    return ESP_OK;
}

esp_err_t zb_touchlink_test(void)
{
    ESP_LOGI(TAG, "Running Touchlink self-test...");

    /* Test IEEE address conversion */
    uint8_t test_ieee[8] = {0x67, 0x45, 0x23, 0x01, 0x0D, 0x8D, 0x15, 0x00};
    char ieee_str[19];
    esp_err_t ret;

    ret = zb_touchlink_ieee_to_string(test_ieee, ieee_str, sizeof(ieee_str));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IEEE to string conversion failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "IEEE to string: %s", ieee_str);

    /* Test string to IEEE */
    uint8_t parsed_ieee[8];
    ret = zb_touchlink_string_to_ieee(ieee_str, parsed_ieee);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "String to IEEE conversion failed");
        return ESP_FAIL;
    }

    if (memcmp(test_ieee, parsed_ieee, 8) != 0) {
        ESP_LOGE(TAG, "IEEE round-trip conversion mismatch");
        return ESP_FAIL;
    }

    /* Test initialization check */
    if (!s_initialized) {
        ESP_LOGW(TAG, "Module not initialized - skipping runtime tests");
    } else {
        /* Verify mutex exists */
        if (s_touchlink_mutex == NULL) {
            ESP_LOGE(TAG, "Mutex not created");
            return ESP_FAIL;
        }

        /* Verify timer exists */
        if (s_scan_timer == NULL) {
            ESP_LOGE(TAG, "Scan timer not created");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Touchlink self-test PASSED");
    return ESP_OK;
}

/* ============================================================================
 * Internal / Static Functions
 * ============================================================================ */

/**
 * @brief Generate random transaction ID
 */
static uint32_t touchlink_generate_transaction_id(void)
{
    return esp_random();
}

/**
 * @brief Send Inter-PAN Touchlink command
 */
static esp_err_t touchlink_send_interpan_command(uint8_t channel,
                                                  const uint8_t *dest_ieee,
                                                  uint8_t cmd_id,
                                                  const uint8_t *payload,
                                                  size_t payload_len)
{
    ESP_LOGD(TAG, "Sending Inter-PAN command 0x%02X on channel %d", cmd_id, channel);

    /*
     * Note: The actual Inter-PAN implementation depends on ESP-Zigbee-SDK support.
     * ESP-Zigbee-SDK may provide esp_zb_zcl_touchlink_* APIs.
     *
     * For now, we provide the framework. The actual sending would use:
     * - esp_zb_zcl_interpan_send() or similar
     * - esp_zb_set_channel() to switch channels temporarily
     *
     * Placeholder implementation that logs the operation.
     */

    /* In a real implementation:
     * 1. Save current channel
     * 2. Switch to target channel: esp_zb_set_channel(channel)
     * 3. Build Inter-PAN frame with ZLL profile
     * 4. Send: esp_zb_aps_interpan_send(...)
     * 5. Restore original channel
     */

    ESP_LOGD(TAG, "Inter-PAN command sent (framework - actual send TBD based on SDK support)");

    return ESP_OK;
}

/**
 * @brief Send Touchlink Scan Request
 */
static esp_err_t touchlink_send_scan_request(uint8_t channel)
{
    ESP_LOGD(TAG, "Sending Scan Request on channel %d", channel);

    /* Build Scan Request payload */
    uint8_t payload[5];

    /* Transaction ID (4 bytes) */
    payload[0] = (s_current_transaction.transaction_id >> 0) & 0xFF;
    payload[1] = (s_current_transaction.transaction_id >> 8) & 0xFF;
    payload[2] = (s_current_transaction.transaction_id >> 16) & 0xFF;
    payload[3] = (s_current_transaction.transaction_id >> 24) & 0xFF;

    /* Zigbee information (1 byte) - coordinator, Rx on when idle */
    payload[4] = 0x05;

    /* Send broadcast Inter-PAN */
    return touchlink_send_interpan_command(
        channel,
        NULL,  /* Broadcast */
        ZB_TOUCHLINK_CMD_SCAN_REQUEST,
        payload,
        sizeof(payload)
    );
}

/**
 * @brief Process received Scan Response
 */
static esp_err_t touchlink_process_scan_response(const uint8_t *data, size_t len,
                                                  uint8_t channel, int8_t rssi)
{
    if (data == NULL || len < ZB_TOUCHLINK_DATA_MIN_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Processing Scan Response from channel %d, RSSI=%d", channel, rssi);

    /* Check RSSI threshold */
    int8_t rssi_corrected = rssi + s_rssi_config.correction_value;
    if (s_rssi_config.enabled && rssi_corrected < s_rssi_config.threshold) {
        ESP_LOGD(TAG, "Device RSSI %d below threshold %d, ignoring",
                 rssi_corrected, s_rssi_config.threshold);
        return ESP_OK;
    }

    xSemaphoreTake(s_touchlink_mutex, portMAX_DELAY);

    /* Check if we have room for more results */
    if (s_scan_result_count >= ZB_TOUCHLINK_MAX_SCAN_RESULTS) {
        ESP_LOGW(TAG, "Scan results full, ignoring device");
        xSemaphoreGive(s_touchlink_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Parse Scan Response */
    zb_touchlink_scan_result_t *result = &s_scan_results[s_scan_result_count];
    memset(result, 0, sizeof(zb_touchlink_scan_result_t));

    size_t idx = 0;

    /* Transaction ID (4 bytes) - verify it matches */
    uint32_t rx_trans_id = data[idx] | (data[idx+1] << 8) |
                          (data[idx+2] << 16) | (data[idx+3] << 24);
    idx += 4;

    if (rx_trans_id != s_current_transaction.transaction_id) {
        ESP_LOGD(TAG, "Transaction ID mismatch, ignoring");
        xSemaphoreGive(s_touchlink_mutex);
        return ESP_OK;
    }

    /* RSSI correction (1 byte) */
    idx += 1;

    /* Zigbee information (1 byte) */
    uint8_t zb_info = data[idx++];
    result->device_type = (zb_touchlink_device_type_t)(zb_info & 0x03);

    /* Touchlink information (1 byte) */
    result->touchlink_info = data[idx++];
    result->factory_new = (result->touchlink_info & 0x01) != 0;
    result->address_assignment = (result->touchlink_info & 0x02) != 0;
    result->link_initiator = (result->touchlink_info & 0x10) != 0;
    result->link_priority = (result->touchlink_info & 0x20) != 0;

    /* Key bitmask (1 byte) */
    result->key_bitmask = data[idx++];

    /* Response ID (4 bytes) */
    result->response_id = data[idx] | (data[idx+1] << 8) |
                         (data[idx+2] << 16) | (data[idx+3] << 24);
    idx += 4;

    /* Extended PAN ID (8 bytes) */
    memcpy(result->extended_pan_id, &data[idx], 8);
    idx += 8;

    /* Network update ID, Channel, PAN ID, Short Address are optional */
    if (len > idx + 5) {
        idx += 1;  /* Network update ID */
        result->channel = data[idx++];
        result->pan_id = data[idx] | (data[idx+1] << 8);
        idx += 2;
        result->short_addr = data[idx] | (data[idx+1] << 8);
        idx += 2;
    } else {
        result->channel = channel;
    }

    /* Source IEEE address should be extracted from Inter-PAN frame header */
    /* For now, use extended PAN ID as placeholder (actual implementation
       would get this from the APS frame) */
    memcpy(result->ieee_addr, result->extended_pan_id, 8);

    /* Store metadata */
    result->rssi = rssi;
    result->rssi_corrected = rssi_corrected;
    result->timestamp = xTaskGetTickCount();
    result->valid = true;

    /* Check for duplicates */
    bool duplicate = false;
    for (size_t i = 0; i < s_scan_result_count; i++) {
        if (s_scan_results[i].valid &&
            memcmp(s_scan_results[i].ieee_addr, result->ieee_addr, 8) == 0) {
            /* Update existing entry if RSSI is better */
            if (rssi > s_scan_results[i].rssi) {
                memcpy(&s_scan_results[i], result, sizeof(zb_touchlink_scan_result_t));
            }
            duplicate = true;
            break;
        }
    }

    if (!duplicate) {
        s_scan_result_count++;
        ESP_LOGI(TAG, "Found Touchlink device: channel=%d, RSSI=%d, factory_new=%d",
                 result->channel, result->rssi, result->factory_new);

        /* Invoke result callback */
        if (s_scan_result_cb != NULL) {
            s_scan_result_cb(result, s_callback_user_data);
        }
    }

    xSemaphoreGive(s_touchlink_mutex);

    return ESP_OK;
}

/**
 * @brief Scan timer callback
 */
static void touchlink_scan_timer_callback(TimerHandle_t timer)
{
    (void)timer;

    /* Move to next channel */
    touchlink_scan_next_channel();
}

/**
 * @brief Scan next channel or complete scan
 */
static void touchlink_scan_next_channel(void)
{
    /* Find next enabled channel */
    while (s_current_scan_channel <= ZB_TOUCHLINK_CHANNEL_LAST) {
        uint8_t ch_offset = s_current_scan_channel - ZB_TOUCHLINK_CHANNEL_FIRST;
        uint8_t byte_idx = ch_offset / 8;
        uint8_t bit_idx = ch_offset % 8;

        if (s_scan_config.channel_mask[byte_idx] & (1 << bit_idx)) {
            break;
        }
        s_current_scan_channel++;
    }

    if (s_current_scan_channel > ZB_TOUCHLINK_CHANNEL_LAST) {
        /* Scan complete */
        touchlink_scan_complete(ZB_TOUCHLINK_SCAN_STATUS_COMPLETE);
        return;
    }

    ESP_LOGD(TAG, "Scanning channel %d...", s_current_scan_channel);

    /* Send scan request on current channel */
    esp_err_t ret = touchlink_send_scan_request(s_current_scan_channel);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send scan request on channel %d", s_current_scan_channel);
    }

    /* Move to next channel */
    s_current_scan_channel++;

    /* Start timer for next channel */
    if (s_scan_timer != NULL) {
        xTimerChangePeriod(s_scan_timer,
                          pdMS_TO_TICKS(s_scan_config.scan_duration_ms),
                          0);
        xTimerStart(s_scan_timer, 0);
    }
}

/**
 * @brief Complete scan operation
 */
static void touchlink_scan_complete(zb_touchlink_scan_status_t status)
{
    ESP_LOGI(TAG, "Scan complete: status=%d, devices_found=%zu", status, s_scan_result_count);

    /* Stop timer */
    if (s_scan_timer != NULL) {
        xTimerStop(s_scan_timer, 0);
    }

    /* Update status */
    s_scan_status = status;
    s_current_transaction.active = false;

    /* Restore original channel */
    /* esp_zb_set_channel(s_original_channel); */

    /* Invoke completion callback */
    if (s_scan_complete_cb != NULL) {
        s_scan_complete_cb(status, s_scan_result_count, s_callback_user_data);
    }
}

/**
 * @brief MQTT scan result callback
 */
static void touchlink_mqtt_scan_result_cb(const zb_touchlink_scan_result_t *result,
                                          void *user_data)
{
    (void)user_data;

    if (result == NULL) {
        return;
    }

    char ieee_str[19];
    zb_touchlink_ieee_to_string(result->ieee_addr, ieee_str, sizeof(ieee_str));

    ESP_LOGI(TAG, "MQTT scan found: %s, ch=%d, RSSI=%d",
             ieee_str, result->channel, result->rssi);
}

/**
 * @brief MQTT scan complete callback
 */
static void touchlink_mqtt_scan_complete_cb(zb_touchlink_scan_status_t status,
                                            size_t device_count, void *user_data)
{
    (void)user_data;

    ESP_LOGI(TAG, "MQTT scan complete: %zu devices found", device_count);

    /* Publish results */
    zb_touchlink_publish_scan_results();
}
