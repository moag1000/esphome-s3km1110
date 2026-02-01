/**
 * @file zb_poll_control.c
 * @brief Zigbee Poll Control Cluster Client Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_poll_control.h"
#include "zb_constants.h"
#include "zb_coordinator.h"
#include "core/gateway_timeouts.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ZB_POLL";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** @brief Module initialization flag */
static bool s_initialized = false;

/** @brief Thread safety mutex */
static SemaphoreHandle_t s_poll_mutex = NULL;

/** @brief Tracked sleepy devices */
static zb_poll_control_device_t s_devices[ZB_POLL_MAX_DEVICES];

/** @brief Number of tracked devices */
static size_t s_device_count = 0;

/** @brief Check-in callback */
static zb_poll_check_in_cb_t s_check_in_cb = NULL;

/** @brief Callback user data */
static void *s_callback_user_data = NULL;

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

/**
 * @brief Find device by short address
 */
static int find_device_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_device_count; i++) {
        if (s_devices[i].short_addr == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Send ZCL command to device
 *
 * Sends a Poll Control cluster command using the ESP-Zigbee-SDK
 * custom cluster command interface.
 *
 * @param short_addr Device short address
 * @param endpoint Device endpoint
 * @param cmd_id Poll Control command ID
 * @param payload Command payload (may be NULL for empty commands)
 * @param payload_len Payload length in bytes
 * @return ESP_OK on success
 */
static esp_err_t send_zcl_command(uint16_t short_addr, uint8_t endpoint,
                                  uint8_t cmd_id, const void *payload,
                                  size_t payload_len)
{
    ESP_LOGD(TAG, "Sending Poll Control command 0x%02X to 0x%04X EP%d",
             cmd_id, short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,  /* Coordinator endpoint */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_POLL_CONTROL,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = cmd_id,
        .data = {
            .type = (payload_len > 0) ? ESP_ZB_ZCL_ATTR_TYPE_SET : ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = payload_len,
            .value = (uint8_t *)payload,
        },
    };

    /* Acquire Zigbee lock for thread safety */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Poll Control command: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_poll_control_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Poll Control already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Poll Control Cluster Client...");

    /* Create mutex */
    s_poll_mutex = xSemaphoreCreateMutex();
    if (s_poll_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize device list */
    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;

    /* Clear callbacks */
    s_check_in_cb = NULL;
    s_callback_user_data = NULL;

    s_initialized = true;
    ESP_LOGI(TAG, "Poll Control Cluster Client initialized");

    return ESP_OK;
}

esp_err_t zb_poll_control_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Poll Control Cluster Client...");

    if (s_poll_mutex != NULL) {
        vSemaphoreDelete(s_poll_mutex);
        s_poll_mutex = NULL;
    }

    s_device_count = 0;
    s_check_in_cb = NULL;
    s_callback_user_data = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "Poll Control Cluster Client deinitialized");
    return ESP_OK;
}

bool zb_poll_control_is_initialized(void)
{
    return s_initialized;
}

esp_err_t zb_poll_control_register_callback(zb_poll_check_in_cb_t callback,
                                            void *user_data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);
    s_check_in_cb = callback;
    s_callback_user_data = user_data;
    xSemaphoreGive(s_poll_mutex);

    ESP_LOGI(TAG, "Check-in callback registered");
    return ESP_OK;
}

esp_err_t zb_poll_control_handle_check_in(uint16_t short_addr, uint8_t endpoint)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Check-in from device 0x%04X EP%d", short_addr, endpoint);

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    /* Find or add device */
    int idx = find_device_index(short_addr);
    zb_poll_control_device_t *device = NULL;

    if (idx >= 0) {
        device = &s_devices[idx];
    } else {
        /* Auto-add device if not found */
        if (s_device_count < ZB_POLL_MAX_DEVICES) {
            idx = s_device_count;
            device = &s_devices[idx];
            memset(device, 0, sizeof(zb_poll_control_device_t));
            device->short_addr = short_addr;
            device->endpoint = endpoint;

            /* Set default configuration */
            zb_poll_control_config_t default_config = ZB_POLL_CONTROL_CONFIG_DEFAULT;
            memcpy(&device->config, &default_config, sizeof(zb_poll_control_config_t));

            s_device_count++;
            ESP_LOGI(TAG, "Auto-added device 0x%04X to poll control tracking", short_addr);
        }
    }

    if (device != NULL) {
        device->last_check_in = xTaskGetTickCount();
        device->endpoint = endpoint;  /* Update endpoint if changed */
    }

    /* Determine response */
    bool start_fast_poll = false;
    uint16_t fast_poll_timeout = ZB_POLL_DEFAULT_FAST_POLL_TIMEOUT;

    if (device != NULL) {
        start_fast_poll = device->config.start_fast_polling;
        fast_poll_timeout = device->config.fast_poll_timeout;

        /* If configuration change pending, start fast poll to apply it */
        if (device->pending_config) {
            start_fast_poll = true;
            device->pending_config = false;
        }
    }

    xSemaphoreGive(s_poll_mutex);

    /* Send Check-In Response */
    esp_err_t ret = zb_poll_control_send_check_in_response(short_addr, endpoint,
                                                            start_fast_poll,
                                                            fast_poll_timeout);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Check-In Response: %s", esp_err_to_name(ret));
    }

    /* Invoke callback */
    if (s_check_in_cb != NULL) {
        s_check_in_cb(short_addr, endpoint, s_callback_user_data);
    }

    return ESP_OK;
}

esp_err_t zb_poll_control_set_interval(uint16_t short_addr, uint32_t interval_qs)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate interval */
    if (interval_qs < ZB_POLL_MIN_CHECK_IN_INTERVAL) {
        interval_qs = ZB_POLL_MIN_CHECK_IN_INTERVAL;
        ESP_LOGW(TAG, "Interval clamped to minimum: %lu qs", (unsigned long)interval_qs);
    }
    if (interval_qs > ZB_POLL_MAX_CHECK_IN_INTERVAL) {
        interval_qs = ZB_POLL_MAX_CHECK_IN_INTERVAL;
        ESP_LOGW(TAG, "Interval clamped to maximum: %lu qs", (unsigned long)interval_qs);
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        ESP_LOGW(TAG, "Device 0x%04X not found in poll control tracking", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    s_devices[idx].config.check_in_interval = interval_qs;
    s_devices[idx].pending_config = true;

    xSemaphoreGive(s_poll_mutex);

    ESP_LOGI(TAG, "Set check-in interval for 0x%04X to %lu qs (%lu sec)",
             short_addr, (unsigned long)interval_qs,
             (unsigned long)zb_poll_qs_to_seconds(interval_qs));

    return ESP_OK;
}

esp_err_t zb_poll_control_set_interval_seconds(uint16_t short_addr,
                                                uint32_t interval_sec)
{
    return zb_poll_control_set_interval(short_addr,
                                        zb_poll_seconds_to_qs(interval_sec));
}

esp_err_t zb_poll_control_get_interval(uint16_t short_addr, uint32_t *interval_qs)
{
    if (interval_qs == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    *interval_qs = s_devices[idx].config.check_in_interval;

    xSemaphoreGive(s_poll_mutex);
    return ESP_OK;
}

esp_err_t zb_poll_control_set_config(uint16_t short_addr,
                                     const zb_poll_control_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(&s_devices[idx].config, config, sizeof(zb_poll_control_config_t));
    s_devices[idx].pending_config = true;

    xSemaphoreGive(s_poll_mutex);

    ESP_LOGI(TAG, "Set poll control config for device 0x%04X", short_addr);
    return ESP_OK;
}

esp_err_t zb_poll_control_get_config(uint16_t short_addr,
                                     zb_poll_control_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(config, &s_devices[idx].config, sizeof(zb_poll_control_config_t));

    xSemaphoreGive(s_poll_mutex);
    return ESP_OK;
}

esp_err_t zb_poll_control_send_check_in_response(uint16_t short_addr,
                                                  uint8_t endpoint,
                                                  bool start_fast_poll,
                                                  uint16_t fast_poll_timeout)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Sending Check-In Response to 0x%04X EP%d (fast_poll=%d, timeout=%u)",
             short_addr, endpoint, start_fast_poll, fast_poll_timeout);

    /* Build payload */
    uint8_t payload[3];
    payload[0] = start_fast_poll ? 0x01 : 0x00;
    payload[1] = (fast_poll_timeout >> 0) & 0xFF;
    payload[2] = (fast_poll_timeout >> 8) & 0xFF;

    return send_zcl_command(short_addr, endpoint,
                           ZB_POLL_CMD_CHECK_IN_RESPONSE,
                           payload, sizeof(payload));
}

esp_err_t zb_poll_control_send_fast_poll_stop(uint16_t short_addr,
                                               uint8_t endpoint)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Sending Fast Poll Stop to 0x%04X EP%d", short_addr, endpoint);

    return send_zcl_command(short_addr, endpoint,
                           ZB_POLL_CMD_FAST_POLL_STOP,
                           NULL, 0);
}

esp_err_t zb_poll_control_send_set_long_poll(uint16_t short_addr,
                                              uint8_t endpoint,
                                              uint32_t interval_qs)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Sending Set Long Poll Interval to 0x%04X EP%d: %lu qs",
             short_addr, endpoint, (unsigned long)interval_qs);

    uint8_t payload[4];
    payload[0] = (interval_qs >> 0) & 0xFF;
    payload[1] = (interval_qs >> 8) & 0xFF;
    payload[2] = (interval_qs >> 16) & 0xFF;
    payload[3] = (interval_qs >> 24) & 0xFF;

    return send_zcl_command(short_addr, endpoint,
                           ZB_POLL_CMD_SET_LONG_POLL_INTERVAL,
                           payload, sizeof(payload));
}

esp_err_t zb_poll_control_send_set_short_poll(uint16_t short_addr,
                                               uint8_t endpoint,
                                               uint16_t interval_qs)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Sending Set Short Poll Interval to 0x%04X EP%d: %u qs",
             short_addr, endpoint, interval_qs);

    uint8_t payload[2];
    payload[0] = (interval_qs >> 0) & 0xFF;
    payload[1] = (interval_qs >> 8) & 0xFF;

    return send_zcl_command(short_addr, endpoint,
                           ZB_POLL_CMD_SET_SHORT_POLL_INTERVAL,
                           payload, sizeof(payload));
}

esp_err_t zb_poll_control_add_device(uint16_t short_addr, uint8_t endpoint,
                                     const uint8_t *ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    /* Check if already tracked */
    int idx = find_device_index(short_addr);
    if (idx >= 0) {
        /* Update endpoint and IEEE */
        s_devices[idx].endpoint = endpoint;
        if (ieee_addr != NULL) {
            memcpy(s_devices[idx].ieee_addr, ieee_addr, 8);
        }
        xSemaphoreGive(s_poll_mutex);
        ESP_LOGI(TAG, "Updated existing poll control device 0x%04X", short_addr);
        return ESP_OK;
    }

    /* Check capacity */
    if (s_device_count >= ZB_POLL_MAX_DEVICES) {
        xSemaphoreGive(s_poll_mutex);
        ESP_LOGE(TAG, "Poll control device list full");
        return ESP_ERR_NO_MEM;
    }

    /* Add new device */
    zb_poll_control_device_t *device = &s_devices[s_device_count];
    memset(device, 0, sizeof(zb_poll_control_device_t));

    device->short_addr = short_addr;
    device->endpoint = endpoint;
    if (ieee_addr != NULL) {
        memcpy(device->ieee_addr, ieee_addr, 8);
    }

    /* Set default configuration */
    zb_poll_control_config_t default_config = ZB_POLL_CONTROL_CONFIG_DEFAULT;
    memcpy(&device->config, &default_config, sizeof(zb_poll_control_config_t));

    device->last_check_in = 0;
    device->bound = false;
    device->pending_config = false;

    s_device_count++;

    xSemaphoreGive(s_poll_mutex);

    ESP_LOGI(TAG, "Added poll control device 0x%04X EP%d (total: %zu)",
             short_addr, endpoint, s_device_count);

    return ESP_OK;
}

esp_err_t zb_poll_control_remove_device(uint16_t short_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Shift remaining devices */
    if ((size_t)idx < s_device_count - 1) {
        memmove(&s_devices[idx], &s_devices[idx + 1],
                (s_device_count - idx - 1) * sizeof(zb_poll_control_device_t));
    }

    s_device_count--;

    xSemaphoreGive(s_poll_mutex);

    ESP_LOGI(TAG, "Removed poll control device 0x%04X (remaining: %zu)",
             short_addr, s_device_count);

    return ESP_OK;
}

esp_err_t zb_poll_control_get_device(uint16_t short_addr,
                                     zb_poll_control_device_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    int idx = find_device_index(short_addr);
    if (idx < 0) {
        xSemaphoreGive(s_poll_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(device, &s_devices[idx], sizeof(zb_poll_control_device_t));

    xSemaphoreGive(s_poll_mutex);
    return ESP_OK;
}

size_t zb_poll_control_get_all_devices(zb_poll_control_device_t *devices,
                                       size_t max_count)
{
    if (devices == NULL || max_count == 0) {
        return 0;
    }

    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);

    size_t count = (s_device_count < max_count) ? s_device_count : max_count;
    memcpy(devices, s_devices, count * sizeof(zb_poll_control_device_t));

    xSemaphoreGive(s_poll_mutex);

    return count;
}

size_t zb_poll_control_get_device_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    size_t count;
    xSemaphoreTake(s_poll_mutex, portMAX_DELAY);
    count = s_device_count;
    xSemaphoreGive(s_poll_mutex);

    return count;
}

esp_err_t zb_poll_control_test(void)
{
    ESP_LOGI(TAG, "Running Poll Control self-test...");

    /* Test time conversion */
    uint32_t test_sec = GW_SECONDS_PER_HOUR;  /* 1 hour */
    uint32_t test_qs = zb_poll_seconds_to_qs(test_sec);
    uint32_t back_sec = zb_poll_qs_to_seconds(test_qs);

    if (test_qs != ZB_POLL_QUARTER_SECONDS_PER_HOUR) {
        ESP_LOGE(TAG, "Seconds to quarter-seconds conversion failed: expected 14400, got %lu",
                 (unsigned long)test_qs);
        return ESP_FAIL;
    }
    if (back_sec != test_sec) {
        ESP_LOGE(TAG, "Quarter-seconds to seconds conversion failed: %lu != %lu",
                 (unsigned long)back_sec, (unsigned long)test_sec);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Time conversion test PASSED");

    /* Test default values */
    if (ZB_POLL_DEFAULT_CHECK_IN_INTERVAL != ZB_POLL_QUARTER_SECONDS_PER_HOUR) {
        ESP_LOGE(TAG, "Default check-in interval incorrect");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Default values test PASSED");

    if (s_initialized) {
        /* Test device tracking */
        uint8_t test_ieee[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        esp_err_t ret;

        ret = zb_poll_control_add_device(0x1234, 1, test_ieee);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Add device failed: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }

        zb_poll_control_device_t device;
        ret = zb_poll_control_get_device(0x1234, &device);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Get device failed: %s", esp_err_to_name(ret));
            zb_poll_control_remove_device(0x1234);
            return ESP_FAIL;
        }

        if (device.short_addr != 0x1234 || device.endpoint != 1) {
            ESP_LOGE(TAG, "Device data mismatch");
            zb_poll_control_remove_device(0x1234);
            return ESP_FAIL;
        }

        ret = zb_poll_control_remove_device(0x1234);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Remove device failed: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Device tracking test PASSED");
    }

    ESP_LOGI(TAG, "Poll Control self-test PASSED");
    return ESP_OK;
}
