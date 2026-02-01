/**
 * @file zb_time_server.c
 * @brief Zigbee Time Cluster Server Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_time_server.h"
#include "zb_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <sys/time.h>

static const char *TAG = "ZB_TIME";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** @brief Module initialization flag */
static bool s_initialized = false;

/** @brief Thread safety mutex */
static SemaphoreHandle_t s_time_mutex = NULL;

/** @brief Current configuration */
static zb_time_server_config_t s_config = {
    .timezone_offset = ZB_TIME_DEFAULT_TIMEZONE,
    .dst_shift = ZB_TIME_DEFAULT_DST_SHIFT,
    .dst_start = 0,
    .dst_end = 0,
    .is_master = true,
    .is_synchronized = false
};

/** @brief Time when server was last set/synchronized */
static uint32_t s_last_set_time = 0;

/** @brief Validity period for time (24 hours by default) */
static uint32_t s_validity_period = 24 * 60 * 60;  /* 24 hours */

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Get current system time as Zigbee UTC
 */
static uint32_t get_current_zigbee_time(void)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) {
        return ZB_TIME_INVALID;
    }

    /* Convert Unix timestamp to Zigbee UTC */
    if (tv.tv_sec < ZB_TIME_EPOCH_OFFSET_SECONDS) {
        /* System time not set (before year 2000) */
        return ZB_TIME_INVALID;
    }

    return (uint32_t)(tv.tv_sec - ZB_TIME_EPOCH_OFFSET_SECONDS);
}

/**
 * @brief Build time status byte
 */
static uint8_t build_time_status(void)
{
    uint8_t status = 0;

    if (s_config.is_master) {
        status |= ZB_TIME_STATUS_MASTER;
    }

    if (s_config.is_synchronized) {
        status |= ZB_TIME_STATUS_SYNCHRONIZED;
    }

    /* We always manage timezone and DST if we're master */
    if (s_config.is_master) {
        status |= ZB_TIME_STATUS_MASTER_ZONE_DST;
    }

    return status;
}

/**
 * @brief Check if DST is currently active
 */
static bool is_dst_active_internal(uint32_t current_time)
{
    if (s_config.dst_shift == 0) {
        return false;  /* No DST configured */
    }

    if (s_config.dst_start == 0 || s_config.dst_end == 0) {
        return false;  /* DST times not configured */
    }

    /* Handle both northern and southern hemisphere DST */
    if (s_config.dst_start < s_config.dst_end) {
        /* Northern hemisphere: DST in summer */
        return (current_time >= s_config.dst_start && current_time < s_config.dst_end);
    } else {
        /* Southern hemisphere: DST in winter */
        return (current_time >= s_config.dst_start || current_time < s_config.dst_end);
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_time_server_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Time server already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Time Cluster Server...");

    /* Create mutex */
    s_time_mutex = xSemaphoreCreateMutex();
    if (s_time_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize with default configuration */
    s_config.is_master = true;
    s_config.is_synchronized = false;
    s_config.timezone_offset = ZB_TIME_DEFAULT_TIMEZONE;
    s_config.dst_shift = ZB_TIME_DEFAULT_DST_SHIFT;
    s_config.dst_start = 0;
    s_config.dst_end = 0;

    /* Check if system time is already valid (NTP may have synced) */
    uint32_t current_time = get_current_zigbee_time();
    if (current_time != ZB_TIME_INVALID) {
        s_config.is_synchronized = true;
        s_last_set_time = current_time;
        ESP_LOGI(TAG, "System time already synchronized: %lu", (unsigned long)current_time);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Time Cluster Server initialized");

    return ESP_OK;
}

esp_err_t zb_time_server_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Time Cluster Server...");

    if (s_time_mutex != NULL) {
        vSemaphoreDelete(s_time_mutex);
        s_time_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Time Cluster Server deinitialized");

    return ESP_OK;
}

bool zb_time_server_is_initialized(void)
{
    return s_initialized;
}

esp_err_t zb_time_server_set_config(const zb_time_server_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    memcpy(&s_config, config, sizeof(zb_time_server_config_t));
    xSemaphoreGive(s_time_mutex);

    ESP_LOGI(TAG, "Configuration updated: tz=%ld, dst_shift=%ld, master=%d, sync=%d",
             (long)s_config.timezone_offset, (long)s_config.dst_shift,
             s_config.is_master, s_config.is_synchronized);

    return ESP_OK;
}

esp_err_t zb_time_server_get_config(zb_time_server_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    memcpy(config, &s_config, sizeof(zb_time_server_config_t));
    xSemaphoreGive(s_time_mutex);

    return ESP_OK;
}

esp_err_t zb_time_server_set_time(uint32_t utc_time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Convert Zigbee UTC to Unix timestamp */
    time_t unix_time = (time_t)(utc_time + ZB_TIME_EPOCH_OFFSET_SECONDS);

    /* Set system time */
    struct timeval tv = {
        .tv_sec = unix_time,
        .tv_usec = 0
    };

    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return ESP_FAIL;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    s_last_set_time = utc_time;
    s_config.is_synchronized = true;
    xSemaphoreGive(s_time_mutex);

    ESP_LOGI(TAG, "Time set to %lu (Zigbee UTC)", (unsigned long)utc_time);

    return ESP_OK;
}

esp_err_t zb_time_server_set_time_unix(time_t unix_time)
{
    if (unix_time < ZB_TIME_EPOCH_OFFSET_SECONDS) {
        ESP_LOGE(TAG, "Unix time too early (before year 2000)");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t zigbee_time = (uint32_t)(unix_time - ZB_TIME_EPOCH_OFFSET_SECONDS);
    return zb_time_server_set_time(zigbee_time);
}

uint32_t zb_time_server_get_time(void)
{
    if (!s_initialized) {
        return ZB_TIME_INVALID;
    }

    return get_current_zigbee_time();
}

time_t zb_time_server_get_time_unix(void)
{
    uint32_t zigbee_time = zb_time_server_get_time();
    if (zigbee_time == ZB_TIME_INVALID) {
        return 0;
    }

    return (time_t)(zigbee_time + ZB_TIME_EPOCH_OFFSET_SECONDS);
}

esp_err_t zb_time_server_set_timezone(int32_t offset_seconds)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    s_config.timezone_offset = offset_seconds;
    xSemaphoreGive(s_time_mutex);

    ESP_LOGI(TAG, "Timezone set to %ld seconds (UTC%+ld)",
             (long)offset_seconds, (long)(offset_seconds / 3600));

    return ESP_OK;
}

int32_t zb_time_server_get_timezone(void)
{
    if (!s_initialized) {
        return 0;
    }

    int32_t tz;
    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    tz = s_config.timezone_offset;
    xSemaphoreGive(s_time_mutex);

    return tz;
}

esp_err_t zb_time_server_set_dst(uint32_t dst_start, uint32_t dst_end, int32_t dst_shift)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    s_config.dst_start = dst_start;
    s_config.dst_end = dst_end;
    s_config.dst_shift = dst_shift;
    xSemaphoreGive(s_time_mutex);

    ESP_LOGI(TAG, "DST configured: start=%lu, end=%lu, shift=%ld",
             (unsigned long)dst_start, (unsigned long)dst_end, (long)dst_shift);

    return ESP_OK;
}

bool zb_time_server_is_dst_active(void)
{
    if (!s_initialized) {
        return false;
    }

    uint32_t current_time = get_current_zigbee_time();
    if (current_time == ZB_TIME_INVALID) {
        return false;
    }

    bool active;
    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    active = is_dst_active_internal(current_time);
    xSemaphoreGive(s_time_mutex);

    return active;
}

esp_err_t zb_time_server_get_state(zb_time_server_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(state, 0, sizeof(zb_time_server_state_t));

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);

    /* Get current UTC time */
    state->utc_time = get_current_zigbee_time();

    /* Build status */
    state->time_status = build_time_status();

    /* Copy configuration values */
    state->timezone = s_config.timezone_offset;
    state->dst_start = s_config.dst_start;
    state->dst_end = s_config.dst_end;
    state->dst_shift = s_config.dst_shift;

    /* Calculate derived values */
    if (state->utc_time != ZB_TIME_INVALID) {
        state->standard_time = state->utc_time + s_config.timezone_offset;

        if (is_dst_active_internal(state->utc_time)) {
            state->local_time = state->standard_time + s_config.dst_shift;
        } else {
            state->local_time = state->standard_time;
        }
    } else {
        state->standard_time = ZB_TIME_INVALID;
        state->local_time = ZB_TIME_INVALID;
    }

    state->last_set_time = s_last_set_time;
    state->valid_until_time = (s_last_set_time != 0) ? (s_last_set_time + s_validity_period) : 0;

    xSemaphoreGive(s_time_mutex);

    return ESP_OK;
}

esp_err_t zb_time_server_set_synchronized(bool synchronized)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    s_config.is_synchronized = synchronized;

    if (synchronized) {
        s_last_set_time = get_current_zigbee_time();
    }
    xSemaphoreGive(s_time_mutex);

    ESP_LOGI(TAG, "Time synchronization status: %s", synchronized ? "synchronized" : "not synchronized");

    return ESP_OK;
}

bool zb_time_server_is_synchronized(void)
{
    if (!s_initialized) {
        return false;
    }

    bool sync;
    xSemaphoreTake(s_time_mutex, portMAX_DELAY);
    sync = s_config.is_synchronized;
    xSemaphoreGive(s_time_mutex);

    return sync;
}

size_t zb_time_server_handle_read(uint16_t short_addr, uint8_t endpoint,
                                  uint16_t attr_id, void *value, size_t max_len)
{
    if (value == NULL || max_len == 0) {
        return 0;
    }

    if (!s_initialized) {
        return 0;
    }

    ESP_LOGD(TAG, "Read attribute 0x%04X from device 0x%04X EP%d",
             attr_id, short_addr, endpoint);

    zb_time_server_state_t state;
    zb_time_server_get_state(&state);

    size_t len = 0;

    switch (attr_id) {
        case ZB_TIME_ATTR_TIME:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.utc_time;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_TIME_STATUS:
            if (max_len >= sizeof(uint8_t)) {
                *(uint8_t *)value = state.time_status;
                len = sizeof(uint8_t);
            }
            break;

        case ZB_TIME_ATTR_TIME_ZONE:
            if (max_len >= sizeof(int32_t)) {
                *(int32_t *)value = state.timezone;
                len = sizeof(int32_t);
            }
            break;

        case ZB_TIME_ATTR_DST_START:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.dst_start;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_DST_END:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.dst_end;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_DST_SHIFT:
            if (max_len >= sizeof(int32_t)) {
                *(int32_t *)value = state.dst_shift;
                len = sizeof(int32_t);
            }
            break;

        case ZB_TIME_ATTR_STANDARD_TIME:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.standard_time;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_LOCAL_TIME:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.local_time;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_LAST_SET_TIME:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.last_set_time;
                len = sizeof(uint32_t);
            }
            break;

        case ZB_TIME_ATTR_VALID_UNTIL_TIME:
            if (max_len >= sizeof(uint32_t)) {
                *(uint32_t *)value = state.valid_until_time;
                len = sizeof(uint32_t);
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown attribute 0x%04X requested", attr_id);
            break;
    }

    return len;
}

esp_err_t zb_time_server_register_cluster(uint8_t endpoint)
{
    ESP_LOGI(TAG, "Registering Time Cluster on endpoint %d", endpoint);

    /* Note: The actual cluster registration depends on ESP-Zigbee-SDK.
     * This would typically use esp_zb_cluster_add_attr() or similar.
     *
     * Example (pseudo-code):
     * esp_zb_attribute_list_t *attr_list = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_TIME);
     * uint32_t time_value = zb_time_server_get_time();
     * esp_zb_time_cluster_add_attr(attr_list, ZB_TIME_ATTR_TIME, &time_value);
     * esp_zb_cluster_list_add_time_cluster(cluster_list, attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
     */

    ESP_LOGI(TAG, "Time Cluster registered (framework - actual registration via SDK)");

    return ESP_OK;
}

uint32_t zb_time_unix_to_zigbee(time_t unix_time)
{
    if (unix_time < ZB_TIME_EPOCH_OFFSET_SECONDS) {
        return ZB_TIME_INVALID;
    }
    return (uint32_t)(unix_time - ZB_TIME_EPOCH_OFFSET_SECONDS);
}

time_t zb_time_zigbee_to_unix(uint32_t zigbee_time)
{
    if (zigbee_time == ZB_TIME_INVALID) {
        return 0;
    }
    return (time_t)(zigbee_time + ZB_TIME_EPOCH_OFFSET_SECONDS);
}

esp_err_t zb_time_server_test(void)
{
    ESP_LOGI(TAG, "Running Time Server self-test...");

    /* Test time conversion */
    time_t test_unix = 1704067200;  /* 2024-01-01 00:00:00 UTC */
    uint32_t zigbee_time = zb_time_unix_to_zigbee(test_unix);
    time_t back_unix = zb_time_zigbee_to_unix(zigbee_time);

    if (back_unix != test_unix) {
        ESP_LOGE(TAG, "Time conversion round-trip failed: %ld != %ld",
                 (long)back_unix, (long)test_unix);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Time conversion test PASSED: Unix %ld -> Zigbee %lu -> Unix %ld",
             (long)test_unix, (unsigned long)zigbee_time, (long)back_unix);

    /* Test epoch offset */
    uint32_t epoch_test = zb_time_unix_to_zigbee(ZB_TIME_EPOCH_OFFSET_SECONDS);
    if (epoch_test != 0) {
        ESP_LOGE(TAG, "Epoch offset test failed: expected 0, got %lu", (unsigned long)epoch_test);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Epoch offset test PASSED");

    /* Test invalid time */
    uint32_t invalid_test = zb_time_unix_to_zigbee(100);  /* Before 2000 */
    if (invalid_test != ZB_TIME_INVALID) {
        ESP_LOGE(TAG, "Invalid time test failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Invalid time test PASSED");

    if (s_initialized) {
        /* Test state retrieval */
        zb_time_server_state_t state;
        esp_err_t ret = zb_time_server_get_state(&state);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Get state failed: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "State retrieval test PASSED");

        /* Test status bits */
        if (s_config.is_master && !(state.time_status & ZB_TIME_STATUS_MASTER)) {
            ESP_LOGE(TAG, "Master status bit not set");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Status bits test PASSED");
    }

    ESP_LOGI(TAG, "Time Server self-test PASSED");
    return ESP_OK;
}
