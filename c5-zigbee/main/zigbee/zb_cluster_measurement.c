/**
 * @file zb_cluster_measurement.c
 * @brief Measurement Cluster Implementations
 *
 * Provides support for:
 * - Illuminance Measurement (0x0400)
 * - Pressure Measurement (0x0403)
 * - PM2.5 Measurement (0x042A)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cluster_measurement.h"
#include "zb_cluster_internal.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ZB_MEAS";

/* ============================================================================
 * Illuminance State Storage
 * ============================================================================ */

static zb_illuminance_state_t *s_illuminance_states = NULL;
static uint16_t *s_illuminance_addrs = NULL;
static size_t s_illuminance_count = 0;
static zb_illuminance_state_cb_t s_illuminance_callback = NULL;

/* ============================================================================
 * Pressure State Storage
 * ============================================================================ */

static zb_pressure_state_t *s_pressure_states = NULL;
static uint16_t *s_pressure_addrs = NULL;
static size_t s_pressure_count = 0;
static zb_pressure_state_cb_t s_pressure_callback = NULL;

/* ============================================================================
 * PM2.5 State Storage
 * ============================================================================ */

static zb_pm25_state_t *s_pm25_states = NULL;
static uint16_t *s_pm25_addrs = NULL;
static size_t s_pm25_count = 0;
static zb_pm25_state_cb_t s_pm25_callback = NULL;

/* ============================================================================
 * Illuminance Internal Helpers
 * ============================================================================ */

/**
 * @brief Find illuminance state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_illuminance_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_illuminance_count; i++) {
        if (s_illuminance_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create illuminance state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_illuminance_state_t* get_or_create_illuminance_state(uint16_t short_addr)
{
    int idx = find_illuminance_state_index(short_addr);
    if (idx >= 0) {
        return &s_illuminance_states[idx];
    }

    /* Create new entry */
    if (s_illuminance_count >= ZB_STATE_MAX_ILLUMINANCE) {
        ESP_LOGW(TAG, "Illuminance state storage full");
        return NULL;
    }

    idx = s_illuminance_count++;
    s_illuminance_addrs[idx] = short_addr;
    memset(&s_illuminance_states[idx], 0, sizeof(zb_illuminance_state_t));
    s_illuminance_states[idx].measured_value = ZB_ZCL_ILLUMINANCE_MEASURED_VALUE_INVALID;

    ESP_LOGI(TAG, "Created illuminance state for device 0x%04X", short_addr);
    return &s_illuminance_states[idx];
}

/* ============================================================================
 * Pressure Internal Helpers
 * ============================================================================ */

/**
 * @brief Find pressure state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_pressure_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_pressure_count; i++) {
        if (s_pressure_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create pressure state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_pressure_state_t* get_or_create_pressure_state(uint16_t short_addr)
{
    int idx = find_pressure_state_index(short_addr);
    if (idx >= 0) {
        return &s_pressure_states[idx];
    }

    /* Create new entry */
    if (s_pressure_count >= ZB_STATE_MAX_PRESSURE) {
        ESP_LOGW(TAG, "Pressure state storage full");
        return NULL;
    }

    idx = s_pressure_count++;
    s_pressure_addrs[idx] = short_addr;
    memset(&s_pressure_states[idx], 0, sizeof(zb_pressure_state_t));
    s_pressure_states[idx].measured_value = ZB_ZCL_PRESSURE_MEASURED_VALUE_INVALID;
    s_pressure_states[idx].has_scaled = false;

    ESP_LOGI(TAG, "Created pressure state for device 0x%04X", short_addr);
    return &s_pressure_states[idx];
}

/* ============================================================================
 * PM2.5 Internal Helpers
 * ============================================================================ */

/**
 * @brief Find PM2.5 state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_pm25_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_pm25_count; i++) {
        if (s_pm25_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Check if float value is NaN
 *
 * @param[in] value Float value to check
 * @return true if value is NaN
 */
static bool is_float_nan(float value)
{
    return value != value;  /* NaN is the only value that is not equal to itself */
}

/**
 * @brief Get or create PM2.5 state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_pm25_state_t* get_or_create_pm25_state(uint16_t short_addr)
{
    int idx = find_pm25_state_index(short_addr);
    if (idx >= 0) {
        return &s_pm25_states[idx];
    }

    /* Create new entry */
    if (s_pm25_count >= ZB_STATE_MAX_PM25) {
        ESP_LOGW(TAG, "PM2.5 state storage full");
        return NULL;
    }

    idx = s_pm25_count++;
    s_pm25_addrs[idx] = short_addr;
    memset(&s_pm25_states[idx], 0, sizeof(zb_pm25_state_t));
    s_pm25_states[idx].is_valid = false;

    ESP_LOGI(TAG, "Created PM2.5 state for device 0x%04X", short_addr);
    return &s_pm25_states[idx];
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_cluster_measurement_init(void)
{
    /* Initialize Illuminance storage */
    s_illuminance_count = 0;
    s_illuminance_callback = NULL;

    /* Allocate Illuminance arrays in PSRAM */
    if (s_illuminance_states == NULL) {
        s_illuminance_states = heap_caps_calloc(ZB_STATE_MAX_ILLUMINANCE,
                                                 sizeof(zb_illuminance_state_t),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_illuminance_states) {
            s_illuminance_states = calloc(ZB_STATE_MAX_ILLUMINANCE,
                                           sizeof(zb_illuminance_state_t));  /* Fallback */
        }
    }
    if (s_illuminance_addrs == NULL) {
        s_illuminance_addrs = heap_caps_calloc(ZB_STATE_MAX_ILLUMINANCE,
                                                sizeof(uint16_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_illuminance_addrs) {
            s_illuminance_addrs = calloc(ZB_STATE_MAX_ILLUMINANCE,
                                          sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_illuminance_states || !s_illuminance_addrs) {
        ESP_LOGE(TAG, "Failed to allocate illuminance state arrays");
        free(s_illuminance_states);
        s_illuminance_states = NULL;
        free(s_illuminance_addrs);
        s_illuminance_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize Pressure storage */
    s_pressure_count = 0;
    s_pressure_callback = NULL;

    /* Allocate Pressure arrays in PSRAM */
    if (s_pressure_states == NULL) {
        s_pressure_states = heap_caps_calloc(ZB_STATE_MAX_PRESSURE,
                                              sizeof(zb_pressure_state_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_pressure_states) {
            s_pressure_states = calloc(ZB_STATE_MAX_PRESSURE,
                                        sizeof(zb_pressure_state_t));  /* Fallback */
        }
    }
    if (s_pressure_addrs == NULL) {
        s_pressure_addrs = heap_caps_calloc(ZB_STATE_MAX_PRESSURE,
                                             sizeof(uint16_t),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_pressure_addrs) {
            s_pressure_addrs = calloc(ZB_STATE_MAX_PRESSURE,
                                       sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_pressure_states || !s_pressure_addrs) {
        ESP_LOGE(TAG, "Failed to allocate pressure state arrays");
        free(s_illuminance_states);
        s_illuminance_states = NULL;
        free(s_illuminance_addrs);
        s_illuminance_addrs = NULL;
        free(s_pressure_states);
        s_pressure_states = NULL;
        free(s_pressure_addrs);
        s_pressure_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize PM2.5 storage */
    s_pm25_count = 0;
    s_pm25_callback = NULL;

    /* Allocate PM2.5 arrays in PSRAM */
    if (s_pm25_states == NULL) {
        s_pm25_states = heap_caps_calloc(ZB_STATE_MAX_PM25,
                                          sizeof(zb_pm25_state_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_pm25_states) {
            s_pm25_states = calloc(ZB_STATE_MAX_PM25,
                                    sizeof(zb_pm25_state_t));  /* Fallback */
        }
    }
    if (s_pm25_addrs == NULL) {
        s_pm25_addrs = heap_caps_calloc(ZB_STATE_MAX_PM25,
                                         sizeof(uint16_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_pm25_addrs) {
            s_pm25_addrs = calloc(ZB_STATE_MAX_PM25,
                                   sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_pm25_states || !s_pm25_addrs) {
        ESP_LOGE(TAG, "Failed to allocate PM2.5 state arrays");
        free(s_illuminance_states);
        s_illuminance_states = NULL;
        free(s_illuminance_addrs);
        s_illuminance_addrs = NULL;
        free(s_pressure_states);
        s_pressure_states = NULL;
        free(s_pressure_addrs);
        s_pressure_addrs = NULL;
        free(s_pm25_states);
        s_pm25_states = NULL;
        free(s_pm25_addrs);
        s_pm25_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Measurement clusters initialized (Illum: %d, Press: %d, PM2.5: %d max)",
             ZB_STATE_MAX_ILLUMINANCE, ZB_STATE_MAX_PRESSURE, ZB_STATE_MAX_PM25);
    return ESP_OK;
}

esp_err_t zb_cluster_measurement_deinit(void)
{
    s_illuminance_count = 0;
    s_illuminance_callback = NULL;
    s_pressure_count = 0;
    s_pressure_callback = NULL;
    s_pm25_count = 0;
    s_pm25_callback = NULL;
    ESP_LOGI(TAG, "Measurement clusters deinitialized");
    return ESP_OK;
}

void zb_cluster_measurement_clear_all(void)
{
    s_illuminance_count = 0;
    s_pressure_count = 0;
    s_pm25_count = 0;
    if (s_illuminance_states) {
        memset(s_illuminance_states, 0,
               ZB_STATE_MAX_ILLUMINANCE * sizeof(zb_illuminance_state_t));
    }
    if (s_illuminance_addrs) {
        memset(s_illuminance_addrs, 0,
               ZB_STATE_MAX_ILLUMINANCE * sizeof(uint16_t));
    }
    if (s_pressure_states) {
        memset(s_pressure_states, 0,
               ZB_STATE_MAX_PRESSURE * sizeof(zb_pressure_state_t));
    }
    if (s_pressure_addrs) {
        memset(s_pressure_addrs, 0,
               ZB_STATE_MAX_PRESSURE * sizeof(uint16_t));
    }
    if (s_pm25_states) {
        memset(s_pm25_states, 0,
               ZB_STATE_MAX_PM25 * sizeof(zb_pm25_state_t));
    }
    if (s_pm25_addrs) {
        memset(s_pm25_addrs, 0,
               ZB_STATE_MAX_PM25 * sizeof(uint16_t));
    }
    ESP_LOGD(TAG, "Cleared all measurement cluster states");
}

void zb_cluster_measurement_remove_device(uint16_t short_addr)
{
    /* Remove from Illuminance storage */
    for (size_t i = 0; i < s_illuminance_count; i++) {
        if (s_illuminance_addrs[i] == short_addr) {
            if (i < s_illuminance_count - 1) {
                s_illuminance_states[i] = s_illuminance_states[s_illuminance_count - 1];
                s_illuminance_addrs[i] = s_illuminance_addrs[s_illuminance_count - 1];
            }
            s_illuminance_count--;
            ESP_LOGD(TAG, "Removed illuminance state for device 0x%04X", short_addr);
            break;
        }
    }

    /* Remove from Pressure storage */
    for (size_t i = 0; i < s_pressure_count; i++) {
        if (s_pressure_addrs[i] == short_addr) {
            if (i < s_pressure_count - 1) {
                s_pressure_states[i] = s_pressure_states[s_pressure_count - 1];
                s_pressure_addrs[i] = s_pressure_addrs[s_pressure_count - 1];
            }
            s_pressure_count--;
            ESP_LOGD(TAG, "Removed pressure state for device 0x%04X", short_addr);
            break;
        }
    }

    /* Remove from PM2.5 storage */
    for (size_t i = 0; i < s_pm25_count; i++) {
        if (s_pm25_addrs[i] == short_addr) {
            if (i < s_pm25_count - 1) {
                s_pm25_states[i] = s_pm25_states[s_pm25_count - 1];
                s_pm25_addrs[i] = s_pm25_addrs[s_pm25_count - 1];
            }
            s_pm25_count--;
            ESP_LOGD(TAG, "Removed PM2.5 state for device 0x%04X", short_addr);
            break;
        }
    }
}

/* ============================================================================
 * Illuminance Cluster Public API
 * ============================================================================ */

esp_err_t zb_illuminance_register_callback(zb_illuminance_state_cb_t callback)
{
    s_illuminance_callback = callback;
    ESP_LOGI(TAG, "Illuminance state callback registered");
    return ESP_OK;
}

bool zb_device_has_illuminance(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
}

esp_err_t zb_illuminance_read_value(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading illuminance from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_ILLUMINANCE_MEASURED_VALUE_ID,
        ZB_ZCL_ATTR_ILLUMINANCE_MIN_MEASURED_ID,
        ZB_ZCL_ATTR_ILLUMINANCE_MAX_MEASURED_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send illuminance read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_illuminance_handle_report(uint16_t short_addr, uint8_t endpoint,
                                        uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_illuminance_state_t *state = get_or_create_illuminance_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_ILLUMINANCE_MEASURED_VALUE_ID:
            if (value_len >= 2) {
                state->measured_value = *(uint16_t *)value;
                float lux = zb_illuminance_to_lux(state->measured_value);
                ESP_LOGI(TAG, "Illuminance 0x%04X: raw=%u, %.1f lux",
                         short_addr, state->measured_value, lux);
            }
            break;

        case ZB_ZCL_ATTR_ILLUMINANCE_MIN_MEASURED_ID:
            if (value_len >= 2) {
                state->min_measured = *(uint16_t *)value;
                ESP_LOGD(TAG, "Illuminance 0x%04X min: %u", short_addr, state->min_measured);
            }
            break;

        case ZB_ZCL_ATTR_ILLUMINANCE_MAX_MEASURED_ID:
            if (value_len >= 2) {
                state->max_measured = *(uint16_t *)value;
                ESP_LOGD(TAG, "Illuminance 0x%04X max: %u", short_addr, state->max_measured);
            }
            break;

        case ZB_ZCL_ATTR_ILLUMINANCE_TOLERANCE_ID:
            if (value_len >= 2) {
                state->tolerance = *(uint16_t *)value;
                ESP_LOGD(TAG, "Illuminance 0x%04X tolerance: %u", short_addr, state->tolerance);
            }
            break;

        case ZB_ZCL_ATTR_ILLUMINANCE_LIGHT_SENSOR_TYPE_ID:
            if (value_len >= 1) {
                state->light_sensor_type = *(uint8_t *)value;
                ESP_LOGD(TAG, "Illuminance 0x%04X sensor type: %d", short_addr, state->light_sensor_type);
            }
            break;

        default:
            ESP_LOGD(TAG, "Illuminance 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_illuminance_callback != NULL) {
        s_illuminance_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_illuminance_get_state(uint16_t short_addr, zb_illuminance_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_illuminance_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_illuminance_states[idx], sizeof(zb_illuminance_state_t));
    return ESP_OK;
}

float zb_illuminance_to_lux(uint16_t raw_value)
{
    /* Handle special values */
    if (raw_value == ZB_ZCL_ILLUMINANCE_MEASURED_VALUE_INVALID ||
        raw_value == ZB_ZCL_ILLUMINANCE_MEASURED_VALUE_TOO_LOW) {
        return 0.0f;
    }

    /* Convert using formula: lux = 10^((value - 1) / ZCL_ILLUMINANCE_SCALE) */
    float exponent = (float)(raw_value - 1) / (float)ZCL_ILLUMINANCE_SCALE;
    return powf(10.0f, exponent);
}

/* ============================================================================
 * Pressure Cluster Public API
 * ============================================================================ */

esp_err_t zb_pressure_register_callback(zb_pressure_state_cb_t callback)
{
    s_pressure_callback = callback;
    ESP_LOGI(TAG, "Pressure state callback registered");
    return ESP_OK;
}

bool zb_device_has_pressure(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
}

esp_err_t zb_pressure_read_value(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading pressure from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_PRESSURE_MEASURED_VALUE_ID,
        ZB_ZCL_ATTR_PRESSURE_SCALED_VALUE_ID,
        ZB_ZCL_ATTR_PRESSURE_SCALE_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send pressure read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_pressure_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_pressure_state_t *state = get_or_create_pressure_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_PRESSURE_MEASURED_VALUE_ID:
            if (value_len >= 2) {
                state->measured_value = *(int16_t *)value;
                float hpa = zb_pressure_to_hpa(state);
                ESP_LOGI(TAG, "Pressure 0x%04X: raw=%d, %.2f hPa",
                         short_addr, state->measured_value, hpa);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_MIN_MEASURED_ID:
            if (value_len >= 2) {
                state->min_measured = *(int16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X min: %d", short_addr, state->min_measured);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_MAX_MEASURED_ID:
            if (value_len >= 2) {
                state->max_measured = *(int16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X max: %d", short_addr, state->max_measured);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_TOLERANCE_ID:
            if (value_len >= 2) {
                state->tolerance = *(uint16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X tolerance: %u", short_addr, state->tolerance);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_SCALED_VALUE_ID:
            if (value_len >= 2) {
                state->scaled_value = *(int16_t *)value;
                state->has_scaled = true;
                ESP_LOGD(TAG, "Pressure 0x%04X scaled value: %d", short_addr, state->scaled_value);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_MIN_SCALED_VALUE_ID:
            if (value_len >= 2) {
                state->min_scaled_value = *(int16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X min scaled: %d", short_addr, state->min_scaled_value);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_MAX_SCALED_VALUE_ID:
            if (value_len >= 2) {
                state->max_scaled_value = *(int16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X max scaled: %d", short_addr, state->max_scaled_value);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_SCALED_TOLERANCE_ID:
            if (value_len >= 2) {
                state->scaled_tolerance = *(uint16_t *)value;
                ESP_LOGD(TAG, "Pressure 0x%04X scaled tolerance: %u", short_addr, state->scaled_tolerance);
            }
            break;

        case ZB_ZCL_ATTR_PRESSURE_SCALE_ID:
            if (value_len >= 1) {
                state->scale = *(int8_t *)value;
                state->has_scaled = true;
                ESP_LOGD(TAG, "Pressure 0x%04X scale: %d", short_addr, state->scale);
            }
            break;

        default:
            ESP_LOGD(TAG, "Pressure 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_pressure_callback != NULL) {
        s_pressure_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_pressure_get_state(uint16_t short_addr, zb_pressure_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_pressure_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_pressure_states[idx], sizeof(zb_pressure_state_t));
    return ESP_OK;
}

float zb_pressure_to_hpa(const zb_pressure_state_t *state)
{
    if (state == NULL) {
        return 0.0f;
    }

    /* Check for invalid value */
    if (state->measured_value == ZB_ZCL_PRESSURE_MEASURED_VALUE_INVALID) {
        return 0.0f;
    }

    /* Use scaled value if available for higher precision */
    if (state->has_scaled && state->scale != 0) {
        /* scaled_value * 10^scale gives pressure in kPa */
        /* Convert to hPa: kPa * 10 = hPa */
        float scale_factor = powf(10.0f, (float)state->scale);
        return (float)state->scaled_value * scale_factor * 10.0f;
    }

    /* Standard value is in 10 Pa (0.1 hPa) units */
    /* So divide by ZCL_PRESSURE_SCALE to get hPa */
    return (float)state->measured_value / (float)ZCL_PRESSURE_SCALE;
}

/* ============================================================================
 * PM2.5 Cluster Public API
 * ============================================================================ */

esp_err_t zb_pm25_register_callback(zb_pm25_state_cb_t callback)
{
    s_pm25_callback = callback;
    ESP_LOGI(TAG, "PM2.5 state callback registered");
    return ESP_OK;
}

bool zb_device_has_pm25(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_PM25_MEASUREMENT);
}

esp_err_t zb_pm25_read_value(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading PM2.5 from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_PM25_MEASUREMENT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_PM25_MEASURED_VALUE_ID,
        ZB_ZCL_ATTR_PM25_MIN_MEASURED_ID,
        ZB_ZCL_ATTR_PM25_MAX_MEASURED_ID,
        ZB_ZCL_ATTR_PM25_TOLERANCE_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send PM2.5 read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_pm25_handle_report(uint16_t short_addr, uint8_t endpoint,
                                 uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_pm25_state_t *state = get_or_create_pm25_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_PM25_MEASURED_VALUE_ID:
            if (value_len >= 4) {
                float measured = *(float *)value;
                state->measured_value = measured;
                state->is_valid = !is_float_nan(measured);
                if (state->is_valid) {
                    ESP_LOGI(TAG, "PM2.5 0x%04X: %.1f ug/m3", short_addr, measured);
                } else {
                    ESP_LOGW(TAG, "PM2.5 0x%04X: invalid value (NaN)", short_addr);
                }
            }
            break;

        case ZB_ZCL_ATTR_PM25_MIN_MEASURED_ID:
            if (value_len >= 4) {
                state->min_measured = *(float *)value;
                ESP_LOGD(TAG, "PM2.5 0x%04X min: %.1f", short_addr, state->min_measured);
            }
            break;

        case ZB_ZCL_ATTR_PM25_MAX_MEASURED_ID:
            if (value_len >= 4) {
                state->max_measured = *(float *)value;
                ESP_LOGD(TAG, "PM2.5 0x%04X max: %.1f", short_addr, state->max_measured);
            }
            break;

        case ZB_ZCL_ATTR_PM25_TOLERANCE_ID:
            if (value_len >= 4) {
                state->tolerance = *(float *)value;
                ESP_LOGD(TAG, "PM2.5 0x%04X tolerance: %.1f", short_addr, state->tolerance);
            }
            break;

        default:
            ESP_LOGD(TAG, "PM2.5 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_pm25_callback != NULL) {
        s_pm25_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_pm25_get_state(uint16_t short_addr, zb_pm25_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_pm25_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_pm25_states[idx], sizeof(zb_pm25_state_t));
    return ESP_OK;
}
