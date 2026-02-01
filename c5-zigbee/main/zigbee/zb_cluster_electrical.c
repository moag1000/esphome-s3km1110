/**
 * @file zb_cluster_electrical.c
 * @brief Electrical Measurement (0x0B04) and Metering (0x0702) Cluster Implementations
 *
 * Provides support for electrical measurement and smart metering devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cluster_electrical.h"
#include "zb_cluster_internal.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "ZB_ELEC";

/* ============================================================================
 * Electrical Measurement State Storage
 * ============================================================================ */

static zb_electrical_state_t *s_electrical_states = NULL;
static uint16_t *s_electrical_addrs = NULL;
static size_t s_electrical_count = 0;
static zb_electrical_state_cb_t s_electrical_callback = NULL;

/* ============================================================================
 * Metering State Storage
 * ============================================================================ */

static zb_metering_state_t *s_metering_states = NULL;
static uint16_t *s_metering_addrs = NULL;
static size_t s_metering_count = 0;
static zb_metering_state_cb_t s_metering_callback = NULL;

/* ============================================================================
 * Electrical Measurement Internal Helpers
 * ============================================================================ */

/**
 * @brief Find electrical measurement state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_electrical_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_electrical_count; i++) {
        if (s_electrical_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create electrical measurement state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_electrical_state_t* get_or_create_electrical_state(uint16_t short_addr)
{
    int idx = find_electrical_state_index(short_addr);
    if (idx >= 0) {
        return &s_electrical_states[idx];
    }

    /* Create new entry */
    if (s_electrical_count >= ZB_STATE_MAX_ELECTRICAL) {
        ESP_LOGW(TAG, "Electrical measurement state storage full");
        return NULL;
    }

    idx = s_electrical_count++;
    s_electrical_addrs[idx] = short_addr;
    memset(&s_electrical_states[idx], 0, sizeof(zb_electrical_state_t));

    /* Set default scaling factors (commonly used defaults) */
    s_electrical_states[idx].voltage_multiplier = 1;
    s_electrical_states[idx].voltage_divisor = 10;                          /* V * 10 */
    s_electrical_states[idx].current_multiplier = 1;
    s_electrical_states[idx].current_divisor = ZB_ELECTRICAL_CURRENT_DIVISOR_MA;  /* mA */
    s_electrical_states[idx].power_multiplier = 1;
    s_electrical_states[idx].power_divisor = 10;        /* W * 10 */
    s_electrical_states[idx].scaling_factors_read = false;
    s_electrical_states[idx].is_valid = false;

    ESP_LOGI(TAG, "Created electrical measurement state for device 0x%04X", short_addr);
    return &s_electrical_states[idx];
}

/* ============================================================================
 * Metering Internal Helpers
 * ============================================================================ */

/**
 * @brief Find metering state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_metering_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_metering_count; i++) {
        if (s_metering_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create metering state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_metering_state_t* get_or_create_metering_state(uint16_t short_addr)
{
    int idx = find_metering_state_index(short_addr);
    if (idx >= 0) {
        return &s_metering_states[idx];
    }

    /* Create new entry */
    if (s_metering_count >= ZB_STATE_MAX_METERING) {
        ESP_LOGW(TAG, "Metering state storage full");
        return NULL;
    }

    idx = s_metering_count++;
    s_metering_addrs[idx] = short_addr;
    memset(&s_metering_states[idx], 0, sizeof(zb_metering_state_t));

    /* Initialize with default values */
    s_metering_states[idx].multiplier = 1;  /* Default multiplier */
    s_metering_states[idx].divisor = 1;     /* Default divisor (avoid division by zero) */
    s_metering_states[idx].unit_of_measure = ZB_METERING_UNIT_KWH;

    ESP_LOGI(TAG, "Created metering state for device 0x%04X", short_addr);
    return &s_metering_states[idx];
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_cluster_electrical_init(void)
{
    /* Initialize Electrical Measurement storage */
    s_electrical_count = 0;
    s_electrical_callback = NULL;

    /* Allocate Electrical arrays in PSRAM */
    if (s_electrical_states == NULL) {
        s_electrical_states = heap_caps_calloc(ZB_STATE_MAX_ELECTRICAL,
                                               sizeof(zb_electrical_state_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_electrical_states) {
            s_electrical_states = calloc(ZB_STATE_MAX_ELECTRICAL,
                                          sizeof(zb_electrical_state_t));  /* Fallback */
        }
    }
    if (s_electrical_addrs == NULL) {
        s_electrical_addrs = heap_caps_calloc(ZB_STATE_MAX_ELECTRICAL,
                                               sizeof(uint16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_electrical_addrs) {
            s_electrical_addrs = calloc(ZB_STATE_MAX_ELECTRICAL,
                                         sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_electrical_states || !s_electrical_addrs) {
        ESP_LOGE(TAG, "Failed to allocate electrical state arrays");
        free(s_electrical_states);
        s_electrical_states = NULL;
        free(s_electrical_addrs);
        s_electrical_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize Metering storage */
    s_metering_count = 0;
    s_metering_callback = NULL;

    /* Allocate Metering arrays in PSRAM */
    if (s_metering_states == NULL) {
        s_metering_states = heap_caps_calloc(ZB_STATE_MAX_METERING,
                                              sizeof(zb_metering_state_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_metering_states) {
            s_metering_states = calloc(ZB_STATE_MAX_METERING,
                                        sizeof(zb_metering_state_t));  /* Fallback */
        }
    }
    if (s_metering_addrs == NULL) {
        s_metering_addrs = heap_caps_calloc(ZB_STATE_MAX_METERING,
                                             sizeof(uint16_t),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_metering_addrs) {
            s_metering_addrs = calloc(ZB_STATE_MAX_METERING,
                                       sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_metering_states || !s_metering_addrs) {
        ESP_LOGE(TAG, "Failed to allocate metering state arrays");
        free(s_electrical_states);
        s_electrical_states = NULL;
        free(s_electrical_addrs);
        s_electrical_addrs = NULL;
        free(s_metering_states);
        s_metering_states = NULL;
        free(s_metering_addrs);
        s_metering_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Electrical clusters initialized (Electrical: %d, Metering: %d max)",
             ZB_STATE_MAX_ELECTRICAL, ZB_STATE_MAX_METERING);
    return ESP_OK;
}

esp_err_t zb_cluster_electrical_deinit(void)
{
    s_electrical_count = 0;
    s_electrical_callback = NULL;
    s_metering_count = 0;
    s_metering_callback = NULL;
    ESP_LOGI(TAG, "Electrical clusters deinitialized");
    return ESP_OK;
}

void zb_cluster_electrical_clear_all(void)
{
    s_electrical_count = 0;
    s_metering_count = 0;
    if (s_electrical_states) {
        memset(s_electrical_states, 0,
               ZB_STATE_MAX_ELECTRICAL * sizeof(zb_electrical_state_t));
    }
    if (s_electrical_addrs) {
        memset(s_electrical_addrs, 0,
               ZB_STATE_MAX_ELECTRICAL * sizeof(uint16_t));
    }
    if (s_metering_states) {
        memset(s_metering_states, 0,
               ZB_STATE_MAX_METERING * sizeof(zb_metering_state_t));
    }
    if (s_metering_addrs) {
        memset(s_metering_addrs, 0,
               ZB_STATE_MAX_METERING * sizeof(uint16_t));
    }
    ESP_LOGD(TAG, "Cleared all electrical cluster states");
}

void zb_cluster_electrical_remove_device(uint16_t short_addr)
{
    /* Remove from Electrical Measurement storage */
    for (size_t i = 0; i < s_electrical_count; i++) {
        if (s_electrical_addrs[i] == short_addr) {
            if (i < s_electrical_count - 1) {
                s_electrical_states[i] = s_electrical_states[s_electrical_count - 1];
                s_electrical_addrs[i] = s_electrical_addrs[s_electrical_count - 1];
            }
            s_electrical_count--;
            ESP_LOGD(TAG, "Removed electrical state for device 0x%04X", short_addr);
            break;
        }
    }

    /* Remove from Metering storage */
    for (size_t i = 0; i < s_metering_count; i++) {
        if (s_metering_addrs[i] == short_addr) {
            if (i < s_metering_count - 1) {
                s_metering_states[i] = s_metering_states[s_metering_count - 1];
                s_metering_addrs[i] = s_metering_addrs[s_metering_count - 1];
            }
            s_metering_count--;
            ESP_LOGD(TAG, "Removed metering state for device 0x%04X", short_addr);
            break;
        }
    }
}

/* ============================================================================
 * Electrical Measurement Cluster Public API
 * ============================================================================ */

esp_err_t zb_electrical_register_callback(zb_electrical_state_cb_t callback)
{
    s_electrical_callback = callback;
    ESP_LOGI(TAG, "Electrical measurement state callback registered");
    return ESP_OK;
}

bool zb_device_has_electrical_measurement(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT);
}

esp_err_t zb_electrical_read_values(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading electrical measurement values from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request for main measurement values */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_ELECTRICAL_RMS_VOLTAGE_ID,
        ZB_ZCL_ATTR_ELECTRICAL_RMS_CURRENT_ID,
        ZB_ZCL_ATTR_ELECTRICAL_ACTIVE_POWER_ID,
        ZB_ZCL_ATTR_ELECTRICAL_POWER_FACTOR_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_FREQUENCY_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send electrical measurement read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_electrical_read_scaling(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading electrical measurement scaling factors from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request for scaling factors */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_MULTIPLIER_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_DIVISOR_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_MULTIPLIER_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_DIVISOR_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_MULTIPLIER_ID,
        ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_DIVISOR_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send electrical scaling read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_electrical_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_electrical_state_t *state = get_or_create_electrical_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        /* Main measurement values */
        case ZB_ZCL_ATTR_ELECTRICAL_RMS_VOLTAGE_ID:
            if (value_len >= 2) {
                state->rms_voltage = *(uint16_t *)value;
                state->is_valid = true;
                float voltage = zb_electrical_get_voltage_v(state);
                ESP_LOGI(TAG, "Electrical 0x%04X voltage: raw=%u, %.1f V",
                         short_addr, state->rms_voltage, voltage);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_RMS_CURRENT_ID:
            if (value_len >= 2) {
                state->rms_current = *(uint16_t *)value;
                state->is_valid = true;
                float current = zb_electrical_get_current_a(state);
                ESP_LOGI(TAG, "Electrical 0x%04X current: raw=%u, %.3f A",
                         short_addr, state->rms_current, current);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_ACTIVE_POWER_ID:
            if (value_len >= 2) {
                state->active_power = *(int16_t *)value;
                state->is_valid = true;
                float power = zb_electrical_get_power_w(state);
                ESP_LOGI(TAG, "Electrical 0x%04X power: raw=%d, %.1f W",
                         short_addr, state->active_power, power);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_REACTIVE_POWER_ID:
            if (value_len >= 2) {
                state->reactive_power = *(int16_t *)value;
                float reactive = zb_electrical_get_reactive_power_var(state);
                ESP_LOGD(TAG, "Electrical 0x%04X reactive power: raw=%d, %.1f VAR",
                         short_addr, state->reactive_power, reactive);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_APPARENT_POWER_ID:
            if (value_len >= 2) {
                state->apparent_power = *(uint16_t *)value;
                float apparent = zb_electrical_get_apparent_power_va(state);
                ESP_LOGD(TAG, "Electrical 0x%04X apparent power: raw=%u, %.1f VA",
                         short_addr, state->apparent_power, apparent);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_POWER_FACTOR_ID:
            if (value_len >= 1) {
                state->power_factor = *(int8_t *)value;
                float pf = zb_electrical_get_power_factor(state);
                ESP_LOGI(TAG, "Electrical 0x%04X power factor: raw=%d, %.2f",
                         short_addr, state->power_factor, pf);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_FREQUENCY_ID:
            if (value_len >= 2) {
                state->ac_frequency = *(uint16_t *)value;
                float freq = zb_electrical_get_frequency_hz(state);
                ESP_LOGD(TAG, "Electrical 0x%04X frequency: raw=%u, %.1f Hz",
                         short_addr, state->ac_frequency, freq);
            }
            break;

        /* Scaling factors */
        case ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_MULTIPLIER_ID:
            if (value_len >= 2) {
                state->voltage_multiplier = *(uint16_t *)value;
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X voltage multiplier: %u",
                         short_addr, state->voltage_multiplier);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_DIVISOR_ID:
            if (value_len >= 2) {
                state->voltage_divisor = *(uint16_t *)value;
                if (state->voltage_divisor == 0) {
                    state->voltage_divisor = 1;  /* Prevent division by zero */
                }
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X voltage divisor: %u",
                         short_addr, state->voltage_divisor);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_MULTIPLIER_ID:
            if (value_len >= 2) {
                state->current_multiplier = *(uint16_t *)value;
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X current multiplier: %u",
                         short_addr, state->current_multiplier);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_DIVISOR_ID:
            if (value_len >= 2) {
                state->current_divisor = *(uint16_t *)value;
                if (state->current_divisor == 0) {
                    state->current_divisor = 1;  /* Prevent division by zero */
                }
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X current divisor: %u",
                         short_addr, state->current_divisor);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_MULTIPLIER_ID:
            if (value_len >= 2) {
                state->power_multiplier = *(uint16_t *)value;
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X power multiplier: %u",
                         short_addr, state->power_multiplier);
            }
            break;

        case ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_DIVISOR_ID:
            if (value_len >= 2) {
                state->power_divisor = *(uint16_t *)value;
                if (state->power_divisor == 0) {
                    state->power_divisor = 1;  /* Prevent division by zero */
                }
                state->scaling_factors_read = true;
                ESP_LOGD(TAG, "Electrical 0x%04X power divisor: %u",
                         short_addr, state->power_divisor);
            }
            break;

        default:
            ESP_LOGD(TAG, "Electrical 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_electrical_callback != NULL) {
        s_electrical_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_electrical_get_state(uint16_t short_addr, zb_electrical_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_electrical_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_electrical_states[idx], sizeof(zb_electrical_state_t));
    return ESP_OK;
}

float zb_electrical_get_voltage_v(const zb_electrical_state_t *state)
{
    if (state == NULL || state->voltage_divisor == 0) {
        return 0.0f;
    }

    return (float)state->rms_voltage * (float)state->voltage_multiplier /
           (float)state->voltage_divisor;
}

float zb_electrical_get_current_a(const zb_electrical_state_t *state)
{
    if (state == NULL || state->current_divisor == 0) {
        return 0.0f;
    }

    return (float)state->rms_current * (float)state->current_multiplier /
           (float)state->current_divisor;
}

float zb_electrical_get_power_w(const zb_electrical_state_t *state)
{
    if (state == NULL || state->power_divisor == 0) {
        return 0.0f;
    }

    return (float)state->active_power * (float)state->power_multiplier /
           (float)state->power_divisor;
}

float zb_electrical_get_reactive_power_var(const zb_electrical_state_t *state)
{
    if (state == NULL || state->power_divisor == 0) {
        return 0.0f;
    }

    /* Reactive power uses the same scaling as active power */
    return (float)state->reactive_power * (float)state->power_multiplier /
           (float)state->power_divisor;
}

float zb_electrical_get_apparent_power_va(const zb_electrical_state_t *state)
{
    if (state == NULL || state->power_divisor == 0) {
        return 0.0f;
    }

    /* Apparent power uses the same scaling as active power */
    return (float)state->apparent_power * (float)state->power_multiplier /
           (float)state->power_divisor;
}

float zb_electrical_get_power_factor(const zb_electrical_state_t *state)
{
    if (state == NULL) {
        return 0.0f;
    }

    /* Power factor is stored as percentage (-100 to 100) */
    return (float)state->power_factor / (float)ZCL_TEMP_SCALE;
}

float zb_electrical_get_frequency_hz(const zb_electrical_state_t *state)
{
    if (state == NULL) {
        return 0.0f;
    }

    /* Frequency is typically stored as Hz * ZCL_POWER_SCALE */
    return (float)state->ac_frequency / (float)ZCL_POWER_SCALE;
}

/* ============================================================================
 * Metering Cluster Public API
 * ============================================================================ */

uint64_t zb_metering_parse_uint48(const void *data, size_t len)
{
    if (data == NULL || len < 6) {
        return 0;
    }

    const uint8_t *bytes = (const uint8_t *)data;
    uint64_t value = 0;

    /* Zigbee uses little-endian format */
    value = (uint64_t)bytes[0] |
            ((uint64_t)bytes[1] << 8) |
            ((uint64_t)bytes[2] << 16) |
            ((uint64_t)bytes[3] << 24) |
            ((uint64_t)bytes[4] << 32) |
            ((uint64_t)bytes[5] << 40);

    /* Mask to 48 bits */
    return value & 0xFFFFFFFFFFFFULL;
}

int32_t zb_metering_parse_int24(const void *data, size_t len)
{
    if (data == NULL || len < 3) {
        return 0;
    }

    const uint8_t *bytes = (const uint8_t *)data;

    /* Zigbee uses little-endian format */
    int32_t value = (int32_t)bytes[0] |
                    ((int32_t)bytes[1] << 8) |
                    ((int32_t)bytes[2] << 16);

    /* Sign extend from 24-bit to 32-bit */
    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    return value;
}

uint32_t zb_metering_parse_uint24(const void *data, size_t len)
{
    if (data == NULL || len < 3) {
        return 0;
    }

    const uint8_t *bytes = (const uint8_t *)data;

    /* Zigbee uses little-endian format */
    uint32_t value = (uint32_t)bytes[0] |
                     ((uint32_t)bytes[1] << 8) |
                     ((uint32_t)bytes[2] << 16);

    return value & 0xFFFFFF;
}

uint8_t zb_metering_get_decimal_places(uint8_t formatting)
{
    /* Bits 0-2: Number of digits to the right of decimal point */
    return formatting & 0x07;
}

const char* zb_metering_get_unit_string(uint8_t unit)
{
    switch (unit) {
        case ZB_METERING_UNIT_KWH:
            return "kWh";
        case ZB_METERING_UNIT_CUBIC_METERS:
            return "m\u00B3";  /* m3 with superscript 3 */
        case ZB_METERING_UNIT_CUBIC_FEET:
            return "ft\u00B3"; /* ft3 with superscript 3 */
        case ZB_METERING_UNIT_CCF:
            return "ccf";
        case ZB_METERING_UNIT_US_GAL:
            return "gal";
        case ZB_METERING_UNIT_IMP_GAL:
            return "gal (IMP)";
        case ZB_METERING_UNIT_BTU:
            return "BTU";
        case ZB_METERING_UNIT_LITERS:
            return "L";
        case ZB_METERING_UNIT_KPA_GAUGE:
        case ZB_METERING_UNIT_KPA_ABSOLUTE:
            return "kPa";
        case ZB_METERING_UNIT_MCF:
            return "mcf";
        case ZB_METERING_UNIT_MJ:
            return "MJ";
        case ZB_METERING_UNIT_KVAR:
            return "kVAr";
        case ZB_METERING_UNIT_UNITLESS:
        default:
            return "";
    }
}

double zb_metering_get_total_energy(const zb_metering_state_t *state)
{
    if (state == NULL) {
        return 0.0;
    }

    /* Apply multiplier and divisor */
    uint32_t divisor = (state->divisor > 0) ? state->divisor : 1;
    uint32_t multiplier = (state->multiplier > 0) ? state->multiplier : 1;

    return ((double)state->current_summation_delivered * (double)multiplier) / (double)divisor;
}

double zb_metering_get_power_w(const zb_metering_state_t *state)
{
    if (state == NULL) {
        return 0.0;
    }

    /* Apply multiplier and divisor */
    uint32_t divisor = (state->divisor > 0) ? state->divisor : 1;
    uint32_t multiplier = (state->multiplier > 0) ? state->multiplier : 1;

    double power = ((double)state->instantaneous_demand * (double)multiplier) / (double)divisor;

    /* For kWh unit, instantaneous demand is typically in kW, convert to W */
    if (state->unit_of_measure == ZB_METERING_UNIT_KWH) {
        power *= 1000.0;
    }

    return power;
}

double zb_metering_get_current_day_energy(const zb_metering_state_t *state)
{
    if (state == NULL) {
        return 0.0;
    }

    /* Apply multiplier and divisor */
    uint32_t divisor = (state->divisor > 0) ? state->divisor : 1;
    uint32_t multiplier = (state->multiplier > 0) ? state->multiplier : 1;

    return ((double)state->current_day_consumption * (double)multiplier) / (double)divisor;
}

double zb_metering_get_previous_day_energy(const zb_metering_state_t *state)
{
    if (state == NULL) {
        return 0.0;
    }

    /* Apply multiplier and divisor */
    uint32_t divisor = (state->divisor > 0) ? state->divisor : 1;
    uint32_t multiplier = (state->multiplier > 0) ? state->multiplier : 1;

    return ((double)state->previous_day_consumption * (double)multiplier) / (double)divisor;
}

esp_err_t zb_metering_register_callback(zb_metering_state_cb_t callback)
{
    s_metering_callback = callback;
    ESP_LOGI(TAG, "Metering state callback registered");
    return ESP_OK;
}

bool zb_device_has_metering(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_METERING);
}

esp_err_t zb_metering_read_values(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading metering values from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_METERING,
    };

    /* Read key attributes: summation, demand, formatting */
    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_METERING_CURRENT_SUMM_DELIVERED_ID,
        ZB_ZCL_ATTR_METERING_CURRENT_SUMM_RECEIVED_ID,
        ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID,
        ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID,
        ZB_ZCL_ATTR_METERING_MULTIPLIER_ID,
        ZB_ZCL_ATTR_METERING_DIVISOR_ID,
        ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID,
        ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send metering read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_metering_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_metering_state_t *state = get_or_create_metering_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_METERING_CURRENT_SUMM_DELIVERED_ID:
            if (value_len >= 6) {
                state->current_summation_delivered = zb_metering_parse_uint48(value, value_len);
                double energy = zb_metering_get_total_energy(state);
                ESP_LOGI(TAG, "Metering 0x%04X total delivered: %llu (%.3f %s)",
                         short_addr, (unsigned long long)state->current_summation_delivered,
                         energy, zb_metering_get_unit_string(state->unit_of_measure));
            }
            break;

        case ZB_ZCL_ATTR_METERING_CURRENT_SUMM_RECEIVED_ID:
            if (value_len >= 6) {
                state->current_summation_received = zb_metering_parse_uint48(value, value_len);
                ESP_LOGI(TAG, "Metering 0x%04X total received: %llu",
                         short_addr, (unsigned long long)state->current_summation_received);
            }
            break;

        case ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID:
            if (value_len >= 3) {
                state->instantaneous_demand = zb_metering_parse_int24(value, value_len);
                double power = zb_metering_get_power_w(state);
                ESP_LOGI(TAG, "Metering 0x%04X instantaneous demand: %ld (%.1f W)",
                         short_addr, (long)state->instantaneous_demand, power);
            }
            break;

        case ZB_ZCL_ATTR_METERING_CURRENT_DAY_CONSUMPTION_ID:
            if (value_len >= 3) {
                state->current_day_consumption = zb_metering_parse_uint24(value, value_len);
                double today = zb_metering_get_current_day_energy(state);
                ESP_LOGI(TAG, "Metering 0x%04X today consumption: %lu (%.3f %s)",
                         short_addr, (unsigned long)state->current_day_consumption,
                         today, zb_metering_get_unit_string(state->unit_of_measure));
            }
            break;

        case ZB_ZCL_ATTR_METERING_PREVIOUS_DAY_CONSUMPTION_ID:
            if (value_len >= 3) {
                state->previous_day_consumption = zb_metering_parse_uint24(value, value_len);
                double yesterday = zb_metering_get_previous_day_energy(state);
                ESP_LOGI(TAG, "Metering 0x%04X yesterday consumption: %lu (%.3f %s)",
                         short_addr, (unsigned long)state->previous_day_consumption,
                         yesterday, zb_metering_get_unit_string(state->unit_of_measure));
            }
            break;

        case ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID:
            if (value_len >= 1) {
                state->unit_of_measure = *(uint8_t *)value;
                ESP_LOGI(TAG, "Metering 0x%04X unit: %s (0x%02X)",
                         short_addr, zb_metering_get_unit_string(state->unit_of_measure),
                         state->unit_of_measure);
            }
            break;

        case ZB_ZCL_ATTR_METERING_MULTIPLIER_ID:
            if (value_len >= 3) {
                state->multiplier = zb_metering_parse_uint24(value, value_len);
                ESP_LOGD(TAG, "Metering 0x%04X multiplier: %lu",
                         short_addr, (unsigned long)state->multiplier);
            }
            break;

        case ZB_ZCL_ATTR_METERING_DIVISOR_ID:
            if (value_len >= 3) {
                state->divisor = zb_metering_parse_uint24(value, value_len);
                if (state->divisor == 0) {
                    state->divisor = 1;  /* Prevent division by zero */
                }
                ESP_LOGD(TAG, "Metering 0x%04X divisor: %lu",
                         short_addr, (unsigned long)state->divisor);
            }
            break;

        case ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID:
            if (value_len >= 1) {
                state->summation_formatting = *(uint8_t *)value;
                ESP_LOGD(TAG, "Metering 0x%04X summation formatting: 0x%02X (%d decimals)",
                         short_addr, state->summation_formatting,
                         zb_metering_get_decimal_places(state->summation_formatting));
            }
            break;

        case ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID:
            if (value_len >= 1) {
                state->demand_formatting = *(uint8_t *)value;
                ESP_LOGD(TAG, "Metering 0x%04X demand formatting: 0x%02X (%d decimals)",
                         short_addr, state->demand_formatting,
                         zb_metering_get_decimal_places(state->demand_formatting));
            }
            break;

        case ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID:
            if (value_len >= 1) {
                state->metering_device_type = *(uint8_t *)value;
                const char *type_str = "unknown";
                switch (state->metering_device_type) {
                    case ZB_METERING_DEVICE_ELECTRIC: type_str = "electric"; break;
                    case ZB_METERING_DEVICE_GAS: type_str = "gas"; break;
                    case ZB_METERING_DEVICE_WATER: type_str = "water"; break;
                    case ZB_METERING_DEVICE_THERMAL: type_str = "thermal"; break;
                    case ZB_METERING_DEVICE_PRESSURE: type_str = "pressure"; break;
                    case ZB_METERING_DEVICE_HEAT: type_str = "heat"; break;
                    case ZB_METERING_DEVICE_COOLING: type_str = "cooling"; break;
                }
                ESP_LOGI(TAG, "Metering 0x%04X device type: %s", short_addr, type_str);
            }
            break;

        case ZB_ZCL_ATTR_METERING_STATUS_ID:
            if (value_len >= 1) {
                state->status = *(uint8_t *)value;
                ESP_LOGD(TAG, "Metering 0x%04X status: 0x%02X", short_addr, state->status);
            }
            break;

        case ZB_ZCL_ATTR_METERING_REMAINING_BATTERY_LIFE_ID:
            if (value_len >= 1) {
                state->battery_percentage = *(uint8_t *)value;
                ESP_LOGI(TAG, "Metering 0x%04X battery: %d%%", short_addr, state->battery_percentage);
            }
            break;

        default:
            ESP_LOGD(TAG, "Metering 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_metering_callback != NULL) {
        s_metering_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_metering_get_state(uint16_t short_addr, zb_metering_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_metering_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_metering_states[idx], sizeof(zb_metering_state_t));
    return ESP_OK;
}
