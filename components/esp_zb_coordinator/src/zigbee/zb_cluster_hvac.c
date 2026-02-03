/**
 * @file zb_cluster_hvac.c
 * @brief Thermostat (0x0201) and Fan Control (0x0202) Cluster Implementations
 *
 * Provides control for HVAC devices including thermostats and fans.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cluster_hvac.h"
#include "zb_cluster_internal.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "ZB_HVAC";

/* ============================================================================
 * Thermostat State Storage
 * ============================================================================ */

static zb_thermostat_state_t *s_thermostat_states = NULL;
static uint16_t *s_thermostat_addrs = NULL;
static size_t s_thermostat_count = 0;
static zb_thermostat_state_cb_t s_thermostat_callback = NULL;

/* ============================================================================
 * Fan Control State Storage
 * ============================================================================ */

static zb_fan_control_state_t *s_fan_control_states = NULL;
static uint16_t *s_fan_control_addrs = NULL;
static size_t s_fan_control_count = 0;
static zb_fan_control_state_cb_t s_fan_control_callback = NULL;

/* ============================================================================
 * Thermostat Internal Helpers
 * ============================================================================ */

/**
 * @brief Find thermostat state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_thermostat_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_thermostat_count; i++) {
        if (s_thermostat_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create thermostat state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_thermostat_state_t* get_or_create_thermostat_state(uint16_t short_addr)
{
    int idx = find_thermostat_state_index(short_addr);
    if (idx >= 0) {
        return &s_thermostat_states[idx];
    }

    /* Create new entry */
    if (s_thermostat_count >= ZB_STATE_MAX_THERMOSTAT) {
        ESP_LOGW(TAG, "Thermostat state storage full");
        return NULL;
    }

    idx = s_thermostat_count++;
    s_thermostat_addrs[idx] = short_addr;
    memset(&s_thermostat_states[idx], 0, sizeof(zb_thermostat_state_t));

    /* Initialize with default values */
    s_thermostat_states[idx].local_temperature = ZB_THERMOSTAT_TEMP_INVALID;
    s_thermostat_states[idx].occupied_heating_setpoint = ZB_THERMOSTAT_DEFAULT_HEAT_SETPOINT;
    s_thermostat_states[idx].occupied_cooling_setpoint = ZB_THERMOSTAT_DEFAULT_COOL_SETPOINT;
    s_thermostat_states[idx].min_heat_setpoint_limit = ZB_THERMOSTAT_MIN_HEAT_SETPOINT;
    s_thermostat_states[idx].max_heat_setpoint_limit = ZB_THERMOSTAT_MAX_HEAT_SETPOINT;
    s_thermostat_states[idx].min_cool_setpoint_limit = ZB_THERMOSTAT_MIN_COOL_SETPOINT;
    s_thermostat_states[idx].max_cool_setpoint_limit = ZB_THERMOSTAT_MAX_COOL_SETPOINT;
    s_thermostat_states[idx].system_mode = ZB_THERMOSTAT_SYSTEM_MODE_OFF;

    ESP_LOGI(TAG, "Created thermostat state for device 0x%04X", short_addr);
    return &s_thermostat_states[idx];
}

/* ============================================================================
 * Fan Control Internal Helpers
 * ============================================================================ */

/**
 * @brief Find fan control state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_fan_control_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_fan_control_count; i++) {
        if (s_fan_control_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create fan control state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_fan_control_state_t* get_or_create_fan_control_state(uint16_t short_addr)
{
    int idx = find_fan_control_state_index(short_addr);
    if (idx >= 0) {
        return &s_fan_control_states[idx];
    }

    /* Create new entry */
    if (s_fan_control_count >= ZB_STATE_MAX_FAN_CONTROL) {
        ESP_LOGW(TAG, "Fan control state storage full");
        return NULL;
    }

    idx = s_fan_control_count++;
    s_fan_control_addrs[idx] = short_addr;
    memset(&s_fan_control_states[idx], 0, sizeof(zb_fan_control_state_t));
    s_fan_control_states[idx].fan_mode = ZB_FAN_MODE_AUTO;
    s_fan_control_states[idx].fan_mode_sequence = ZB_FAN_MODE_SEQ_LOW_MED_HIGH_AUTO;

    ESP_LOGI(TAG, "Created fan control state for device 0x%04X", short_addr);
    return &s_fan_control_states[idx];
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_cluster_hvac_init(void)
{
    /* Initialize Thermostat storage */
    s_thermostat_count = 0;
    s_thermostat_callback = NULL;

    /* Allocate Thermostat arrays in PSRAM */
    if (s_thermostat_states == NULL) {
        s_thermostat_states = heap_caps_calloc(ZB_STATE_MAX_THERMOSTAT,
                                                sizeof(zb_thermostat_state_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_thermostat_states) {
            s_thermostat_states = calloc(ZB_STATE_MAX_THERMOSTAT,
                                          sizeof(zb_thermostat_state_t));  /* Fallback */
        }
    }
    if (s_thermostat_addrs == NULL) {
        s_thermostat_addrs = heap_caps_calloc(ZB_STATE_MAX_THERMOSTAT,
                                               sizeof(uint16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_thermostat_addrs) {
            s_thermostat_addrs = calloc(ZB_STATE_MAX_THERMOSTAT,
                                         sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_thermostat_states || !s_thermostat_addrs) {
        ESP_LOGE(TAG, "Failed to allocate thermostat state arrays");
        free(s_thermostat_states);
        s_thermostat_states = NULL;
        free(s_thermostat_addrs);
        s_thermostat_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize Fan Control storage */
    s_fan_control_count = 0;
    s_fan_control_callback = NULL;

    /* Allocate Fan Control arrays in PSRAM */
    if (s_fan_control_states == NULL) {
        s_fan_control_states = heap_caps_calloc(ZB_STATE_MAX_FAN_CONTROL,
                                                 sizeof(zb_fan_control_state_t),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fan_control_states) {
            s_fan_control_states = calloc(ZB_STATE_MAX_FAN_CONTROL,
                                           sizeof(zb_fan_control_state_t));  /* Fallback */
        }
    }
    if (s_fan_control_addrs == NULL) {
        s_fan_control_addrs = heap_caps_calloc(ZB_STATE_MAX_FAN_CONTROL,
                                                sizeof(uint16_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fan_control_addrs) {
            s_fan_control_addrs = calloc(ZB_STATE_MAX_FAN_CONTROL,
                                          sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_fan_control_states || !s_fan_control_addrs) {
        ESP_LOGE(TAG, "Failed to allocate fan control state arrays");
        free(s_thermostat_states);
        s_thermostat_states = NULL;
        free(s_thermostat_addrs);
        s_thermostat_addrs = NULL;
        free(s_fan_control_states);
        s_fan_control_states = NULL;
        free(s_fan_control_addrs);
        s_fan_control_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HVAC clusters initialized (Thermostat: %d, Fan Control: %d max)",
             ZB_STATE_MAX_THERMOSTAT, ZB_STATE_MAX_FAN_CONTROL);
    return ESP_OK;
}

esp_err_t zb_cluster_hvac_deinit(void)
{
    s_thermostat_count = 0;
    s_thermostat_callback = NULL;
    s_fan_control_count = 0;
    s_fan_control_callback = NULL;
    ESP_LOGI(TAG, "HVAC clusters deinitialized");
    return ESP_OK;
}

void zb_cluster_hvac_clear_all(void)
{
    s_thermostat_count = 0;
    s_fan_control_count = 0;
    if (s_thermostat_states) {
        memset(s_thermostat_states, 0,
               ZB_STATE_MAX_THERMOSTAT * sizeof(zb_thermostat_state_t));
    }
    if (s_thermostat_addrs) {
        memset(s_thermostat_addrs, 0,
               ZB_STATE_MAX_THERMOSTAT * sizeof(uint16_t));
    }
    if (s_fan_control_states) {
        memset(s_fan_control_states, 0,
               ZB_STATE_MAX_FAN_CONTROL * sizeof(zb_fan_control_state_t));
    }
    if (s_fan_control_addrs) {
        memset(s_fan_control_addrs, 0,
               ZB_STATE_MAX_FAN_CONTROL * sizeof(uint16_t));
    }
    ESP_LOGD(TAG, "Cleared all HVAC cluster states");
}

void zb_cluster_hvac_remove_device(uint16_t short_addr)
{
    /* Remove from Thermostat storage */
    for (size_t i = 0; i < s_thermostat_count; i++) {
        if (s_thermostat_addrs[i] == short_addr) {
            if (i < s_thermostat_count - 1) {
                s_thermostat_states[i] = s_thermostat_states[s_thermostat_count - 1];
                s_thermostat_addrs[i] = s_thermostat_addrs[s_thermostat_count - 1];
            }
            s_thermostat_count--;
            ESP_LOGD(TAG, "Removed thermostat state for device 0x%04X", short_addr);
            break;
        }
    }

    /* Remove from Fan Control storage */
    for (size_t i = 0; i < s_fan_control_count; i++) {
        if (s_fan_control_addrs[i] == short_addr) {
            if (i < s_fan_control_count - 1) {
                s_fan_control_states[i] = s_fan_control_states[s_fan_control_count - 1];
                s_fan_control_addrs[i] = s_fan_control_addrs[s_fan_control_count - 1];
            }
            s_fan_control_count--;
            ESP_LOGD(TAG, "Removed fan control state for device 0x%04X", short_addr);
            break;
        }
    }
}

/* ============================================================================
 * Thermostat Cluster Public API
 * ============================================================================ */

esp_err_t zb_thermostat_register_callback(zb_thermostat_state_cb_t callback)
{
    s_thermostat_callback = callback;
    ESP_LOGI(TAG, "Thermostat state callback registered");
    return ESP_OK;
}

bool zb_device_has_thermostat(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_THERMOSTAT);
}

esp_err_t zb_thermostat_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading thermostat state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_THERMOSTAT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
        ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
        ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
        ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
        ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send thermostat read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_thermostat_set_heating_setpoint(uint16_t short_addr, uint8_t endpoint,
                                              int16_t temperature)
{
    ESP_LOGI(TAG, "Setting heating setpoint to %.2fC for 0x%04X EP%d",
             temperature / (float)ZCL_TEMP_SCALE, short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_S16,
            .size = sizeof(int16_t),
            .value = (void *)&temperature,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set heating setpoint: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_thermostat_set_cooling_setpoint(uint16_t short_addr, uint8_t endpoint,
                                              int16_t temperature)
{
    ESP_LOGI(TAG, "Setting cooling setpoint to %.2fC for 0x%04X EP%d",
             temperature / (float)ZCL_TEMP_SCALE, short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_S16,
            .size = sizeof(int16_t),
            .value = (void *)&temperature,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set cooling setpoint: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_thermostat_set_system_mode(uint16_t short_addr, uint8_t endpoint,
                                         zb_thermostat_system_mode_t mode)
{
    const char *mode_str = "unknown";
    switch (mode) {
        case ZB_THERMOSTAT_SYSTEM_MODE_OFF: mode_str = "off"; break;
        case ZB_THERMOSTAT_SYSTEM_MODE_AUTO: mode_str = "auto"; break;
        case ZB_THERMOSTAT_SYSTEM_MODE_COOL: mode_str = "cool"; break;
        case ZB_THERMOSTAT_SYSTEM_MODE_HEAT: mode_str = "heat"; break;
        case ZB_THERMOSTAT_SYSTEM_MODE_FAN_ONLY: mode_str = "fan_only"; break;
        case ZB_THERMOSTAT_SYSTEM_MODE_DRY: mode_str = "dry"; break;
        default: break;
    }

    ESP_LOGI(TAG, "Setting thermostat system mode to %s for 0x%04X EP%d",
             mode_str, short_addr, endpoint);

    uint8_t mode_value = (uint8_t)mode;

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
            .size = sizeof(uint8_t),
            .value = (void *)&mode_value,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set system mode: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_thermostat_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_thermostat_state_t *state = get_or_create_thermostat_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID:
            if (value_len >= 2) {
                state->local_temperature = *(int16_t *)value;
                ESP_LOGI(TAG, "Thermostat 0x%04X local temp: %.2fC",
                         short_addr, state->local_temperature / (float)ZCL_TEMP_SCALE);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID:
            if (value_len >= 2) {
                state->occupied_cooling_setpoint = *(int16_t *)value;
                ESP_LOGI(TAG, "Thermostat 0x%04X cooling setpoint: %.2fC",
                         short_addr, state->occupied_cooling_setpoint / (float)ZCL_TEMP_SCALE);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID:
            if (value_len >= 2) {
                state->occupied_heating_setpoint = *(int16_t *)value;
                ESP_LOGI(TAG, "Thermostat 0x%04X heating setpoint: %.2fC",
                         short_addr, state->occupied_heating_setpoint / (float)ZCL_TEMP_SCALE);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID:
            if (value_len >= 2) {
                state->min_heat_setpoint_limit = *(int16_t *)value;
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID:
            if (value_len >= 2) {
                state->max_heat_setpoint_limit = *(int16_t *)value;
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID:
            if (value_len >= 2) {
                state->min_cool_setpoint_limit = *(int16_t *)value;
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID:
            if (value_len >= 2) {
                state->max_cool_setpoint_limit = *(int16_t *)value;
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID:
            if (value_len >= 1) {
                state->system_mode = (zb_thermostat_system_mode_t)(*(uint8_t *)value);
                const char *mode_str = "unknown";
                switch (state->system_mode) {
                    case ZB_THERMOSTAT_SYSTEM_MODE_OFF: mode_str = "off"; break;
                    case ZB_THERMOSTAT_SYSTEM_MODE_AUTO: mode_str = "auto"; break;
                    case ZB_THERMOSTAT_SYSTEM_MODE_COOL: mode_str = "cool"; break;
                    case ZB_THERMOSTAT_SYSTEM_MODE_HEAT: mode_str = "heat"; break;
                    case ZB_THERMOSTAT_SYSTEM_MODE_FAN_ONLY: mode_str = "fan_only"; break;
                    case ZB_THERMOSTAT_SYSTEM_MODE_DRY: mode_str = "dry"; break;
                    default: break;
                }
                ESP_LOGI(TAG, "Thermostat 0x%04X system mode: %s", short_addr, mode_str);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID:
            if (value_len >= 1) {
                state->running_mode = (zb_thermostat_running_mode_t)(*(uint8_t *)value);
                ESP_LOGD(TAG, "Thermostat 0x%04X running mode: %d",
                         short_addr, state->running_mode);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID:
            if (value_len >= 1) {
                state->pi_heating_demand = *(uint8_t *)value;
                ESP_LOGD(TAG, "Thermostat 0x%04X heating demand: %d%%",
                         short_addr, state->pi_heating_demand);
            }
            break;

        case ZB_ZCL_ATTR_THERMOSTAT_PI_COOLING_DEMAND_ID:
            if (value_len >= 1) {
                state->pi_cooling_demand = *(uint8_t *)value;
                ESP_LOGD(TAG, "Thermostat 0x%04X cooling demand: %d%%",
                         short_addr, state->pi_cooling_demand);
            }
            break;

        default:
            ESP_LOGD(TAG, "Thermostat 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_thermostat_callback != NULL) {
        s_thermostat_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_thermostat_get_state(uint16_t short_addr, zb_thermostat_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_thermostat_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_thermostat_states[idx], sizeof(zb_thermostat_state_t));
    return ESP_OK;
}

/* ============================================================================
 * Fan Control Cluster Public API
 * ============================================================================ */

esp_err_t zb_fan_control_register_callback(zb_fan_control_state_cb_t callback)
{
    s_fan_control_callback = callback;
    ESP_LOGI(TAG, "Fan control state callback registered");
    return ESP_OK;
}

bool zb_device_has_fan_control(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_FAN_CONTROL);
}

esp_err_t zb_fan_control_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading fan control state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
        ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send fan control read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_fan_control_set_mode(uint16_t short_addr, uint8_t endpoint,
                                   zb_fan_mode_t mode)
{
    const char *mode_str = "unknown";
    switch (mode) {
        case ZB_FAN_MODE_OFF: mode_str = "off"; break;
        case ZB_FAN_MODE_LOW: mode_str = "low"; break;
        case ZB_FAN_MODE_MEDIUM: mode_str = "medium"; break;
        case ZB_FAN_MODE_HIGH: mode_str = "high"; break;
        case ZB_FAN_MODE_ON: mode_str = "on"; break;
        case ZB_FAN_MODE_AUTO: mode_str = "auto"; break;
        case ZB_FAN_MODE_SMART: mode_str = "smart"; break;
    }

    ESP_LOGI(TAG, "Setting fan mode to %s for 0x%04X EP%d", mode_str, short_addr, endpoint);

    uint8_t mode_value = (uint8_t)mode;

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
            .size = sizeof(uint8_t),
            .value = (void *)&mode_value,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set fan mode: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_fan_control_handle_report(uint16_t short_addr, uint8_t endpoint,
                                        uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_fan_control_state_t *state = get_or_create_fan_control_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID:
            if (value_len >= 1) {
                state->fan_mode = (zb_fan_mode_t)(*(uint8_t *)value);
                const char *mode_str = "unknown";
                switch (state->fan_mode) {
                    case ZB_FAN_MODE_OFF: mode_str = "off"; break;
                    case ZB_FAN_MODE_LOW: mode_str = "low"; break;
                    case ZB_FAN_MODE_MEDIUM: mode_str = "medium"; break;
                    case ZB_FAN_MODE_HIGH: mode_str = "high"; break;
                    case ZB_FAN_MODE_ON: mode_str = "on"; break;
                    case ZB_FAN_MODE_AUTO: mode_str = "auto"; break;
                    case ZB_FAN_MODE_SMART: mode_str = "smart"; break;
                }
                ESP_LOGI(TAG, "Fan control 0x%04X mode: %s", short_addr, mode_str);
            }
            break;

        case ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID:
            if (value_len >= 1) {
                state->fan_mode_sequence = (zb_fan_mode_sequence_t)(*(uint8_t *)value);
                ESP_LOGD(TAG, "Fan control 0x%04X mode sequence: %d",
                         short_addr, state->fan_mode_sequence);
            }
            break;

        default:
            ESP_LOGD(TAG, "Fan control 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_fan_control_callback != NULL) {
        s_fan_control_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_fan_control_get_state(uint16_t short_addr, zb_fan_control_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_fan_control_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_fan_control_states[idx], sizeof(zb_fan_control_state_t));
    return ESP_OK;
}
