/**
 * @file zb_hvac_dehumid.c
 * @brief Zigbee Dehumidification Control Cluster (0x0203) Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_hvac_dehumid.h"
#include "zb_constants.h"
#include "zb_device_handler.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include <string.h>

static const char *TAG = "ZB_DEHUMID";

/* ============================================================================
 * Module State
 * ============================================================================ */

static bool s_dehumid_initialized = false;
static zb_dehumid_state_t s_dehumid_states[ZB_STATE_MAX_DEHUMID];
static uint16_t s_dehumid_addrs[ZB_STATE_MAX_DEHUMID];
static size_t s_dehumid_count = 0;
static zb_dehumid_state_cb_t s_dehumid_callback = NULL;

/* ============================================================================
 * Internal Helpers
 * ============================================================================ */

/**
 * @brief Find dehumidification state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_dehumid_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_dehumid_count; i++) {
        if (s_dehumid_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create dehumidification state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_dehumid_state_t* get_or_create_dehumid_state(uint16_t short_addr)
{
    int idx = find_dehumid_state_index(short_addr);
    if (idx >= 0) {
        return &s_dehumid_states[idx];
    }

    /* Create new entry */
    if (s_dehumid_count >= ZB_STATE_MAX_DEHUMID) {
        ESP_LOGW(TAG, "Dehumidification state storage full");
        return NULL;
    }

    idx = s_dehumid_count++;
    s_dehumid_addrs[idx] = short_addr;
    memset(&s_dehumid_states[idx], 0, sizeof(zb_dehumid_state_t));

    /* Initialize with default values */
    s_dehumid_states[idx].relative_humidity = 0xFF;                    /* Unknown */
    s_dehumid_states[idx].rh_setpoint = 50;                            /* 50% default */
    s_dehumid_states[idx].dehumid_lockout = ZB_DEHUMID_LOCKOUT_ALLOWED;
    s_dehumid_states[idx].dehumid_hysteresis = 5;                      /* 5% hysteresis */
    s_dehumid_states[idx].dehumid_max_cool = ZB_DEHUMID_PERCENT_MAX;   /* 100% max cool */
    s_dehumid_states[idx].relative_humidity_display = ZB_DEHUMID_DISPLAY_DISPLAYED;

    ESP_LOGI(TAG, "Created dehumidification state for device 0x%04X", short_addr);
    return &s_dehumid_states[idx];
}

/**
 * @brief Get mode string for relative humidity mode
 *
 * @param[in] mode Mode value
 * @return Mode string
 */
static const char* rh_mode_to_str(uint8_t mode)
{
    switch (mode) {
        case ZB_DEHUMID_RH_MODE_MEASURED_LOCALLY:
            return "measured_locally";
        case ZB_DEHUMID_RH_MODE_UPDATED_OVER_NETWORK:
            return "updated_over_network";
        default:
            return "unknown";
    }
}

/**
 * @brief Get lockout string for dehumidification lockout
 *
 * @param[in] lockout Lockout value
 * @return Lockout string
 */
static const char* lockout_to_str(uint8_t lockout)
{
    switch (lockout) {
        case ZB_DEHUMID_LOCKOUT_NOT_ALLOWED:
            return "not_allowed";
        case ZB_DEHUMID_LOCKOUT_ALLOWED:
            return "allowed";
        default:
            return "unknown";
    }
}

/* ============================================================================
 * Initialization and Deinitialization
 * ============================================================================ */

esp_err_t zb_dehumid_init(void)
{
    if (s_dehumid_initialized) {
        ESP_LOGW(TAG, "Dehumidification control already initialized");
        return ESP_OK;
    }

    /* Clear state storage */
    memset(s_dehumid_states, 0, sizeof(s_dehumid_states));
    memset(s_dehumid_addrs, 0, sizeof(s_dehumid_addrs));
    s_dehumid_count = 0;
    s_dehumid_callback = NULL;

    s_dehumid_initialized = true;
    ESP_LOGI(TAG, "Dehumidification control initialized (max devices: %d)",
             ZB_STATE_MAX_DEHUMID);

    return ESP_OK;
}

esp_err_t zb_dehumid_deinit(void)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clear state storage */
    memset(s_dehumid_states, 0, sizeof(s_dehumid_states));
    memset(s_dehumid_addrs, 0, sizeof(s_dehumid_addrs));
    s_dehumid_count = 0;
    s_dehumid_callback = NULL;

    s_dehumid_initialized = false;
    ESP_LOGI(TAG, "Dehumidification control deinitialized");

    return ESP_OK;
}

/* ============================================================================
 * State Management
 * ============================================================================ */

esp_err_t zb_dehumid_register_callback(zb_dehumid_state_cb_t callback)
{
    s_dehumid_callback = callback;
    ESP_LOGI(TAG, "Dehumidification state callback registered");
    return ESP_OK;
}

bool zb_device_has_dehumid_control(uint16_t short_addr)
{
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        return false;
    }

    for (uint16_t i = 0; i < device->cluster_count; i++) {
        if (device->clusters[i] == ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL) {
            return true;
        }
    }
    return false;
}

esp_err_t zb_dehumid_handle_report(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t attr_id, void *value, size_t value_len)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_dehumid_state_t *state = get_or_create_dehumid_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_ID:
            if (value_len >= 1) {
                state->relative_humidity = *(uint8_t *)value;
                ESP_LOGI(TAG, "Dehumid 0x%04X relative humidity: %d%%",
                         short_addr, state->relative_humidity);
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_DEHUMIDIFICATION_COOLING_ID:
            if (value_len >= 1) {
                state->dehumid_cooling = *(uint8_t *)value;
                ESP_LOGI(TAG, "Dehumid 0x%04X cooling effect: %d%%",
                         short_addr, state->dehumid_cooling);
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_RH_DEHUMID_SETPOINT_ID:
            if (value_len >= 1) {
                state->rh_setpoint = *(uint8_t *)value;
                ESP_LOGI(TAG, "Dehumid 0x%04X RH setpoint: %d%%",
                         short_addr, state->rh_setpoint);
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_MODE_ID:
            if (value_len >= 1) {
                state->relative_humidity_mode = *(uint8_t *)value;
                ESP_LOGI(TAG, "Dehumid 0x%04X RH mode: %s",
                         short_addr, rh_mode_to_str(state->relative_humidity_mode));
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_DEHUMID_LOCKOUT_ID:
            if (value_len >= 1) {
                state->dehumid_lockout = *(uint8_t *)value;
                ESP_LOGI(TAG, "Dehumid 0x%04X lockout: %s",
                         short_addr, lockout_to_str(state->dehumid_lockout));
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_DEHUMID_HYSTERESIS_ID:
            if (value_len >= 1) {
                state->dehumid_hysteresis = *(uint8_t *)value;
                ESP_LOGD(TAG, "Dehumid 0x%04X hysteresis: %d",
                         short_addr, state->dehumid_hysteresis);
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_DEHUMID_MAX_COOL_ID:
            if (value_len >= 1) {
                state->dehumid_max_cool = *(uint8_t *)value;
                ESP_LOGD(TAG, "Dehumid 0x%04X max cool: %d%%",
                         short_addr, state->dehumid_max_cool);
            }
            break;

        case ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_DISPLAY_ID:
            if (value_len >= 1) {
                state->relative_humidity_display = *(uint8_t *)value;
                ESP_LOGD(TAG, "Dehumid 0x%04X RH display: %d",
                         short_addr, state->relative_humidity_display);
            }
            break;

        default:
            ESP_LOGD(TAG, "Dehumid 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_dehumid_callback != NULL) {
        s_dehumid_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_dehumid_get_state(uint16_t short_addr, zb_dehumid_state_t *state)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_dehumid_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_dehumid_states[idx], sizeof(zb_dehumid_state_t));
    return ESP_OK;
}

/* ============================================================================
 * Control Commands
 * ============================================================================ */

esp_err_t zb_dehumid_read_state(uint16_t short_addr, uint8_t endpoint)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading dehumidification state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_ID,
        ZB_ZCL_ATTR_DEHUMID_DEHUMIDIFICATION_COOLING_ID,
        ZB_ZCL_ATTR_DEHUMID_RH_DEHUMID_SETPOINT_ID,
        ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_MODE_ID,
        ZB_ZCL_ATTR_DEHUMID_DEHUMID_LOCKOUT_ID,
        ZB_ZCL_ATTR_DEHUMID_DEHUMID_HYSTERESIS_ID,
        ZB_ZCL_ATTR_DEHUMID_DEHUMID_MAX_COOL_ID,
        ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_DISPLAY_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send dehumidification read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_dehumid_set_setpoint(uint16_t short_addr, uint8_t endpoint,
                                   uint8_t setpoint)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp setpoint to valid range */
    if (setpoint > ZB_DEHUMID_PERCENT_MAX) {
        ESP_LOGW(TAG, "Setpoint %d clamped to %d%%", setpoint, ZB_DEHUMID_PERCENT_MAX);
        setpoint = ZB_DEHUMID_PERCENT_MAX;
    }

    ESP_LOGI(TAG, "Setting dehumidification setpoint to %d%% for 0x%04X EP%d",
             setpoint, short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_DEHUMID_RH_DEHUMID_SETPOINT_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .size = sizeof(uint8_t),
            .value = (void *)&setpoint,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set dehumidification setpoint: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_dehumid_set_hysteresis(uint16_t short_addr, uint8_t endpoint,
                                     uint8_t hysteresis)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting dehumidification hysteresis to %d for 0x%04X EP%d",
             hysteresis, short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_DEHUMID_DEHUMID_HYSTERESIS_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .size = sizeof(uint8_t),
            .value = (void *)&hysteresis,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set dehumidification hysteresis: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_dehumid_set_max_cool(uint16_t short_addr, uint8_t endpoint,
                                   uint8_t max_cool)
{
    if (!s_dehumid_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp max_cool to valid range */
    if (max_cool > ZB_DEHUMID_PERCENT_MAX) {
        ESP_LOGW(TAG, "Max cool %d clamped to %d%%", max_cool, ZB_DEHUMID_PERCENT_MAX);
        max_cool = ZB_DEHUMID_PERCENT_MAX;
    }

    ESP_LOGI(TAG, "Setting dehumidification max cool to %d%% for 0x%04X EP%d",
             max_cool, short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_DEHUMID_DEHUMID_MAX_COOL_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .size = sizeof(uint8_t),
            .value = (void *)&max_cool,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set dehumidification max cool: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* ============================================================================
 * Testing
 * ============================================================================ */

esp_err_t zb_dehumid_test(void)
{
    ESP_LOGI(TAG, "Running dehumidification control self-test...");

    /* Save current state */
    bool was_initialized = s_dehumid_initialized;
    size_t saved_count = s_dehumid_count;

    /* Ensure module is initialized for testing */
    if (!s_dehumid_initialized) {
        esp_err_t ret = zb_dehumid_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize for test");
            return ESP_FAIL;
        }
    }

    /* Test state creation */
    uint16_t test_addr = 0xFFFF;
    zb_dehumid_state_t *state = get_or_create_dehumid_state(test_addr);
    if (state == NULL) {
        ESP_LOGE(TAG, "Failed to create test state");
        return ESP_FAIL;
    }

    /* Test state retrieval */
    zb_dehumid_state_t retrieved_state;
    esp_err_t ret = zb_dehumid_get_state(test_addr, &retrieved_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to retrieve test state");
        return ESP_FAIL;
    }

    /* Verify default values */
    if (retrieved_state.rh_setpoint != 50 ||
        retrieved_state.dehumid_lockout != ZB_DEHUMID_LOCKOUT_ALLOWED) {
        ESP_LOGE(TAG, "Default values incorrect");
        return ESP_FAIL;
    }

    /* Test attribute report handling */
    uint8_t test_humidity = ZB_DEHUMID_TEST_HUMIDITY_PERCENT;
    ret = zb_dehumid_handle_report(test_addr, 1,
                                    ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_ID,
                                    &test_humidity, sizeof(test_humidity));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to handle test report");
        return ESP_FAIL;
    }

    /* Verify update */
    ret = zb_dehumid_get_state(test_addr, &retrieved_state);
    if (ret != ESP_OK || retrieved_state.relative_humidity != ZB_DEHUMID_TEST_HUMIDITY_PERCENT) {
        ESP_LOGE(TAG, "State update verification failed");
        return ESP_FAIL;
    }

    /* Clean up test entry by restoring original count */
    s_dehumid_count = saved_count;

    /* Restore initialization state if we initialized for testing */
    if (!was_initialized) {
        zb_dehumid_deinit();
    }

    ESP_LOGI(TAG, "Dehumidification control self-test PASSED");
    return ESP_OK;
}
