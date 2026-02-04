/**
 * @file zb_cluster_closures.c
 * @brief Window Covering Cluster (0x0102) Implementation
 *
 * Provides control for blinds, shades, curtains, and other window
 * covering devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cluster_closures.h"
#include "zb_cluster_internal.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "ZB_CLOSURES";

/* ============================================================================
 * Window Covering State Storage
 * ============================================================================ */

static zb_window_covering_state_t *s_window_covering_states = NULL;
static uint16_t *s_window_covering_addrs = NULL;
static size_t s_window_covering_count = 0;
static zb_window_covering_state_cb_t s_window_covering_callback = NULL;

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Find window covering state by device address
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state or NULL if not found
 */
static zb_window_covering_state_t* find_window_covering_state(uint16_t short_addr)
{
    for (size_t i = 0; i < s_window_covering_count; i++) {
        if (s_window_covering_addrs[i] == short_addr) {
            return &s_window_covering_states[i];
        }
    }
    return NULL;
}

/**
 * @brief Add or get window covering state for a device
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state or NULL if full
 */
static zb_window_covering_state_t* get_or_create_window_covering_state(uint16_t short_addr)
{
    /* Check if already exists */
    zb_window_covering_state_t *state = find_window_covering_state(short_addr);
    if (state != NULL) {
        return state;
    }

    /* Create new entry */
    if (s_window_covering_count >= ZB_STATE_MAX_WINDOW_COVERING) {
        ESP_LOGW(TAG, "Window covering state storage full");
        return NULL;
    }

    s_window_covering_addrs[s_window_covering_count] = short_addr;
    state = &s_window_covering_states[s_window_covering_count];
    memset(state, 0, sizeof(zb_window_covering_state_t));
    state->current_position_lift = 0xFF; /* Unknown */
    state->current_position_tilt = 0xFF; /* Unknown */
    s_window_covering_count++;

    ESP_LOGI(TAG, "Created window covering state for device 0x%04X", short_addr);
    return state;
}

/**
 * @brief Generic helper for window covering simple commands (up/down/stop)
 *
 * Consolidates three nearly identical functions into one generic implementation.
 *
 * @param short_addr Device short address
 * @param endpoint Device endpoint
 * @param cmd_id Command ID (UP_OPEN, DOWN_CLOSE, or STOP)
 * @param is_moving Target moving state (true for up/down, false for stop)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t zb_window_covering_cmd(uint16_t short_addr, uint8_t endpoint,
                                        uint8_t cmd_id, bool is_moving)
{
    if (!zb_device_handler_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = cmd_id,
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
        ESP_LOGE(TAG, "Failed to send window covering command 0x%02X to 0x%04X EP%d: %s",
                 cmd_id, short_addr, endpoint, esp_err_to_name(ret));
    } else {
        /* Update local state */
        zb_window_covering_state_t *state =
            is_moving ? get_or_create_window_covering_state(short_addr)
                      : find_window_covering_state(short_addr);
        if (state) {
            state->is_moving = is_moving;
        }
    }

    return ret;
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_cluster_closures_init(void)
{
    s_window_covering_count = 0;
    s_window_covering_callback = NULL;

    /* Allocate in PSRAM */
    if (s_window_covering_states == NULL) {
        s_window_covering_states = heap_caps_calloc(ZB_STATE_MAX_WINDOW_COVERING,
                                                     sizeof(zb_window_covering_state_t),
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_window_covering_states) {
            s_window_covering_states = calloc(ZB_STATE_MAX_WINDOW_COVERING,
                                               sizeof(zb_window_covering_state_t));  /* Fallback */
        }
    }
    if (s_window_covering_addrs == NULL) {
        s_window_covering_addrs = heap_caps_calloc(ZB_STATE_MAX_WINDOW_COVERING,
                                                    sizeof(uint16_t),
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_window_covering_addrs) {
            s_window_covering_addrs = calloc(ZB_STATE_MAX_WINDOW_COVERING,
                                              sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_window_covering_states || !s_window_covering_addrs) {
        ESP_LOGE(TAG, "Failed to allocate window covering state arrays");
        free(s_window_covering_states);
        s_window_covering_states = NULL;
        free(s_window_covering_addrs);
        s_window_covering_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Window Covering cluster initialized (max: %d devices)",
             ZB_STATE_MAX_WINDOW_COVERING);
    return ESP_OK;
}

esp_err_t zb_cluster_closures_deinit(void)
{
    s_window_covering_count = 0;
    s_window_covering_callback = NULL;
    ESP_LOGI(TAG, "Window Covering cluster deinitialized");
    return ESP_OK;
}

void zb_cluster_closures_clear_all(void)
{
    s_window_covering_count = 0;
    if (s_window_covering_states) {
        memset(s_window_covering_states, 0,
               ZB_STATE_MAX_WINDOW_COVERING * sizeof(zb_window_covering_state_t));
    }
    if (s_window_covering_addrs) {
        memset(s_window_covering_addrs, 0,
               ZB_STATE_MAX_WINDOW_COVERING * sizeof(uint16_t));
    }
    ESP_LOGD(TAG, "Cleared all window covering states");
}

void zb_cluster_closures_remove_device(uint16_t short_addr)
{
    for (size_t i = 0; i < s_window_covering_count; i++) {
        if (s_window_covering_addrs[i] == short_addr) {
            /* Move last entry to this slot */
            if (i < s_window_covering_count - 1) {
                s_window_covering_states[i] = s_window_covering_states[s_window_covering_count - 1];
                s_window_covering_addrs[i] = s_window_covering_addrs[s_window_covering_count - 1];
            }
            s_window_covering_count--;
            ESP_LOGD(TAG, "Removed window covering state for device 0x%04X", short_addr);
            return;
        }
    }
}

/* ============================================================================
 * Window Covering Cluster Public API
 * ============================================================================ */

esp_err_t zb_window_covering_register_callback(zb_window_covering_state_cb_t callback)
{
    s_window_covering_callback = callback;
    ESP_LOGI(TAG, "Window covering callback registered");
    return ESP_OK;
}

bool zb_device_has_window_covering(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
}

esp_err_t zb_window_covering_read_position(uint16_t short_addr, uint8_t endpoint)
{
    if (!zb_device_handler_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading window covering position from 0x%04X EP%d", short_addr, endpoint);

    /* Create read attributes request */
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
    };

    /* Read CurrentPositionLiftPercentage attribute */
    uint16_t attr_list[] = {
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_LIFT_PERCENT_ID,
        ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_TILT_PERCENT_ID
    };
    read_cmd.attr_number = sizeof(attr_list) / sizeof(uint16_t);
    read_cmd.attr_field = attr_list;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&read_cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read position command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_window_covering_cmd_up(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Up/Open command to 0x%04X EP%d", short_addr, endpoint);
    return zb_window_covering_cmd(short_addr, endpoint, ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN,
                                   true);
}

esp_err_t zb_window_covering_cmd_down(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Down/Close command to 0x%04X EP%d", short_addr, endpoint);
    return zb_window_covering_cmd(short_addr, endpoint, ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE,
                                   true);
}

esp_err_t zb_window_covering_cmd_stop(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending Stop command to 0x%04X EP%d", short_addr, endpoint);
    return zb_window_covering_cmd(short_addr, endpoint, ZB_ZCL_CMD_WINDOW_COVERING_STOP,
                                   false);
}

esp_err_t zb_window_covering_cmd_goto_lift_percent(uint16_t short_addr, uint8_t endpoint,
                                                    uint8_t percentage)
{
    if (!zb_device_handler_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (percentage > ZB_WINDOW_COVERING_PERCENT_MAX) {
        ESP_LOGW(TAG, "Clamping percentage %d to %d", percentage, ZB_WINDOW_COVERING_PERCENT_MAX);
        percentage = ZB_WINDOW_COVERING_PERCENT_MAX;
    }

    ESP_LOGI(TAG, "Sending GoToLiftPercentage(%d%%) to 0x%04X EP%d",
             percentage, short_addr, endpoint);

    /* Prepare payload: single byte percentage value */
    uint8_t payload = percentage;

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENT,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_U8,
            .size = 1,
            .value = &payload,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GoToLiftPercentage command: %s", esp_err_to_name(ret));
    } else {
        /* Update local state */
        zb_window_covering_state_t *state = get_or_create_window_covering_state(short_addr);
        if (state) {
            state->is_moving = true;
        }
    }

    return ret;
}

esp_err_t zb_window_covering_handle_report(uint16_t short_addr, uint8_t endpoint,
                                            uint16_t attr_id, void *value, size_t value_len)
{
    if (!zb_device_handler_is_initialized() || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Window covering report from 0x%04X EP%d: attr=0x%04X len=%d",
             short_addr, endpoint, attr_id, (int)value_len);

    /* Get or create state for this device */
    zb_window_covering_state_t *state = get_or_create_window_covering_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Process attribute */
    switch (attr_id) {
        case ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_LIFT_PERCENT_ID:
            if (value_len >= 1) {
                uint8_t position = *(uint8_t *)value;
                ESP_LOGI(TAG, "Device 0x%04X lift position: %d%%", short_addr, position);
                state->current_position_lift = position;
                state->is_moving = false; /* Position report typically means movement completed */
            }
            break;

        case ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_TILT_PERCENT_ID:
            if (value_len >= 1) {
                uint8_t position = *(uint8_t *)value;
                ESP_LOGI(TAG, "Device 0x%04X tilt position: %d%%", short_addr, position);
                state->current_position_tilt = position;
            }
            break;

        case ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ID:
            if (value_len >= 1) {
                state->covering_type = *(uint8_t *)value;
                ESP_LOGI(TAG, "Device 0x%04X covering type: %d", short_addr, state->covering_type);
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled window covering attribute 0x%04X", attr_id);
            break;
    }

    /* Invoke callback if registered */
    if (s_window_covering_callback != NULL) {
        s_window_covering_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_window_covering_get_state(uint16_t short_addr, zb_window_covering_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_window_covering_state_t *stored = find_window_covering_state(short_addr);
    if (stored == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, stored, sizeof(zb_window_covering_state_t));
    return ESP_OK;
}
