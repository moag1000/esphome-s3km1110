/**
 * @file zb_cluster_security.c
 * @brief IAS Zone (0x0500) and Door Lock (0x0101) Cluster Implementations
 *
 * Provides support for security sensors and door lock devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cluster_security.h"
#include "zb_cluster_internal.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "ZB_SECURITY";

/* ============================================================================
 * Door Lock State Storage
 * ============================================================================ */

static zb_door_lock_state_struct_t *s_door_lock_states = NULL;
static uint16_t *s_door_lock_addrs = NULL;
static size_t s_door_lock_count = 0;
static zb_door_lock_state_cb_t s_door_lock_callback = NULL;

/* ============================================================================
 * IAS Zone State Storage
 * ============================================================================ */

static zb_ias_zone_state_struct_t *s_ias_zone_states = NULL;
static uint16_t *s_ias_zone_addrs = NULL;
static size_t s_ias_zone_count = 0;
static zb_ias_zone_state_cb_t s_ias_zone_callback = NULL;
static bool s_ias_zone_auto_enroll = true;  /* Auto-enroll by default */
static uint8_t s_ias_zone_next_id = 0;      /* Next zone ID to assign */

/* ============================================================================
 * Door Lock Internal Helpers
 * ============================================================================ */

/**
 * @brief Find door lock state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_door_lock_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_door_lock_count; i++) {
        if (s_door_lock_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create door lock state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_door_lock_state_struct_t* get_or_create_door_lock_state(uint16_t short_addr)
{
    int idx = find_door_lock_state_index(short_addr);
    if (idx >= 0) {
        return &s_door_lock_states[idx];
    }

    /* Create new entry */
    if (s_door_lock_count >= ZB_STATE_MAX_DOOR_LOCK) {
        ESP_LOGW(TAG, "Door lock state storage full");
        return NULL;
    }

    idx = s_door_lock_count++;
    s_door_lock_addrs[idx] = short_addr;
    memset(&s_door_lock_states[idx], 0, sizeof(zb_door_lock_state_struct_t));
    s_door_lock_states[idx].lock_state = ZB_DOOR_LOCK_STATE_UNLOCKED;

    return &s_door_lock_states[idx];
}

/* ============================================================================
 * IAS Zone Internal Helpers
 * ============================================================================ */

/**
 * @brief Find IAS Zone state index for device
 *
 * @param[in] short_addr Device short address
 * @return Index in state array or -1 if not found
 */
static int find_ias_zone_state_index(uint16_t short_addr)
{
    for (size_t i = 0; i < s_ias_zone_count; i++) {
        if (s_ias_zone_addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Get or create IAS Zone state entry
 *
 * @param[in] short_addr Device short address
 * @return Pointer to state structure or NULL if full
 */
static zb_ias_zone_state_struct_t* get_or_create_ias_zone_state(uint16_t short_addr)
{
    int idx = find_ias_zone_state_index(short_addr);
    if (idx >= 0) {
        return &s_ias_zone_states[idx];
    }

    /* Create new entry */
    if (s_ias_zone_count >= ZB_STATE_MAX_IAS_ZONE) {
        ESP_LOGW(TAG, "IAS Zone state storage full");
        return NULL;
    }

    idx = s_ias_zone_count++;
    s_ias_zone_addrs[idx] = short_addr;
    memset(&s_ias_zone_states[idx], 0, sizeof(zb_ias_zone_state_struct_t));
    s_ias_zone_states[idx].zone_state = ZB_IAS_ZONE_STATE_NOT_ENROLLED;
    s_ias_zone_states[idx].zone_type = ZB_IAS_ZONE_TYPE_INVALID;
    s_ias_zone_states[idx].zone_id = 0xFF;  /* Invalid zone ID */

    ESP_LOGI(TAG, "Created IAS Zone state for device 0x%04X", short_addr);
    return &s_ias_zone_states[idx];
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_cluster_security_init(void)
{
    /* Initialize Door Lock storage */
    s_door_lock_count = 0;
    s_door_lock_callback = NULL;

    /* Allocate Door Lock arrays in PSRAM */
    if (s_door_lock_states == NULL) {
        s_door_lock_states = heap_caps_calloc(ZB_STATE_MAX_DOOR_LOCK,
                                               sizeof(zb_door_lock_state_struct_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_door_lock_states) {
            s_door_lock_states = calloc(ZB_STATE_MAX_DOOR_LOCK,
                                         sizeof(zb_door_lock_state_struct_t));  /* Fallback */
        }
    }
    if (s_door_lock_addrs == NULL) {
        s_door_lock_addrs = heap_caps_calloc(ZB_STATE_MAX_DOOR_LOCK,
                                              sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_door_lock_addrs) {
            s_door_lock_addrs = calloc(ZB_STATE_MAX_DOOR_LOCK,
                                        sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_door_lock_states || !s_door_lock_addrs) {
        ESP_LOGE(TAG, "Failed to allocate door lock state arrays");
        free(s_door_lock_states);
        s_door_lock_states = NULL;
        free(s_door_lock_addrs);
        s_door_lock_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Initialize IAS Zone storage */
    s_ias_zone_count = 0;
    s_ias_zone_callback = NULL;
    s_ias_zone_auto_enroll = true;
    s_ias_zone_next_id = 0;

    /* Allocate IAS Zone arrays in PSRAM */
    if (s_ias_zone_states == NULL) {
        s_ias_zone_states = heap_caps_calloc(ZB_STATE_MAX_IAS_ZONE,
                                              sizeof(zb_ias_zone_state_struct_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_ias_zone_states) {
            s_ias_zone_states = calloc(ZB_STATE_MAX_IAS_ZONE,
                                        sizeof(zb_ias_zone_state_struct_t));  /* Fallback */
        }
    }
    if (s_ias_zone_addrs == NULL) {
        s_ias_zone_addrs = heap_caps_calloc(ZB_STATE_MAX_IAS_ZONE,
                                             sizeof(uint16_t),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_ias_zone_addrs) {
            s_ias_zone_addrs = calloc(ZB_STATE_MAX_IAS_ZONE,
                                       sizeof(uint16_t));  /* Fallback */
        }
    }

    if (!s_ias_zone_states || !s_ias_zone_addrs) {
        ESP_LOGE(TAG, "Failed to allocate IAS Zone state arrays");
        free(s_door_lock_states);
        s_door_lock_states = NULL;
        free(s_door_lock_addrs);
        s_door_lock_addrs = NULL;
        free(s_ias_zone_states);
        s_ias_zone_states = NULL;
        free(s_ias_zone_addrs);
        s_ias_zone_addrs = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Security clusters initialized (Door Lock: %d, IAS Zone: %d max)",
             ZB_STATE_MAX_DOOR_LOCK, ZB_STATE_MAX_IAS_ZONE);
    return ESP_OK;
}

esp_err_t zb_cluster_security_deinit(void)
{
    s_door_lock_count = 0;
    s_door_lock_callback = NULL;
    s_ias_zone_count = 0;
    s_ias_zone_callback = NULL;
    ESP_LOGI(TAG, "Security clusters deinitialized");
    return ESP_OK;
}

void zb_cluster_security_clear_all(void)
{
    s_door_lock_count = 0;
    s_ias_zone_count = 0;
    if (s_door_lock_states) {
        memset(s_door_lock_states, 0,
               ZB_STATE_MAX_DOOR_LOCK * sizeof(zb_door_lock_state_struct_t));
    }
    if (s_door_lock_addrs) {
        memset(s_door_lock_addrs, 0,
               ZB_STATE_MAX_DOOR_LOCK * sizeof(uint16_t));
    }
    if (s_ias_zone_states) {
        memset(s_ias_zone_states, 0,
               ZB_STATE_MAX_IAS_ZONE * sizeof(zb_ias_zone_state_struct_t));
    }
    if (s_ias_zone_addrs) {
        memset(s_ias_zone_addrs, 0,
               ZB_STATE_MAX_IAS_ZONE * sizeof(uint16_t));
    }
    ESP_LOGD(TAG, "Cleared all security cluster states");
}

void zb_cluster_security_remove_device(uint16_t short_addr)
{
    /* Remove from Door Lock storage */
    for (size_t i = 0; i < s_door_lock_count; i++) {
        if (s_door_lock_addrs[i] == short_addr) {
            if (i < s_door_lock_count - 1) {
                s_door_lock_states[i] = s_door_lock_states[s_door_lock_count - 1];
                s_door_lock_addrs[i] = s_door_lock_addrs[s_door_lock_count - 1];
            }
            s_door_lock_count--;
            ESP_LOGD(TAG, "Removed door lock state for device 0x%04X", short_addr);
            break;
        }
    }

    /* Remove from IAS Zone storage */
    for (size_t i = 0; i < s_ias_zone_count; i++) {
        if (s_ias_zone_addrs[i] == short_addr) {
            if (i < s_ias_zone_count - 1) {
                s_ias_zone_states[i] = s_ias_zone_states[s_ias_zone_count - 1];
                s_ias_zone_addrs[i] = s_ias_zone_addrs[s_ias_zone_count - 1];
            }
            s_ias_zone_count--;
            ESP_LOGD(TAG, "Removed IAS Zone state for device 0x%04X", short_addr);
            break;
        }
    }
}

/* ============================================================================
 * Door Lock Cluster Public API
 * ============================================================================ */

esp_err_t zb_door_lock_register_callback(zb_door_lock_state_cb_t callback)
{
    s_door_lock_callback = callback;
    ESP_LOGI(TAG, "Door lock state callback registered");
    return ESP_OK;
}

bool zb_device_has_door_lock(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_DOOR_LOCK);
}

esp_err_t zb_door_lock_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading door lock state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
    };

    uint16_t attr_ids[] = { ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID };
    cmd_req.attr_number = 1;
    cmd_req.attr_field = attr_ids;

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send door lock read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_door_lock_cmd_lock(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending LOCK command to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_DOOR_LOCK_LOCK_DOOR_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = 0,
            .value = NULL,
        },
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send lock command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_door_lock_cmd_unlock(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending UNLOCK command to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_DOOR_LOCK_UNLOCK_DOOR_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = 0,
            .value = NULL,
        },
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send unlock command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_door_lock_handle_report(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_door_lock_state_struct_t *state = get_or_create_door_lock_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID:
            if (value_len >= 1) {
                uint8_t lock_state = *(uint8_t *)value;
                state->lock_state = (zb_door_lock_state_t)lock_state;

                const char *state_str = "UNKNOWN";
                switch (state->lock_state) {
                    case ZB_DOOR_LOCK_STATE_NOT_FULLY_LOCKED:
                        state_str = "NOT_FULLY_LOCKED";
                        break;
                    case ZB_DOOR_LOCK_STATE_LOCKED:
                        state_str = "LOCKED";
                        break;
                    case ZB_DOOR_LOCK_STATE_UNLOCKED:
                        state_str = "UNLOCKED";
                        break;
                }
                ESP_LOGI(TAG, "Door lock 0x%04X state: %s", short_addr, state_str);
            }
            break;

        case ZB_ZCL_ATTR_DOOR_LOCK_LOCK_TYPE_ID:
            if (value_len >= 1) {
                state->lock_type = *(uint8_t *)value;
                ESP_LOGD(TAG, "Door lock 0x%04X type: %d", short_addr, state->lock_type);
            }
            break;

        case ZB_ZCL_ATTR_DOOR_LOCK_ACTUATOR_ENABLED_ID:
            if (value_len >= 1) {
                state->actuator_enabled = (*(uint8_t *)value != 0);
                ESP_LOGD(TAG, "Door lock 0x%04X actuator: %s",
                         short_addr, state->actuator_enabled ? "enabled" : "disabled");
            }
            break;

        default:
            ESP_LOGD(TAG, "Door lock 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_door_lock_callback != NULL) {
        s_door_lock_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_door_lock_get_state(uint16_t short_addr, zb_door_lock_state_struct_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_door_lock_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_door_lock_states[idx], sizeof(zb_door_lock_state_struct_t));
    return ESP_OK;
}

/* ============================================================================
 * IAS Zone Cluster Public API
 * ============================================================================ */

esp_err_t zb_ias_zone_register_callback(zb_ias_zone_state_cb_t callback)
{
    s_ias_zone_callback = callback;
    ESP_LOGI(TAG, "IAS Zone state callback registered");
    return ESP_OK;
}

bool zb_device_has_ias_zone(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_IAS_ZONE);
}

esp_err_t zb_ias_zone_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading IAS Zone state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_IAS_ZONE,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_IAS_ZONE_STATE_ID,
        ZB_ZCL_ATTR_IAS_ZONE_TYPE_ID,
        ZB_ZCL_ATTR_IAS_ZONE_STATUS_ID,
        ZB_ZCL_ATTR_IAS_ZONE_ID_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send IAS Zone read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_ias_zone_write_cie_address(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Writing CIE address to IAS Zone device 0x%04X EP%d", short_addr, endpoint);

    /* Get coordinator's IEEE address */
    esp_zb_ieee_addr_t cie_addr;
    esp_zb_get_long_address(cie_addr);

    ESP_LOGI(TAG, "CIE address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             cie_addr[7], cie_addr[6], cie_addr[5], cie_addr[4],
             cie_addr[3], cie_addr[2], cie_addr[1], cie_addr[0]);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_IAS_ZONE,
        .attr_number = 1,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_IAS_ZONE_CIE_ADDRESS_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_IEEE_ADDR,
            .size = 8,
            .value = (void *)cie_addr,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CIE address: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_ias_zone_enroll_response(uint16_t short_addr, uint8_t endpoint,
                                       zb_ias_zone_enroll_response_code_t response_code,
                                       uint8_t zone_id)
{
    const char *code_str = "unknown";
    switch (response_code) {
        case ZB_IAS_ZONE_ENROLL_SUCCESS: code_str = "success"; break;
        case ZB_IAS_ZONE_ENROLL_NOT_SUPPORTED: code_str = "not_supported"; break;
        case ZB_IAS_ZONE_ENROLL_NO_ENROLL_PERMIT: code_str = "no_permit"; break;
        case ZB_IAS_ZONE_ENROLL_TOO_MANY_ZONES: code_str = "too_many"; break;
    }

    ESP_LOGI(TAG, "Sending Zone Enroll Response to 0x%04X EP%d: %s, zone_id=%d",
             short_addr, endpoint, code_str, zone_id);

    /* Build Zone Enroll Response payload: response_code (1 byte) + zone_id (1 byte) */
    uint8_t payload[2] = { (uint8_t)response_code, zone_id };

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_IAS_ZONE,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_IAS_ZONE_ENROLL_RESPONSE_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_NULL,
            .size = sizeof(payload),
            .value = payload,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Zone Enroll Response: %s", esp_err_to_name(ret));
    } else if (response_code == ZB_IAS_ZONE_ENROLL_SUCCESS) {
        /* Update local state */
        zb_ias_zone_state_struct_t *state = get_or_create_ias_zone_state(short_addr);
        if (state) {
            state->zone_state = ZB_IAS_ZONE_STATE_ENROLLED;
            state->zone_id = zone_id;
        }
    }

    return ret;
}

void zb_ias_zone_parse_status(uint16_t zone_status, zb_ias_zone_state_struct_t *state)
{
    if (state == NULL) {
        return;
    }

    state->zone_status = zone_status;
    state->alarm1 = (zone_status & ZB_IAS_ZONE_STATUS_ALARM1) != 0;
    state->alarm2 = (zone_status & ZB_IAS_ZONE_STATUS_ALARM2) != 0;
    state->tamper = (zone_status & ZB_IAS_ZONE_STATUS_TAMPER) != 0;
    state->battery_low = (zone_status & ZB_IAS_ZONE_STATUS_BATTERY_LOW) != 0;
    state->supervision_reports = (zone_status & ZB_IAS_ZONE_STATUS_SUPERVISION) != 0;
    state->restore_reports = (zone_status & ZB_IAS_ZONE_STATUS_RESTORE_REPORTS) != 0;
    state->trouble = (zone_status & ZB_IAS_ZONE_STATUS_TROUBLE) != 0;
    state->ac_mains_fault = (zone_status & ZB_IAS_ZONE_STATUS_AC_MAINS_FAULT) != 0;
    state->test_mode = (zone_status & ZB_IAS_ZONE_STATUS_TEST) != 0;
    state->battery_defect = (zone_status & ZB_IAS_ZONE_STATUS_BATTERY_DEFECT) != 0;
}

esp_err_t zb_ias_zone_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_ias_zone_state_struct_t *state = get_or_create_ias_zone_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_IAS_ZONE_STATE_ID:
            if (value_len >= 1) {
                state->zone_state = (zb_ias_zone_state_t)(*(uint8_t *)value);
                ESP_LOGI(TAG, "IAS Zone 0x%04X state: %s",
                         short_addr,
                         state->zone_state == ZB_IAS_ZONE_STATE_ENROLLED ? "enrolled" : "not_enrolled");
            }
            break;

        case ZB_ZCL_ATTR_IAS_ZONE_TYPE_ID:
            if (value_len >= 2) {
                state->zone_type = (zb_ias_zone_type_t)(*(uint16_t *)value);
                ESP_LOGI(TAG, "IAS Zone 0x%04X type: 0x%04X (%s)",
                         short_addr, state->zone_type,
                         zb_ias_zone_get_device_class(state->zone_type));

                /* Update device type based on zone type - requires device handler access */
                SemaphoreHandle_t mutex = zb_device_get_mutex();
                xSemaphoreTake(mutex, portMAX_DELAY);
                zb_device_t *device = zb_device_find_by_short_addr_unlocked(short_addr);
                if (device != NULL) {
                    switch (state->zone_type) {
                        case ZB_IAS_ZONE_TYPE_MOTION_SENSOR:
                            device->device_type = ZB_DEVICE_TYPE_MOTION_SENSOR;
                            break;
                        case ZB_IAS_ZONE_TYPE_CONTACT_SWITCH:
                            device->device_type = ZB_DEVICE_TYPE_DOOR_SENSOR;
                            break;
                        case ZB_IAS_ZONE_TYPE_FIRE_SENSOR:
                            device->device_type = ZB_DEVICE_TYPE_FIRE_SENSOR;
                            break;
                        case ZB_IAS_ZONE_TYPE_WATER_SENSOR:
                            device->device_type = ZB_DEVICE_TYPE_WATER_LEAK_SENSOR;
                            break;
                        case ZB_IAS_ZONE_TYPE_GAS_SENSOR:
                        case ZB_IAS_ZONE_TYPE_CARBON_MONOXIDE:
                            device->device_type = ZB_DEVICE_TYPE_GAS_SENSOR;
                            break;
                        case ZB_IAS_ZONE_TYPE_VIBRATION_SENSOR:
                            device->device_type = ZB_DEVICE_TYPE_VIBRATION_SENSOR;
                            break;
                        default:
                            device->device_type = ZB_DEVICE_TYPE_IAS_ZONE;
                            break;
                    }
                }
                xSemaphoreGive(mutex);
            }
            break;

        case ZB_ZCL_ATTR_IAS_ZONE_STATUS_ID:
            if (value_len >= 2) {
                uint16_t zone_status = *(uint16_t *)value;
                zb_ias_zone_parse_status(zone_status, state);
                ESP_LOGI(TAG, "IAS Zone 0x%04X status: 0x%04X (alarm1=%d, alarm2=%d, tamper=%d, battery_low=%d)",
                         short_addr, zone_status,
                         state->alarm1, state->alarm2, state->tamper, state->battery_low);
            }
            break;

        case ZB_ZCL_ATTR_IAS_ZONE_CIE_ADDRESS_ID:
            if (value_len >= 8) {
                memcpy(state->cie_address, value, 8);
                ESP_LOGI(TAG, "IAS Zone 0x%04X CIE address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                         short_addr,
                         state->cie_address[7], state->cie_address[6],
                         state->cie_address[5], state->cie_address[4],
                         state->cie_address[3], state->cie_address[2],
                         state->cie_address[1], state->cie_address[0]);
            }
            break;

        case ZB_ZCL_ATTR_IAS_ZONE_ID_ID:
            if (value_len >= 1) {
                state->zone_id = *(uint8_t *)value;
                ESP_LOGI(TAG, "IAS Zone 0x%04X zone_id: %d", short_addr, state->zone_id);
            }
            break;

        default:
            ESP_LOGD(TAG, "IAS Zone 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_ias_zone_callback != NULL) {
        s_ias_zone_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_ias_zone_handle_status_change(uint16_t short_addr, uint8_t endpoint,
                                            uint16_t zone_status, uint8_t extended_status,
                                            uint8_t zone_id, uint16_t delay)
{
    ESP_LOGI(TAG, "IAS Zone 0x%04X Status Change: status=0x%04X, ext=0x%02X, zone_id=%d, delay=%d",
             short_addr, zone_status, extended_status, zone_id, delay);

    zb_ias_zone_state_struct_t *state = get_or_create_ias_zone_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Parse and store the zone status */
    zb_ias_zone_parse_status(zone_status, state);
    state->zone_id = zone_id;

    ESP_LOGI(TAG, "IAS Zone 0x%04X: alarm1=%d, alarm2=%d, tamper=%d, battery_low=%d, trouble=%d",
             short_addr, state->alarm1, state->alarm2, state->tamper,
             state->battery_low, state->trouble);

    /* Invoke callback if registered */
    if (s_ias_zone_callback != NULL) {
        s_ias_zone_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_ias_zone_handle_enroll_request(uint16_t short_addr, uint8_t endpoint,
                                             uint16_t zone_type, uint16_t manufacturer_code)
{
    ESP_LOGI(TAG, "IAS Zone Enroll Request from 0x%04X EP%d: type=0x%04X, mfr=0x%04X",
             short_addr, endpoint, zone_type, manufacturer_code);

    /* Update state with zone type */
    zb_ias_zone_state_struct_t *state = get_or_create_ias_zone_state(short_addr);
    if (state != NULL) {
        state->zone_type = (zb_ias_zone_type_t)zone_type;
    }

    /* Auto-enroll if enabled */
    if (s_ias_zone_auto_enroll) {
        /* Assign next available zone ID */
        uint8_t assigned_zone_id = s_ias_zone_next_id++;
        if (s_ias_zone_next_id > 0xFE) {
            s_ias_zone_next_id = 0;  /* Wrap around */
        }

        ESP_LOGI(TAG, "Auto-enrolling IAS Zone device 0x%04X with zone_id=%d",
                 short_addr, assigned_zone_id);

        return zb_ias_zone_enroll_response(short_addr, endpoint,
                                            ZB_IAS_ZONE_ENROLL_SUCCESS,
                                            assigned_zone_id);
    }

    ESP_LOGW(TAG, "IAS Zone auto-enroll disabled, device 0x%04X not enrolled", short_addr);
    return ESP_OK;
}

esp_err_t zb_ias_zone_get_state(uint16_t short_addr, zb_ias_zone_state_struct_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_ias_zone_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_ias_zone_states[idx], sizeof(zb_ias_zone_state_struct_t));
    return ESP_OK;
}

const char* zb_ias_zone_get_device_class(zb_ias_zone_type_t zone_type)
{
    switch (zone_type) {
        case ZB_IAS_ZONE_TYPE_MOTION_SENSOR:
            return "motion";
        case ZB_IAS_ZONE_TYPE_CONTACT_SWITCH:
            return "door";  /* or "window" - Home Assistant uses "door" for generic contact */
        case ZB_IAS_ZONE_TYPE_FIRE_SENSOR:
            return "smoke";
        case ZB_IAS_ZONE_TYPE_WATER_SENSOR:
            return "moisture";
        case ZB_IAS_ZONE_TYPE_GAS_SENSOR:
            return "gas";
        case ZB_IAS_ZONE_TYPE_CARBON_MONOXIDE:
            return "carbon_monoxide";
        case ZB_IAS_ZONE_TYPE_VIBRATION_SENSOR:
            return "vibration";
        case ZB_IAS_ZONE_TYPE_GLASS_BREAK_SENSOR:
            return "sound";  /* Glass break is detected by sound */
        case ZB_IAS_ZONE_TYPE_PERSONAL_EMERGENCY:
            return "safety";
        case ZB_IAS_ZONE_TYPE_STANDARD_WARNING:
            return "problem";
        case ZB_IAS_ZONE_TYPE_REMOTE_CONTROL:
        case ZB_IAS_ZONE_TYPE_KEY_FOB:
        case ZB_IAS_ZONE_TYPE_KEYPAD:
            return NULL;  /* No device_class for these */
        default:
            return NULL;  /* Generic binary sensor */
    }
}

esp_err_t zb_ias_zone_set_auto_enroll(bool enable)
{
    s_ias_zone_auto_enroll = enable;
    ESP_LOGI(TAG, "IAS Zone auto-enroll %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

bool zb_ias_zone_get_auto_enroll(void)
{
    return s_ias_zone_auto_enroll;
}
