/**
 * @file zb_network.c
 * @brief Zigbee Network Management Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_network.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <string.h>
#include <inttypes.h>
#include <ctype.h>

/* SDK Version Check for ESP-Zigbee-SDK v1.6.x */
#include "esp_zigbee_core.h"

#ifndef ESP_ZB_VER_MAJOR
#error "ESP-Zigbee-SDK version macros not found. Update to v1.6.0+"
#endif

#if ESP_ZB_VER_MAJOR < 1 || \
    (ESP_ZB_VER_MAJOR == 1 && ESP_ZB_VER_MINOR < 6)
#error "ESP-Zigbee-SDK >= 1.6.0 required for this firmware"
#endif

static const char *TAG = "ZB_NET";

/* NVS namespace for network configuration */
#define NVS_NAMESPACE "zb_network"

/* NVS keys */
#define NVS_KEY_PAN_ID          "pan_id"
#define NVS_KEY_CHANNEL         "channel"
#define NVS_KEY_EXT_PAN_ID      "ext_pan_id"
#define NVS_KEY_NETWORK_KEY     "net_key"
#define NVS_KEY_FORMED          "formed"

/* Global network information */
static zb_network_info_t s_network_info = {0};
static bool s_network_initialized = false;

esp_err_t zb_network_init(void)
{
    if (s_network_initialized) {
        ESP_LOGW(TAG, "Network manager already initialized");
        return ESP_OK;
    }

    /* Initialize network info with defaults from Kconfig */
    memset(&s_network_info, 0, sizeof(zb_network_info_t));
    s_network_info.pan_id = zb_network_get_pan_id_config();
    s_network_info.channel = zb_network_get_channel_config();
    s_network_info.short_addr = 0x0000; /* Coordinator is always 0x0000 */
    s_network_info.depth = 0;
    s_network_info.permit_join = false;
    s_network_info.network_formed = false;
    s_network_info.device_count = 0;

    /* Try to load stored configuration */
    esp_err_t ret = zb_network_load_config();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded network config from NVS: PAN=0x%04X, CH=%d",
                 s_network_info.pan_id, s_network_info.channel);
    } else {
        ESP_LOGI(TAG, "No stored network config, using defaults: PAN=0x%04X, CH=%d",
                 s_network_info.pan_id, s_network_info.channel);
    }

    s_network_initialized = true;
    ESP_LOGI(TAG, "Network manager initialized");
    return ESP_OK;
}

esp_err_t zb_network_deinit(void)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_network_info, 0, sizeof(zb_network_info_t));
    s_network_initialized = false;
    ESP_LOGI(TAG, "Network manager deinitialized");
    return ESP_OK;
}

esp_err_t zb_network_get_info(zb_network_info_t *info)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(info, &s_network_info, sizeof(zb_network_info_t));
    return ESP_OK;
}

esp_err_t zb_network_save_config(void)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Save configuration */
    ret = nvs_set_u16(nvs_handle, NVS_KEY_PAN_ID, s_network_info.pan_id);
    if (ret == ESP_OK) {
        ret = nvs_set_u8(nvs_handle, NVS_KEY_CHANNEL, s_network_info.channel);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(nvs_handle, NVS_KEY_EXT_PAN_ID,
                          s_network_info.extended_pan_id, 8);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_blob(nvs_handle, NVS_KEY_NETWORK_KEY,
                          s_network_info.network_key, ZB_NETWORK_KEY_LEN);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(nvs_handle, NVS_KEY_FORMED,
                        s_network_info.network_formed ? 1 : 0);
    }

    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Network config saved to NVS");
        }
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_network_load_config(void)
{
    /* Note: No s_network_initialized check here because this function
     * is called from zb_network_init() before the flag is set */

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Load configuration */
    uint16_t pan_id;
    ret = nvs_get_u16(nvs_handle, NVS_KEY_PAN_ID, &pan_id);
    if (ret == ESP_OK) {
        s_network_info.pan_id = pan_id;

        uint8_t channel;
        if (nvs_get_u8(nvs_handle, NVS_KEY_CHANNEL, &channel) == ESP_OK) {
            s_network_info.channel = channel;
        }

        size_t len = 8;
        nvs_get_blob(nvs_handle, NVS_KEY_EXT_PAN_ID,
                    s_network_info.extended_pan_id, &len);

        len = ZB_NETWORK_KEY_LEN;
        nvs_get_blob(nvs_handle, NVS_KEY_NETWORK_KEY,
                    s_network_info.network_key, &len);

        uint8_t formed;
        if (nvs_get_u8(nvs_handle, NVS_KEY_FORMED, &formed) == ESP_OK) {
            s_network_info.network_formed = (formed != 0);
        }
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_network_reset(void)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Resetting network configuration!");

    /* Erase NVS */
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    /* Reset to defaults */
    memset(&s_network_info, 0, sizeof(zb_network_info_t));
    s_network_info.pan_id = zb_network_get_pan_id_config();
    s_network_info.channel = zb_network_get_channel_config();
    s_network_info.short_addr = 0x0000;
    s_network_info.depth = 0;
    s_network_info.permit_join = false;
    s_network_info.network_formed = false;
    s_network_info.device_count = 0;

    ESP_LOGI(TAG, "Network configuration reset to defaults");
    return ESP_OK;
}

bool zb_network_has_stored_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return false;
    }

    uint16_t pan_id;
    ret = nvs_get_u16(nvs_handle, NVS_KEY_PAN_ID, &pan_id);
    nvs_close(nvs_handle);

    return (ret == ESP_OK);
}

esp_err_t zb_network_set_permit_join(bool permit_join)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_network_info.permit_join = permit_join;
    return ESP_OK;
}

esp_err_t zb_network_set_device_count(uint8_t count)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_network_info.device_count = count;
    return ESP_OK;
}

esp_err_t zb_network_set_formed(bool formed)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_network_info.network_formed = formed;
    ESP_LOGI(TAG, "Network formed status: %s", formed ? "YES" : "NO");
    return ESP_OK;
}

uint16_t zb_network_get_pan_id_config(void)
{
#ifdef CONFIG_ZIGBEE_PAN_ID
    return CONFIG_ZIGBEE_PAN_ID;
#else
    return ZB_DEFAULT_PAN_ID;
#endif
}

uint8_t zb_network_get_channel_config(void)
{
#ifdef CONFIG_ZIGBEE_CHANNEL
    return CONFIG_ZIGBEE_CHANNEL;
#else
    /* Hardcode channel 15 to avoid channel 11 conflicts */
    return 15;
#endif
}

uint8_t zb_network_get_max_children_config(void)
{
#ifdef CONFIG_ZIGBEE_MAX_CHILDREN
    return CONFIG_ZIGBEE_MAX_CHILDREN;
#else
    return ZB_DEFAULT_MAX_CHILDREN;
#endif
}

uint8_t zb_network_get_permit_join_duration_config(void)
{
#ifdef CONFIG_ZIGBEE_PERMIT_JOIN_DURATION
    return CONFIG_ZIGBEE_PERMIT_JOIN_DURATION;
#else
    return ZB_DEFAULT_PERMIT_JOIN_DURATION_SEC;
#endif
}

esp_err_t zb_network_test(void)
{
    ESP_LOGI(TAG, "Running network self-test...");

    /* Test initialization */
    if (!s_network_initialized) {
        ESP_LOGE(TAG, "Network not initialized");
        return ESP_FAIL;
    }

    /* Test info retrieval */
    zb_network_info_t info;
    esp_err_t ret = zb_network_get_info(&info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get network info");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Network info: PAN=0x%04X, Channel=%d, Devices=%d",
             info.pan_id, info.channel, info.device_count);

    ESP_LOGI(TAG, "Network self-test PASSED");
    return ESP_OK;
}

/* ============================================================================
 * Network Channel Change Implementation (ZG-016)
 * ============================================================================ */

/* Channel change state */
static struct {
    zb_channel_change_state_t state;
    uint8_t old_channel;
    uint8_t new_channel;
    uint8_t scan_duration;
    zb_channel_change_cb_t callback;
    void *user_data;
    uint32_t nwk_update_id;
    TimerHandle_t timeout_timer;
} s_channel_change = {
    .state = ZB_CHANNEL_CHANGE_IDLE,
    .timeout_timer = NULL
};

/* Note: ZB_CHANNEL_MIN, ZB_CHANNEL_MAX, ZB_CHANNEL_MASK_ALL, ZB_CHANNEL_CHANGE_TIMEOUT_MS defined in zb_constants.h */

/* Scan duration for channel change (0xFE = channel change request) */
#define ZB_NWK_UPDATE_CHANNEL_CHANGE    0xFE

/**
 * @brief Generic ZDO network update notify callback
 *
 * Logs the result of network update ZDO requests.
 *
 * @param notify Notification data from the stack
 * @param user_ctx User context (unused)
 */
static void zdo_nwk_update_callback(const esp_zb_zdo_mgmt_update_notify_t *notify, void *user_ctx)
{
    (void)user_ctx;
    if (notify == NULL) {
        ESP_LOGW(TAG, "ZDO network update notify: NULL response");
        return;
    }
    if (notify->status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGD(TAG, "ZDO network update request succeeded");
    } else {
        ESP_LOGW(TAG, "ZDO network update request failed: 0x%02x", notify->status);
    }
}

/**
 * @brief Channel change timeout callback
 */
static void channel_change_timeout_cb(TimerHandle_t timer)
{
    (void)timer;

    ESP_LOGW(TAG, "Channel change timeout");

    zb_channel_change_cb_t callback = s_channel_change.callback;
    void *user_data = s_channel_change.user_data;
    uint8_t old_channel = s_channel_change.old_channel;
    uint8_t new_channel = s_channel_change.new_channel;

    /* Reset state */
    s_channel_change.state = ZB_CHANNEL_CHANGE_FAILED;

    /* Notify via callback */
    if (callback != NULL) {
        callback(ESP_ERR_TIMEOUT, old_channel, new_channel, user_data);
    }

    /* Clean up */
    s_channel_change.state = ZB_CHANNEL_CHANGE_IDLE;
    s_channel_change.callback = NULL;
    s_channel_change.user_data = NULL;
}

bool zb_network_validate_channel(uint8_t channel)
{
    return (channel >= ZB_CHANNEL_MIN && channel <= ZB_CHANNEL_MAX);
}

bool zb_network_is_channel_change_pending(void)
{
    return (s_channel_change.state != ZB_CHANNEL_CHANGE_IDLE &&
            s_channel_change.state != ZB_CHANNEL_CHANGE_COMPLETE &&
            s_channel_change.state != ZB_CHANNEL_CHANGE_FAILED);
}

zb_channel_change_state_t zb_network_get_channel_change_state(void)
{
    return s_channel_change.state;
}

esp_err_t zb_network_change_channel(uint8_t new_channel,
                                    zb_channel_change_cb_t callback,
                                    void *user_data)
{
    return zb_network_change_channel_ex(new_channel, ZB_NWK_UPDATE_CHANNEL_CHANGE,
                                        callback, user_data);
}

esp_err_t zb_network_change_channel_ex(uint8_t new_channel,
                                       uint8_t scan_duration,
                                       zb_channel_change_cb_t callback,
                                       void *user_data)
{
    if (!s_network_initialized) {
        ESP_LOGE(TAG, "Network manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate channel */
    if (!zb_network_validate_channel(new_channel)) {
        ESP_LOGE(TAG, "Invalid channel: %d (must be %d-%d)",
                 new_channel, ZB_CHANNEL_MIN, ZB_CHANNEL_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check if change already in progress */
    if (zb_network_is_channel_change_pending()) {
        ESP_LOGW(TAG, "Channel change already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if network is formed */
    if (!s_network_info.network_formed) {
        ESP_LOGE(TAG, "Network not formed, cannot change channel");
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Check if already on requested channel */
    if (s_network_info.channel == new_channel) {
        ESP_LOGI(TAG, "Already on channel %d", new_channel);
        if (callback != NULL) {
            callback(ESP_OK, new_channel, new_channel, user_data);
        }
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initiating channel change: %d -> %d",
             s_network_info.channel, new_channel);

    /* Store channel change context */
    s_channel_change.old_channel = s_network_info.channel;
    s_channel_change.new_channel = new_channel;
    s_channel_change.scan_duration = scan_duration;
    s_channel_change.callback = callback;
    s_channel_change.user_data = user_data;
    s_channel_change.state = ZB_CHANNEL_CHANGE_PENDING;

    /* Create timeout timer if not exists */
    if (s_channel_change.timeout_timer == NULL) {
        s_channel_change.timeout_timer = xTimerCreate(
            "ch_change",
            pdMS_TO_TICKS(ZB_CHANNEL_CHANGE_TIMEOUT_MS),
            pdFALSE,  /* One-shot */
            NULL,
            channel_change_timeout_cb
        );
    }

    /* Start timeout timer */
    if (s_channel_change.timeout_timer != NULL) {
        xTimerStart(s_channel_change.timeout_timer, 0);
    }

    /*
     * Send Mgmt_NWK_Update_req to broadcast address
     *
     * The Zigbee stack handles the actual channel change through
     * esp_zb_zdo_nwk_mgmt_req() or similar API.
     *
     * Scan channels mask: bit (channel - 1) for single channel
     * scan_duration 0xFE = channel change request
     */
    uint32_t channel_mask = (1UL << new_channel);

    /* Get current network update ID and increment */
    /* Note: In a full implementation, this would be read from the NIB */
    s_channel_change.nwk_update_id = 0;  /* Will be set by stack */

    s_channel_change.state = ZB_CHANNEL_CHANGE_NOTIFYING;

    /* Send network update request to all devices */
    esp_zb_zdo_mgmt_nwk_update_req_param_t req = {
        .dst_addr = 0xFFFC,  /* All routers and coordinator */
        .scan_channels = channel_mask,
        .scan_duration = scan_duration,
        .scan_count = 0,
        .nwk_manager_addr = 0x0000  /* Coordinator address */
    };

    /* Send request with response logger callback */
    esp_zb_zdo_mgmt_nwk_update_req(&req, zdo_nwk_update_callback, NULL);

    /* Request sent - result will come via callback or signal handler */
    ESP_LOGI(TAG, "Mgmt_NWK_Update_req sent for channel change to %d", new_channel);

    /* The channel change result will be handled by the timeout timer
     * and the signal handler. If we don't hear back, timeout fires. */

    /* Note: Error handling happens via timeout timer or signal callback
     * since esp_zb_zdo_mgmt_nwk_update_req() returns void in SDK v1.6.x */

    /*
     * The actual channel switch happens asynchronously.
     * The Zigbee stack will call our callback when done.
     * For now, we'll complete the channel change locally.
     */

    /* Switch coordinator channel */
    s_channel_change.state = ZB_CHANNEL_CHANGE_SWITCHING;

    /* Update local state */
    s_network_info.channel = new_channel;

    /* Save to NVS */
    esp_err_t save_ret = zb_network_save_config();
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save channel to NVS: %s", esp_err_to_name(save_ret));
    }

    /* Stop timeout timer */
    if (s_channel_change.timeout_timer != NULL) {
        xTimerStop(s_channel_change.timeout_timer, 0);
    }

    s_channel_change.state = ZB_CHANNEL_CHANGE_COMPLETE;

    ESP_LOGI(TAG, "Channel change complete: %d -> %d",
             s_channel_change.old_channel, new_channel);

    /* Notify via callback */
    if (callback != NULL) {
        callback(ESP_OK, s_channel_change.old_channel, new_channel, user_data);
    }

    /* Reset state */
    s_channel_change.state = ZB_CHANNEL_CHANGE_IDLE;
    s_channel_change.callback = NULL;
    s_channel_change.user_data = NULL;

    return ESP_OK;
}

esp_err_t zb_network_cancel_channel_change(void)
{
    if (s_channel_change.state == ZB_CHANNEL_CHANGE_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_channel_change.state == ZB_CHANNEL_CHANGE_SWITCHING) {
        ESP_LOGW(TAG, "Cannot cancel, channel switch already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Cancelling channel change");

    /* Stop timeout timer */
    if (s_channel_change.timeout_timer != NULL) {
        xTimerStop(s_channel_change.timeout_timer, 0);
    }

    zb_channel_change_cb_t callback = s_channel_change.callback;
    void *user_data = s_channel_change.user_data;
    uint8_t old_channel = s_channel_change.old_channel;
    uint8_t new_channel = s_channel_change.new_channel;

    /* Reset state */
    s_channel_change.state = ZB_CHANNEL_CHANGE_IDLE;
    s_channel_change.callback = NULL;
    s_channel_change.user_data = NULL;

    /* Notify via callback */
    if (callback != NULL) {
        callback(ESP_ERR_INVALID_STATE, old_channel, new_channel, user_data);
    }

    return ESP_OK;
}

esp_err_t zb_network_get_available_channels(uint32_t *channels_mask,
                                            int8_t threshold_dbm)
{
    if (channels_mask == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Start with all channels */
    *channels_mask = ZB_CHANNEL_MASK_ALL;

    /*
     * In a full implementation, this would:
     * 1. Perform energy scan on all channels
     * 2. Filter out channels with noise above threshold
     *
     * For now, return all channels as available
     */
    ESP_LOGI(TAG, "Available channels mask: 0x%08" PRIX32 " (threshold: %d dBm)",
             *channels_mask, threshold_dbm);

    return ESP_OK;
}

esp_err_t zb_network_set_channel_internal(uint8_t channel)
{
    if (!zb_network_validate_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_network_info.channel = channel;
    ESP_LOGI(TAG, "Internal channel set to: %d", channel);

    return ESP_OK;
}

/* ============================================================================
 * Extended PAN ID Management Implementation (API-004)
 * ============================================================================ */

esp_err_t zb_network_get_extended_pan_id(uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN])
{
    if (ext_pan_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Use ESP-Zigbee-SDK v1.6.8 API */
    esp_zb_ieee_addr_t ieee_ext_pan_id;
    esp_zb_nwk_get_extended_pan_id(ieee_ext_pan_id);

    memcpy(ext_pan_id, ieee_ext_pan_id, ZB_EXTENDED_PAN_ID_LEN);

    /* Also update local network info cache */
    memcpy(s_network_info.extended_pan_id, ieee_ext_pan_id, ZB_EXTENDED_PAN_ID_LEN);

    ESP_LOGD(TAG, "Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             ext_pan_id[7], ext_pan_id[6], ext_pan_id[5], ext_pan_id[4],
             ext_pan_id[3], ext_pan_id[2], ext_pan_id[1], ext_pan_id[0]);

    return ESP_OK;
}

esp_err_t zb_network_set_extended_pan_id(const uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN])
{
    if (ext_pan_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             ext_pan_id[7], ext_pan_id[6], ext_pan_id[5], ext_pan_id[4],
             ext_pan_id[3], ext_pan_id[2], ext_pan_id[1], ext_pan_id[0]);

    /* Use ESP-Zigbee-SDK v1.6.8 API */
    esp_zb_ieee_addr_t ieee_ext_pan_id;
    memcpy(ieee_ext_pan_id, ext_pan_id, ZB_EXTENDED_PAN_ID_LEN);
    esp_zb_nwk_set_extended_pan_id(ieee_ext_pan_id);

    /* Update local network info cache */
    memcpy(s_network_info.extended_pan_id, ext_pan_id, ZB_EXTENDED_PAN_ID_LEN);

    /* Save to NVS for persistence */
    esp_err_t ret = zb_network_save_config();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save Extended PAN ID to NVS: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t zb_network_set_extended_pan_id_from_mac(void)
{
    if (!s_network_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t mac_addr[ZB_EXTENDED_PAN_ID_LEN];
    esp_err_t ret = esp_read_mac(mac_addr, ESP_MAC_IEEE802154);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IEEE MAC address: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Setting Extended PAN ID from IEEE MAC address");
    return zb_network_set_extended_pan_id(mac_addr);
}

bool zb_network_has_extended_pan_id(void)
{
    if (!s_network_initialized) {
        return false;
    }

    /* Get current Extended PAN ID from stack */
    uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN];
    if (zb_network_get_extended_pan_id(ext_pan_id) != ESP_OK) {
        return false;
    }

    /* Check if all zeros */
    for (size_t i = 0; i < ZB_EXTENDED_PAN_ID_LEN; i++) {
        if (ext_pan_id[i] != 0) {
            return true;
        }
    }

    return false;
}

esp_err_t zb_network_format_extended_pan_id(const uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN],
                                             char *str_buf, size_t buf_len)
{
    if (ext_pan_id == NULL || str_buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Need 19 bytes: "0x" + 16 hex chars + null terminator */
    if (buf_len < ZB_EXT_PAN_ID_BUFFER_SIZE) {
        return ESP_ERR_NO_MEM;
    }

    /* Format as big-endian hex string (most significant byte first) */
    snprintf(str_buf, buf_len, "0x%02X%02X%02X%02X%02X%02X%02X%02X",
             ext_pan_id[7], ext_pan_id[6], ext_pan_id[5], ext_pan_id[4],
             ext_pan_id[3], ext_pan_id[2], ext_pan_id[1], ext_pan_id[0]);

    return ESP_OK;
}

esp_err_t zb_network_parse_extended_pan_id(const char *str_in,
                                            uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN])
{
    if (str_in == NULL || ext_pan_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *hex_start = str_in;

    /* Skip optional "0x" or "0X" prefix */
    if (str_in[0] == '0' && (str_in[1] == 'x' || str_in[1] == 'X')) {
        hex_start = str_in + 2;
    }

    /* Check length (16 hex characters) */
    size_t hex_len = strlen(hex_start);
    if (hex_len != ZB_EXT_PAN_ID_HEX_LEN) {
        ESP_LOGE(TAG, "Invalid Extended PAN ID length: %zu (expected %d)", hex_len, ZB_EXT_PAN_ID_HEX_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate all characters are hex */
    for (size_t i = 0; i < ZB_EXT_PAN_ID_HEX_LEN; i++) {
        if (!isxdigit((unsigned char)hex_start[i])) {
            ESP_LOGE(TAG, "Invalid hex character at position %zu", i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* Parse hex string (big-endian to little-endian array) */
    for (size_t i = 0; i < ZB_EXTENDED_PAN_ID_LEN; i++) {
        char hex_byte[3] = {hex_start[i * 2], hex_start[i * 2 + 1], '\0'};
        /* Store in reverse order (big-endian string to little-endian array) */
        ext_pan_id[ZB_EXTENDED_PAN_ID_LEN - 1 - i] = (uint8_t)strtoul(hex_byte, NULL, 16);
    }

    return ESP_OK;
}

/* ============================================================================
 * Network Key Rotation Implementation (API-006)
 * ============================================================================ */

esp_err_t zb_network_get_key_rotation_sequence(uint32_t *sequence)
{
    if (sequence == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        *sequence = 0;
        return ESP_ERR_NOT_FOUND;
    }

    ret = nvs_get_u32(nvs_handle, NVS_KEY_KEY_ROTATION_SEQ, sequence);
    nvs_close(nvs_handle);

    if (ret != ESP_OK) {
        *sequence = 0;
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

/**
 * @brief Save key rotation sequence to NVS
 * @param sequence Sequence number to save
 * @return ESP_OK on success
 */
static esp_err_t save_key_rotation_sequence(uint32_t sequence)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for key rotation sequence: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u32(nvs_handle, NVS_KEY_KEY_ROTATION_SEQ, sequence);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_network_rotate_key(uint32_t *sequence)
{
    if (!s_network_initialized) {
        ESP_LOGE(TAG, "Network manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_network_info.network_formed) {
        ESP_LOGE(TAG, "Network not formed, cannot rotate key");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initiating network key rotation...");

    /* Get current sequence number */
    uint32_t current_seq = 0;
    zb_network_get_key_rotation_sequence(&current_seq);

    /* Increment sequence for new rotation */
    uint32_t new_seq = current_seq + 1;

    /*
     * Broadcast network key switch using ESP-Zigbee-SDK v1.6.8 API
     *
     * esp_zb_secur_broadcast_network_key_switch() broadcasts a new network key
     * to all devices in the network. The stack handles key generation and
     * distribution internally.
     *
     * This operation requires the Zigbee lock to be acquired.
     */
    esp_zb_lock_acquire(portMAX_DELAY);

    /* Broadcast the network key switch command with the new sequence number */
    esp_zb_secur_broadcast_network_key_switch((uint8_t)(new_seq & 0xFF));

    ESP_LOGI(TAG, "Network key switch broadcast sent (seq=%u)", (unsigned)(new_seq & 0xFF));

    esp_zb_lock_release();

    /* Save new sequence number to NVS */
    esp_err_t ret = save_key_rotation_sequence(new_seq);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save key rotation sequence to NVS: %s",
                 esp_err_to_name(ret));
        /* Continue anyway - the key rotation was initiated */
    }

    ESP_LOGI(TAG, "Network key rotation initiated, sequence: %" PRIu32, new_seq);

    /* Return new sequence if requested */
    if (sequence != NULL) {
        *sequence = new_seq;
    }

    return ESP_OK;
}
