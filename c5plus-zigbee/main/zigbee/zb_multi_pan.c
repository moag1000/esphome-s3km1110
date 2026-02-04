/**
 * @file zb_multi_pan.c
 * @brief Multiple Zigbee Network (Multi-PAN) Support Implementation
 *
 * This module implements time-division multiplexing for managing multiple
 * Zigbee networks on a single 802.15.4 radio.
 *
 * @note EXPERIMENTAL FEATURE - Not recommended for production use!
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_multi_pan.h"
#include "zb_coordinator.h"
#include "zb_network.h"
#include "zb_constants.h"
#include "gateway_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "core/compat_stubs.h"
#include "utils/json_utils.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ZB_MULTI_PAN";

/* NVS namespace and keys */
#define NVS_NAMESPACE           "zb_multi_pan"
#define NVS_KEY_CONFIG          "config"
#define NVS_KEY_PRIMARY_CTX     "primary"
#define NVS_KEY_SECONDARY_CTX   "secondary"

/* MQTT topics */
#define MQTT_TOPIC_STATUS       "zigbee2mqtt/bridge/multipan/status"
#define MQTT_TOPIC_SWITCH       "zigbee2mqtt/bridge/multipan/switch"
#define MQTT_TOPIC_CONFIG       "zigbee2mqtt/bridge/multipan/config"

/* Timer periods */
#define TIMER_PERIOD_MS         100

/* Module state */
static struct {
    bool initialized;
    zb_multi_pan_state_t state;
    zb_multi_pan_network_id_t active_network;

    /* Network contexts */
    zb_multi_pan_network_ctx_t networks[ZB_MULTI_PAN_MAX_NETWORKS];
    bool network_valid[ZB_MULTI_PAN_MAX_NETWORKS];

    /* Configuration */
    zb_multi_pan_config_t config;

    /* FreeRTOS resources */
    SemaphoreHandle_t mutex;
    TimerHandle_t switch_timer;
    TaskHandle_t switch_task;

    /* Timing */
    uint32_t start_time_ms;
    uint32_t last_switch_time_ms;
    uint32_t primary_active_ms;
    uint32_t secondary_active_ms;
    uint32_t current_slot_start_ms;

    /* Statistics */
    uint32_t total_switches;
    uint32_t failed_switches;

    /* Callbacks */
    zb_multi_pan_switch_cb_t switch_callback;
    void *callback_user_data;

    /* Pending transactions tracking */
    uint32_t pending_transactions;

} s_multi_pan = {
    .initialized = false,
    .state = ZB_MULTI_PAN_STATE_DISABLED,
    .active_network = ZB_MULTI_PAN_NETWORK_NONE
};

/* Forward declarations */
static void switch_timer_callback(TimerHandle_t timer);
static void switch_task_func(void *pvParameters);
static esp_err_t perform_network_switch(zb_multi_pan_network_id_t target,
                                        zb_multi_pan_switch_reason_t reason);
static esp_err_t save_network_context(zb_multi_pan_network_id_t network_id);
static esp_err_t load_network_context(zb_multi_pan_network_id_t network_id);
static esp_err_t synchronize_with_stack(zb_multi_pan_network_id_t network_id);
static void publish_status_mqtt(void);
static uint32_t get_time_ms(void);

/**
 * @brief Get current time in milliseconds
 */
static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Take mutex with timeout
 */
static bool take_mutex(uint32_t timeout_ms)
{
    if (s_multi_pan.mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_multi_pan.mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief Release mutex
 */
static void give_mutex(void)
{
    if (s_multi_pan.mutex != NULL) {
        xSemaphoreGive(s_multi_pan.mutex);
    }
}

esp_err_t zb_multi_pan_init(void)
{
#ifndef CONFIG_ZIGBEE_MULTI_PAN_ENABLED
    ESP_LOGW(TAG, "Multi-PAN not enabled in Kconfig");
    return ESP_ERR_NOT_SUPPORTED;
#else
    ESP_LOGI(TAG, "Initializing Multi-PAN module (EXPERIMENTAL)");

    if (s_multi_pan.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create mutex */
    s_multi_pan.mutex = xSemaphoreCreateMutex();
    if (s_multi_pan.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize network contexts */
    memset(s_multi_pan.networks, 0, sizeof(s_multi_pan.networks));
    memset(s_multi_pan.network_valid, 0, sizeof(s_multi_pan.network_valid));

    /* Set default configuration */
#ifdef CONFIG_ZIGBEE_PRIMARY_CHANNEL
    s_multi_pan.config.primary_channel = CONFIG_ZIGBEE_PRIMARY_CHANNEL;
#else
    s_multi_pan.config.primary_channel = ZB_DEFAULT_PRIMARY_CHANNEL;
#endif

#ifdef CONFIG_ZIGBEE_SECONDARY_CHANNEL
    s_multi_pan.config.secondary_channel = CONFIG_ZIGBEE_SECONDARY_CHANNEL;
#else
    s_multi_pan.config.secondary_channel = ZB_DEFAULT_SECONDARY_CHANNEL;
#endif

#ifdef CONFIG_ZIGBEE_TIME_RATIO
    s_multi_pan.config.time_ratio = CONFIG_ZIGBEE_TIME_RATIO;
#else
    s_multi_pan.config.time_ratio = ZB_MULTI_PAN_DEFAULT_RATIO;
#endif

    s_multi_pan.config.switch_interval_ms = ZB_MULTI_PAN_DEFAULT_SWITCH_MS;
    s_multi_pan.config.graceful_switch = true;
    s_multi_pan.config.switch_timeout_ms = ZB_MULTI_PAN_SWITCH_TIMEOUT_MS;

    /* Try to load saved configuration */
    esp_err_t ret = zb_multi_pan_load_config();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded saved configuration");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved configuration, using defaults");
    } else {
        ESP_LOGW(TAG, "Failed to load configuration: %s", esp_err_to_name(ret));
    }

    /* Create switch timer */
    s_multi_pan.switch_timer = xTimerCreate(
        "multi_pan_timer",
        pdMS_TO_TICKS(TIMER_PERIOD_MS),
        pdTRUE,  /* Auto-reload */
        NULL,
        switch_timer_callback
    );

    if (s_multi_pan.switch_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create switch timer");
        vSemaphoreDelete(s_multi_pan.mutex);
        s_multi_pan.mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Reset statistics */
    s_multi_pan.total_switches = 0;
    s_multi_pan.failed_switches = 0;
    s_multi_pan.pending_transactions = 0;

    s_multi_pan.state = ZB_MULTI_PAN_STATE_INITIALIZING;
    s_multi_pan.initialized = true;

    ESP_LOGI(TAG, "Multi-PAN initialized - Primary: Ch%d, Secondary: Ch%d, Ratio: %d%%",
             s_multi_pan.config.primary_channel,
             s_multi_pan.config.secondary_channel,
             s_multi_pan.config.time_ratio);

    return ESP_OK;
#endif
}

esp_err_t zb_multi_pan_deinit(void)
{
    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Multi-PAN module");

    /* Stop if running */
    if (s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING) {
        zb_multi_pan_stop();
    }

    /* Delete timer */
    if (s_multi_pan.switch_timer != NULL) {
        xTimerDelete(s_multi_pan.switch_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS);
        s_multi_pan.switch_timer = NULL;
    }

    /* Delete mutex */
    if (s_multi_pan.mutex != NULL) {
        vSemaphoreDelete(s_multi_pan.mutex);
        s_multi_pan.mutex = NULL;
    }

    s_multi_pan.state = ZB_MULTI_PAN_STATE_DISABLED;
    s_multi_pan.initialized = false;

    ESP_LOGI(TAG, "Multi-PAN deinitialized");
    return ESP_OK;
}

esp_err_t zb_multi_pan_start(void)
{
    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    /* Check that at least primary network is configured */
    if (!s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_PRIMARY]) {
        ESP_LOGE(TAG, "Primary network not configured");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting Multi-PAN operation");

    if (!take_mutex(1000)) {
        return ESP_ERR_TIMEOUT;
    }

    /* Initialize timing */
    s_multi_pan.start_time_ms = get_time_ms();
    s_multi_pan.last_switch_time_ms = s_multi_pan.start_time_ms;
    s_multi_pan.current_slot_start_ms = s_multi_pan.start_time_ms;
    s_multi_pan.primary_active_ms = 0;
    s_multi_pan.secondary_active_ms = 0;

    /* Start with primary network */
    s_multi_pan.active_network = ZB_MULTI_PAN_NETWORK_PRIMARY;
    esp_err_t ret = synchronize_with_stack(ZB_MULTI_PAN_NETWORK_PRIMARY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to synchronize with primary network");
        give_mutex();
        return ret;
    }

    /* Start the switch timer */
    if (s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_SECONDARY]) {
        if (xTimerStart(s_multi_pan.switch_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS) != pdPASS) {
            ESP_LOGE(TAG, "Failed to start switch timer");
            give_mutex();
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Time-division enabled: %d%% primary / %d%% secondary",
                 s_multi_pan.config.time_ratio,
                 100 - s_multi_pan.config.time_ratio);
    } else {
        ESP_LOGI(TAG, "Secondary network not configured - single network mode");
    }

    s_multi_pan.state = ZB_MULTI_PAN_STATE_RUNNING;

    give_mutex();

    /* Publish initial status */
    publish_status_mqtt();

    ESP_LOGI(TAG, "Multi-PAN started on channel %d",
             s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_PRIMARY].channel);

    return ESP_OK;
}

esp_err_t zb_multi_pan_stop(void)
{
    if (!s_multi_pan.initialized ||
        s_multi_pan.state != ZB_MULTI_PAN_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping Multi-PAN operation");

    if (!take_mutex(1000)) {
        return ESP_ERR_TIMEOUT;
    }

    /* Stop the timer */
    if (s_multi_pan.switch_timer != NULL) {
        xTimerStop(s_multi_pan.switch_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS);
    }

    /* Update statistics */
    uint32_t now = get_time_ms();
    if (s_multi_pan.active_network == ZB_MULTI_PAN_NETWORK_PRIMARY) {
        s_multi_pan.primary_active_ms += now - s_multi_pan.current_slot_start_ms;
    } else if (s_multi_pan.active_network == ZB_MULTI_PAN_NETWORK_SECONDARY) {
        s_multi_pan.secondary_active_ms += now - s_multi_pan.current_slot_start_ms;
    }

    /* Ensure we're on primary network when stopped */
    if (s_multi_pan.active_network != ZB_MULTI_PAN_NETWORK_PRIMARY &&
        s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_PRIMARY]) {
        synchronize_with_stack(ZB_MULTI_PAN_NETWORK_PRIMARY);
        s_multi_pan.active_network = ZB_MULTI_PAN_NETWORK_PRIMARY;
    }

    s_multi_pan.state = ZB_MULTI_PAN_STATE_DISABLED;

    give_mutex();

    /* Publish final status */
    publish_status_mqtt();

    ESP_LOGI(TAG, "Multi-PAN stopped");
    return ESP_OK;
}

esp_err_t zb_multi_pan_add_network(
    zb_multi_pan_network_id_t network_id,
    uint16_t pan_id,
    uint8_t channel,
    const char *name)
{
    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        ESP_LOGE(TAG, "Invalid network ID: %d", network_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (channel < ZB_MULTI_PAN_CHANNEL_MIN || channel > ZB_MULTI_PAN_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid channel: %d (must be %d-%d)",
                 channel, ZB_MULTI_PAN_CHANNEL_MIN, ZB_MULTI_PAN_CHANNEL_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex(1000)) {
        return ESP_ERR_TIMEOUT;
    }

    zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[network_id];

    /* Initialize context */
    memset(ctx, 0, sizeof(zb_multi_pan_network_ctx_t));
    ctx->pan_id = pan_id;
    ctx->channel = channel;
    ctx->enabled = true;

    if (name != NULL) {
        strncpy(ctx->name, name, sizeof(ctx->name) - 1);
        ctx->name[sizeof(ctx->name) - 1] = '\0';
    } else {
        snprintf(ctx->name, sizeof(ctx->name), "Network_%d", network_id);
    }

    s_multi_pan.network_valid[network_id] = true;

    give_mutex();

    ESP_LOGI(TAG, "Added network %s: PAN=0x%04X, Channel=%d",
             ctx->name, pan_id, channel);

    return ESP_OK;
}

esp_err_t zb_multi_pan_remove_network(zb_multi_pan_network_id_t network_id)
{
    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_multi_pan.active_network == network_id &&
        s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING) {
        ESP_LOGE(TAG, "Cannot remove active network");
        return ESP_ERR_INVALID_STATE;
    }

    if (!take_mutex(1000)) {
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Removing network: %s",
             s_multi_pan.networks[network_id].name);

    memset(&s_multi_pan.networks[network_id], 0, sizeof(zb_multi_pan_network_ctx_t));
    s_multi_pan.network_valid[network_id] = false;

    give_mutex();

    return ESP_OK;
}

esp_err_t zb_multi_pan_switch(zb_multi_pan_network_id_t network_id)
{
    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_multi_pan.state != ZB_MULTI_PAN_STATE_RUNNING) {
        ESP_LOGW(TAG, "Multi-PAN not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_multi_pan.active_network == network_id) {
        ESP_LOGD(TAG, "Already on requested network");
        return ESP_OK;
    }

    return perform_network_switch(network_id, ZB_MULTI_PAN_SWITCH_MANUAL);
}

zb_multi_pan_network_id_t zb_multi_pan_get_active(void)
{
    if (!s_multi_pan.initialized ||
        s_multi_pan.state != ZB_MULTI_PAN_STATE_RUNNING) {
        return ZB_MULTI_PAN_NETWORK_NONE;
    }

    return s_multi_pan.active_network;
}

esp_err_t zb_multi_pan_get_network_ctx(
    zb_multi_pan_network_id_t network_id,
    zb_multi_pan_network_ctx_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(ctx, &s_multi_pan.networks[network_id], sizeof(zb_multi_pan_network_ctx_t));

    give_mutex();

    return ESP_OK;
}

esp_err_t zb_multi_pan_set_config(const zb_multi_pan_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate configuration */
    if (config->primary_channel < ZB_MULTI_PAN_CHANNEL_MIN ||
        config->primary_channel > ZB_MULTI_PAN_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid primary channel: %d", config->primary_channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->secondary_channel < ZB_MULTI_PAN_CHANNEL_MIN ||
        config->secondary_channel > ZB_MULTI_PAN_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid secondary channel: %d", config->secondary_channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->time_ratio < ZB_MULTI_PAN_MIN_RATIO ||
        config->time_ratio > ZB_MULTI_PAN_MAX_RATIO) {
        ESP_LOGE(TAG, "Invalid time ratio: %d%% (must be %d-%d%%)",
                 config->time_ratio, ZB_MULTI_PAN_MIN_RATIO, ZB_MULTI_PAN_MAX_RATIO);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->switch_interval_ms < ZB_MULTI_PAN_MIN_SWITCH_MS) {
        ESP_LOGE(TAG, "Switch interval too short: %lu ms (min %d)",
                 (unsigned long)config->switch_interval_ms, ZB_MULTI_PAN_MIN_SWITCH_MS);
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(&s_multi_pan.config, config, sizeof(zb_multi_pan_config_t));

    /* Update network channels if configured */
    if (s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_PRIMARY]) {
        s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_PRIMARY].channel = config->primary_channel;
    }
    if (s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_SECONDARY]) {
        s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_SECONDARY].channel = config->secondary_channel;
    }

    give_mutex();

    ESP_LOGI(TAG, "Configuration updated: Primary Ch%d, Secondary Ch%d, Ratio %d%%",
             config->primary_channel, config->secondary_channel, config->time_ratio);

    return ESP_OK;
}

esp_err_t zb_multi_pan_get_config(zb_multi_pan_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(config, &s_multi_pan.config, sizeof(zb_multi_pan_config_t));

    give_mutex();

    return ESP_OK;
}

esp_err_t zb_multi_pan_get_status(zb_multi_pan_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    status->state = s_multi_pan.state;
    status->active_network = s_multi_pan.active_network;
    status->uptime_ms = s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING ?
                        get_time_ms() - s_multi_pan.start_time_ms : 0;
    status->total_switches = s_multi_pan.total_switches;
    status->failed_switches = s_multi_pan.failed_switches;
    status->pending_transactions = s_multi_pan.pending_transactions;

    give_mutex();

    return ESP_OK;
}

esp_err_t zb_multi_pan_register_switch_callback(
    zb_multi_pan_switch_cb_t callback,
    void *user_data)
{
    s_multi_pan.switch_callback = callback;
    s_multi_pan.callback_user_data = user_data;
    return ESP_OK;
}

esp_err_t zb_multi_pan_set_network_key(
    zb_multi_pan_network_id_t network_id,
    const uint8_t key[ZB_MULTI_PAN_KEY_LEN])
{
    if (key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(s_multi_pan.networks[network_id].network_key, key, ZB_MULTI_PAN_KEY_LEN);
    s_multi_pan.networks[network_id].key_set = true;

    give_mutex();

    ESP_LOGI(TAG, "Network key set for %s",
             s_multi_pan.networks[network_id].name);

    return ESP_OK;
}

esp_err_t zb_multi_pan_set_ext_pan_id(
    zb_multi_pan_network_id_t network_id,
    const uint8_t ext_pan_id[ZB_MULTI_PAN_EXT_PAN_ID_LEN])
{
    if (ext_pan_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(s_multi_pan.networks[network_id].ext_pan_id, ext_pan_id,
           ZB_MULTI_PAN_EXT_PAN_ID_LEN);

    give_mutex();

    return ESP_OK;
}

esp_err_t zb_multi_pan_permit_join(
    zb_multi_pan_network_id_t network_id,
    uint8_t duration)
{
    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    s_multi_pan.networks[network_id].permit_join = (duration > 0);
    s_multi_pan.networks[network_id].permit_join_remaining = duration;

    /* If this is the active network, actually enable permit join */
    if (s_multi_pan.active_network == network_id &&
        s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING) {
        give_mutex();
        return zb_coordinator_permit_join(duration);
    }

    give_mutex();

    ESP_LOGI(TAG, "Permit join %s for %s (duration: %ds)",
             duration > 0 ? "enabled" : "disabled",
             s_multi_pan.networks[network_id].name,
             duration);

    return ESP_OK;
}

esp_err_t zb_multi_pan_add_device(
    zb_multi_pan_network_id_t network_id,
    uint16_t short_addr,
    const uint8_t ieee_addr[8])
{
    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[network_id];

    /* Check if device already exists */
    for (int i = 0; i < ctx->device_count; i++) {
        if (ctx->devices[i].short_addr == short_addr) {
            /* Update existing device */
            memcpy(ctx->devices[i].ieee_addr, ieee_addr, 8);
            ctx->devices[i].last_seen = get_time_ms();
            ctx->devices[i].online = true;
            give_mutex();
            return ESP_OK;
        }
    }

    /* Add new device */
    if (ctx->device_count >= ZB_MULTI_PAN_MAX_DEVICES) {
        give_mutex();
        ESP_LOGW(TAG, "Device table full for %s", ctx->name);
        return ESP_ERR_NO_MEM;
    }

    zb_multi_pan_device_t *dev = &ctx->devices[ctx->device_count];
    dev->short_addr = short_addr;
    memcpy(dev->ieee_addr, ieee_addr, 8);
    dev->online = true;
    dev->last_seen = get_time_ms();
    ctx->device_count++;

    give_mutex();

    ESP_LOGI(TAG, "Added device 0x%04X to %s (total: %d)",
             short_addr, ctx->name, ctx->device_count);

    return ESP_OK;
}

esp_err_t zb_multi_pan_remove_device(
    zb_multi_pan_network_id_t network_id,
    uint16_t short_addr)
{
    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!take_mutex(500)) {
        return ESP_ERR_TIMEOUT;
    }

    zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[network_id];

    for (int i = 0; i < ctx->device_count; i++) {
        if (ctx->devices[i].short_addr == short_addr) {
            /* Move last device to this position */
            if (i < ctx->device_count - 1) {
                memcpy(&ctx->devices[i], &ctx->devices[ctx->device_count - 1],
                       sizeof(zb_multi_pan_device_t));
            }
            ctx->device_count--;
            give_mutex();

            ESP_LOGI(TAG, "Removed device 0x%04X from %s", short_addr, ctx->name);
            return ESP_OK;
        }
    }

    give_mutex();
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_multi_pan_save_config(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!take_mutex(1000)) {
        nvs_close(handle);
        return ESP_ERR_TIMEOUT;
    }

    /* Save configuration */
    ret = nvs_set_blob(handle, NVS_KEY_CONFIG, &s_multi_pan.config,
                       sizeof(zb_multi_pan_config_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Save primary network context if valid */
    if (s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_PRIMARY]) {
        ret = nvs_set_blob(handle, NVS_KEY_PRIMARY_CTX,
                          &s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_PRIMARY],
                          sizeof(zb_multi_pan_network_ctx_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save primary context: %s", esp_err_to_name(ret));
            goto cleanup;
        }
    }

    /* Save secondary network context if valid */
    if (s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_SECONDARY]) {
        ret = nvs_set_blob(handle, NVS_KEY_SECONDARY_CTX,
                          &s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_SECONDARY],
                          sizeof(zb_multi_pan_network_ctx_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save secondary context: %s", esp_err_to_name(ret));
            goto cleanup;
        }
    }

    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    }

cleanup:
    give_mutex();
    nvs_close(handle);
    return ret;
}

esp_err_t zb_multi_pan_load_config(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!take_mutex(1000)) {
        nvs_close(handle);
        return ESP_ERR_TIMEOUT;
    }

    /* Load configuration */
    size_t len = sizeof(zb_multi_pan_config_t);
    ret = nvs_get_blob(handle, NVS_KEY_CONFIG, &s_multi_pan.config, &len);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to load config: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Load primary network context */
    len = sizeof(zb_multi_pan_network_ctx_t);
    ret = nvs_get_blob(handle, NVS_KEY_PRIMARY_CTX,
                       &s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_PRIMARY], &len);
    if (ret == ESP_OK) {
        s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_PRIMARY] = true;
        ESP_LOGI(TAG, "Loaded primary network: %s",
                 s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_PRIMARY].name);
    }

    /* Load secondary network context */
    len = sizeof(zb_multi_pan_network_ctx_t);
    ret = nvs_get_blob(handle, NVS_KEY_SECONDARY_CTX,
                       &s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_SECONDARY], &len);
    if (ret == ESP_OK) {
        s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_SECONDARY] = true;
        ESP_LOGI(TAG, "Loaded secondary network: %s",
                 s_multi_pan.networks[ZB_MULTI_PAN_NETWORK_SECONDARY].name);
    }

    ret = ESP_OK;

cleanup:
    give_mutex();
    nvs_close(handle);
    return ret;
}

esp_err_t zb_multi_pan_reset_config(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ESP_LOGW(TAG, "Resetting Multi-PAN configuration!");

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Configuration reset complete");
    }

    return ret;
}

bool zb_multi_pan_is_enabled(void)
{
#ifdef CONFIG_ZIGBEE_MULTI_PAN_ENABLED
    return true;
#else
    return false;
#endif
}

bool zb_multi_pan_is_running(void)
{
    return s_multi_pan.initialized &&
           s_multi_pan.state == ZB_MULTI_PAN_STATE_RUNNING;
}

const char *zb_multi_pan_state_to_str(zb_multi_pan_state_t state)
{
    switch (state) {
        case ZB_MULTI_PAN_STATE_DISABLED:     return "disabled";
        case ZB_MULTI_PAN_STATE_INITIALIZING: return "initializing";
        case ZB_MULTI_PAN_STATE_RUNNING:      return "running";
        case ZB_MULTI_PAN_STATE_SWITCHING:    return "switching";
        case ZB_MULTI_PAN_STATE_ERROR:        return "error";
        default:                               return "unknown";
    }
}

const char *zb_multi_pan_network_to_str(zb_multi_pan_network_id_t network_id)
{
    switch (network_id) {
        case ZB_MULTI_PAN_NETWORK_PRIMARY:    return "primary";
        case ZB_MULTI_PAN_NETWORK_SECONDARY:  return "secondary";
        case ZB_MULTI_PAN_NETWORK_NONE:       return "none";
        default:                               return "unknown";
    }
}

/* ==================== Internal Functions ==================== */

/**
 * @brief Timer callback for automatic network switching
 */
static void switch_timer_callback(TimerHandle_t timer)
{
    (void)timer;

    if (s_multi_pan.state != ZB_MULTI_PAN_STATE_RUNNING) {
        return;
    }

    if (!s_multi_pan.network_valid[ZB_MULTI_PAN_NETWORK_SECONDARY]) {
        return;
    }

    uint32_t now = get_time_ms();
    uint32_t slot_duration = now - s_multi_pan.current_slot_start_ms;

    /* Calculate time slot durations based on ratio */
    uint32_t primary_slot_ms = (s_multi_pan.config.switch_interval_ms *
                                s_multi_pan.config.time_ratio) / 100;
    uint32_t secondary_slot_ms = s_multi_pan.config.switch_interval_ms - primary_slot_ms;

    bool should_switch = false;
    zb_multi_pan_network_id_t target_network = s_multi_pan.active_network;

    if (s_multi_pan.active_network == ZB_MULTI_PAN_NETWORK_PRIMARY) {
        if (slot_duration >= primary_slot_ms) {
            should_switch = true;
            target_network = ZB_MULTI_PAN_NETWORK_SECONDARY;
        }
    } else if (s_multi_pan.active_network == ZB_MULTI_PAN_NETWORK_SECONDARY) {
        if (slot_duration >= secondary_slot_ms) {
            should_switch = true;
            target_network = ZB_MULTI_PAN_NETWORK_PRIMARY;
        }
    }

    if (should_switch) {
        /* Update statistics for current slot */
        if (s_multi_pan.active_network == ZB_MULTI_PAN_NETWORK_PRIMARY) {
            s_multi_pan.primary_active_ms += slot_duration;
        } else {
            s_multi_pan.secondary_active_ms += slot_duration;
        }

        /* Perform the switch */
        perform_network_switch(target_network, ZB_MULTI_PAN_SWITCH_TIMER);
    }
}

/**
 * @brief Perform network switch with state preservation
 */
static esp_err_t perform_network_switch(
    zb_multi_pan_network_id_t target,
    zb_multi_pan_switch_reason_t reason)
{
    if (target == s_multi_pan.active_network) {
        return ESP_OK;
    }

    if (!s_multi_pan.network_valid[target]) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGD(TAG, "Switching from %s to %s (reason: %d)",
             zb_multi_pan_network_to_str(s_multi_pan.active_network),
             zb_multi_pan_network_to_str(target),
             reason);

    zb_multi_pan_network_id_t from_network = s_multi_pan.active_network;

    /* Check for graceful switch */
    if (s_multi_pan.config.graceful_switch && s_multi_pan.pending_transactions > 0) {
        uint32_t wait_start = get_time_ms();
        while (s_multi_pan.pending_transactions > 0) {
            if (get_time_ms() - wait_start > s_multi_pan.config.switch_timeout_ms) {
                ESP_LOGW(TAG, "Graceful switch timeout - %lu transactions pending",
                         (unsigned long)s_multi_pan.pending_transactions);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(GW_TIMEOUT_MINIMAL_MS));
        }
    }

    s_multi_pan.state = ZB_MULTI_PAN_STATE_SWITCHING;

    /* Save current network state */
    if (from_network >= 0 && s_multi_pan.network_valid[from_network]) {
        save_network_context(from_network);
    }

    /* Synchronize with Zigbee stack for target network */
    esp_err_t ret = synchronize_with_stack(target);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to synchronize with target network");
        s_multi_pan.failed_switches++;
        s_multi_pan.state = ZB_MULTI_PAN_STATE_RUNNING;
        return ret;
    }

    /* Update state */
    s_multi_pan.active_network = target;
    s_multi_pan.current_slot_start_ms = get_time_ms();
    s_multi_pan.last_switch_time_ms = s_multi_pan.current_slot_start_ms;
    s_multi_pan.total_switches++;
    s_multi_pan.networks[target].switch_count++;
    s_multi_pan.networks[target].last_active = s_multi_pan.current_slot_start_ms;

    s_multi_pan.state = ZB_MULTI_PAN_STATE_RUNNING;

    /* Notify callback */
    if (s_multi_pan.switch_callback != NULL) {
        s_multi_pan.switch_callback(from_network, target, reason,
                                    s_multi_pan.callback_user_data);
    }

    /* Publish MQTT status on manual switches */
    if (reason == ZB_MULTI_PAN_SWITCH_MANUAL) {
        publish_status_mqtt();
    }

    ESP_LOGD(TAG, "Switch complete - now on %s (channel %d)",
             s_multi_pan.networks[target].name,
             s_multi_pan.networks[target].channel);

    return ESP_OK;
}

/**
 * @brief Save current network context state
 */
static esp_err_t save_network_context(zb_multi_pan_network_id_t network_id)
{
    if (network_id < 0 || network_id >= ZB_MULTI_PAN_MAX_NETWORKS) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[network_id];

    /* Update last active time */
    ctx->last_active = get_time_ms();

    /* Update active time statistics */
    uint32_t slot_time = ctx->last_active - s_multi_pan.current_slot_start_ms;
    ctx->active_time_ms += slot_time;

    ESP_LOGD(TAG, "Saved context for %s (active time: %lu ms)",
             ctx->name, (unsigned long)ctx->active_time_ms);

    return ESP_OK;
}

/**
 * @brief Load network context from storage
 */
static esp_err_t load_network_context(zb_multi_pan_network_id_t network_id)
{
    /* Context is already in memory, this is a placeholder for future
     * implementation that might need to load from NVS on switch */
    (void)network_id;
    return ESP_OK;
}

/**
 * @brief Synchronize with Zigbee stack for network switch
 *
 * This function handles the actual radio/stack reconfiguration.
 * Note: This is a simplified implementation. Real implementation would
 * need to interact with the ESP-Zigbee-SDK for channel switching.
 */
static esp_err_t synchronize_with_stack(zb_multi_pan_network_id_t network_id)
{
    if (!s_multi_pan.network_valid[network_id]) {
        return ESP_ERR_NOT_FOUND;
    }

    zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[network_id];

    ESP_LOGD(TAG, "Synchronizing stack with %s (PAN: 0x%04X, Channel: %d)",
             ctx->name, ctx->pan_id, ctx->channel);

    /*
     * TODO: Implement actual Zigbee stack synchronization
     *
     * This would involve:
     * 1. Suspend current network operations
     * 2. Save MAC/NWK layer state
     * 3. Reconfigure radio channel
     * 4. Update PAN ID in stack
     * 5. Load device binding tables for new network
     * 6. Resume network operations
     *
     * Example (pseudo-code for ESP-Zigbee-SDK v1.6.x - API-004):
     *   esp_zb_radio_set_channel(ctx->channel);
     *   esp_zb_set_pan_id(ctx->pan_id);
     *   esp_zb_nwk_set_extended_pan_id(ctx->ext_pan_id);
     *   if (ctx->key_set) {
     *       esp_zb_set_network_key(ctx->network_key);
     *   }
     *
     * For now, this is a placeholder that logs the operation.
     */

    /* Simulate switch delay */
    vTaskDelay(pdMS_TO_TICKS(GW_TIMEOUT_MINIMAL_MS));

    /* Restore permit join state for this network */
    if (ctx->permit_join && ctx->permit_join_remaining > 0) {
        /* Would call: zb_coordinator_permit_join(ctx->permit_join_remaining); */
        ESP_LOGD(TAG, "Restored permit join: %ds remaining", ctx->permit_join_remaining);
    }

    return ESP_OK;
}

/**
 * @brief Publish Multi-PAN status to MQTT
 */
static void publish_status_mqtt(void)
{
    if (!mqtt_client_is_connected()) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return;
    }

    /* Add status fields */
    cJSON_AddStringToObject(root, "state",
                            zb_multi_pan_state_to_str(s_multi_pan.state));
    cJSON_AddStringToObject(root, "active_network",
                            zb_multi_pan_network_to_str(s_multi_pan.active_network));
    cJSON_AddBoolToObject(root, "enabled", zb_multi_pan_is_enabled());

    /* Add configuration */
    cJSON *config = cJSON_CreateObject();
    if (config != NULL) {
        cJSON_AddNumberToObject(config, "primary_channel",
                                s_multi_pan.config.primary_channel);
        cJSON_AddNumberToObject(config, "secondary_channel",
                                s_multi_pan.config.secondary_channel);
        cJSON_AddNumberToObject(config, "time_ratio",
                                s_multi_pan.config.time_ratio);
        cJSON_AddNumberToObject(config, "switch_interval_ms",
                                s_multi_pan.config.switch_interval_ms);
        cJSON_AddItemToObject(root, "config", config);
    }

    /* Add statistics */
    cJSON *stats = cJSON_CreateObject();
    if (stats != NULL) {
        cJSON_AddNumberToObject(stats, "total_switches", s_multi_pan.total_switches);
        cJSON_AddNumberToObject(stats, "failed_switches", s_multi_pan.failed_switches);
        cJSON_AddNumberToObject(stats, "primary_active_ms", s_multi_pan.primary_active_ms);
        cJSON_AddNumberToObject(stats, "secondary_active_ms", s_multi_pan.secondary_active_ms);
        cJSON_AddItemToObject(root, "statistics", stats);
    }

    /* Add network information */
    cJSON *networks = cJSON_CreateArray();
    if (networks != NULL) {
        for (int i = 0; i < ZB_MULTI_PAN_MAX_NETWORKS; i++) {
            if (s_multi_pan.network_valid[i]) {
                zb_multi_pan_network_ctx_t *ctx = &s_multi_pan.networks[i];
                cJSON *net = cJSON_CreateObject();
                if (net != NULL) {
                    cJSON_AddStringToObject(net, "id", zb_multi_pan_network_to_str(i));
                    cJSON_AddStringToObject(net, "name", ctx->name);
                    char pan_str[7];
                    snprintf(pan_str, sizeof(pan_str), "0x%04X", ctx->pan_id);
                    cJSON_AddStringToObject(net, "pan_id", pan_str);
                    cJSON_AddNumberToObject(net, "channel", ctx->channel);
                    cJSON_AddNumberToObject(net, "device_count", ctx->device_count);
                    cJSON_AddBoolToObject(net, "permit_join", ctx->permit_join);
                    cJSON_AddBoolToObject(net, "active",
                                          i == s_multi_pan.active_network);
                    cJSON_AddItemToArray(networks, net);
                }
            }
        }
        cJSON_AddItemToObject(root, "networks", networks);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str != NULL) {
        mqtt_client_publish(MQTT_TOPIC_STATUS, json_str, strlen(json_str), 0, true);
        free(json_str);
    }
}

esp_err_t zb_multi_pan_test(void)
{
    ESP_LOGI(TAG, "Running Multi-PAN self-test...");

    esp_err_t ret;
    int passed = 0;
    int failed = 0;

    /* Test 1: Check enabled status */
    bool enabled = zb_multi_pan_is_enabled();
    ESP_LOGI(TAG, "Test 1: Enabled check = %s",
             enabled ? "ENABLED" : "DISABLED");
    passed++;

#ifdef CONFIG_ZIGBEE_MULTI_PAN_ENABLED
    /* Test 2: Initialize */
    ret = zb_multi_pan_init();
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Test 2: Init - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 2: Init - FAILED (%s)", esp_err_to_name(ret));
        failed++;
    }

    /* Test 3: Add primary network */
    ret = zb_multi_pan_add_network(ZB_MULTI_PAN_NETWORK_PRIMARY,
                                   0x1A62, 11, "Test_Primary");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Test 3: Add primary network - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 3: Add primary network - FAILED (%s)",
                 esp_err_to_name(ret));
        failed++;
    }

    /* Test 4: Add secondary network */
    ret = zb_multi_pan_add_network(ZB_MULTI_PAN_NETWORK_SECONDARY,
                                   0x2B73, 15, "Test_Secondary");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Test 4: Add secondary network - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 4: Add secondary network - FAILED (%s)",
                 esp_err_to_name(ret));
        failed++;
    }

    /* Test 5: Get config */
    zb_multi_pan_config_t config;
    ret = zb_multi_pan_get_config(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Test 5: Get config - PASSED (ratio=%d%%)", config.time_ratio);
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 5: Get config - FAILED");
        failed++;
    }

    /* Test 6: Set config */
    config.time_ratio = 75;
    ret = zb_multi_pan_set_config(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Test 6: Set config - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 6: Set config - FAILED");
        failed++;
    }

    /* Test 7: Get network context */
    zb_multi_pan_network_ctx_t ctx;
    ret = zb_multi_pan_get_network_ctx(ZB_MULTI_PAN_NETWORK_PRIMARY, &ctx);
    if (ret == ESP_OK && ctx.channel == ZB_DEFAULT_PRIMARY_CHANNEL) {
        ESP_LOGI(TAG, "Test 7: Get network context - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 7: Get network context - FAILED");
        failed++;
    }

    /* Test 8: State to string */
    const char *state_str = zb_multi_pan_state_to_str(ZB_MULTI_PAN_STATE_RUNNING);
    if (strcmp(state_str, "running") == 0) {
        ESP_LOGI(TAG, "Test 8: State to string - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 8: State to string - FAILED");
        failed++;
    }

    /* Test 9: Network to string */
    const char *net_str = zb_multi_pan_network_to_str(ZB_MULTI_PAN_NETWORK_PRIMARY);
    if (strcmp(net_str, "primary") == 0) {
        ESP_LOGI(TAG, "Test 9: Network to string - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 9: Network to string - FAILED");
        failed++;
    }

    /* Cleanup */
    zb_multi_pan_remove_network(ZB_MULTI_PAN_NETWORK_SECONDARY);
    zb_multi_pan_remove_network(ZB_MULTI_PAN_NETWORK_PRIMARY);
#else
    /* Test 2: Verify init returns not supported */
    ret = zb_multi_pan_init();
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "Test 2: Init returns NOT_SUPPORTED - PASSED");
        passed++;
    } else {
        ESP_LOGE(TAG, "Test 2: Init should return NOT_SUPPORTED");
        failed++;
    }
#endif

    ESP_LOGI(TAG, "Self-test complete: %d passed, %d failed", passed, failed);

    return (failed == 0) ? ESP_OK : ESP_FAIL;
}
