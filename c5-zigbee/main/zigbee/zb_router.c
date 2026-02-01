/**
 * @file zb_router.c
 * @brief Zigbee Router Implementation
 *
 * Implements Zigbee router functionality for mesh network extension.
 * Handles network discovery, joining, route maintenance, and reconnection.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_router.h"
#include "zb_network.h"
#include "zb_callbacks.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "core/compat_stubs.h"
#include "utils/json_utils.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ZB_ROUTER";

/* NVS namespace and keys for router configuration */
#define NVS_ROUTER_NAMESPACE    "zb_router"
#define NVS_KEY_CONFIG          "config"
#define NVS_KEY_JOINED_PAN      "joined_pan"
#define NVS_KEY_JOINED_CHANNEL  "joined_ch"

/* Router task configuration */
#define ZB_ROUTER_ENDPOINT          1
#define ZB_ROUTER_STACK_SIZE        (6 * 1024)
#define ZB_ROUTER_PRIORITY          5
#define ZB_ROUTER_SCAN_DURATION_MS  3000
#define ZB_ROUTER_REJOIN_DELAY_MS   5000
#define ZB_ROUTER_STATUS_INTERVAL   60000  /* Publish status every 60 seconds */

/* MQTT Topics for Router */
#define MQTT_TOPIC_ROUTER_STATE     "zigbee2mqtt/bridge/router/state"
#define MQTT_TOPIC_ROUTER_PARENT    "zigbee2mqtt/bridge/router/parent"
#define MQTT_TOPIC_ROUTER_NEIGHBORS "zigbee2mqtt/bridge/router/neighbors"
#define MQTT_TOPIC_ROUTER_STATS     "zigbee2mqtt/bridge/router/stats"

/* Router state */
static zb_router_state_t s_router_state = ZB_ROUTER_STATE_UNINITIALIZED;
static SemaphoreHandle_t s_router_mutex = NULL;
static TaskHandle_t s_router_task_handle = NULL;
static esp_timer_handle_t s_rejoin_timer = NULL;
static esp_timer_handle_t s_status_timer = NULL;

/* Router configuration */
static zb_router_config_t s_router_config;

/* Parent and network information */
static zb_router_parent_info_t s_parent_info;
static bool s_has_parent_info = false;

/* Neighbor table */
static zb_router_neighbor_t s_neighbors[ZB_ROUTER_MAX_NEIGHBORS];
static uint8_t s_neighbor_count = 0;

/* Statistics */
static zb_router_stats_t s_router_stats;
static uint32_t s_join_timestamp = 0;

/* Callbacks */
static zb_router_state_callback_t s_state_callback = NULL;
static zb_router_network_found_callback_t s_network_callback = NULL;

/* Scan results */
#define MAX_SCAN_RESULTS 8
static zb_router_network_t s_scan_results[MAX_SCAN_RESULTS];
static uint8_t s_scan_result_count = 0;
static bool s_scanning = false;

/* Rejoin tracking */
static uint8_t s_rejoin_attempts = 0;

/* Forward declarations */
static void set_router_state(zb_router_state_t state);
static esp_err_t configure_router_endpoint(void);
static void esp_zb_router_task(void *pvParameters);
static void rejoin_timer_callback(void *arg);
static void status_timer_callback(void *arg);
static esp_err_t save_config_to_nvs(void);
static esp_err_t load_config_from_nvs(void);
static void handle_network_steering_done(bool success);
static void handle_network_left(void);
static void update_parent_info(void);
static void update_neighbor_table(void);
static esp_err_t publish_router_state_mqtt(void);
static esp_err_t publish_parent_info_mqtt(void);
static esp_err_t publish_neighbors_mqtt(void);

/**
 * @brief Zigbee signal handler for router mode
 */
static esp_err_t zb_router_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);

#if CONFIG_ZIGBEE_DEVICE_TYPE_ROUTER
/**
 * @brief Zigbee application signal handler for router mode
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    esp_zb_app_signal_type_t sig_type = *signal->p_app_signal;
    esp_err_t err_status = signal->esp_err_status;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            /* Start network steering (join) */
            if (s_router_config.auto_join) {
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Device %s, starting network steering",
                         sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "first start" : "reboot");
                set_router_state(ZB_ROUTER_STATE_SEARCHING);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGW(TAG, "Device startup failed: 0x%x", err_status);
                set_router_state(ZB_ROUTER_STATE_ERROR);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Successfully joined network");
                set_router_state(ZB_ROUTER_STATE_JOINED);
                s_router_stats.join_count++;
                s_rejoin_attempts = 0;
                s_join_timestamp = (uint32_t)(esp_timer_get_time() / 1000000);

                /* Get network info using ESP-Zigbee-SDK v1.6.x API (API-004) */
                esp_zb_ieee_addr_t ext_pan_id;
                esp_zb_nwk_get_extended_pan_id(ext_pan_id);
                uint16_t pan_id = esp_zb_get_pan_id();
                uint16_t short_addr = esp_zb_get_short_address();

                ESP_LOGI(TAG, "Network: PAN=0x%04X, Short=0x%04X", pan_id, short_addr);

                /* Update parent info and start status publishing */
                update_parent_info();
                if (s_status_timer) {
                    esp_timer_start_periodic(s_status_timer, ZB_ROUTER_STATUS_INTERVAL * 1000);
                }

                /* Publish status */
                zb_router_publish_status();
            } else {
                ESP_LOGW(TAG, "Network steering failed: 0x%x", err_status);
                handle_network_steering_done(false);
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            ESP_LOGI(TAG, "Left network");
            handle_network_left();
            break;

        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            ESP_LOGD(TAG, "Permit join status changed");
            break;

        case ESP_ZB_NLME_STATUS_INDICATION: {
            /* Network layer status - check for parent loss */
            ESP_LOGD(TAG, "NLME status indication received");
            break;
        }

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            ESP_LOGD(TAG, "Production config ready");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled signal: %d, status: 0x%x", sig_type, err_status);
            break;
    }
}
#endif /* CONFIG_ZIGBEE_DEVICE_TYPE_ROUTER */

/**
 * @brief ZCL core action handler for router mode
 */
static esp_err_t zb_router_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ESP_LOGD(TAG, "Set attribute value callback");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled action callback: 0x%04x", callback_id);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    return ret;
}

esp_err_t zb_router_init(void)
{
    if (s_router_state != ZB_ROUTER_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Router already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee router...");

    /* Create router mutex */
    s_router_mutex = xSemaphoreCreateMutex();
    if (s_router_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create router mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Load configuration (from NVS or defaults) */
    esp_err_t ret = load_config_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No stored config, using defaults");
        zb_router_get_default_config(&s_router_config);
    }

    /* Initialize sub-modules */
    ret = zb_network_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network manager: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = zb_callbacks_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize callbacks: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Initialize Zigbee stack in router mode */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = ZB_DEFAULT_MAX_CHILDREN,  /* Maximum number of children */
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    /* Register action handler for ZCL callbacks */
    esp_zb_core_action_handler_register(zb_router_action_handler);

    /* Configure router endpoint */
    ret = configure_router_endpoint();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure router: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Configure channel mask */
    uint32_t channel_mask;
    if (s_router_config.preferred_channel > 0 &&
        s_router_config.preferred_channel >= ZB_CHANNEL_MIN &&
        s_router_config.preferred_channel <= ZB_CHANNEL_MAX) {
        channel_mask = (1 << s_router_config.preferred_channel);
    } else {
        channel_mask = ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK;
    }
    esp_zb_set_primary_network_channel_set(channel_mask);

    /* Create rejoin timer */
    const esp_timer_create_args_t rejoin_timer_args = {
        .callback = rejoin_timer_callback,
        .arg = NULL,
        .name = "router_rejoin"
    };
    ret = esp_timer_create(&rejoin_timer_args, &s_rejoin_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create rejoin timer: %s", esp_err_to_name(ret));
    }

    /* Create status timer */
    const esp_timer_create_args_t status_timer_args = {
        .callback = status_timer_callback,
        .arg = NULL,
        .name = "router_status"
    };
    ret = esp_timer_create(&status_timer_args, &s_status_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create status timer: %s", esp_err_to_name(ret));
    }

    /* Initialize statistics */
    memset(&s_router_stats, 0, sizeof(s_router_stats));

    set_router_state(ZB_ROUTER_STATE_INITIALIZED);
    ESP_LOGI(TAG, "Zigbee router initialized successfully");
    return ESP_OK;

cleanup:
    if (s_router_mutex != NULL) {
        vSemaphoreDelete(s_router_mutex);
        s_router_mutex = NULL;
    }
    return ret;
}

esp_err_t zb_router_start(void)
{
    if (s_router_state != ZB_ROUTER_STATE_INITIALIZED) {
        ESP_LOGE(TAG, "Router not initialized or already running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting Zigbee router...");

    /* Create router task */
    BaseType_t task_created = xTaskCreate(
        esp_zb_router_task,
        "zb_router",
        ZB_ROUTER_STACK_SIZE,
        NULL,
        ZB_ROUTER_PRIORITY,
        &s_router_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create router task");
        set_router_state(ZB_ROUTER_STATE_ERROR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zigbee router task started");
    return ESP_OK;
}

esp_err_t zb_router_stop(void)
{
    if (s_router_state == ZB_ROUTER_STATE_UNINITIALIZED ||
        s_router_state == ZB_ROUTER_STATE_STOPPED) {
        ESP_LOGW(TAG, "Router not running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping Zigbee router...");

    /* Stop timers */
    if (s_rejoin_timer) {
        esp_timer_stop(s_rejoin_timer);
    }
    if (s_status_timer) {
        esp_timer_stop(s_status_timer);
    }

    /* Leave network if joined */
    if (s_router_state == ZB_ROUTER_STATE_JOINED) {
        zb_router_leave_network();
    }

    /* Delete router task */
    if (s_router_task_handle != NULL) {
        vTaskDelete(s_router_task_handle);
        s_router_task_handle = NULL;
    }

    set_router_state(ZB_ROUTER_STATE_STOPPED);
    ESP_LOGI(TAG, "Zigbee router stopped");
    return ESP_OK;
}

zb_router_state_t zb_router_get_state(void)
{
    return s_router_state;
}

const char *zb_router_state_to_str(zb_router_state_t state)
{
    static const char *state_strings[] = {
        "uninitialized",
        "initialized",
        "searching",
        "joining",
        "joined",
        "lost",
        "error",
        "stopped"
    };

    if (state < sizeof(state_strings) / sizeof(state_strings[0])) {
        return state_strings[state];
    }
    return "unknown";
}

bool zb_router_is_joined(void)
{
    return (s_router_state == ZB_ROUTER_STATE_JOINED);
}

esp_err_t zb_router_get_parent_info(zb_router_parent_info_t *parent_info)
{
    if (parent_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!zb_router_is_joined() || !s_has_parent_info) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_router_mutex, portMAX_DELAY);
    memcpy(parent_info, &s_parent_info, sizeof(zb_router_parent_info_t));
    xSemaphoreGive(s_router_mutex);

    return ESP_OK;
}

esp_err_t zb_router_get_neighbors(zb_router_neighbor_t *neighbors, uint8_t *count)
{
    if (neighbors == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!zb_router_is_joined()) {
        *count = 0;
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_router_mutex, portMAX_DELAY);

    /* Update neighbor table */
    update_neighbor_table();

    uint8_t copy_count = (*count < s_neighbor_count) ? *count : s_neighbor_count;
    memcpy(neighbors, s_neighbors, copy_count * sizeof(zb_router_neighbor_t));
    *count = copy_count;

    xSemaphoreGive(s_router_mutex);

    return ESP_OK;
}

esp_err_t zb_router_get_stats(zb_router_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_router_mutex, portMAX_DELAY);
    memcpy(stats, &s_router_stats, sizeof(zb_router_stats_t));

    /* Calculate uptime if joined */
    if (zb_router_is_joined() && s_join_timestamp > 0) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000);
        stats->uptime_seconds = now - s_join_timestamp;
    }

    xSemaphoreGive(s_router_mutex);

    return ESP_OK;
}

esp_err_t zb_router_set_config(const zb_router_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_router_mutex, portMAX_DELAY);
    memcpy(&s_router_config, config, sizeof(zb_router_config_t));
    xSemaphoreGive(s_router_mutex);

    /* Save to NVS */
    save_config_to_nvs();

    ESP_LOGI(TAG, "Router configuration updated");
    return ESP_OK;
}

esp_err_t zb_router_get_config(zb_router_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_router_mutex, portMAX_DELAY);
    memcpy(config, &s_router_config, sizeof(zb_router_config_t));
    xSemaphoreGive(s_router_mutex);

    return ESP_OK;
}

esp_err_t zb_router_scan_networks(uint32_t channel_mask, uint32_t duration_ms)
{
    if (s_scanning) {
        ESP_LOGW(TAG, "Scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting network scan (mask=0x%08lX, duration=%lu ms)",
             (unsigned long)channel_mask, (unsigned long)duration_ms);

    s_scanning = true;
    s_scan_result_count = 0;

    /* Set channel mask for scan */
    if (channel_mask == 0) {
        channel_mask = ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK;
    }
    esp_zb_set_primary_network_channel_set(channel_mask);

    /* Start network steering which includes scanning */
    set_router_state(ZB_ROUTER_STATE_SEARCHING);
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

    return ESP_OK;
}

esp_err_t zb_router_join_network(uint16_t pan_id, uint8_t channel)
{
    if (s_router_state != ZB_ROUTER_STATE_INITIALIZED &&
        s_router_state != ZB_ROUTER_STATE_SEARCHING &&
        s_router_state != ZB_ROUTER_STATE_LOST) {
        ESP_LOGE(TAG, "Cannot join from current state: %s",
                 zb_router_state_to_str(s_router_state));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Joining network: PAN=0x%04X, Channel=%d", pan_id, channel);

    /* Set specific channel */
    uint32_t channel_mask = (1 << channel);
    esp_zb_set_primary_network_channel_set(channel_mask);

    /* Store preferred PAN for steering */
    s_router_config.preferred_pan_id = pan_id;
    s_router_config.preferred_channel = channel;

    set_router_state(ZB_ROUTER_STATE_JOINING);
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

    return ESP_OK;
}

esp_err_t zb_router_leave_network(void)
{
    if (!zb_router_is_joined()) {
        ESP_LOGW(TAG, "Not currently joined to a network");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Leaving network...");

    /* Send leave request */
    esp_zb_zdo_mgmt_leave_req_param_t leave_req = {
        .dst_nwk_addr = esp_zb_get_short_address(),
        .remove_children = false,
        .rejoin = false
    };
    memset(leave_req.device_address, 0, sizeof(leave_req.device_address));

    esp_zb_zdo_device_leave_req(&leave_req, NULL, NULL);

    return ESP_OK;
}

esp_err_t zb_router_rejoin(bool secured)
{
    if (s_router_state != ZB_ROUTER_STATE_LOST &&
        s_router_state != ZB_ROUTER_STATE_JOINED) {
        ESP_LOGW(TAG, "Cannot rejoin from current state");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initiating %s rejoin", secured ? "secured" : "unsecured");

    s_router_stats.rejoin_count++;
    s_rejoin_attempts++;

    set_router_state(ZB_ROUTER_STATE_JOINING);
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

    return ESP_OK;
}

esp_err_t zb_router_register_state_callback(zb_router_state_callback_t callback)
{
    s_state_callback = callback;
    return ESP_OK;
}

esp_err_t zb_router_register_network_callback(zb_router_network_found_callback_t callback)
{
    s_network_callback = callback;
    return ESP_OK;
}

esp_err_t zb_router_publish_status(void)
{
    esp_err_t ret;

    ret = publish_router_state_mqtt();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish router state");
    }

    if (zb_router_is_joined()) {
        ret = publish_parent_info_mqtt();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to publish parent info");
        }

        ret = publish_neighbors_mqtt();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to publish neighbors");
        }
    }

    return ESP_OK;
}

SemaphoreHandle_t zb_router_get_mutex(void)
{
    return s_router_mutex;
}

esp_err_t zb_router_self_test(void)
{
    ESP_LOGI(TAG, "Running router self-test...");

    /* Check initialization */
    if (s_router_state == ZB_ROUTER_STATE_UNINITIALIZED) {
        ESP_LOGE(TAG, "Router not initialized");
        return ESP_FAIL;
    }

    /* Check mutex */
    if (s_router_mutex == NULL) {
        ESP_LOGE(TAG, "Router mutex not created");
        return ESP_FAIL;
    }

    /* Test config get/set */
    zb_router_config_t test_config;
    if (zb_router_get_config(&test_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Router self-test PASSED");
    return ESP_OK;
}

esp_err_t zb_router_get_default_config(zb_router_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(config, 0, sizeof(zb_router_config_t));

#ifdef CONFIG_ZIGBEE_ROUTER_AUTO_JOIN
    config->auto_join = CONFIG_ZIGBEE_ROUTER_AUTO_JOIN;
#else
    config->auto_join = true;
#endif

#ifdef CONFIG_ZIGBEE_PAN_ID
    config->preferred_pan_id = CONFIG_ZIGBEE_PAN_ID;
#else
    config->preferred_pan_id = 0xFFFF;  /* Any PAN */
#endif

#ifdef CONFIG_ZIGBEE_CHANNEL
    config->preferred_channel = CONFIG_ZIGBEE_CHANNEL;
#else
    config->preferred_channel = 0;  /* Any channel */
#endif

    config->scan_duration_ms = ZB_ROUTER_SCAN_DURATION_MS;
    config->rejoin_interval_ms = ZB_ROUTER_REJOIN_DELAY_MS;
    config->max_rejoin_attempts = 0;  /* Infinite */
    config->route_discovery_enabled = true;

#ifdef CONFIG_ZIGBEE_MAX_CHILDREN
    config->max_children = CONFIG_ZIGBEE_MAX_CHILDREN;
#else
    config->max_children = ZB_DEFAULT_MAX_CHILDREN;
#endif

    snprintf(config->friendly_name, sizeof(config->friendly_name),
             "ESP32-C5 Router");

    return ESP_OK;
}

/* Internal functions */

static void set_router_state(zb_router_state_t state)
{
    zb_router_state_t old_state = s_router_state;
    s_router_state = state;

    ESP_LOGI(TAG, "State: %s -> %s",
             zb_router_state_to_str(old_state),
             zb_router_state_to_str(state));

    /* Notify callback */
    if (s_state_callback != NULL && old_state != state) {
        s_state_callback(old_state, state);
    }

    /* Publish state change to MQTT */
    if (mqtt_client_is_connected()) {
        publish_router_state_mqtt();
    }
}

static esp_err_t configure_router_endpoint(void)
{
    ESP_LOGI(TAG, "Configuring router endpoint...");

    /* Create Basic cluster (mandatory) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);

    /* Add manufacturer name */
    static char manufacturer[] = "Espressif";
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
                                   ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                   manufacturer);

    /* Add model identifier */
    static char model[] = "ESP32-C5-Router";
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
                                   ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                   model);

    /* Create Identify cluster (mandatory) */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);

    /* Create cluster list */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster,
                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Create endpoint list */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    /* Create endpoint configuration */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ZB_ROUTER_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0
    };

    /* Add endpoint to list */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    /* Register device */
    esp_zb_device_register(esp_zb_ep_list);

    ESP_LOGI(TAG, "Router endpoint configured (EP=%d)", ZB_ROUTER_ENDPOINT);
    return ESP_OK;
}

static void esp_zb_router_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Zigbee router task started");

    /* Get IEEE address for logging */
    esp_zb_ieee_addr_t ieee_addr;
    esp_read_mac(ieee_addr, ESP_MAC_IEEE802154);

    ESP_LOGI(TAG, "IEEE Address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));

    ESP_LOGI(TAG, "Zigbee router running, searching for networks...");
    set_router_state(ZB_ROUTER_STATE_SEARCHING);

    /* Main event loop - esp_zb_stack_main_loop() blocks forever */
    esp_zb_stack_main_loop();

    ESP_LOGW(TAG, "Zigbee router task exiting");
    set_router_state(ZB_ROUTER_STATE_STOPPED);
    vTaskDelete(NULL);
}

static void rejoin_timer_callback(void *arg)
{
    ESP_LOGI(TAG, "Rejoin timer fired, attempting rejoin...");

    if (s_router_config.max_rejoin_attempts > 0 &&
        s_rejoin_attempts >= s_router_config.max_rejoin_attempts) {
        ESP_LOGW(TAG, "Max rejoin attempts (%d) reached", s_router_config.max_rejoin_attempts);
        set_router_state(ZB_ROUTER_STATE_ERROR);
        return;
    }

    zb_router_rejoin(true);
}

static void status_timer_callback(void *arg)
{
    if (zb_router_is_joined()) {
        /* Update info and publish */
        update_parent_info();
        update_neighbor_table();
        zb_router_publish_status();

        /* Update uptime in stats */
        if (s_join_timestamp > 0) {
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000000);
            s_router_stats.uptime_seconds = now - s_join_timestamp;
        }
    }
}

static void handle_network_steering_done(bool success)
{
    s_scanning = false;

    if (!success) {
        ESP_LOGW(TAG, "Network steering failed");

        if (s_router_config.auto_join) {
            /* Schedule rejoin attempt */
            set_router_state(ZB_ROUTER_STATE_LOST);
            s_router_stats.network_lost_count++;

            if (s_rejoin_timer) {
                esp_timer_start_once(s_rejoin_timer, s_router_config.rejoin_interval_ms * 1000);
                ESP_LOGI(TAG, "Scheduled rejoin in %lu ms",
                         (unsigned long)s_router_config.rejoin_interval_ms);
            }
        } else {
            set_router_state(ZB_ROUTER_STATE_INITIALIZED);
        }
    }
}

static void handle_network_left(void)
{
    s_has_parent_info = false;
    memset(&s_parent_info, 0, sizeof(s_parent_info));
    s_neighbor_count = 0;

    /* Stop status timer */
    if (s_status_timer) {
        esp_timer_stop(s_status_timer);
    }

    if (s_router_config.auto_join) {
        set_router_state(ZB_ROUTER_STATE_LOST);

        /* Schedule rejoin */
        if (s_rejoin_timer) {
            esp_timer_start_once(s_rejoin_timer, s_router_config.rejoin_interval_ms * 1000);
        }
    } else {
        set_router_state(ZB_ROUTER_STATE_INITIALIZED);
    }

    /* Publish offline status */
    if (mqtt_client_is_connected()) {
        publish_router_state_mqtt();
    }
}

static void update_parent_info(void)
{
    xSemaphoreTake(s_router_mutex, portMAX_DELAY);

    /* Note: esp_zb_get_parent_short_address() and esp_zb_get_network_depth()
     * are not available in SDK v1.6.x. Using default values.
     * Parent info can be obtained from neighbor table in future implementation. */
    s_parent_info.short_addr = 0x0000;  /* Assume parent is coordinator */
    s_parent_info.is_coordinator = true;
    s_parent_info.depth = 1;  /* Default depth */

    s_parent_info.last_seen = (uint32_t)(esp_timer_get_time() / 1000000);
    s_has_parent_info = true;

    xSemaphoreGive(s_router_mutex);

    ESP_LOGI(TAG, "Parent: 0x%04X (depth=%d, coordinator=%s)",
             s_parent_info.short_addr, s_parent_info.depth,
             s_parent_info.is_coordinator ? "yes" : "no");
}

static void update_neighbor_table(void)
{
    /* Note: In a full implementation, this would query the NWK layer
     * neighbor table. For now, we maintain a simplified version. */

    /* Get parent as first neighbor */
    if (s_has_parent_info && s_neighbor_count == 0) {
        s_neighbors[0].short_addr = s_parent_info.short_addr;
        memcpy(s_neighbors[0].ieee_addr, s_parent_info.ieee_addr, 8);
        s_neighbors[0].device_type = s_parent_info.is_coordinator ? 0 : 1;
        s_neighbors[0].relationship = 0;  /* Parent */
        s_neighbors[0].rx_on_idle = true;
        s_neighbors[0].lqi = s_parent_info.lqi;
        s_neighbors[0].rssi = s_parent_info.rssi;
        s_neighbors[0].last_seen = s_parent_info.last_seen;
        s_neighbor_count = 1;
    }
}

static esp_err_t save_config_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_ROUTER_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_CONFIG, &s_router_config, sizeof(s_router_config));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(ret));
    } else {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Router config saved to NVS");
    }

    nvs_close(handle);
    return ret;
}

static esp_err_t load_config_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_ROUTER_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t size = sizeof(s_router_config);
    ret = nvs_get_blob(handle, NVS_KEY_CONFIG, &s_router_config, &size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Router config loaded from NVS");
    }

    nvs_close(handle);
    return ret;
}

static esp_err_t publish_router_state_mqtt(void)
{
    if (!mqtt_client_is_connected()) {
        return ESP_FAIL;
    }

    char json_buf[256];
    int len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"state\":\"%s\","
        "\"joined\":%s,"
        "\"short_address\":\"0x%04X\","
        "\"network_depth\":%d,"
        "\"rejoin_attempts\":%d"
        "}",
        zb_router_state_to_str(s_router_state),
        zb_router_is_joined() ? "true" : "false",
        zb_router_is_joined() ? esp_zb_get_short_address() : 0,
        zb_router_is_joined() ? s_parent_info.depth : 0,
        s_rejoin_attempts
    );

    if (len < 0 || len >= (int)sizeof(json_buf)) {
        return ESP_ERR_NO_MEM;
    }

    return mqtt_client_publish(MQTT_TOPIC_ROUTER_STATE, json_buf, 0, 1, false);
}

static esp_err_t publish_parent_info_mqtt(void)
{
    if (!mqtt_client_is_connected() || !s_has_parent_info) {
        return ESP_FAIL;
    }

    char json_buf[256];
    int len = snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"short_address\":\"0x%04X\","
        "\"ieee_address\":\"%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"is_coordinator\":%s,"
        "\"depth\":%d,"
        "\"lqi\":%d,"
        "\"rssi\":%d"
        "}",
        s_parent_info.short_addr,
        s_parent_info.ieee_addr[7], s_parent_info.ieee_addr[6],
        s_parent_info.ieee_addr[5], s_parent_info.ieee_addr[4],
        s_parent_info.ieee_addr[3], s_parent_info.ieee_addr[2],
        s_parent_info.ieee_addr[1], s_parent_info.ieee_addr[0],
        s_parent_info.is_coordinator ? "true" : "false",
        s_parent_info.depth,
        s_parent_info.lqi,
        s_parent_info.rssi
    );

    if (len < 0 || len >= (int)sizeof(json_buf)) {
        return ESP_ERR_NO_MEM;
    }

    return mqtt_client_publish(MQTT_TOPIC_ROUTER_PARENT, json_buf, 0, 1, false);
}

static esp_err_t publish_neighbors_mqtt(void)
{
    if (!mqtt_client_is_connected()) {
        return ESP_FAIL;
    }

    /* Build JSON array of neighbors */
    char json_buf[512];
    int offset = 0;

    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, "[");

    for (uint8_t i = 0; i < s_neighbor_count && i < ZB_ROUTER_MAX_NEIGHBORS; i++) {
        if (i > 0) {
            offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, ",");
        }

        const char *rel_str = "unknown";
        switch (s_neighbors[i].relationship) {
            case 0: rel_str = "parent"; break;
            case 1: rel_str = "child"; break;
            case 2: rel_str = "sibling"; break;
        }

        const char *type_str = "unknown";
        switch (s_neighbors[i].device_type) {
            case 0: type_str = "coordinator"; break;
            case 1: type_str = "router"; break;
            case 2: type_str = "end_device"; break;
        }

        offset += snprintf(json_buf + offset, sizeof(json_buf) - offset,
            "{"
            "\"short_address\":\"0x%04X\","
            "\"type\":\"%s\","
            "\"relationship\":\"%s\","
            "\"lqi\":%d,"
            "\"rssi\":%d"
            "}",
            s_neighbors[i].short_addr,
            type_str,
            rel_str,
            s_neighbors[i].lqi,
            s_neighbors[i].rssi
        );

        if (offset >= (int)sizeof(json_buf) - 10) {
            break;  /* Buffer almost full */
        }
    }

    offset += snprintf(json_buf + offset, sizeof(json_buf) - offset, "]");

    return mqtt_client_publish(MQTT_TOPIC_ROUTER_NEIGHBORS, json_buf, 0, 1, false);
}
