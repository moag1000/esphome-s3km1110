/**
 * @file zb_coordinator.c
 * @brief Zigbee Coordinator Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_coordinator.h"
#include "zb_network.h"
#include "zb_device_handler.h"
#include "zb_callbacks.h"
#include "zb_interview.h"
#include "zb_availability.h"
#include "zb_install_codes.h"
#include "zb_alarms.h"
#include "zb_demand_response.h"
#include "zb_tuya.h"
#include "zigbee/zb_diagnostics.h"
#include "zigbee/zb_time_server.h"
#include "zigbee/zb_poll_control.h"
#include "core/compat_stubs.h"
#include "utils/version.h"
#include "utils/freertos_helpers.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <string.h>
#include <inttypes.h>

#if CONFIG_BT_SCANNER_ENABLED
#endif

/* ============================================================================
 * SDK Version Checks (API-016)
 * ============================================================================
 * Compile-time version validation for ESP-Zigbee-SDK compatibility.
 * Ensures minimum required SDK version and warns about recommended versions.
 */
#include "esp_zigbee_core.h"

/* ESP-Zigbee-SDK version macro fallbacks for older SDK versions
 * Some SDK versions don't define ESP_ZB_VERSION_MAJOR etc., so we provide
 * default values that indicate "version unknown but presumed compatible" */
#if !defined(ESP_ZB_VERSION_MAJOR)
#define ESP_ZB_VERSION_MAJOR    1
#define ESP_ZB_VERSION_MINOR    6
#define ESP_ZB_VERSION_PATCH    0
#define ESP_ZB_VERSION_UNDEFINED 1
#endif

/* Minimum required version: v1.6.0 (only check if version macros are defined) */
#if !defined(ESP_ZB_VERSION_UNDEFINED)
#if ESP_ZB_VERSION_MAJOR < 1
#error "ESP-Zigbee-SDK v1.0.0 or higher required"
#endif

#if ESP_ZB_VERSION_MAJOR == 1 && ESP_ZB_VERSION_MINOR < 6
#error "ESP-Zigbee-SDK v1.6.0 or higher required for this firmware"
#endif

/* Recommended version warning: v1.6.8+ for full feature support */
#if ESP_ZB_VERSION_MAJOR == 1 && ESP_ZB_VERSION_MINOR == 6 && defined(ESP_ZB_VERSION_PATCH)
#if ESP_ZB_VERSION_PATCH < 8
#warning "ESP-Zigbee-SDK v1.6.8+ recommended for full feature support (key rotation, power descriptors)"
#endif
#endif
#endif /* !ESP_ZB_VERSION_UNDEFINED */

static const char *TAG = "ZB_COORD";

/* Coordinator state */
static zb_coordinator_state_t s_coordinator_state = ZB_COORD_STATE_UNINITIALIZED;
static SemaphoreHandle_t s_coordinator_mutex = NULL;
static TaskHandle_t s_coordinator_task_handle = NULL;

/* Permit join state and timer */
static zb_permit_join_state_t s_permit_join_state = {0};
static esp_timer_handle_t s_permit_join_timer = NULL;
static esp_timer_handle_t s_permit_join_update_timer = NULL;

/* BLE coexistence: tracks whether we paused BLE for permit_join */
#if CONFIG_BT_SCANNER_ENABLED
static bool s_ble_paused_for_join = false;
#endif

/* Coordinator statistics for health check */
static int64_t s_coordinator_start_time = 0;
static uint32_t s_message_count_tx = 0;
static uint32_t s_message_count_rx = 0;
static int8_t s_last_rssi = 0;
static uint8_t s_last_lqi = 0;

/* API-009: Enhanced signal handler statistics */
static uint32_t s_route_error_count = 0;     /**< NLME route errors/discovery failures */
static uint32_t s_tc_rejoin_count = 0;       /**< Trust Center rejoin completions */

/* Zigbee stack configuration */
#define ZB_COORDINATOR_ENDPOINT     1
#define ZB_COORDINATOR_STACK_SIZE   (6 * 1024)
#define ZB_COORDINATOR_PRIORITY     5

/* Forward declarations */
static void set_coordinator_state(zb_coordinator_state_t state);
static esp_err_t configure_coordinator(void);
static void esp_zb_task(void *pvParameters);
static void permit_join_timer_callback(void *arg);
static void permit_join_update_callback(void *arg);
static esp_err_t start_permit_join_timers(uint8_t duration);
static void stop_permit_join_timers(void);
static void update_permit_join_remaining_time(void);

esp_err_t zb_coordinator_init(void)
{
    if (s_coordinator_state != ZB_COORD_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Coordinator already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee coordinator...");

    /* Log SDK version at runtime (API-016) */
    ESP_LOGI(TAG, "ESP-Zigbee-SDK version: %d.%d.%d",
             ESP_ZB_VERSION_MAJOR, ESP_ZB_VERSION_MINOR,
#ifdef ESP_ZB_VERSION_PATCH
             ESP_ZB_VERSION_PATCH
#else
             0
#endif
    );

    /* Create coordinator mutex */
    s_coordinator_mutex = xSemaphoreCreateMutex();
    if (s_coordinator_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create coordinator mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize sub-modules */
    esp_err_t ret;

    ret = zb_network_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network manager: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = zb_device_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize device handler: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Load persisted devices from NVS */
    size_t loaded_devices = zb_device_load_all_from_nvs();
    if (loaded_devices > 0) {
        ESP_LOGI(TAG, "Restored %zu devices from NVS persistence", loaded_devices);
    }

    ret = zb_callbacks_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize callbacks: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = zb_interview_init(NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize interview module: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without interview support */
    }

    esp_err_t avail_ret = zb_availability_init(NULL);
    if (avail_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize availability module: %s", esp_err_to_name(avail_ret));
        /* Non-fatal - continue without availability tracking */
    }

    /* Register NVS-restored devices with availability tracker */
    if (loaded_devices > 0 && avail_ret == ESP_OK) {
        size_t count = zb_device_get_count();
        for (size_t i = 0; i < count; i++) {
            zb_device_t dev;
            if (zb_device_copy_by_index(i, &dev) == ESP_OK &&
                dev.short_addr != 0 && dev.short_addr != 0xFFFF) {
                /* Determine power type from stored power source info */
                zb_availability_power_type_t power = ZB_AVAIL_POWER_UNKNOWN;
                if (dev.power_info.power_info_valid) {
                    if (dev.power_info.current_power_source & 0x04) {  /* Bit 2 = battery */
                        power = ZB_AVAIL_POWER_BATTERY;
                    } else if (dev.power_info.current_power_source & 0x01) {  /* Bit 0 = mains */
                        power = ZB_AVAIL_POWER_MAINS;
                    }
                }
                zb_availability_add_device(dev.short_addr, power);
                ESP_LOGI(TAG, "Registered restored device 0x%04X in availability tracker",
                         dev.short_addr);
            }
        }
    }

    ret = zb_install_codes_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize install codes: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without install code support */
    } else {
        ESP_LOGI(TAG, "Install codes module initialized (%d entries loaded)",
                 zb_install_codes_count());
    }

    ret = zb_diagnostics_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize diagnostics: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without diagnostics support */
    }

    ret = zb_time_server_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize time server: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without time server support */
    }

    ret = zb_poll_control_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize poll control: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without poll control support */
    }

    ret = zb_alarms_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize alarms module: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without alarms support */
    }

    ret = zb_demand_response_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize demand response: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without demand response support */
    }

    ret = zb_tuya_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize Tuya module: %s", esp_err_to_name(ret));
        /* Non-fatal - continue without Tuya support */
    }

    /* Configure network parameters before init (ESP-Zigbee-SDK v1.6.x+) */
    esp_zb_overall_network_size_set(ZB_COORDINATOR_MAX_NETWORK_DEVICES);
    esp_zb_io_buffer_size_set(ZB_COORDINATOR_IO_BUFFER_SIZE);
    esp_zb_aps_src_binding_table_size_set(ZB_COORDINATOR_BINDING_TABLE_SIZE);
    esp_zb_aps_dst_binding_table_size_set(ZB_COORDINATOR_BINDING_TABLE_SIZE);
    esp_zb_zcl_scenes_table_set_size(ZB_COORDINATOR_SCENE_TABLE_SIZE);

    ESP_LOGI(TAG, "Network configured: max_devices=%d, io_buffer=%d, bindings=%d/%d, scenes=%d",
             ZB_COORDINATOR_MAX_NETWORK_DEVICES, ZB_COORDINATOR_IO_BUFFER_SIZE,
             ZB_COORDINATOR_BINDING_TABLE_SIZE, ZB_COORDINATOR_BINDING_TABLE_SIZE,
             ZB_COORDINATOR_SCENE_TABLE_SIZE);

    /* Initialize Zigbee stack with coordinator configuration */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = ZB_COORDINATOR_MAX_CHILDREN,
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    /* Configure coordinator endpoint and clusters */
    ret = configure_coordinator();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure coordinator: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Set signal handler */
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    /* Create permit join expiry timer */
    esp_timer_create_args_t timer_args = {
        .callback = permit_join_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pj_expire"
    };
    ret = esp_timer_create(&timer_args, &s_permit_join_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create permit join timer: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Create permit join update timer (periodic) */
    esp_timer_create_args_t update_timer_args = {
        .callback = permit_join_update_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pj_update"
    };
    ret = esp_timer_create(&update_timer_args, &s_permit_join_update_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create permit join update timer: %s", esp_err_to_name(ret));
        esp_timer_delete(s_permit_join_timer);
        s_permit_join_timer = NULL;
        goto cleanup;
    }

    /* Initialize permit join state */
    memset(&s_permit_join_state, 0, sizeof(s_permit_join_state));

    set_coordinator_state(ZB_COORD_STATE_INITIALIZED);
    ESP_LOGI(TAG, "Zigbee coordinator initialized successfully");
    return ESP_OK;

cleanup:
    if (s_coordinator_mutex != NULL) {
        vSemaphoreDelete(s_coordinator_mutex);
        s_coordinator_mutex = NULL;
    }
    return ret;
}

esp_err_t zb_coordinator_start(void)
{
    if (s_coordinator_state != ZB_COORD_STATE_INITIALIZED) {
        ESP_LOGE(TAG, "Coordinator not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting Zigbee coordinator...");

    set_coordinator_state(ZB_COORD_STATE_STARTING);

    /* Create coordinator task (must use internal RAM - Zigbee stack accesses
     * DMA and radio hardware that requires internal SRAM stack) */
    BaseType_t task_created = xTaskCreate(
        esp_zb_task,
        "zb_coord",
        ZB_COORDINATOR_STACK_SIZE,
        NULL,
        ZB_COORDINATOR_PRIORITY,
        &s_coordinator_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create coordinator task");
        set_coordinator_state(ZB_COORD_STATE_ERROR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zigbee coordinator task started");
    return ESP_OK;
}

esp_err_t zb_coordinator_stop(void)
{
    if (s_coordinator_state != ZB_COORD_STATE_RUNNING) {
        ESP_LOGW(TAG, "Coordinator not running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping Zigbee coordinator...");

    /* Publish bridge event: network_stopped */
    bridge_event_network_stopped();

    /* Delete coordinator task */
    if (s_coordinator_task_handle != NULL) {
        vTaskDelete(s_coordinator_task_handle);
        s_coordinator_task_handle = NULL;
    }

    set_coordinator_state(ZB_COORD_STATE_STOPPED);
    ESP_LOGI(TAG, "Zigbee coordinator stopped");
    return ESP_OK;
}

bool zb_coordinator_is_running(void)
{
    /* Thread-safe state check */
    if (s_coordinator_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);
    bool running = (s_coordinator_state == ZB_COORD_STATE_RUNNING);
    xSemaphoreGive(s_coordinator_mutex);
    return running;
}

zb_coordinator_state_t zb_coordinator_get_state(void)
{
    /* Thread-safe state retrieval */
    if (s_coordinator_mutex == NULL) {
        return ZB_COORD_STATE_UNINITIALIZED;
    }
    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);
    zb_coordinator_state_t state = s_coordinator_state;
    xSemaphoreGive(s_coordinator_mutex);
    return state;
}

esp_err_t zb_coordinator_permit_join(uint8_t duration)
{
    /* Delegate to the extended function with default options */
    zb_permit_join_options_t options = {
        .duration = duration,
        .has_target_device = false
    };
    memset(options.target_ieee_addr, 0, sizeof(options.target_ieee_addr));

    return zb_coordinator_permit_join_with_options(&options);
}

esp_err_t zb_coordinator_permit_join_with_options(const zb_permit_join_options_t *options)
{
    if (options == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!zb_coordinator_is_running()) {
        ESP_LOGE(TAG, "Coordinator not running");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t duration = options->duration;

    if (options->has_target_device) {
        ESP_LOGI(TAG, "Opening network for specific device joining (duration: %d seconds)", duration);
        ESP_LOGI(TAG, "Target IEEE: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 options->target_ieee_addr[7], options->target_ieee_addr[6],
                 options->target_ieee_addr[5], options->target_ieee_addr[4],
                 options->target_ieee_addr[3], options->target_ieee_addr[2],
                 options->target_ieee_addr[1], options->target_ieee_addr[0]);
    } else {
        ESP_LOGI(TAG, "Opening network for device joining (duration: %d seconds)", duration);
    }

    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);

    /* Stop any existing timers */
    stop_permit_join_timers();

    esp_err_t ret;

    if (duration == 0) {
        /* Close network */
        esp_zb_lock_acquire(portMAX_DELAY);
        ret = esp_zb_bdb_open_network(0);
        esp_zb_lock_release();
        if (ret == ESP_OK) {
            s_permit_join_state.enabled = false;
            s_permit_join_state.remaining_time = 0;
            s_permit_join_state.initial_duration = 0;
            s_permit_join_state.has_target_device = false;

            /* Resume BLE scanning if we paused it */
#if CONFIG_BT_SCANNER_ENABLED
            if (s_ble_paused_for_join) {
                s_ble_paused_for_join = false;
                esp_err_t ble_ret = ble_scanner_start();
                ESP_LOGI(TAG, "BLE scanning resumed after permit_join: %s",
                         esp_err_to_name(ble_ret));
            }
#endif

            /* Update LED status */
            led_status_manager_set_condition(LED_COND_PERMIT_JOIN, false);

            /* Publish permit join ended event */
            bridge_event_permit_join(false, 0);

            ESP_LOGI(TAG, "Network closed for joining");
        }
    } else {
        /* Open network */
        /* Note: esp_zb_bdb_open_network() takes duration in SECONDS as uint8_t (0-254, 255=forever)
         * Do NOT multiply by 1000 - that was causing the bug where 180 became 32! */
        uint8_t duration_sec = (duration == ZB_PERMIT_JOIN_FOREVER) ? 255 : duration;
        esp_zb_lock_acquire(portMAX_DELAY);
        ret = esp_zb_bdb_open_network(duration_sec);
        esp_zb_lock_release();

        if (ret == ESP_OK) {
            /* Update permit join state */
            s_permit_join_state.enabled = true;
            s_permit_join_state.initial_duration = duration;
            s_permit_join_state.remaining_time = duration;
            s_permit_join_state.start_time_us = esp_timer_get_time();
            s_permit_join_state.has_target_device = options->has_target_device;

            if (options->has_target_device) {
                memcpy(s_permit_join_state.target_ieee_addr, options->target_ieee_addr, 8);
            } else {
                memset(s_permit_join_state.target_ieee_addr, 0, 8);
            }

            /* Start timers (unless duration is forever) */
            if (duration != ZB_PERMIT_JOIN_FOREVER) {
                start_permit_join_timers(duration);
            }

            /* Pause BLE scanning to give Zigbee exclusive radio access */
#if CONFIG_BT_SCANNER_ENABLED
            if (ble_scanner_is_running()) {
                esp_err_t ble_ret = ble_scanner_stop();
                if (ble_ret == ESP_OK) {
                    s_ble_paused_for_join = true;
                    ESP_LOGI(TAG, "BLE scanning paused for Zigbee permit_join");
                } else {
                    ESP_LOGW(TAG, "Failed to pause BLE scanning: %s",
                             esp_err_to_name(ble_ret));
                }
            }
#endif

            /* Update LED status */
            led_status_manager_set_condition(LED_COND_PERMIT_JOIN, true);

            /* Publish permit join started event */
            bridge_event_permit_join(true, duration);

            ESP_LOGI(TAG, "Network opened successfully (duration: %d seconds)", duration);
        }
    }

    xSemaphoreGive(s_coordinator_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set permit join: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_coordinator_get_permit_join_state(zb_permit_join_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);

    /* Update remaining time before returning */
    update_permit_join_remaining_time();

    memcpy(state, &s_permit_join_state, sizeof(zb_permit_join_state_t));

    xSemaphoreGive(s_coordinator_mutex);

    return ESP_OK;
}

bool zb_coordinator_is_permit_join_enabled(void)
{
    /* Thread-safe permit join check */
    if (s_coordinator_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);
    bool enabled = s_permit_join_state.enabled;
    xSemaphoreGive(s_coordinator_mutex);
    return enabled;
}

uint8_t zb_coordinator_get_permit_join_remaining(void)
{
    /* Thread-safe remaining time retrieval */
    if (s_coordinator_mutex == NULL) {
        return 0;
    }
    xSemaphoreTake(s_coordinator_mutex, portMAX_DELAY);

    if (!s_permit_join_state.enabled) {
        xSemaphoreGive(s_coordinator_mutex);
        return 0;
    }

    if (s_permit_join_state.remaining_time == ZB_PERMIT_JOIN_FOREVER) {
        xSemaphoreGive(s_coordinator_mutex);
        return ZB_PERMIT_JOIN_FOREVER;
    }

    update_permit_join_remaining_time();
    uint8_t remaining = s_permit_join_state.remaining_time;
    xSemaphoreGive(s_coordinator_mutex);
    return remaining;
}

void zb_coordinator_task(void *pvParameters)
{
    /* This is a wrapper for compatibility - actual task is esp_zb_task */
    esp_zb_task(pvParameters);
}

SemaphoreHandle_t zb_coordinator_get_mutex(void)
{
    return s_coordinator_mutex;
}

esp_err_t zb_coordinator_self_test(void)
{
    ESP_LOGI(TAG, "Running coordinator self-test...");

    /* Check initialization */
    if (s_coordinator_state == ZB_COORD_STATE_UNINITIALIZED) {
        ESP_LOGE(TAG, "Coordinator not initialized");
        return ESP_FAIL;
    }

    /* Check mutex */
    if (s_coordinator_mutex == NULL) {
        ESP_LOGE(TAG, "Coordinator mutex not created");
        return ESP_FAIL;
    }

    /* Test sub-modules */
    esp_err_t ret;

    ret = zb_network_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network manager test failed");
        return ESP_FAIL;
    }

    ret = zb_device_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device handler test failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Coordinator self-test PASSED");
    return ESP_OK;
}

/* Internal functions */
static void set_coordinator_state(zb_coordinator_state_t state)
{
    s_coordinator_state = state;
    const char *state_str[] = {
        "UNINITIALIZED", "INITIALIZED", "STARTING", "RUNNING", "ERROR", "STOPPED"
    };
    ESP_LOGI(TAG, "State: %s", state_str[state]);
}

static esp_err_t configure_coordinator(void)
{
    ESP_LOGI(TAG, "Configuring coordinator endpoint...");

    /* Create Basic cluster (mandatory) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);

    /* Create Identify cluster (mandatory for coordinator) */
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);

    /* Create cluster list */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Add Tuya private cluster (0xEF00) as CLIENT for sending commands to Tuya devices.
     * The custom cluster must be registered on the coordinator to use esp_zb_zcl_custom_cluster_cmd_req() */
    esp_zb_attribute_list_t *tuya_cluster = esp_zb_zcl_attr_list_create(ZB_TUYA_CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, tuya_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    ESP_LOGI(TAG, "Added Tuya cluster (0x%04X) as client", ZB_TUYA_CLUSTER_ID);

    /* Create endpoint list */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    /* Create endpoint configuration */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ZB_COORDINATOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0
    };

    /* Add endpoint 1 to list */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    /* Add endpoint 242 (0xF2) — Tuya gateway endpoint.
     * Some Tuya devices check the source endpoint and only respond to EP 242. */
    esp_zb_cluster_list_t *ep242_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *tuya_cluster_ep242 = esp_zb_zcl_attr_list_create(ZB_TUYA_CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(ep242_cluster_list, tuya_cluster_ep242,
                                           ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t ep242_config = {
        .endpoint = 242,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HOME_GATEWAY_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, ep242_cluster_list, ep242_config);
    ESP_LOGI(TAG, "Added Tuya endpoint (EP=242) with cluster 0x%04X", ZB_TUYA_CLUSTER_ID);

    /* Register device */
    esp_zb_device_register(esp_zb_ep_list);

    ESP_LOGI(TAG, "Coordinator endpoints configured (EP=%d, EP=242)", ZB_COORDINATOR_ENDPOINT);
    return ESP_OK;
}

/* ============================================================================
 * Permit Join Timer Functions
 * ============================================================================ */

/**
 * @brief Update remaining permit join time based on elapsed time
 */
static void update_permit_join_remaining_time(void)
{
    if (!s_permit_join_state.enabled) {
        return;
    }

    if (s_permit_join_state.remaining_time == ZB_PERMIT_JOIN_FOREVER) {
        return;
    }

    int64_t elapsed_us = esp_timer_get_time() - s_permit_join_state.start_time_us;
    int64_t elapsed_sec = elapsed_us / 1000000;

    if (elapsed_sec >= s_permit_join_state.initial_duration) {
        s_permit_join_state.remaining_time = 0;
    } else {
        s_permit_join_state.remaining_time = s_permit_join_state.initial_duration - (uint8_t)elapsed_sec;
    }
}

/**
 * @brief Timer callback when permit join expires
 *
 * Note: Timer callbacks run from esp_timer task context.
 * We use a short timeout to avoid blocking the timer task.
 */
static void permit_join_timer_callback(void *arg)
{
    ESP_LOGI(TAG, "Permit join timer expired");

    /* Close the network - Zigbee API needs lock for thread safety */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_bdb_open_network(0);
    esp_zb_lock_release();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to close network after timer: %s", esp_err_to_name(ret));
    }

    /* Thread-safe state update */
    if (s_coordinator_mutex != NULL &&
        xSemaphoreTake(s_coordinator_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_permit_join_state.enabled = false;
        s_permit_join_state.remaining_time = 0;
        s_permit_join_state.has_target_device = false;
        xSemaphoreGive(s_coordinator_mutex);
    } else {
        /* Fallback: update without lock (better than leaving stale state) */
        ESP_LOGW(TAG, "Could not acquire mutex in timer callback");
        s_permit_join_state.enabled = false;
        s_permit_join_state.remaining_time = 0;
        s_permit_join_state.has_target_device = false;
    }

    /* Stop the update timer */
    if (s_permit_join_update_timer != NULL) {
        esp_timer_stop(s_permit_join_update_timer);
    }

    /* Resume BLE scanning if we paused it */
#if CONFIG_BT_SCANNER_ENABLED
    if (s_ble_paused_for_join) {
        s_ble_paused_for_join = false;
        esp_err_t ble_ret = ble_scanner_start();
        ESP_LOGI(TAG, "BLE scanning resumed after permit_join expired: %s",
                 esp_err_to_name(ble_ret));
    }
#endif

    /* Update LED status */
    led_status_manager_set_condition(LED_COND_PERMIT_JOIN, false);

    /* Publish permit join ended event */
    bridge_event_permit_join(false, 0);

    ESP_LOGI(TAG, "Network closed (timer expired)");
}

/**
 * @brief Periodic timer callback to publish remaining time
 *
 * Note: Timer callbacks run from esp_timer task context.
 * We use a short timeout to avoid blocking the timer task.
 */
static void permit_join_update_callback(void *arg)
{
    uint8_t remaining = 0;
    bool enabled = false;

    /* Thread-safe state access */
    if (s_coordinator_mutex != NULL &&
        xSemaphoreTake(s_coordinator_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        enabled = s_permit_join_state.enabled;
        if (enabled) {
            update_permit_join_remaining_time();
            remaining = s_permit_join_state.remaining_time;
        }
        xSemaphoreGive(s_coordinator_mutex);
    } else {
        /* Could not acquire mutex - skip this update cycle */
        return;
    }

    if (!enabled) {
        return;
    }

    ESP_LOGD(TAG, "Permit join remaining: %d seconds", remaining);

    /* Publish update event with remaining time */
    bridge_event_permit_join(true, remaining);
}

/**
 * @brief Start permit join timers
 *
 * @param duration Duration in seconds
 * @return ESP_OK on success
 */
static esp_err_t start_permit_join_timers(uint8_t duration)
{
    esp_err_t ret;

    /* Start expiry timer (one-shot) */
    if (s_permit_join_timer != NULL) {
        uint64_t timeout_us = (uint64_t)duration * 1000000;
        ret = esp_timer_start_once(s_permit_join_timer, timeout_us);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start permit join timer: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Start update timer (periodic, every 10 seconds) */
    if (s_permit_join_update_timer != NULL && duration > ZB_PERMIT_JOIN_UPDATE_INTERVAL) {
        uint64_t update_us = ZB_PERMIT_JOIN_UPDATE_INTERVAL * 1000000;
        ret = esp_timer_start_periodic(s_permit_join_update_timer, update_us);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start permit join update timer: %s", esp_err_to_name(ret));
            /* Non-fatal, continue without periodic updates */
        }
    }

    return ESP_OK;
}

/**
 * @brief Stop permit join timers
 */
static void stop_permit_join_timers(void)
{
    if (s_permit_join_timer != NULL) {
        esp_timer_stop(s_permit_join_timer);
    }
    if (s_permit_join_update_timer != NULL) {
        esp_timer_stop(s_permit_join_update_timer);
    }
}

static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Zigbee stack task started");

    /* Get network configuration */
    uint8_t channel = zb_network_get_channel_config();
    uint16_t pan_id = zb_network_get_pan_id_config();

    ESP_LOGI(TAG, "Network config: PAN=0x%04X, Channel=%d", pan_id, channel);

    /* Configure extended PAN ID using ESP-Zigbee-SDK v1.6.x API (API-004) */
    esp_zb_ieee_addr_t extended_pan_id;
    esp_zb_nwk_get_extended_pan_id(extended_pan_id);

    /* Check if extended PAN ID is all zeros (not yet configured) */
    bool ext_pan_id_is_zero = true;
    for (size_t i = 0; i < sizeof(esp_zb_ieee_addr_t); i++) {
        if (extended_pan_id[i] != 0) {
            ext_pan_id_is_zero = false;
            break;
        }
    }

    if (ext_pan_id_is_zero) {
        /* Extended PAN ID not set - use IEEE MAC address */
        esp_err_t mac_ret = esp_read_mac(extended_pan_id, ESP_MAC_IEEE802154);
        if (mac_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read IEEE MAC address: %s", esp_err_to_name(mac_ret));
        } else {
            esp_zb_nwk_set_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Extended PAN ID set from IEEE MAC");
        }
    } else {
        ESP_LOGI(TAG, "Extended PAN ID already configured");
    }

    ESP_LOGI(TAG, "Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
             extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);

    /* Set channel mask */
    uint32_t channel_mask = (1 << channel);
    esp_zb_set_channel_mask(channel_mask);

    /* Start Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Record start time for uptime calculation */
    s_coordinator_start_time = esp_timer_get_time();

    set_coordinator_state(ZB_COORD_STATE_RUNNING);
    ESP_LOGI(TAG, "Zigbee coordinator running");

    /* Start availability tracking now that Zigbee stack is running */
    esp_err_t avail_ret = zb_availability_start();
    if (avail_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start availability tracking: %s", esp_err_to_name(avail_ret));
        /* Non-fatal - coordinator continues without availability tracking */
    } else {
        ESP_LOGI(TAG, "Availability tracking started");
    }

    /* Main event loop - esp_zb_stack_main_loop() blocks forever */
    esp_zb_stack_main_loop();

    /* Task should never reach here, but handle cleanup if it does */
    ESP_LOGW(TAG, "Zigbee stack task exiting");
    set_coordinator_state(ZB_COORD_STATE_STOPPED);
    vTaskDelete(NULL);
}

/* ============================================================================
 * Coordinator Settings and Info Implementation (ZG-012)
 * ============================================================================ */

esp_err_t zb_coordinator_get_settings(zb_coordinator_settings_t *settings)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_coordinator_state == ZB_COORD_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(settings, 0, sizeof(zb_coordinator_settings_t));

    /* Get network configuration from zb_network module */
    settings->pan_id = zb_network_get_pan_id_config();
    settings->channel = zb_network_get_channel_config();
    settings->transmit_power = ZB_DEFAULT_TX_POWER;
    settings->permit_join_default = false;
    settings->max_children = ZB_COORDINATOR_MAX_CHILDREN;
    settings->network_formed = (s_coordinator_state == ZB_COORD_STATE_RUNNING);

    /* Get extended PAN ID using ESP-Zigbee-SDK v1.6.x API (API-004) */
    esp_zb_ieee_addr_t ext_pan_id;
    esp_zb_nwk_get_extended_pan_id(ext_pan_id);
    memcpy(settings->extended_pan_id, ext_pan_id, sizeof(esp_zb_ieee_addr_t));

    /* Network key is sensitive - we don't expose it */
    memset(settings->network_key, 0, sizeof(settings->network_key));

    /* Get network update ID */
    settings->network_update_id = 0;  /* TODO: Get from Zigbee stack if available */

    ESP_LOGI(TAG, "Settings: PAN=0x%04X, Channel=%d, Formed=%d",
             settings->pan_id, settings->channel, settings->network_formed);

    return ESP_OK;
}

esp_err_t zb_coordinator_get_version(zb_coordinator_version_t *version)
{
    if (version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(version, 0, sizeof(zb_coordinator_version_t));

    /* Coordinator type */
    snprintf(version->type, sizeof(version->type), "ESP32-C5");

    /* Firmware version from version module */
    snprintf(version->firmware_version, sizeof(version->firmware_version),
             "%s", version_get_number());
    snprintf(version->firmware_build, sizeof(version->firmware_build),
             "%s", version_get_build_date());

    /* Zigbee stack version */
    snprintf(version->zigbee_version, sizeof(version->zigbee_version), "ESP-Zigbee");

    /* Coordinator IEEE address */
    esp_zb_ieee_addr_t ieee_addr;
    esp_read_mac(ieee_addr, ESP_MAC_IEEE802154);
    snprintf(version->ieee_address, sizeof(version->ieee_address),
             "0x%02X%02X%02X%02X%02X%02X%02X%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

    /* Short address is always 0x0000 for coordinator */
    version->short_address = 0x0000;

    ESP_LOGI(TAG, "Version: %s, Build: %s, IEEE: %s",
             version->firmware_version, version->firmware_build, version->ieee_address);

    return ESP_OK;
}

esp_err_t zb_coordinator_health_check(zb_coordinator_health_t *health)
{
    if (health == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(health, 0, sizeof(zb_coordinator_health_t));

    /* Network and stack status */
    health->network_up = (s_coordinator_state == ZB_COORD_STATE_RUNNING);
    health->zigbee_stack_running = (s_coordinator_state == ZB_COORD_STATE_RUNNING);

    /* Calculate uptime */
    if (s_coordinator_start_time > 0) {
        int64_t now_us = esp_timer_get_time();
        health->uptime_seconds = (uint32_t)((now_us - s_coordinator_start_time) / 1000000);
    } else {
        health->uptime_seconds = 0;
    }

    /* Device count */
    health->joined_devices = zb_device_get_count();

    /* Message statistics */
    health->message_count_tx = s_message_count_tx;
    health->message_count_rx = s_message_count_rx;

    /* Signal quality */
    health->last_rssi = s_last_rssi;
    health->last_lqi = s_last_lqi;

    /* Overall health - healthy if network is up and running */
    health->healthy = health->network_up && health->zigbee_stack_running;

    ESP_LOGI(TAG, "Health check: %s (uptime=%lus, devices=%lu, tx=%lu, rx=%lu)",
             health->healthy ? "healthy" : "unhealthy",
             (unsigned long)health->uptime_seconds,
             (unsigned long)health->joined_devices,
             (unsigned long)health->message_count_tx,
             (unsigned long)health->message_count_rx);

    return health->healthy ? ESP_OK : ESP_FAIL;
}

esp_err_t zb_coordinator_get_version_string(char *version_buf, size_t buf_len)
{
    if (version_buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_coordinator_version_t version;
    esp_err_t ret = zb_coordinator_get_version(&version);
    if (ret != ESP_OK) {
        return ret;
    }

    snprintf(version_buf, buf_len, "%s (%s)", version.firmware_version, version.firmware_build);
    return ESP_OK;
}

/* Statistics update functions - to be called from message handlers */
void zb_coordinator_update_tx_count(void)
{
    s_message_count_tx++;
}

void zb_coordinator_update_rx_count(void)
{
    s_message_count_rx++;
}

void zb_coordinator_update_signal_quality(int8_t rssi, uint8_t lqi)
{
    s_last_rssi = rssi;
    s_last_lqi = lqi;
}

/* ============================================================================
 * API-009: Enhanced Signal Handler Statistics
 * ============================================================================ */

void zb_coordinator_update_route_error_count(void)
{
    s_route_error_count++;
    ESP_LOGD(TAG, "Route error count: %" PRIu32, s_route_error_count);
}

void zb_coordinator_update_tc_rejoin_count(void)
{
    s_tc_rejoin_count++;
    ESP_LOGI(TAG, "TC rejoin count: %" PRIu32, s_tc_rejoin_count);
}

uint32_t zb_coordinator_get_route_error_count(void)
{
    return s_route_error_count;
}

uint32_t zb_coordinator_get_tc_rejoin_count(void)
{
    return s_tc_rejoin_count;
}

/* ============================================================================
 * SDK Version Query Functions (API-016)
 * ============================================================================ */

esp_err_t zb_coordinator_get_sdk_version(zb_sdk_version_t *version)
{
    if (version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    version->major = ESP_ZB_VERSION_MAJOR;
    version->minor = ESP_ZB_VERSION_MINOR;
#ifdef ESP_ZB_VERSION_PATCH
    version->patch = ESP_ZB_VERSION_PATCH;
#else
    version->patch = 0;
#endif

    /* Create version string */
    snprintf(version->version_string, sizeof(version->version_string),
             "%d.%d.%d", version->major, version->minor, version->patch);

    /* Set minimum required version for feature checks */
    version->min_major = 1;
    version->min_minor = 6;
    version->min_patch = 0;

    /* Check if current version meets minimum requirements */
    version->meets_minimum = (version->major > version->min_major) ||
                            (version->major == version->min_major &&
                             version->minor >= version->min_minor);

    ESP_LOGD(TAG, "SDK version: %s (min: %d.%d.%d, meets_min: %s)",
             version->version_string,
             version->min_major, version->min_minor, version->min_patch,
             version->meets_minimum ? "yes" : "no");

    return ESP_OK;
}

esp_err_t zb_coordinator_get_sdk_version_string(char *version_buf, size_t buf_len)
{
    if (version_buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(version_buf, buf_len, "%d.%d.%d",
             ESP_ZB_VERSION_MAJOR, ESP_ZB_VERSION_MINOR,
#ifdef ESP_ZB_VERSION_PATCH
             ESP_ZB_VERSION_PATCH
#else
             0
#endif
    );

    return ESP_OK;
}
