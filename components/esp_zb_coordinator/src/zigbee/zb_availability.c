/**
 * @file zb_availability.c
 * @brief Zigbee Device Availability Tracking Implementation
 *
 * Implements active and passive availability checking for Zigbee devices:
 * - Router devices: Active ZCL Basic cluster ping with exponential backoff
 * - Battery devices: Passive timeout-based detection
 * - MQTT availability publishing on state changes
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_availability.h"
#include "zb_device_handler.h"
#include "zb_coordinator.h"
#include "compat_stubs.h"
#include "gateway_defaults.h"
#include "uart_bridge.h"
#include "uart/uart_protocol.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos_helpers.h"  /* For PSRAM task creation */
#include <string.h>
#include <time.h>

static const char *TAG = "ZB_AVAIL";

/* ============================================================================
 * NVS Keys
 * ============================================================================ */

#define ZB_AVAIL_NVS_NAMESPACE      "zb_avail"
#define ZB_AVAIL_NVS_KEY_CONFIG     "config"

/* ============================================================================
 * Task Configuration
 * ============================================================================ */

#define ZB_AVAIL_TASK_STACK_SIZE    (4 * 1024)
#define ZB_AVAIL_TASK_PRIORITY      4
#define ZB_AVAIL_TASK_LOOP_MS       1000    /**< Main loop interval */
#define ZB_AVAIL_PING_TIMEOUT_MS    10000   /**< Ping response timeout */

/* ============================================================================
 * Module State
 * ============================================================================ */

/**
 * @brief Module state enumeration
 */
typedef enum {
    ZB_AVAIL_MODULE_UNINITIALIZED = 0,
    ZB_AVAIL_MODULE_INITIALIZED,
    ZB_AVAIL_MODULE_RUNNING,
    ZB_AVAIL_MODULE_STOPPED
} zb_avail_module_state_t;

/**
 * @brief Module context structure
 */
typedef struct {
    zb_avail_module_state_t state;
    zb_availability_config_t config;
    zb_availability_device_t devices[ZB_AVAIL_MAX_DEVICES];
    uint16_t device_count;
    psram_task_handle_t psram_task;  /**< PSRAM-backed task (saves internal RAM) */
    SemaphoreHandle_t mutex;
    zb_availability_state_cb_t state_callback;
    uint32_t check_counter;
} zb_avail_context_t;

/* Static module context */
static zb_avail_context_t s_ctx = {0};

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void zb_availability_task(void *pvParameters);
static zb_availability_device_t* find_device(uint16_t short_addr);
static esp_err_t send_ping(uint16_t short_addr, uint8_t endpoint);
static void check_device_timeouts(void);
static void process_pending_checks(void);
static void set_device_state(zb_availability_device_t *device, zb_availability_state_t new_state);
static uint32_t get_device_timeout(const zb_availability_device_t *device);
static void calculate_next_check(zb_availability_device_t *device);

/* ============================================================================
 * Initialization and Control
 * ============================================================================ */

esp_err_t zb_availability_init(zb_availability_config_t *config)
{
    if (s_ctx.state != ZB_AVAIL_MODULE_UNINITIALIZED) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing availability tracking...");

    /* Create mutex */
    s_ctx.mutex = xSemaphoreCreateMutex();
    if (s_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize device list */
    memset(s_ctx.devices, 0, sizeof(s_ctx.devices));
    s_ctx.device_count = 0;
    s_ctx.state_callback = NULL;
    s_ctx.check_counter = 0;

    /* Set configuration */
    if (config != NULL) {
        s_ctx.config = *config;
    } else {
        /* Use defaults */
        s_ctx.config.router_check_interval = ZB_AVAIL_DEFAULT_ROUTER_INTERVAL;
        s_ctx.config.battery_timeout = ZB_AVAIL_DEFAULT_BATTERY_TIMEOUT;
        s_ctx.config.router_timeout = ZB_AVAIL_DEFAULT_ROUTER_TIMEOUT;
#ifdef CONFIG_ZB_AVAILABILITY_CHECK_ENABLED
        s_ctx.config.active_check_enabled = true;
#else
        s_ctx.config.active_check_enabled = true;  /* Default enabled */
#endif
    }

    /* Try to load saved configuration */
    esp_err_t ret = zb_availability_load_config();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded configuration from NVS");
    } else {
        ESP_LOGI(TAG, "Using default configuration");
    }

    ESP_LOGI(TAG, "Config: router_interval=%lus, battery_timeout=%lus, router_timeout=%lus, active=%s",
             (unsigned long)s_ctx.config.router_check_interval,
             (unsigned long)s_ctx.config.battery_timeout,
             (unsigned long)s_ctx.config.router_timeout,
             s_ctx.config.active_check_enabled ? "yes" : "no");

    s_ctx.state = ZB_AVAIL_MODULE_INITIALIZED;
    ESP_LOGI(TAG, "Availability tracking initialized");
    return ESP_OK;
}

esp_err_t zb_availability_deinit(void)
{
    if (s_ctx.state == ZB_AVAIL_MODULE_UNINITIALIZED) {
        return ESP_OK;
    }

    /* Stop if running */
    if (s_ctx.state == ZB_AVAIL_MODULE_RUNNING) {
        zb_availability_stop();
    }

    /* Delete mutex */
    if (s_ctx.mutex != NULL) {
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
    }

    s_ctx.state = ZB_AVAIL_MODULE_UNINITIALIZED;
    ESP_LOGI(TAG, "Availability tracking deinitialized");
    return ESP_OK;
}

esp_err_t zb_availability_start(void)
{
    if (s_ctx.state != ZB_AVAIL_MODULE_INITIALIZED &&
        s_ctx.state != ZB_AVAIL_MODULE_STOPPED) {
        ESP_LOGE(TAG, "Cannot start: invalid state %d", s_ctx.state);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting availability tracking...");

    /* Create task with PSRAM stack (saves ~4KB internal RAM) */
    esp_err_t ret = psram_task_create(
        zb_availability_task,
        "zb_avail",
        ZB_AVAIL_TASK_STACK_SIZE,
        NULL,
        ZB_AVAIL_TASK_PRIORITY,
        &s_ctx.psram_task
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create task: %s", esp_err_to_name(ret));
        return ret;
    }

    s_ctx.state = ZB_AVAIL_MODULE_RUNNING;
    ESP_LOGI(TAG, "Availability tracking started (PSRAM stack)");
    return ESP_OK;
}

esp_err_t zb_availability_stop(void)
{
    if (s_ctx.state != ZB_AVAIL_MODULE_RUNNING) {
        ESP_LOGW(TAG, "Not running");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping availability tracking...");

    /* Delete task and free PSRAM resources */
    psram_task_delete(&s_ctx.psram_task);

    s_ctx.state = ZB_AVAIL_MODULE_STOPPED;
    ESP_LOGI(TAG, "Availability tracking stopped");
    return ESP_OK;
}

bool zb_availability_is_running(void)
{
    return (s_ctx.state == ZB_AVAIL_MODULE_RUNNING);
}

/* ============================================================================
 * Device Tracking
 * ============================================================================ */

esp_err_t zb_availability_add_device(uint16_t short_addr, zb_availability_power_type_t power_type)
{
    if (short_addr == 0x0000 || short_addr == 0xFFFF) {
        ESP_LOGE(TAG, "Invalid device address: 0x%04X", short_addr);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    /* Check if device already exists */
    zb_availability_device_t *existing = find_device(short_addr);
    if (existing != NULL) {
        ESP_LOGD(TAG, "Device 0x%04X already tracked, updating power type", short_addr);
        existing->power_type = power_type;
        existing->last_seen = time(NULL);
        xSemaphoreGive(s_ctx.mutex);
        return ESP_OK;
    }

    /* Find empty slot */
    if (s_ctx.device_count >= ZB_AVAIL_MAX_DEVICES) {
        ESP_LOGE(TAG, "Device list full, cannot add 0x%04X", short_addr);
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Add device */
    zb_availability_device_t *device = NULL;
    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        if (s_ctx.devices[i].short_addr == 0) {
            device = &s_ctx.devices[i];
            break;
        }
    }

    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NO_MEM;
    }

    time_t now = time(NULL);

    device->short_addr = short_addr;
    device->state = ZB_AVAIL_UNKNOWN;
    device->power_type = power_type;
    device->last_seen = now;
    device->last_check = 0;
    device->consecutive_failures = 0;
    device->backoff_multiplier = 1;
    device->custom_timeout = 0;
    device->pending_response = false;

    /* Calculate initial next check time */
    calculate_next_check(device);

    s_ctx.device_count++;

    ESP_LOGI(TAG, "Added device 0x%04X for tracking (power=%s, count=%d)",
             short_addr, zb_availability_power_type_to_str(power_type),
             s_ctx.device_count);

    xSemaphoreGive(s_ctx.mutex);

    /* Mark as online initially (device just joined) */
    set_device_state(device, ZB_AVAIL_ONLINE);

    return ESP_OK;
}

esp_err_t zb_availability_remove_device(uint16_t short_addr)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        ESP_LOGW(TAG, "Device 0x%04X not found for removal", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Clear device slot */
    memset(device, 0, sizeof(zb_availability_device_t));
    s_ctx.device_count--;

    ESP_LOGI(TAG, "Removed device 0x%04X from tracking (count=%d)",
             short_addr, s_ctx.device_count);

    xSemaphoreGive(s_ctx.mutex);
    return ESP_OK;
}

esp_err_t zb_availability_update_last_seen(uint16_t short_addr)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        /* Device not tracked - this is normal for devices before they're added */
        ESP_LOGD(TAG, "Device 0x%04X not tracked", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    time_t now = time(NULL);
    device->last_seen = now;
    device->consecutive_failures = 0;
    device->backoff_multiplier = 1;
    device->pending_response = false;

    /* Recalculate next check time */
    calculate_next_check(device);

    zb_availability_state_t old_state = device->state;

    xSemaphoreGive(s_ctx.mutex);

    /* Mark as online if not already */
    if (old_state != ZB_AVAIL_ONLINE) {
        set_device_state(device, ZB_AVAIL_ONLINE);
    }

    ESP_LOGD(TAG, "Updated last_seen for device 0x%04X", short_addr);
    return ESP_OK;
}

esp_err_t zb_availability_check_device(uint16_t short_addr)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        ESP_LOGW(TAG, "Device 0x%04X not found for check", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    if (device->pending_response) {
        xSemaphoreGive(s_ctx.mutex);
        ESP_LOGW(TAG, "Check already pending for device 0x%04X", short_addr);
        return ESP_ERR_INVALID_STATE;
    }

    /* Get device endpoint */
    zb_device_t *zb_dev = zb_device_get(short_addr);
    uint8_t endpoint = (zb_dev != NULL) ? zb_dev->endpoint : 1;

    device->pending_response = true;
    device->last_check = time(NULL);

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Sending availability ping to 0x%04X (EP=%d)", short_addr, endpoint);

    /* Send ZCL Read Attribute request to Basic cluster */
    esp_err_t ret = send_ping(short_addr, endpoint);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send ping to 0x%04X: %s", short_addr, esp_err_to_name(ret));

        xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
        device->pending_response = false;
        xSemaphoreGive(s_ctx.mutex);
    }

    return ret;
}

zb_availability_state_t zb_availability_get_state(uint16_t short_addr)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    zb_availability_state_t state = (device != NULL) ? device->state : ZB_AVAIL_UNKNOWN;

    xSemaphoreGive(s_ctx.mutex);
    return state;
}

esp_err_t zb_availability_get_device_data(uint16_t short_addr, zb_availability_device_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(data, device, sizeof(zb_availability_device_t));

    xSemaphoreGive(s_ctx.mutex);
    return ESP_OK;
}

/* ============================================================================
 * Configuration
 * ============================================================================ */

esp_err_t zb_availability_set_device_timeout(uint16_t short_addr, uint32_t timeout_sec)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    device->custom_timeout = timeout_sec;
    calculate_next_check(device);

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Set custom timeout for 0x%04X: %lu seconds",
             short_addr, (unsigned long)timeout_sec);
    return ESP_OK;
}

esp_err_t zb_availability_set_device_power_type(uint16_t short_addr,
                                                  zb_availability_power_type_t power_type)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    device->power_type = power_type;
    calculate_next_check(device);

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Set power type for 0x%04X: %s",
             short_addr, zb_availability_power_type_to_str(power_type));
    return ESP_OK;
}

esp_err_t zb_availability_set_config(const zb_availability_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    if (config->router_check_interval > 0) {
        s_ctx.config.router_check_interval = config->router_check_interval;
    }
    if (config->battery_timeout > 0) {
        s_ctx.config.battery_timeout = config->battery_timeout;
    }
    if (config->router_timeout > 0) {
        s_ctx.config.router_timeout = config->router_timeout;
    }
    s_ctx.config.active_check_enabled = config->active_check_enabled;

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Configuration updated");
    return ESP_OK;
}

esp_err_t zb_availability_get_config(zb_availability_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    *config = s_ctx.config;
    xSemaphoreGive(s_ctx.mutex);

    return ESP_OK;
}

esp_err_t zb_availability_save_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(ZB_AVAIL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    ret = nvs_set_blob(nvs_handle, ZB_AVAIL_NVS_KEY_CONFIG,
                       &s_ctx.config, sizeof(zb_availability_config_t));

    xSemaphoreGive(s_ctx.mutex);

    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save configuration: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_availability_load_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    ret = nvs_open(ZB_AVAIL_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    zb_availability_config_t loaded_config;
    size_t size = sizeof(zb_availability_config_t);

    ret = nvs_get_blob(nvs_handle, ZB_AVAIL_NVS_KEY_CONFIG, &loaded_config, &size);

    nvs_close(nvs_handle);

    if (ret == ESP_OK && size == sizeof(zb_availability_config_t)) {
        xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
        s_ctx.config = loaded_config;
        xSemaphoreGive(s_ctx.mutex);
        ESP_LOGI(TAG, "Configuration loaded from NVS");
    }

    return ret;
}

/* ============================================================================
 * MQTT Publishing
 * ============================================================================ */

esp_err_t zb_availability_publish_state(uint16_t short_addr)
{
    /* Get device from device handler */
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X not found in device handler", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Get availability state */
    zb_availability_state_t state = zb_availability_get_state(short_addr);

    /* Check MQTT connection */
    REQUIRE_MQTT_CONNECTED(ESP_ERR_INVALID_STATE);

    /* Build topic: zigbee2mqtt/[friendly_name]/availability */
    char topic[128];
    esp_err_t ret = mqtt_topic_device_availability(device->friendly_name, topic, sizeof(topic));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build availability topic");
        return ret;
    }

    /* Payload is simply "online" or "offline" */
    const char *payload = zb_availability_state_to_str(state);

    /* Publish with retain flag */
    ret = mqtt_client_publish(topic, payload, strlen(payload), 1, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish availability for 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Published availability for %s: %s",
                 device->friendly_name, payload);
    }

    return ret;
}

esp_err_t zb_availability_publish_all(void)
{
    ESP_LOGI(TAG, "Publishing availability for all devices...");

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        if (s_ctx.devices[i].short_addr != 0) {
            uint16_t addr = s_ctx.devices[i].short_addr;
            xSemaphoreGive(s_ctx.mutex);

            zb_availability_publish_state(addr);

            xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
        }
    }

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Finished publishing all availability states");
    return ESP_OK;
}

/* ============================================================================
 * Callbacks
 * ============================================================================ */

esp_err_t zb_availability_register_callback(zb_availability_state_cb_t callback)
{
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    s_ctx.state_callback = callback;
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "State change callback registered");
    return ESP_OK;
}

esp_err_t zb_availability_unregister_callback(void)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    s_ctx.state_callback = NULL;
    xSemaphoreGive(s_ctx.mutex);

    return ESP_OK;
}

/* ============================================================================
 * Response Handlers
 * ============================================================================ */

esp_err_t zb_availability_handle_read_response(uint16_t short_addr, uint8_t status)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    device->pending_response = false;

    if (status == 0) {  /* ESP_ZB_ZCL_STATUS_SUCCESS */
        ESP_LOGI(TAG, "Ping response from 0x%04X: SUCCESS", short_addr);

        device->last_seen = time(NULL);
        device->consecutive_failures = 0;
        device->backoff_multiplier = 1;
        calculate_next_check(device);

        zb_availability_state_t old_state = device->state;
        xSemaphoreGive(s_ctx.mutex);

        if (old_state != ZB_AVAIL_ONLINE) {
            set_device_state(device, ZB_AVAIL_ONLINE);
        }
    } else {
        ESP_LOGW(TAG, "Ping response from 0x%04X: FAILED (status=0x%02X)", short_addr, status);

        device->consecutive_failures++;
        xSemaphoreGive(s_ctx.mutex);

        /* Check if we should mark offline */
        zb_availability_handle_timeout(short_addr);
    }

    return ESP_OK;
}

esp_err_t zb_availability_handle_timeout(uint16_t short_addr)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_device_t *device = find_device(short_addr);
    if (device == NULL) {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    device->pending_response = false;
    device->consecutive_failures++;

    ESP_LOGW(TAG, "Ping timeout for 0x%04X (failures=%d)",
             short_addr, device->consecutive_failures);

    /* Apply exponential backoff */
    if (device->backoff_multiplier < ZB_AVAIL_MAX_BACKOFF) {
        device->backoff_multiplier *= ZB_AVAIL_BACKOFF_BASE;
    }
    calculate_next_check(device);

    /* Check if we should mark offline */
    if (device->consecutive_failures >= ZB_AVAIL_MAX_FAILURES) {
        zb_availability_state_t old_state = device->state;
        xSemaphoreGive(s_ctx.mutex);

        if (old_state != ZB_AVAIL_OFFLINE) {
            set_device_state(device, ZB_AVAIL_OFFLINE);
        }
    } else {
        xSemaphoreGive(s_ctx.mutex);
    }

    return ESP_OK;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

const char* zb_availability_state_to_str(zb_availability_state_t state)
{
    switch (state) {
        case ZB_AVAIL_ONLINE:  return "online";
        case ZB_AVAIL_OFFLINE: return "offline";
        case ZB_AVAIL_UNKNOWN:
        default:               return "unknown";
    }
}

const char* zb_availability_power_type_to_str(zb_availability_power_type_t power_type)
{
    switch (power_type) {
        case ZB_AVAIL_POWER_MAINS:   return "mains";
        case ZB_AVAIL_POWER_BATTERY: return "battery";
        case ZB_AVAIL_POWER_UNKNOWN:
        default:                     return "unknown";
    }
}

esp_err_t zb_availability_get_stats(uint16_t *online_count, uint16_t *offline_count,
                                     uint16_t *unknown_count)
{
    uint16_t online = 0, offline = 0, unknown = 0;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        if (s_ctx.devices[i].short_addr != 0) {
            switch (s_ctx.devices[i].state) {
                case ZB_AVAIL_ONLINE:  online++;  break;
                case ZB_AVAIL_OFFLINE: offline++; break;
                default:               unknown++; break;
            }
        }
    }

    xSemaphoreGive(s_ctx.mutex);

    if (online_count)  *online_count = online;
    if (offline_count) *offline_count = offline;
    if (unknown_count) *unknown_count = unknown;

    return ESP_OK;
}

esp_err_t zb_availability_test(void)
{
    ESP_LOGI(TAG, "Running availability module self-test...");

    /* Test initialization */
    if (s_ctx.state == ZB_AVAIL_MODULE_UNINITIALIZED) {
        ESP_LOGE(TAG, "Test failed: module not initialized");
        return ESP_FAIL;
    }

    /* Test mutex */
    if (s_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Test failed: mutex not created");
        return ESP_FAIL;
    }

    /* Test add/remove device */
    esp_err_t ret = zb_availability_add_device(0x1234, ZB_AVAIL_POWER_MAINS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test failed: add device");
        return ESP_FAIL;
    }

    zb_availability_state_t state = zb_availability_get_state(0x1234);
    if (state != ZB_AVAIL_ONLINE) {
        ESP_LOGE(TAG, "Test failed: expected ONLINE state");
        return ESP_FAIL;
    }

    ret = zb_availability_update_last_seen(0x1234);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test failed: update last_seen");
        return ESP_FAIL;
    }

    ret = zb_availability_remove_device(0x1234);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test failed: remove device");
        return ESP_FAIL;
    }

    state = zb_availability_get_state(0x1234);
    if (state != ZB_AVAIL_UNKNOWN) {
        ESP_LOGE(TAG, "Test failed: expected UNKNOWN after removal");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Availability module self-test PASSED");
    return ESP_OK;
}

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

/**
 * @brief Find a device in the tracking list
 * @note Caller must hold mutex
 */
static zb_availability_device_t* find_device(uint16_t short_addr)
{
    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        if (s_ctx.devices[i].short_addr == short_addr) {
            return &s_ctx.devices[i];
        }
    }
    return NULL;
}

/**
 * @brief Send ZCL Read Attribute request as ping
 */
static esp_err_t send_ping(uint16_t short_addr, uint8_t endpoint)
{
    if (!zb_coordinator_is_running()) {
        ESP_LOGW(TAG, "Coordinator not running, cannot send ping");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read ZCL Version attribute from Basic cluster as ping */
    esp_zb_zcl_read_attr_cmd_t read_req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        .attr_number = 1,
        .attr_field = (uint16_t[]){ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID},
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&read_req);
    esp_zb_lock_release();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read attribute request: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Check all devices for timeout and mark offline if needed
 */
static void check_device_timeouts(void)
{
    time_t now = time(NULL);

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        zb_availability_device_t *device = &s_ctx.devices[i];
        if (device->short_addr == 0) {
            continue;
        }

        uint32_t timeout = get_device_timeout(device);
        time_t elapsed = now - device->last_seen;

        /* Check pending response timeout */
        if (device->pending_response) {
            time_t ping_elapsed = now - device->last_check;
            if (ping_elapsed > (ZB_AVAIL_PING_TIMEOUT_MS / 1000)) {
                uint16_t addr = device->short_addr;
                xSemaphoreGive(s_ctx.mutex);

                ESP_LOGW(TAG, "Ping timeout detected for 0x%04X", addr);
                zb_availability_handle_timeout(addr);

                xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
                continue;  /* Device state may have changed */
            }
        }

        /* Check if device has timed out */
        if (device->state == ZB_AVAIL_ONLINE && (uint32_t)elapsed > timeout) {
            uint16_t addr = device->short_addr;
            zb_availability_state_t old_state = device->state;

            /* For battery devices, mark offline immediately on timeout */
            /* For routers, we rely on failed pings */
            if (device->power_type == ZB_AVAIL_POWER_BATTERY ||
                device->power_type == ZB_AVAIL_POWER_UNKNOWN) {
                ESP_LOGW(TAG, "Device 0x%04X timed out (elapsed=%ld, timeout=%lu)",
                         addr, (long)elapsed, (unsigned long)timeout);

                xSemaphoreGive(s_ctx.mutex);
                set_device_state(device, ZB_AVAIL_OFFLINE);
                xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
            }
        }
    }

    xSemaphoreGive(s_ctx.mutex);
}

/**
 * @brief Process pending availability checks for router devices
 */
static void process_pending_checks(void)
{
    if (!s_ctx.config.active_check_enabled) {
        return;
    }

    time_t now = time(NULL);

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    for (int i = 0; i < ZB_AVAIL_MAX_DEVICES; i++) {
        zb_availability_device_t *device = &s_ctx.devices[i];
        if (device->short_addr == 0) {
            continue;
        }

        /* Only active check for mains-powered (router) devices */
        if (device->power_type != ZB_AVAIL_POWER_MAINS) {
            continue;
        }

        /* Skip if already waiting for response */
        if (device->pending_response) {
            continue;
        }

        /* Check if it's time for the next check */
        if (now >= device->next_check) {
            uint16_t addr = device->short_addr;
            xSemaphoreGive(s_ctx.mutex);

            ESP_LOGD(TAG, "Scheduled availability check for 0x%04X", addr);
            zb_availability_check_device(addr);

            xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
        }
    }

    xSemaphoreGive(s_ctx.mutex);
}

/**
 * @brief Set device state and trigger callbacks/publishing
 */
static void set_device_state(zb_availability_device_t *device, zb_availability_state_t new_state)
{
    if (device == NULL) {
        return;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    zb_availability_state_t old_state = device->state;
    uint16_t short_addr = device->short_addr;

    if (old_state == new_state) {
        xSemaphoreGive(s_ctx.mutex);
        return;
    }

    device->state = new_state;

    /* Get callback reference while holding mutex */
    zb_availability_state_cb_t callback = s_ctx.state_callback;

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Device 0x%04X state changed: %s -> %s",
             short_addr,
             zb_availability_state_to_str(old_state),
             zb_availability_state_to_str(new_state));

    /* Update device handler online status */
    zb_device_set_online(short_addr, (new_state == ZB_AVAIL_ONLINE));

    /* Publish availability change over UART */
    if (uart_bridge_is_enabled()) {
        zb_device_t *avail_dev = zb_device_get(short_addr);
        char ieee_str[20] = "unknown";
        if (avail_dev) {
            uart_proto_ieee_to_str(avail_dev->ieee_addr, ieee_str, sizeof(ieee_str));
        }
        char json_buf[160];
        snprintf(json_buf, sizeof(json_buf),
                 "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\",\"online\":%s,\"old_state\":\"%s\",\"new_state\":\"%s\"}",
                 ieee_str, short_addr,
                 (new_state == ZB_AVAIL_ONLINE) ? "true" : "false",
                 zb_availability_state_to_str(old_state),
                 zb_availability_state_to_str(new_state));
        uart_bridge_publish_event("availability", json_buf);
    }

    /* Publish to MQTT (legacy) */
    zb_availability_publish_state(short_addr);

    /* Call state change callback */
    if (callback != NULL) {
        callback(short_addr, old_state, new_state);
    }
}

/**
 * @brief Get effective timeout for a device
 */
static uint32_t get_device_timeout(const zb_availability_device_t *device)
{
    if (device->custom_timeout > 0) {
        return device->custom_timeout;
    }

    switch (device->power_type) {
        case ZB_AVAIL_POWER_MAINS:
            return s_ctx.config.router_timeout;
        case ZB_AVAIL_POWER_BATTERY:
        case ZB_AVAIL_POWER_UNKNOWN:
        default:
            return s_ctx.config.battery_timeout;
    }
}

/**
 * @brief Calculate next check time for a device
 */
static void calculate_next_check(zb_availability_device_t *device)
{
    time_t now = time(NULL);
    uint32_t interval;

    if (device->power_type == ZB_AVAIL_POWER_MAINS) {
        /* Router: use check interval with backoff */
        interval = s_ctx.config.router_check_interval * device->backoff_multiplier;
    } else {
        /* Battery device: no active checks, just use timeout for reference */
        interval = s_ctx.config.battery_timeout;
    }

    device->next_check = now + interval;
}

/**
 * @brief Main availability tracking task
 */
static void zb_availability_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Availability task started");

    /* Wait for coordinator to be running */
    while (!zb_coordinator_is_running()) {
        vTaskDelay(pdMS_TO_TICKS(GW_WIFI_STABILIZE_DELAY_MS));
    }

    ESP_LOGI(TAG, "Coordinator running, starting availability checks");

    /* Initial delay to let devices settle */
    vTaskDelay(pdMS_TO_TICKS(GW_TIMEOUT_VERY_LONG_MS));

    while (1) {
        s_ctx.check_counter++;

        /* Check for device timeouts */
        check_device_timeouts();

        /* Process scheduled active checks */
        process_pending_checks();

        /* Log statistics periodically (every 60 iterations = 1 minute) */
        if ((s_ctx.check_counter % 60) == 0) {
            uint16_t online, offline, unknown;
            zb_availability_get_stats(&online, &offline, &unknown);
            ESP_LOGI(TAG, "Status: online=%d, offline=%d, unknown=%d",
                     online, offline, unknown);

            /* Log stack watermark */
            UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Stack watermark: %d bytes", watermark * sizeof(StackType_t));
        }

        vTaskDelay(pdMS_TO_TICKS(ZB_AVAIL_TASK_LOOP_MS));
    }

    ESP_LOGW(TAG, "Availability task exiting");
    vTaskDelete(NULL);
}
