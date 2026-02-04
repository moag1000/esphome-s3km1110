/**
 * @file zb_green_power.c
 * @brief Zigbee Green Power Implementation for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module implements Zigbee Green Power (ZGP) functionality including:
 * - GP Proxy Table management
 * - GP Translation Table
 * - Commissioning mode handling
 * - GP Frame parsing and security verification
 * - NVS persistence
 * - MQTT integration
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_green_power.h"
#include "zb_constants.h"
#include "gateway_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <string.h>
#include <time.h>
#include <math.h>

/* Log tag */
static const char *TAG = "ZB_GP";

/* NVS namespace and keys */
#define GP_NVS_NAMESPACE        "gp_storage"
#define GP_NVS_KEY_DEVICES      "gp_devices"
#define GP_NVS_KEY_TRANS        "gp_trans"
#define GP_NVS_KEY_SHARED_KEY   "gp_shared_key"
#define GP_NVS_KEY_STATS        "gp_stats"

/* Forward declarations for internal functions */
static void gp_commissioning_timer_callback(TimerHandle_t timer);
static esp_err_t gp_process_frame(const zb_gp_device_t *device, const zb_gp_frame_t *frame);
static esp_err_t gp_execute_translation(const zb_gp_device_t *device,
                                         const zb_gp_translation_entry_t *entry,
                                         const zb_gp_frame_t *frame);
static bool gp_addr_equal(const zb_gp_addr_t *a, const zb_gp_addr_t *b);
static void gp_generate_friendly_name(zb_gp_device_t *device);
static esp_err_t gp_derive_key(const zb_gp_addr_t *gp_addr,
                                zb_gp_key_type_t key_type,
                                uint8_t out_key[ZB_GP_SECURITY_KEY_SIZE]);

/* ============================================================================
 * Module State
 * ============================================================================ */

/**
 * @brief GP Module state structure
 */
static struct {
    /* Initialization */
    bool initialized;
    SemaphoreHandle_t mutex;

    /* Proxy Table (devices) */
    zb_gp_device_t proxy_table[ZB_GP_MAX_DEVICES];
    size_t device_count;

    /* Translation Table */
    zb_gp_translation_entry_t translation_table[ZB_GP_MAX_TRANSLATION_ENTRIES];
    size_t translation_count;

    /* Commissioning */
    zb_gp_commissioning_state_t commissioning_state;
    zb_gp_commissioning_config_t commissioning_config;
    TimerHandle_t commissioning_timer;
    time_t commissioning_start_time;

    /* Security */
    uint8_t shared_key[ZB_GP_SECURITY_KEY_SIZE];
    bool shared_key_valid;

    /* Callbacks */
    zb_gp_frame_callback_t frame_callback;
    zb_gp_commissioning_callback_t commissioning_callback;
    zb_gp_device_callback_t device_callback;

    /* Statistics */
    zb_gp_stats_t stats;

} s_gp = {
    .initialized = false,
    .commissioning_state = ZB_GP_COMMISSIONING_STATE_IDLE,
    .shared_key_valid = false,
};

/* ============================================================================
 * GP Device Type and Command Names
 * ============================================================================ */

/**
 * @brief GP Device type name mapping
 */
static const struct {
    zb_gp_device_type_t type;
    const char *name;
} s_gp_device_type_names[] = {
    { ZB_GP_DEVICE_TYPE_SIMPLE_GENERIC_1STATE_SWITCH, "Simple 1-State Switch" },
    { ZB_GP_DEVICE_TYPE_SIMPLE_GENERIC_2STATE_SWITCH, "Simple 2-State Switch" },
    { ZB_GP_DEVICE_TYPE_ON_OFF_SWITCH, "On/Off Switch" },
    { ZB_GP_DEVICE_TYPE_LEVEL_CONTROL_SWITCH, "Level Control Switch" },
    { ZB_GP_DEVICE_TYPE_SIMPLE_SENSOR, "Simple Sensor" },
    { ZB_GP_DEVICE_TYPE_ADVANCED_GENERIC_1STATE_SWITCH, "Advanced 1-State Switch" },
    { ZB_GP_DEVICE_TYPE_ADVANCED_GENERIC_2STATE_SWITCH, "Advanced 2-State Switch" },
    { ZB_GP_DEVICE_TYPE_GENERIC_SWITCH, "Generic Switch" },
    { ZB_GP_DEVICE_TYPE_COLOR_DIMMER_SWITCH, "Color Dimmer Switch" },
    { ZB_GP_DEVICE_TYPE_LIGHT_SENSOR, "Light Sensor" },
    { ZB_GP_DEVICE_TYPE_OCCUPANCY_SENSOR, "Occupancy Sensor" },
    { ZB_GP_DEVICE_TYPE_DOOR_SENSOR, "Door Sensor" },
    { ZB_GP_DEVICE_TYPE_TEMPERATURE_SENSOR, "Temperature Sensor" },
    { ZB_GP_DEVICE_TYPE_PRESSURE_SENSOR, "Pressure Sensor" },
    { ZB_GP_DEVICE_TYPE_FLOW_SENSOR, "Flow Sensor" },
    { ZB_GP_DEVICE_TYPE_ENVIRONMENT_SENSOR, "Environment Sensor" },
    { ZB_GP_DEVICE_TYPE_MFR_SPECIFIC, "Manufacturer Specific" },
    { ZB_GP_DEVICE_TYPE_UNDEFINED, "Undefined" },
};

/**
 * @brief GP Command name mapping
 */
static const struct {
    uint8_t cmd_id;
    const char *name;
} s_gp_cmd_names[] = {
    { ZB_GP_CMD_IDENTIFY, "Identify" },
    { ZB_GP_CMD_OFF, "Off" },
    { ZB_GP_CMD_ON, "On" },
    { ZB_GP_CMD_TOGGLE, "Toggle" },
    { ZB_GP_CMD_RELEASE, "Release" },
    { ZB_GP_CMD_MOVE_UP, "Move Up" },
    { ZB_GP_CMD_MOVE_DOWN, "Move Down" },
    { ZB_GP_CMD_STEP_UP, "Step Up" },
    { ZB_GP_CMD_STEP_DOWN, "Step Down" },
    { ZB_GP_CMD_LEVEL_CONTROL_STOP, "Level Stop" },
    { ZB_GP_CMD_PRESS_1_OF_1, "Press 1 of 1" },
    { ZB_GP_CMD_RELEASE_1_OF_1, "Release 1 of 1" },
    { ZB_GP_CMD_PRESS_1_OF_2, "Press 1 of 2" },
    { ZB_GP_CMD_RELEASE_1_OF_2, "Release 1 of 2" },
    { ZB_GP_CMD_PRESS_2_OF_2, "Press 2 of 2" },
    { ZB_GP_CMD_RELEASE_2_OF_2, "Release 2 of 2" },
    { ZB_GP_CMD_SHORT_PRESS_1_OF_1, "Short Press 1 of 1" },
    { ZB_GP_CMD_SHORT_PRESS_1_OF_2, "Short Press 1 of 2" },
    { ZB_GP_CMD_SHORT_PRESS_2_OF_2, "Short Press 2 of 2" },
    { ZB_GP_CMD_8BIT_VECTOR_PRESS, "8-bit Vector Press" },
    { ZB_GP_CMD_8BIT_VECTOR_RELEASE, "8-bit Vector Release" },
    { ZB_GP_CMD_ATTRIBUTE_REPORTING, "Attribute Reporting" },
    { ZB_GP_CMD_COMMISSIONING, "Commissioning" },
    { ZB_GP_CMD_DECOMMISSIONING, "Decommissioning" },
    { ZB_GP_CMD_SUCCESS, "Success" },
    { ZB_GP_CMD_CHANNEL_REQUEST, "Channel Request" },
};

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Compare two GP addresses for equality
 */
static bool gp_addr_equal(const zb_gp_addr_t *a, const zb_gp_addr_t *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }

    if (a->app_id != b->app_id) {
        return false;
    }

    if (a->app_id == ZB_GP_APP_ID_SRC_ID) {
        return a->src_id == b->src_id;
    } else if (a->app_id == ZB_GP_APP_ID_IEEE) {
        return (memcmp(a->ieee.ieee_addr, b->ieee.ieee_addr, 8) == 0) &&
               (a->ieee.endpoint == b->ieee.endpoint);
    }

    return false;
}

/**
 * @brief Generate a friendly name for a GP device
 */
static void gp_generate_friendly_name(zb_gp_device_t *device)
{
    if (device == NULL) {
        return;
    }

    const char *type_name = zb_gp_device_type_to_string(device->device_type);

    if (device->gp_addr.app_id == ZB_GP_APP_ID_SRC_ID) {
        snprintf(device->friendly_name, ZB_GP_DEVICE_NAME_LEN,
                 "GP_%s_0x%08lX",
                 type_name,
                 (unsigned long)device->gp_addr.src_id);
    } else {
        snprintf(device->friendly_name, ZB_GP_DEVICE_NAME_LEN,
                 "GP_%s_%02X%02X%02X%02X",
                 type_name,
                 device->gp_addr.ieee.ieee_addr[4],
                 device->gp_addr.ieee.ieee_addr[5],
                 device->gp_addr.ieee.ieee_addr[6],
                 device->gp_addr.ieee.ieee_addr[7]);
    }
}

/**
 * @brief Derive GP security key based on key type
 */
static esp_err_t gp_derive_key(const zb_gp_addr_t *gp_addr,
                                zb_gp_key_type_t key_type,
                                uint8_t out_key[ZB_GP_SECURITY_KEY_SIZE])
{
    if (out_key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (key_type) {
        case ZB_GP_KEY_TYPE_NONE:
            memset(out_key, 0, ZB_GP_SECURITY_KEY_SIZE);
            break;

        case ZB_GP_KEY_TYPE_GP_GROUP_KEY:
            /* Use shared key */
            if (s_gp.shared_key_valid) {
                memcpy(out_key, s_gp.shared_key, ZB_GP_SECURITY_KEY_SIZE);
            } else {
                /* Use default GP shared key */
                uint8_t default_key[] = ZB_GP_DEFAULT_SHARED_KEY;
                memcpy(out_key, default_key, ZB_GP_SECURITY_KEY_SIZE);
            }
            break;

        case ZB_GP_KEY_TYPE_NWK_KEY_DERIVED_GP_GROUP:
            /* TODO: Derive from network key using HMAC-MMO */
            ESP_LOGW(TAG, "Network key derived GP key not yet implemented");
            return ESP_ERR_NOT_SUPPORTED;

        case ZB_GP_KEY_TYPE_DERIVED_INDIVIDUAL_GPD:
            /* TODO: Derive individual key from GPD ID */
            ESP_LOGW(TAG, "Individual GPD key derivation not yet implemented");
            return ESP_ERR_NOT_SUPPORTED;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Find a free slot in the proxy table
 */
static int gp_find_free_proxy_slot(void)
{
    for (int i = 0; i < ZB_GP_MAX_DEVICES; i++) {
        if (!s_gp.proxy_table[i].in_use) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find a device in the proxy table
 */
static int gp_find_device(const zb_gp_addr_t *gp_addr)
{
    for (int i = 0; i < ZB_GP_MAX_DEVICES; i++) {
        if (s_gp.proxy_table[i].in_use &&
            gp_addr_equal(&s_gp.proxy_table[i].gp_addr, gp_addr)) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find a free slot in the translation table
 */
static int gp_find_free_translation_slot(void)
{
    for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES; i++) {
        if (!s_gp.translation_table[i].in_use) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find a translation entry
 */
static int gp_find_translation(const zb_gp_addr_t *gp_addr, uint8_t gp_cmd_id)
{
    for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES; i++) {
        if (s_gp.translation_table[i].in_use &&
            gp_addr_equal(&s_gp.translation_table[i].gp_addr, gp_addr) &&
            s_gp.translation_table[i].gp_cmd_id == gp_cmd_id) {
            return i;
        }
    }
    return -1;
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_gp_init(void)
{
    if (s_gp.initialized) {
        ESP_LOGW(TAG, "GP module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Green Power module");

    /* Create mutex */
    s_gp.mutex = xSemaphoreCreateMutex();
    if (s_gp.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create GP mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize proxy table */
    memset(s_gp.proxy_table, 0, sizeof(s_gp.proxy_table));
    s_gp.device_count = 0;

    /* Initialize translation table */
    memset(s_gp.translation_table, 0, sizeof(s_gp.translation_table));
    s_gp.translation_count = 0;

    /* Initialize commissioning state */
    s_gp.commissioning_state = ZB_GP_COMMISSIONING_STATE_IDLE;
    memset(&s_gp.commissioning_config, 0, sizeof(s_gp.commissioning_config));

    /* Create commissioning timer */
    s_gp.commissioning_timer = xTimerCreate(
        "gp_comm_timer",
        GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS,
        pdTRUE,  /* Auto-reload */
        NULL,
        gp_commissioning_timer_callback
    );
    if (s_gp.commissioning_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create commissioning timer");
        vSemaphoreDelete(s_gp.mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Initialize security with default shared key */
    uint8_t default_key[] = ZB_GP_DEFAULT_SHARED_KEY;
    memcpy(s_gp.shared_key, default_key, ZB_GP_SECURITY_KEY_SIZE);
    s_gp.shared_key_valid = true;

    /* Reset statistics */
    memset(&s_gp.stats, 0, sizeof(s_gp.stats));

    /* Clear callbacks */
    s_gp.frame_callback = NULL;
    s_gp.commissioning_callback = NULL;
    s_gp.device_callback = NULL;

    /* Load saved state from NVS */
    esp_err_t ret = zb_gp_load_from_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded %zu GP devices from NVS", s_gp.device_count);
    } else if (ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to load GP state from NVS: %s", esp_err_to_name(ret));
    }

    s_gp.initialized = true;
    ESP_LOGI(TAG, "Green Power module initialized (max %d devices)", ZB_GP_MAX_DEVICES);

    return ESP_OK;
}

esp_err_t zb_gp_deinit(void)
{
    if (!s_gp.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing Green Power module");

    /* Stop commissioning if active */
    if (s_gp.commissioning_state != ZB_GP_COMMISSIONING_STATE_IDLE) {
        zb_gp_disable_commissioning();
    }

    /* Save state to NVS */
    zb_gp_save_to_nvs();

    /* Delete timer */
    if (s_gp.commissioning_timer != NULL) {
        xTimerDelete(s_gp.commissioning_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS);
        s_gp.commissioning_timer = NULL;
    }

    /* Delete mutex */
    if (s_gp.mutex != NULL) {
        vSemaphoreDelete(s_gp.mutex);
        s_gp.mutex = NULL;
    }

    s_gp.initialized = false;
    return ESP_OK;
}

bool zb_gp_is_initialized(void)
{
    return s_gp.initialized;
}

SemaphoreHandle_t zb_gp_get_mutex(void)
{
    return s_gp.mutex;
}

/* ============================================================================
 * Commissioning Mode
 * ============================================================================ */

/**
 * @brief Commissioning timer callback
 */
static void gp_commissioning_timer_callback(TimerHandle_t timer)
{
    if (!s_gp.initialized ||
        s_gp.commissioning_state != ZB_GP_COMMISSIONING_STATE_ACTIVE) {
        xTimerStop(timer, 0);
        return;
    }

    time_t now = time(NULL);
    time_t elapsed = now - s_gp.commissioning_start_time;

    if (elapsed >= s_gp.commissioning_config.window_sec) {
        ESP_LOGI(TAG, "Commissioning window expired");

        if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_TICKS) == pdTRUE) {
            s_gp.commissioning_state = ZB_GP_COMMISSIONING_STATE_IDLE;
            xSemaphoreGive(s_gp.mutex);
        }

        xTimerStop(timer, 0);

        /* Notify callback */
        if (s_gp.commissioning_callback) {
            s_gp.commissioning_callback(ZB_GP_COMMISSIONING_STATE_IDLE, NULL, NULL);
        }

        /* Save updated state */
        zb_gp_save_to_nvs();
    }
}

esp_err_t zb_gp_enable_commissioning(const zb_gp_commissioning_config_t *config)
{
    if (!s_gp.initialized) {
        ESP_LOGE(TAG, "GP module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Use default config if not provided */
    if (config != NULL) {
        memcpy(&s_gp.commissioning_config, config, sizeof(zb_gp_commissioning_config_t));
    } else {
        memset(&s_gp.commissioning_config, 0, sizeof(zb_gp_commissioning_config_t));
        s_gp.commissioning_config.window_sec = ZB_GP_COMMISSIONING_TIMEOUT_SEC;
        s_gp.commissioning_config.exit_on_pairing_success = false;
    }

    /* Validate window duration */
    if (s_gp.commissioning_config.window_sec == 0) {
        s_gp.commissioning_config.window_sec = ZB_GP_COMMISSIONING_TIMEOUT_SEC;
    }

    s_gp.commissioning_state = ZB_GP_COMMISSIONING_STATE_ACTIVE;
    s_gp.commissioning_start_time = time(NULL);
    s_gp.stats.commissioning_attempts++;

    xSemaphoreGive(s_gp.mutex);

    /* Start timer */
    xTimerStart(s_gp.commissioning_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS);

    ESP_LOGI(TAG, "GP commissioning mode enabled (window: %u sec)",
             s_gp.commissioning_config.window_sec);

    /* Notify callback */
    if (s_gp.commissioning_callback) {
        s_gp.commissioning_callback(ZB_GP_COMMISSIONING_STATE_ACTIVE, NULL, NULL);
    }

    return ESP_OK;
}

esp_err_t zb_gp_disable_commissioning(void)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Stop timer */
    xTimerStop(s_gp.commissioning_timer, GW_DEFAULT_MUTEX_TIMEOUT_TICKS);

    s_gp.commissioning_state = ZB_GP_COMMISSIONING_STATE_IDLE;

    xSemaphoreGive(s_gp.mutex);

    ESP_LOGI(TAG, "GP commissioning mode disabled");

    /* Notify callback */
    if (s_gp.commissioning_callback) {
        s_gp.commissioning_callback(ZB_GP_COMMISSIONING_STATE_IDLE, NULL, NULL);
    }

    return ESP_OK;
}

bool zb_gp_is_commissioning_active(void)
{
    return s_gp.commissioning_state == ZB_GP_COMMISSIONING_STATE_ACTIVE;
}

zb_gp_commissioning_state_t zb_gp_get_commissioning_state(void)
{
    return s_gp.commissioning_state;
}

uint16_t zb_gp_get_commissioning_remaining_time(void)
{
    if (s_gp.commissioning_state != ZB_GP_COMMISSIONING_STATE_ACTIVE) {
        return 0;
    }

    time_t now = time(NULL);
    time_t elapsed = now - s_gp.commissioning_start_time;

    if (elapsed >= s_gp.commissioning_config.window_sec) {
        return 0;
    }

    return (uint16_t)(s_gp.commissioning_config.window_sec - elapsed);
}

/* ============================================================================
 * Device Management
 * ============================================================================ */

esp_err_t zb_gp_add_device(const zb_gp_device_t *device)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Check if device already exists */
    int existing = gp_find_device(&device->gp_addr);
    if (existing >= 0) {
        /* Update existing device */
        memcpy(&s_gp.proxy_table[existing], device, sizeof(zb_gp_device_t));
        s_gp.proxy_table[existing].in_use = true;
        xSemaphoreGive(s_gp.mutex);

        ESP_LOGI(TAG, "Updated existing GP device");
        return ESP_OK;
    }

    /* Find free slot */
    int slot = gp_find_free_proxy_slot();
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        ESP_LOGE(TAG, "GP proxy table full");
        return ESP_ERR_NO_MEM;
    }

    /* Add device */
    memcpy(&s_gp.proxy_table[slot], device, sizeof(zb_gp_device_t));
    s_gp.proxy_table[slot].in_use = true;
    s_gp.proxy_table[slot].commissioned = true;

    /* Generate friendly name if not set */
    if (s_gp.proxy_table[slot].friendly_name[0] == '\0') {
        gp_generate_friendly_name(&s_gp.proxy_table[slot]);
    }

    s_gp.device_count++;

    xSemaphoreGive(s_gp.mutex);

    if (device->gp_addr.app_id == ZB_GP_APP_ID_SRC_ID) {
        ESP_LOGI(TAG, "Added GP device: 0x%08lX (%s)",
                 (unsigned long)device->gp_addr.src_id,
                 s_gp.proxy_table[slot].friendly_name);
    } else {
        ESP_LOGI(TAG, "Added GP device: IEEE (%s)",
                 s_gp.proxy_table[slot].friendly_name);
    }

    /* Notify callback */
    if (s_gp.device_callback) {
        s_gp.device_callback(&s_gp.proxy_table[slot], true);
    }

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_gp_remove_device(const zb_gp_addr_t *gp_addr)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (gp_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int slot = gp_find_device(gp_addr);
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    zb_gp_device_t removed_device;
    memcpy(&removed_device, &s_gp.proxy_table[slot], sizeof(zb_gp_device_t));

    /* Remove device */
    memset(&s_gp.proxy_table[slot], 0, sizeof(zb_gp_device_t));
    s_gp.device_count--;

    /* Remove associated translations */
    for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES; i++) {
        if (s_gp.translation_table[i].in_use &&
            gp_addr_equal(&s_gp.translation_table[i].gp_addr, gp_addr)) {
            memset(&s_gp.translation_table[i], 0, sizeof(zb_gp_translation_entry_t));
            s_gp.translation_count--;
        }
    }

    xSemaphoreGive(s_gp.mutex);

    ESP_LOGI(TAG, "Removed GP device: %s", removed_device.friendly_name);

    /* Notify callback */
    if (s_gp.device_callback) {
        s_gp.device_callback(&removed_device, false);
    }

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_gp_remove_device_by_src_id(zb_gp_src_id_t src_id)
{
    zb_gp_addr_t addr = {
        .app_id = ZB_GP_APP_ID_SRC_ID,
        .src_id = src_id
    };
    return zb_gp_remove_device(&addr);
}

const zb_gp_device_t* zb_gp_get_device(const zb_gp_addr_t *gp_addr)
{
    if (!s_gp.initialized || gp_addr == NULL) {
        return NULL;
    }

    int slot = gp_find_device(gp_addr);
    if (slot < 0) {
        return NULL;
    }

    return &s_gp.proxy_table[slot];
}

const zb_gp_device_t* zb_gp_get_device_by_src_id(zb_gp_src_id_t src_id)
{
    zb_gp_addr_t addr = {
        .app_id = ZB_GP_APP_ID_SRC_ID,
        .src_id = src_id
    };
    return zb_gp_get_device(&addr);
}

size_t zb_gp_get_devices(zb_gp_device_t *devices, size_t max_count)
{
    if (!s_gp.initialized || devices == NULL || max_count == 0) {
        return 0;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return 0;
    }

    size_t count = 0;
    for (int i = 0; i < ZB_GP_MAX_DEVICES && count < max_count; i++) {
        if (s_gp.proxy_table[i].in_use) {
            memcpy(&devices[count], &s_gp.proxy_table[i], sizeof(zb_gp_device_t));
            count++;
        }
    }

    xSemaphoreGive(s_gp.mutex);
    return count;
}

size_t zb_gp_get_device_count(void)
{
    return s_gp.device_count;
}

esp_err_t zb_gp_set_device_name(const zb_gp_addr_t *gp_addr, const char *name)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (gp_addr == NULL || name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t name_len = strlen(name);
    if (name_len >= ZB_GP_DEVICE_NAME_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int slot = gp_find_device(gp_addr);
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(s_gp.proxy_table[slot].friendly_name, name, ZB_GP_DEVICE_NAME_LEN - 1);
    s_gp.proxy_table[slot].friendly_name[ZB_GP_DEVICE_NAME_LEN - 1] = '\0';

    xSemaphoreGive(s_gp.mutex);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

/* ============================================================================
 * Translation Table
 * ============================================================================ */

esp_err_t zb_gp_add_translation(const zb_gp_translation_entry_t *entry)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (entry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Check if entry already exists */
    int existing = gp_find_translation(&entry->gp_addr, entry->gp_cmd_id);
    if (existing >= 0) {
        /* Update existing entry */
        memcpy(&s_gp.translation_table[existing], entry, sizeof(zb_gp_translation_entry_t));
        s_gp.translation_table[existing].in_use = true;
        xSemaphoreGive(s_gp.mutex);
        return ESP_OK;
    }

    /* Find free slot */
    int slot = gp_find_free_translation_slot();
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        ESP_LOGE(TAG, "Translation table full");
        return ESP_ERR_NO_MEM;
    }

    /* Add entry */
    memcpy(&s_gp.translation_table[slot], entry, sizeof(zb_gp_translation_entry_t));
    s_gp.translation_table[slot].in_use = true;
    s_gp.translation_count++;

    xSemaphoreGive(s_gp.mutex);

    ESP_LOGD(TAG, "Added translation: GP cmd 0x%02X -> ZCL cluster 0x%04X cmd 0x%02X",
             entry->gp_cmd_id, entry->zcl_cluster_id, entry->zcl_cmd_id);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_gp_remove_translation(const zb_gp_addr_t *gp_addr, uint8_t gp_cmd_id)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (gp_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int slot = gp_find_translation(gp_addr, gp_cmd_id);
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memset(&s_gp.translation_table[slot], 0, sizeof(zb_gp_translation_entry_t));
    s_gp.translation_count--;

    xSemaphoreGive(s_gp.mutex);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_gp_remove_all_translations(const zb_gp_addr_t *gp_addr)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (gp_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES; i++) {
        if (s_gp.translation_table[i].in_use &&
            gp_addr_equal(&s_gp.translation_table[i].gp_addr, gp_addr)) {
            memset(&s_gp.translation_table[i], 0, sizeof(zb_gp_translation_entry_t));
            s_gp.translation_count--;
        }
    }

    xSemaphoreGive(s_gp.mutex);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

size_t zb_gp_get_translations(const zb_gp_addr_t *gp_addr,
                               zb_gp_translation_entry_t *entries,
                               size_t max_count)
{
    if (!s_gp.initialized || gp_addr == NULL || entries == NULL || max_count == 0) {
        return 0;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return 0;
    }

    size_t count = 0;
    for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES && count < max_count; i++) {
        if (s_gp.translation_table[i].in_use &&
            gp_addr_equal(&s_gp.translation_table[i].gp_addr, gp_addr)) {
            memcpy(&entries[count], &s_gp.translation_table[i],
                   sizeof(zb_gp_translation_entry_t));
            count++;
        }
    }

    xSemaphoreGive(s_gp.mutex);
    return count;
}

/* ============================================================================
 * Frame Handling
 * ============================================================================ */

esp_err_t zb_gp_handle_notification(const zb_gp_frame_t *frame)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_gp.stats.frames_received++;

    /* Look up device */
    const zb_gp_device_t *device = zb_gp_get_device(&frame->gp_addr);
    if (device == NULL) {
        /* Unknown device - check if commissioning mode */
        if (frame->auto_commissioning &&
            s_gp.commissioning_state == ZB_GP_COMMISSIONING_STATE_ACTIVE) {
            ESP_LOGI(TAG, "Auto-commissioning frame from unknown device");
            s_gp.stats.frames_dropped_unknown++;
            /* Would trigger commissioning flow here */
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGD(TAG, "GP frame from unknown device");
        s_gp.stats.frames_dropped_unknown++;
        return ESP_ERR_NOT_FOUND;
    }

    /* Verify security if required */
    if (device->security_level != ZB_GP_SECURITY_LEVEL_NO_SECURITY) {
        esp_err_t ret = zb_gp_verify_security(device, frame);
        if (ret != ESP_OK) {
            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "GP frame replay attack detected");
                s_gp.stats.frames_dropped_replay++;
            } else {
                ESP_LOGW(TAG, "GP frame security verification failed");
                s_gp.stats.frames_dropped_security++;
            }
            return ret;
        }
    }

    /* Update device state (cast away const for internal update) */
    zb_gp_device_t *dev_mut = (zb_gp_device_t *)device;
    dev_mut->last_seen = time(NULL);
    dev_mut->link_quality = frame->lqi;
    dev_mut->rssi = frame->rssi;
    dev_mut->rx_count++;

    /* Update frame counter */
    if (frame->frame_counter > dev_mut->frame_counter) {
        dev_mut->frame_counter = frame->frame_counter;
    }

    /* Process the frame */
    esp_err_t ret = gp_process_frame(device, frame);

    s_gp.stats.frames_processed++;

    return ret;
}

/**
 * @brief Process a validated GP frame
 */
static esp_err_t gp_process_frame(const zb_gp_device_t *device, const zb_gp_frame_t *frame)
{
    ESP_LOGI(TAG, "GP frame from %s: cmd=0x%02X (%s)",
             device->friendly_name, frame->gp_cmd_id,
             zb_gp_cmd_to_string(frame->gp_cmd_id));

    /* Look for matching translation */
    int trans_slot = gp_find_translation(&device->gp_addr, frame->gp_cmd_id);
    if (trans_slot >= 0) {
        s_gp.stats.translations_triggered++;
        esp_err_t ret = gp_execute_translation(device,
                                                &s_gp.translation_table[trans_slot],
                                                frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Translation execution failed: %s", esp_err_to_name(ret));
        }
    }

    /* Call frame callback */
    if (s_gp.frame_callback) {
        s_gp.frame_callback(device, frame);
    }

    /* Publish to MQTT */
    zb_gp_publish_device_state(device, frame);

    return ESP_OK;
}

/**
 * @brief Execute a translation entry (send ZCL command)
 */
static esp_err_t gp_execute_translation(const zb_gp_device_t *device,
                                         const zb_gp_translation_entry_t *entry,
                                         const zb_gp_frame_t *frame)
{
    (void)device;  /* May be used for logging */
    (void)frame;   /* May contain additional payload */

    ESP_LOGD(TAG, "Executing translation: cluster 0x%04X cmd 0x%02X on ep %d",
             entry->zcl_cluster_id, entry->zcl_cmd_id, entry->zcl_endpoint);

    /* TODO: Send ZCL command via Zigbee stack */
    /* This would typically call the Zigbee coordinator to send a ZCL command
     * to the target device or broadcast to a group */

    return ESP_OK;
}

esp_err_t zb_gp_handle_commissioning(const zb_gp_commissioning_frame_t *frame)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gp.commissioning_state != ZB_GP_COMMISSIONING_STATE_ACTIVE) {
        ESP_LOGD(TAG, "Commissioning frame ignored - not in commissioning mode");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Processing GP commissioning frame");

    /* Create new device entry */
    zb_gp_device_t new_device = {0};
    new_device.in_use = true;
    new_device.commissioned = true;
    memcpy(&new_device.gp_addr, &frame->gp_addr, sizeof(zb_gp_addr_t));
    new_device.device_type = frame->device_type;
    new_device.options = frame->options;
    new_device.security_level = frame->security_level;
    new_device.key_type = frame->key_type;
    new_device.frame_counter = frame->frame_counter;
    new_device.last_seen = time(NULL);

    /* Copy key if provided */
    if (frame->has_key) {
        memcpy(&new_device.security_key, &frame->key, sizeof(zb_gp_security_key_t));
        new_device.security_key_valid = true;
    }

    /* Copy command list */
    new_device.num_gp_cmds = frame->num_gp_cmds;
    if (frame->num_gp_cmds > 0 && frame->num_gp_cmds <= ZB_GREEN_POWER_MAX_CMDS) {
        memcpy(new_device.gp_cmd_list, frame->gp_cmd_list, frame->num_gp_cmds);
    }

    /* Copy manufacturer/model IDs */
    new_device.manufacturer_id[0] = frame->manufacturer_id & 0xFF;
    new_device.manufacturer_id[1] = (frame->manufacturer_id >> 8) & 0xFF;
    new_device.model_id[0] = frame->model_id & 0xFF;
    new_device.model_id[1] = (frame->model_id >> 8) & 0xFF;

    /* Generate friendly name */
    gp_generate_friendly_name(&new_device);

    /* Add device */
    esp_err_t ret = zb_gp_add_device(&new_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add commissioned device: %s", esp_err_to_name(ret));

        if (s_gp.commissioning_callback) {
            s_gp.commissioning_callback(ZB_GP_COMMISSIONING_STATE_FAILED, frame, NULL);
        }
        return ret;
    }

    s_gp.stats.commissioning_success++;

    ESP_LOGI(TAG, "GP device commissioned: %s (type: %s)",
             new_device.friendly_name,
             zb_gp_device_type_to_string(new_device.device_type));

    /* Notify callback */
    if (s_gp.commissioning_callback) {
        s_gp.commissioning_callback(ZB_GP_COMMISSIONING_STATE_SUCCESS, frame, &new_device);
    }

    /* Check if we should exit commissioning mode */
    if (s_gp.commissioning_config.exit_on_pairing_success) {
        zb_gp_disable_commissioning();
    }

    return ESP_OK;
}

esp_err_t zb_gp_parse_frame(const uint8_t *data, size_t data_len, zb_gp_frame_t *frame)
{
    if (data == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (data_len < 4) {  /* Minimum: NWK FC + Ext NWK FC + GPD SrcID/Cmd */
        return ESP_ERR_INVALID_SIZE;
    }

    memset(frame, 0, sizeof(zb_gp_frame_t));

    size_t offset = 0;

    /* Network Frame Control */
    uint8_t nwk_fc = data[offset++];
    frame->frame_type = (zb_gp_frame_type_t)(nwk_fc & 0x03);
    frame->nwk_frame_control_ext = (nwk_fc & 0x80) != 0;
    frame->auto_commissioning = (nwk_fc & 0x40) != 0;

    /* Extended Network Frame Control (if present) */
    uint8_t app_id = ZB_GP_APP_ID_SRC_ID;
    uint8_t security_level = 0;
    bool has_security = false;

    if (frame->nwk_frame_control_ext && offset < data_len) {
        uint8_t ext_fc = data[offset++];
        app_id = ext_fc & 0x07;
        security_level = (ext_fc >> 3) & 0x03;
        has_security = (security_level > 0);
        (void)has_security;  /* Used for parsing logic */
    }

    frame->gp_addr.app_id = (zb_gp_app_id_t)app_id;

    /* Source Address */
    if (app_id == ZB_GP_APP_ID_SRC_ID) {
        if (offset + 4 > data_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        frame->gp_addr.src_id = data[offset] |
                                (data[offset + 1] << 8) |
                                (data[offset + 2] << 16) |
                                (data[offset + 3] << 24);
        offset += 4;
    } else if (app_id == ZB_GP_APP_ID_IEEE) {
        if (offset + 9 > data_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(frame->gp_addr.ieee.ieee_addr, &data[offset], 8);
        offset += 8;
        frame->gp_addr.ieee.endpoint = data[offset++];
    }

    /* Security Frame Counter (if security level > 0) */
    if (security_level >= ZB_GP_SECURITY_LEVEL_FULL_COUNTER_MIC) {
        if (offset + 4 > data_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        frame->frame_counter = data[offset] |
                               (data[offset + 1] << 8) |
                               (data[offset + 2] << 16) |
                               (data[offset + 3] << 24);
        offset += 4;
    } else if (security_level == ZB_GP_SECURITY_LEVEL_1LSB_COUNTER_MIC) {
        /* Only 1 LSB of counter */
        if (offset + 1 > data_len) {
            return ESP_ERR_INVALID_SIZE;
        }
        frame->frame_counter = data[offset++];
    }

    /* GP Command ID */
    if (offset >= data_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    frame->gp_cmd_id = data[offset++];

    /* Payload */
    size_t mic_len = 0;
    if (security_level == ZB_GP_SECURITY_LEVEL_1LSB_COUNTER_MIC) {
        mic_len = 2;
    } else if (security_level >= ZB_GP_SECURITY_LEVEL_FULL_COUNTER_MIC) {
        mic_len = 4;
    }

    if (data_len > offset + mic_len) {
        frame->payload_len = data_len - offset - mic_len;
        if (frame->payload_len > ZB_GP_MAX_PAYLOAD_SIZE) {
            frame->payload_len = ZB_GP_MAX_PAYLOAD_SIZE;
        }
        memcpy(frame->payload, &data[offset], frame->payload_len);
        offset += frame->payload_len;
    }

    /* MIC */
    if (mic_len > 0 && offset + mic_len <= data_len) {
        memcpy(frame->mic, &data[offset], mic_len);
    }

    return ESP_OK;
}

/* ============================================================================
 * Security
 * ============================================================================ */

esp_err_t zb_gp_set_shared_key(const uint8_t key[ZB_GP_SECURITY_KEY_SIZE])
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(s_gp.shared_key, key, ZB_GP_SECURITY_KEY_SIZE);
    s_gp.shared_key_valid = true;

    xSemaphoreGive(s_gp.mutex);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    ESP_LOGI(TAG, "GP shared key updated");
    return ESP_OK;
}

esp_err_t zb_gp_get_shared_key(uint8_t key[ZB_GP_SECURITY_KEY_SIZE])
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(key, s_gp.shared_key, ZB_GP_SECURITY_KEY_SIZE);
    return ESP_OK;
}

esp_err_t zb_gp_set_device_key(const zb_gp_addr_t *gp_addr,
                                const uint8_t key[ZB_GP_SECURITY_KEY_SIZE])
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (gp_addr == NULL || key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_gp.mutex, GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int slot = gp_find_device(gp_addr);
    if (slot < 0) {
        xSemaphoreGive(s_gp.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(s_gp.proxy_table[slot].security_key.key, key, ZB_GP_SECURITY_KEY_SIZE);
    s_gp.proxy_table[slot].security_key_valid = true;
    s_gp.proxy_table[slot].key_type = ZB_GP_KEY_TYPE_OUT_OF_BOX_GPD_KEY;

    xSemaphoreGive(s_gp.mutex);

    /* Save to NVS */
    zb_gp_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_gp_verify_security(const zb_gp_device_t *device,
                                 const zb_gp_frame_t *frame)
{
    if (device == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check frame counter for replay protection */
    if (device->security_level >= ZB_GP_SECURITY_LEVEL_FULL_COUNTER_MIC) {
        if (frame->frame_counter <= device->frame_counter) {
            ESP_LOGW(TAG, "Frame counter replay: got %lu, expected > %lu",
                     (unsigned long)frame->frame_counter,
                     (unsigned long)device->frame_counter);
            return ESP_ERR_INVALID_STATE;  /* Replay attack */
        }
    }

    /* Get the key to use */
    uint8_t key[ZB_GP_SECURITY_KEY_SIZE];
    if (device->security_key_valid) {
        memcpy(key, device->security_key.key, ZB_GP_SECURITY_KEY_SIZE);
    } else {
        esp_err_t ret = gp_derive_key(&device->gp_addr, device->key_type, key);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    /* TODO: Implement actual MIC verification using AES-CCM
     * This would require the full frame data and crypto operations */
    (void)key;  /* Silence unused warning for now */

    ESP_LOGD(TAG, "Security verification passed (MIC check not yet implemented)");
    return ESP_OK;
}

/* ============================================================================
 * Callbacks
 * ============================================================================ */

esp_err_t zb_gp_register_frame_callback(zb_gp_frame_callback_t callback)
{
    s_gp.frame_callback = callback;
    return ESP_OK;
}

esp_err_t zb_gp_register_commissioning_callback(zb_gp_commissioning_callback_t callback)
{
    s_gp.commissioning_callback = callback;
    return ESP_OK;
}

esp_err_t zb_gp_register_device_callback(zb_gp_device_callback_t callback)
{
    s_gp.device_callback = callback;
    return ESP_OK;
}

/* ============================================================================
 * Statistics
 * ============================================================================ */

esp_err_t zb_gp_get_stats(zb_gp_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(stats, &s_gp.stats, sizeof(zb_gp_stats_t));
    return ESP_OK;
}

esp_err_t zb_gp_reset_stats(void)
{
    memset(&s_gp.stats, 0, sizeof(zb_gp_stats_t));
    return ESP_OK;
}

const char* zb_gp_device_type_to_string(zb_gp_device_type_t device_type)
{
    for (size_t i = 0; i < sizeof(s_gp_device_type_names) / sizeof(s_gp_device_type_names[0]); i++) {
        if (s_gp_device_type_names[i].type == device_type) {
            return s_gp_device_type_names[i].name;
        }
    }
    return "Unknown";
}

const char* zb_gp_cmd_to_string(uint8_t cmd_id)
{
    for (size_t i = 0; i < sizeof(s_gp_cmd_names) / sizeof(s_gp_cmd_names[0]); i++) {
        if (s_gp_cmd_names[i].cmd_id == cmd_id) {
            return s_gp_cmd_names[i].name;
        }
    }

    /* Check for scene commands */
    if (cmd_id >= ZB_GP_CMD_RECALL_SCENE_0 && cmd_id <= ZB_GP_CMD_RECALL_SCENE_7) {
        return "Recall Scene";
    }
    if (cmd_id >= ZB_GP_CMD_STORE_SCENE_0 && cmd_id <= ZB_GP_CMD_STORE_SCENE_7) {
        return "Store Scene";
    }

    return "Unknown";
}

/* ============================================================================
 * NVS Persistence
 * ============================================================================ */

esp_err_t zb_gp_save_to_nvs(void)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(GP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Save proxy table */
    ret = nvs_set_blob(handle, GP_NVS_KEY_DEVICES,
                       s_gp.proxy_table, sizeof(s_gp.proxy_table));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save proxy table: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    /* Save translation table */
    ret = nvs_set_blob(handle, GP_NVS_KEY_TRANS,
                       s_gp.translation_table, sizeof(s_gp.translation_table));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save translation table: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    /* Save shared key */
    if (s_gp.shared_key_valid) {
        ret = nvs_set_blob(handle, GP_NVS_KEY_SHARED_KEY,
                           s_gp.shared_key, ZB_GP_SECURITY_KEY_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save shared key: %s", esp_err_to_name(ret));
        }
    }

    /* Commit */
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);

    ESP_LOGD(TAG, "GP state saved to NVS (%zu devices, %zu translations)",
             s_gp.device_count, s_gp.translation_count);

    return ret;
}

esp_err_t zb_gp_load_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(GP_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved GP state found");
        return ESP_ERR_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Load proxy table */
    size_t size = sizeof(s_gp.proxy_table);
    ret = nvs_get_blob(handle, GP_NVS_KEY_DEVICES, s_gp.proxy_table, &size);
    if (ret == ESP_OK) {
        /* Count devices */
        s_gp.device_count = 0;
        for (int i = 0; i < ZB_GP_MAX_DEVICES; i++) {
            if (s_gp.proxy_table[i].in_use) {
                s_gp.device_count++;
            }
        }
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to load proxy table: %s", esp_err_to_name(ret));
    }

    /* Load translation table */
    size = sizeof(s_gp.translation_table);
    ret = nvs_get_blob(handle, GP_NVS_KEY_TRANS, s_gp.translation_table, &size);
    if (ret == ESP_OK) {
        /* Count translations */
        s_gp.translation_count = 0;
        for (int i = 0; i < ZB_GP_MAX_TRANSLATION_ENTRIES; i++) {
            if (s_gp.translation_table[i].in_use) {
                s_gp.translation_count++;
            }
        }
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to load translation table: %s", esp_err_to_name(ret));
    }

    /* Load shared key */
    size = ZB_GP_SECURITY_KEY_SIZE;
    ret = nvs_get_blob(handle, GP_NVS_KEY_SHARED_KEY, s_gp.shared_key, &size);
    if (ret == ESP_OK) {
        s_gp.shared_key_valid = true;
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded GP state from NVS (%zu devices, %zu translations)",
             s_gp.device_count, s_gp.translation_count);

    return ESP_OK;
}

esp_err_t zb_gp_clear_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(GP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "GP NVS storage cleared");
    return ret;
}

/* ============================================================================
 * MQTT Integration
 * ============================================================================ */

esp_err_t zb_gp_publish_devices(void)
{
    if (!s_gp.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Build JSON array of devices */
    /* Format: [{"src_id":"0x12345678","friendly_name":"GP_Switch","device_type":"On/Off Switch",...}] */

    char json[2048];
    int offset = 0;
    offset += snprintf(json + offset, sizeof(json) - offset, "[");

    bool first = true;
    for (int i = 0; i < ZB_GP_MAX_DEVICES; i++) {
        if (!s_gp.proxy_table[i].in_use) {
            continue;
        }

        const zb_gp_device_t *dev = &s_gp.proxy_table[i];

        if (!first) {
            offset += snprintf(json + offset, sizeof(json) - offset, ",");
        }
        first = false;

        if (dev->gp_addr.app_id == ZB_GP_APP_ID_SRC_ID) {
            offset += snprintf(json + offset, sizeof(json) - offset,
                "{\"src_id\":\"0x%08lX\","
                "\"friendly_name\":\"%s\","
                "\"device_type\":\"%s\","
                "\"security_level\":%d,"
                "\"commissioned\":%s,"
                "\"rx_count\":%lu}",
                (unsigned long)dev->gp_addr.src_id,
                dev->friendly_name,
                zb_gp_device_type_to_string(dev->device_type),
                dev->security_level,
                dev->commissioned ? "true" : "false",
                (unsigned long)dev->rx_count);
        } else {
            offset += snprintf(json + offset, sizeof(json) - offset,
                "{\"ieee\":\"%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\","
                "\"endpoint\":%d,"
                "\"friendly_name\":\"%s\","
                "\"device_type\":\"%s\","
                "\"security_level\":%d,"
                "\"commissioned\":%s,"
                "\"rx_count\":%lu}",
                dev->gp_addr.ieee.ieee_addr[0], dev->gp_addr.ieee.ieee_addr[1],
                dev->gp_addr.ieee.ieee_addr[2], dev->gp_addr.ieee.ieee_addr[3],
                dev->gp_addr.ieee.ieee_addr[4], dev->gp_addr.ieee.ieee_addr[5],
                dev->gp_addr.ieee.ieee_addr[6], dev->gp_addr.ieee.ieee_addr[7],
                dev->gp_addr.ieee.endpoint,
                dev->friendly_name,
                zb_gp_device_type_to_string(dev->device_type),
                dev->security_level,
                dev->commissioned ? "true" : "false",
                (unsigned long)dev->rx_count);
        }

        if ((size_t)offset >= sizeof(json) - ZB_JSON_BUFFER_SAFETY_MARGIN) {
            ESP_LOGW(TAG, "GP device list JSON truncated");
            break;
        }
    }

    offset += snprintf(json + offset, sizeof(json) - offset, "]");

    ESP_LOGD(TAG, "Publishing GP devices: %s", json);

    /* TODO: Actually publish via MQTT client
     * mqtt_publish("zigbee2mqtt/bridge/greenpower/devices", json, strlen(json), 0, true);
     */

    return ESP_OK;
}

esp_err_t zb_gp_publish_device_state(const zb_gp_device_t *device,
                                      const zb_gp_frame_t *frame)
{
    if (device == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build state JSON based on command */
    char json[512];
    int offset = 0;

    offset += snprintf(json + offset, sizeof(json) - offset,
        "{\"action\":\"%s\"", zb_gp_cmd_to_string(frame->gp_cmd_id));

    /* Add command-specific state */
    switch (frame->gp_cmd_id) {
        case ZB_GP_CMD_ON:
            offset += snprintf(json + offset, sizeof(json) - offset,
                ",\"state\":\"ON\"");
            break;

        case ZB_GP_CMD_OFF:
            offset += snprintf(json + offset, sizeof(json) - offset,
                ",\"state\":\"OFF\"");
            break;

        case ZB_GP_CMD_TOGGLE:
            offset += snprintf(json + offset, sizeof(json) - offset,
                ",\"state\":\"TOGGLE\"");
            break;

        case ZB_GP_CMD_PRESS_1_OF_1:
        case ZB_GP_CMD_PRESS_1_OF_2:
        case ZB_GP_CMD_PRESS_2_OF_2:
        case ZB_GP_CMD_SHORT_PRESS_1_OF_1:
        case ZB_GP_CMD_SHORT_PRESS_1_OF_2:
        case ZB_GP_CMD_SHORT_PRESS_2_OF_2:
            /* Button press - include button number */
            {
                int button = 1;
                if (frame->gp_cmd_id == ZB_GP_CMD_PRESS_2_OF_2 ||
                    frame->gp_cmd_id == ZB_GP_CMD_SHORT_PRESS_2_OF_2) {
                    button = 2;
                }
                offset += snprintf(json + offset, sizeof(json) - offset,
                    ",\"button\":%d", button);
            }
            break;

        case ZB_GP_CMD_8BIT_VECTOR_PRESS:
        case ZB_GP_CMD_8BIT_VECTOR_RELEASE:
            /* Include button vector */
            if (frame->payload_len >= 1) {
                offset += snprintf(json + offset, sizeof(json) - offset,
                    ",\"buttons\":\"0x%02X\"", frame->payload[0]);
            }
            break;

        default:
            break;
    }

    /* Add link quality */
    offset += snprintf(json + offset, sizeof(json) - offset,
        ",\"linkquality\":%d", frame->lqi);

    offset += snprintf(json + offset, sizeof(json) - offset, "}");

    ESP_LOGD(TAG, "Publishing GP state for %s: %s", device->friendly_name, json);

    /* TODO: Actually publish via MQTT client
     * char topic[128];
     * snprintf(topic, sizeof(topic), "zigbee2mqtt/%s", device->friendly_name);
     * mqtt_publish(topic, json, strlen(json), 0, false);
     */

    return ESP_OK;
}

esp_err_t zb_gp_mqtt_handle_commission_request(const char *payload)
{
    ESP_LOGI(TAG, "MQTT commissioning request received");

    /* Parse optional parameters from JSON payload */
    zb_gp_commissioning_config_t config = {0};
    config.window_sec = ZB_GP_COMMISSIONING_TIMEOUT_SEC;

    if (payload != NULL && strlen(payload) > 0) {
        /* TODO: Parse JSON for optional parameters like timeout */
        ESP_LOGD(TAG, "Commissioning payload: %s", payload);
    }

    return zb_gp_enable_commissioning(&config);
}

esp_err_t zb_gp_mqtt_handle_remove_request(const char *gp_id_str)
{
    if (gp_id_str == NULL || strlen(gp_id_str) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "MQTT remove request for GP ID: %s", gp_id_str);

    /* Parse source ID from hex string */
    uint32_t src_id = 0;
    if (gp_id_str[0] == '0' && (gp_id_str[1] == 'x' || gp_id_str[1] == 'X')) {
        src_id = strtoul(gp_id_str + 2, NULL, 16);
    } else {
        src_id = strtoul(gp_id_str, NULL, 16);
    }

    return zb_gp_remove_device_by_src_id(src_id);
}

/* ============================================================================
 * Self-Test
 * ============================================================================ */

esp_err_t zb_gp_self_test(void)
{
    ESP_LOGI(TAG, "Starting GP self-test");

    esp_err_t ret;

    /* Test 1: Verify initialization */
    if (!s_gp.initialized) {
        ESP_LOGE(TAG, "FAIL: Module not initialized");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Module initialized");

    /* Test 2: Add a test device */
    zb_gp_device_t test_device = {0};
    test_device.gp_addr.app_id = ZB_GP_APP_ID_SRC_ID;
    test_device.gp_addr.src_id = 0x12345678;
    test_device.device_type = ZB_GP_DEVICE_TYPE_ON_OFF_SWITCH;
    test_device.security_level = ZB_GP_SECURITY_LEVEL_NO_SECURITY;

    ret = zb_gp_add_device(&test_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Add device failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Device added");

    /* Test 3: Look up device */
    const zb_gp_device_t *found = zb_gp_get_device_by_src_id(0x12345678);
    if (found == NULL) {
        ESP_LOGE(TAG, "FAIL: Device lookup failed");
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Device lookup succeeded");

    /* Test 4: Set device name */
    zb_gp_addr_t addr = { .app_id = ZB_GP_APP_ID_SRC_ID, .src_id = 0x12345678 };
    ret = zb_gp_set_device_name(&addr, "TestSwitch");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Set name failed: %s", esp_err_to_name(ret));
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    found = zb_gp_get_device_by_src_id(0x12345678);
    if (found == NULL || strcmp(found->friendly_name, "TestSwitch") != 0) {
        ESP_LOGE(TAG, "FAIL: Name not set correctly");
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Device name set");

    /* Test 5: Add translation */
    zb_gp_translation_entry_t trans = {0};
    trans.gp_addr = addr;
    trans.gp_cmd_id = ZB_GP_CMD_ON;
    trans.zcl_cluster_id = 0x0006;  /* On/Off cluster */
    trans.zcl_cmd_id = 0x01;        /* On command */

    ret = zb_gp_add_translation(&trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Add translation failed: %s", esp_err_to_name(ret));
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Translation added");

    /* Test 6: Get translations */
    zb_gp_translation_entry_t entries[8];
    size_t count = zb_gp_get_translations(&addr, entries, 8);
    if (count != 1) {
        ESP_LOGE(TAG, "FAIL: Expected 1 translation, got %zu", count);
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Translation retrieved");

    /* Test 7: Commissioning mode */
    ret = zb_gp_enable_commissioning(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Enable commissioning failed: %s", esp_err_to_name(ret));
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    if (!zb_gp_is_commissioning_active()) {
        ESP_LOGE(TAG, "FAIL: Commissioning not active");
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ret = zb_gp_disable_commissioning();
    if (ret != ESP_OK || zb_gp_is_commissioning_active()) {
        ESP_LOGE(TAG, "FAIL: Disable commissioning failed");
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Commissioning mode toggle");

    /* Test 8: Statistics */
    zb_gp_stats_t stats;
    ret = zb_gp_get_stats(&stats);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Get stats failed: %s", esp_err_to_name(ret));
        zb_gp_remove_device_by_src_id(0x12345678);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Statistics retrieved (commissioning attempts: %lu)",
             (unsigned long)stats.commissioning_attempts);

    /* Cleanup: Remove test device */
    ret = zb_gp_remove_device_by_src_id(0x12345678);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Remove device failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Device removed");

    /* Verify device is gone */
    found = zb_gp_get_device_by_src_id(0x12345678);
    if (found != NULL) {
        ESP_LOGE(TAG, "FAIL: Device still exists after removal");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASS: Device removal verified");

    ESP_LOGI(TAG, "GP self-test PASSED (all tests)");
    return ESP_OK;
}
