/**
 * @file zb_device_handler.c
 * @brief Zigbee Device Management Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_device_handler.h"
#include "zb_constants.h"
#include "zb_hvac_dehumid.h"
#include "zb_cluster_hvac.h"
#include "zb_cluster_measurement.h"
#include "zb_cluster_electrical.h"
#include "zb_cluster_security.h"
#include "zb_cluster_closures.h"
#include "zb_tuya.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "ZB_DEV";

/* Device registry */
static zb_device_t *s_device_registry = NULL;
static size_t s_device_count = 0;
static bool s_device_handler_initialized = false;
static SemaphoreHandle_t s_device_mutex = NULL;

/* Mutex debugging - track who holds the mutex */
static const char *s_mutex_holder = NULL;
static TickType_t s_mutex_acquire_time = 0;

/* NVS save debouncing - dirty flags and timer */
static bool s_nvs_dirty_flags[ZB_MAX_DEVICES] = {false};
static TimerHandle_t s_nvs_save_timer = NULL;
static const uint32_t NVS_SAVE_DEBOUNCE_MS = 5000;  /* 5 second debounce */

/* ============================================================================
 * Mutex Debug Helpers
 * ============================================================================ */

/**
 * @brief Acquire mutex with debug tracking
 */
static inline bool mutex_take_debug(const char *func_name, TickType_t timeout)
{
    if (xSemaphoreTake(s_device_mutex, timeout) == pdTRUE) {
        s_mutex_holder = func_name;
        s_mutex_acquire_time = xTaskGetTickCount();
        return true;
    }
    /* Timeout - log who was holding it */
    TickType_t hold_time = (s_mutex_acquire_time > 0) ?
        (xTaskGetTickCount() - s_mutex_acquire_time) * portTICK_PERIOD_MS : 0;
    ESP_LOGE(TAG, "Mutex timeout in %s! Held by: %s for %lu ms",
             func_name, s_mutex_holder ? s_mutex_holder : "unknown",
             (unsigned long)hold_time);
    return false;
}

/**
 * @brief Release mutex with debug tracking
 */
static inline void mutex_give_debug(void)
{
    s_mutex_holder = NULL;
    s_mutex_acquire_time = 0;
    xSemaphoreGive(s_device_mutex);
}

/* ============================================================================
 * Generic State Registry Macros and Helpers
 *
 * These macros reduce code duplication for state storage systems.
 * Each state type (multistate, binary output, binary value) uses the same
 * pattern: arrays for states and addresses, plus a count variable.
 * ============================================================================ */

/**
 * @brief Generic state registry find function
 *
 * Searches for a state entry by short address in the address array.
 *
 * @param addrs Array of short addresses
 * @param count Number of entries in arrays
 * @param short_addr Address to find
 * @return Index if found, -1 otherwise
 */
static inline int generic_find_state_index(const uint16_t *addrs, size_t count,
                                           uint16_t short_addr)
{
    for (size_t i = 0; i < count; i++) {
        if (addrs[i] == short_addr) {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief Macro to define state storage for a state type
 *
 * Generates: s_[name]_states[], s_[name]_addrs[], s_[name]_count, s_[name]_callback
 */
#define DEFINE_STATE_STORAGE(name, type, cb_type, max_count)    \
    static type s_##name##_states[max_count];                   \
    static uint16_t s_##name##_addrs[max_count];                \
    static size_t s_##name##_count = 0;                         \
    static cb_type s_##name##_callback = NULL

/**
 * @brief Macro to implement find function for a simple state type (address-only lookup)
 *
 * For state types that only need address lookup (binary output, binary value).
 */
#define IMPL_FIND_STATE_INDEX_SIMPLE(name)                                  \
    static int find_##name##_state_index(uint16_t short_addr) {             \
        return generic_find_state_index(s_##name##_addrs,                   \
                                        s_##name##_count, short_addr);      \
    }

/**
 * @brief Macro to implement get_or_create function for a simple state type
 *
 * For state types without endpoint-specific handling.
 */
#define IMPL_GET_OR_CREATE_STATE_SIMPLE(name, type, max_count, log_name)            \
    static type* get_or_create_##name##_state(uint16_t short_addr) {                \
        int idx = find_##name##_state_index(short_addr);                            \
        if (idx >= 0) {                                                             \
            return &s_##name##_states[idx];                                         \
        }                                                                           \
        if (s_##name##_count >= (max_count)) {                                      \
            ESP_LOGW(TAG, log_name " state storage full");                          \
            return NULL;                                                            \
        }                                                                           \
        idx = s_##name##_count++;                                                   \
        s_##name##_addrs[idx] = short_addr;                                         \
        memset(&s_##name##_states[idx], 0, sizeof(type));                           \
        ESP_LOGI(TAG, "Created " log_name " state for device 0x%04X", short_addr);  \
        return &s_##name##_states[idx];                                             \
    }

/**
 * @brief Parse binary status flags from ZCL attribute
 *
 * Works for both Binary Output (0x0010) and Binary Value (0x0011) clusters
 * which share the same status flag definitions.
 *
 * @param status_flags Raw status flags byte (bitmap8)
 * @param out_in_alarm Output: in alarm flag (bit 0)
 * @param out_fault Output: fault flag (bit 1)
 * @param out_overridden Output: overridden flag (bit 2)
 * @param out_out_of_service Output: out of service flag (bit 3)
 */
static inline void parse_binary_status_flags_generic(uint8_t status_flags,
                                                     bool *out_in_alarm,
                                                     bool *out_fault,
                                                     bool *out_overridden,
                                                     bool *out_out_of_service)
{
    if (out_in_alarm) {
        *out_in_alarm = (status_flags & ZB_BINARY_STATUS_IN_ALARM) != 0;
    }
    if (out_fault) {
        *out_fault = (status_flags & ZB_BINARY_STATUS_FAULT) != 0;
    }
    if (out_overridden) {
        *out_overridden = (status_flags & ZB_BINARY_STATUS_OVERRIDDEN) != 0;
    }
    if (out_out_of_service) {
        *out_out_of_service = (status_flags & ZB_BINARY_STATUS_OUT_OF_SERVICE) != 0;
    }
}

/* Multistate Input/Output/Value state storage (has endpoint handling, custom impl) */
DEFINE_STATE_STORAGE(multistate, zb_multistate_state_t,
                     zb_multistate_state_cb_t, ZB_STATE_MAX_MULTISTATE);

/* Binary Output state storage - use simple macros */
DEFINE_STATE_STORAGE(binary_output, zb_binary_output_state_t,
                     zb_binary_output_state_cb_t, ZB_STATE_MAX_BINARY_OUTPUT);
IMPL_FIND_STATE_INDEX_SIMPLE(binary_output)
IMPL_GET_OR_CREATE_STATE_SIMPLE(binary_output, zb_binary_output_state_t,
                                ZB_STATE_MAX_BINARY_OUTPUT, "Binary Output")

/* Binary Value state storage - use simple macros */
DEFINE_STATE_STORAGE(binary_value, zb_binary_value_state_t,
                     zb_binary_value_state_cb_t, ZB_STATE_MAX_BINARY_VALUE);
IMPL_FIND_STATE_INDEX_SIMPLE(binary_value)
IMPL_GET_OR_CREATE_STATE_SIMPLE(binary_value, zb_binary_value_state_t,
                                ZB_STATE_MAX_BINARY_VALUE, "Binary Value")

/* Forward declarations */
static zb_device_t* find_device_by_short_addr(uint16_t short_addr);
static zb_device_t* find_device_by_ieee_addr(esp_zb_ieee_addr_t ieee_addr);
static bool ieee_addr_equal(esp_zb_ieee_addr_t addr1, esp_zb_ieee_addr_t addr2);

esp_err_t zb_device_handler_init(void)
{
    if (s_device_handler_initialized) {
        ESP_LOGW(TAG, "Device handler already initialized");
        return ESP_OK;
    }

    /* Create mutex */
    s_device_mutex = xSemaphoreCreateMutex();
    if (s_device_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create device mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate device registry in PSRAM if available, otherwise in heap */
#if CONFIG_SPIRAM
    s_device_registry = heap_caps_calloc(ZB_MAX_DEVICES, sizeof(zb_device_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    s_device_registry = calloc(ZB_MAX_DEVICES, sizeof(zb_device_t));
#endif

    if (s_device_registry == NULL) {
        ESP_LOGE(TAG, "Failed to allocate device registry");
        vSemaphoreDelete(s_device_mutex);
        return ESP_ERR_NO_MEM;
    }

    s_device_count = 0;
    s_device_handler_initialized = true;

    /* Initialize cluster modules */
    esp_err_t ret;

    ret = zb_cluster_hvac_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HVAC cluster init failed: %s", esp_err_to_name(ret));
    }

    ret = zb_cluster_measurement_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Measurement cluster init failed: %s", esp_err_to_name(ret));
    }

    ret = zb_cluster_electrical_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Electrical cluster init failed: %s", esp_err_to_name(ret));
    }

    ret = zb_cluster_security_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Security cluster init failed: %s", esp_err_to_name(ret));
    }

    ret = zb_cluster_closures_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Closures cluster init failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Device handler initialized (max devices: %d)", ZB_MAX_DEVICES);
    return ESP_OK;
}

esp_err_t zb_device_handler_deinit(void)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Deinitialize cluster modules */
    zb_cluster_closures_deinit();
    zb_cluster_security_deinit();
    zb_cluster_electrical_deinit();
    zb_cluster_measurement_deinit();
    zb_cluster_hvac_deinit();

    xSemaphoreTake(s_device_mutex, portMAX_DELAY);

    if (s_device_registry != NULL) {
        free(s_device_registry);
        s_device_registry = NULL;
    }

    s_device_count = 0;
    s_device_handler_initialized = false;

    xSemaphoreGive(s_device_mutex);
    vSemaphoreDelete(s_device_mutex);
    s_device_mutex = NULL;

    ESP_LOGI(TAG, "Device handler deinitialized");
    return ESP_OK;
}

esp_err_t zb_device_add(esp_zb_ieee_addr_t ieee_addr, uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (short_addr == 0xFFFF || short_addr == 0x0000) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!mutex_take_debug("zb_device_add", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    /* Check if device already exists */
    zb_device_t *existing = find_device_by_ieee_addr(ieee_addr);
    if (existing != NULL) {
        /* Update short address */
        existing->short_addr = short_addr;
        existing->online = true;
        existing->last_seen = time(NULL);
        mutex_give_debug();
        ESP_LOGI(TAG, "Updated existing device: 0x%04X", short_addr);
        return ESP_OK;
    }

    /* Check if registry is full */
    if (s_device_count >= ZB_MAX_DEVICES) {
        mutex_give_debug();
        ESP_LOGE(TAG, "Device registry full");
        return ESP_ERR_NO_MEM;
    }

    /* Add new device */
    zb_device_t *device = &s_device_registry[s_device_count];
    memset(device, 0, sizeof(zb_device_t));
    memcpy(device->ieee_addr, ieee_addr, sizeof(esp_zb_ieee_addr_t));
    device->short_addr = short_addr;
    device->endpoint = 1; /* Default endpoint */
    device->online = true;
    device->last_seen = time(NULL);
    device->device_type = ZB_DEVICE_TYPE_UNKNOWN;
    snprintf(device->friendly_name, ZB_DEVICE_FRIENDLY_NAME_LEN,
             "Device_0x%04X", short_addr);

    /* API-005: APS Authentication State
     *
     * Note: ESP-Zigbee-SDK v1.6.x does not provide a per-device APS authentication
     * state API. The esp_zb_aps_is_authenticated() function only checks the local
     * coordinator's APS layer state, not individual devices.
     *
     * For per-device trust/authentication tracking, consider:
     * - Using install codes (zb_install_codes.c) for secure joining
     * - Tracking trust based on successful key exchange during join
     * - Future SDK versions may expose per-device authentication state
     *
     * The authentication state logging is done in zb_callbacks.c:zb_callback_device_join()
     */

    s_device_count++;

    mutex_give_debug();

    ESP_LOGI(TAG, "Added device: 0x%04X (total: %d)", short_addr, s_device_count);
    return ESP_OK;
}

esp_err_t zb_device_remove(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_zb_ieee_addr_t ieee_addr_copy;
    bool found = false;

    if (!mutex_take_debug("zb_device_remove", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    /* Find device */
    for (size_t i = 0; i < s_device_count; i++) {
        if (s_device_registry[i].short_addr == short_addr) {
            /* Copy IEEE address before removal for NVS cleanup */
            memcpy(ieee_addr_copy, s_device_registry[i].ieee_addr, sizeof(esp_zb_ieee_addr_t));
            found = true;

            /* Remove by shifting remaining devices */
            if (i < s_device_count - 1) {
                memmove(&s_device_registry[i], &s_device_registry[i + 1],
                       (s_device_count - i - 1) * sizeof(zb_device_t));
                /* Also shift dirty flags */
                memmove(&s_nvs_dirty_flags[i], &s_nvs_dirty_flags[i + 1],
                       (s_device_count - i - 1) * sizeof(bool));
            }
            s_device_count--;
            s_nvs_dirty_flags[s_device_count] = false;
            break;
        }
    }

    mutex_give_debug();

    if (found) {
        /* Delete from NVS persistence */
        esp_err_t nvs_ret = zb_device_delete_from_nvs(ieee_addr_copy);
        if (nvs_ret != ESP_OK && nvs_ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Failed to delete device from NVS: %s", esp_err_to_name(nvs_ret));
        }
        ESP_LOGI(TAG, "Removed device: 0x%04X (remaining: %zu)", short_addr, s_device_count);
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

zb_device_t* zb_device_get(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return NULL;
    }

    /* Use timeout instead of blocking forever to prevent deadlocks
     * 5 seconds should be enough for any legitimate mutex hold */
    if (!mutex_take_debug("zb_device_get", pdMS_TO_TICKS(5000))) {
        return NULL;
    }
    zb_device_t *device = find_device_by_short_addr(short_addr);
    mutex_give_debug();

    return device;
}

zb_device_t* zb_device_get_by_ieee(esp_zb_ieee_addr_t ieee_addr)
{
    if (!s_device_handler_initialized) {
        return NULL;
    }

    if (!mutex_take_debug("zb_device_get_by_ieee", pdMS_TO_TICKS(5000))) {
        return NULL;
    }
    zb_device_t *device = find_device_by_ieee_addr(ieee_addr);
    mutex_give_debug();

    return device;
}

zb_device_t* zb_device_get_or_log(uint16_t short_addr, const char *context)
{
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGW(TAG, "%s: Device 0x%04X not found",
                 context ? context : "unknown", short_addr);
    }
    return device;
}

size_t zb_device_get_all(zb_device_t *devices, size_t max_count)
{
    if (!s_device_handler_initialized || devices == NULL) {
        return 0;
    }

    if (!mutex_take_debug("zb_device_get_all", pdMS_TO_TICKS(5000))) {
        return 0;
    }

    size_t count = (s_device_count < max_count) ? s_device_count : max_count;
    memcpy(devices, s_device_registry, count * sizeof(zb_device_t));

    mutex_give_debug();

    return count;
}

size_t zb_device_get_count(void)
{
    if (!s_device_handler_initialized) {
        return 0;
    }

    if (!mutex_take_debug("zb_device_get_count", pdMS_TO_TICKS(5000))) {
        return 0;
    }
    size_t count = s_device_count;
    mutex_give_debug();

    return count;
}

size_t zb_device_iterate(zb_device_iterator_cb_t callback, void *user_data)
{
    if (!s_device_handler_initialized || callback == NULL) {
        return 0;
    }

    if (!mutex_take_debug("zb_device_iterate", pdMS_TO_TICKS(5000))) {
        return 0;
    }

    size_t iterated = 0;
    for (size_t i = 0; i < s_device_count; i++) {
        iterated++;
        if (!callback(&s_device_registry[i], user_data)) {
            break;  /* Callback requested early termination */
        }
    }

    mutex_give_debug();

    return iterated;
}

zb_device_t* zb_device_find_by_name(const char *friendly_name)
{
    if (!s_device_handler_initialized || friendly_name == NULL) {
        return NULL;
    }

    if (!mutex_take_debug("zb_device_find_by_name", pdMS_TO_TICKS(5000))) {
        return NULL;
    }

    zb_device_t *found = NULL;
    for (size_t i = 0; i < s_device_count; i++) {
        if (strcmp(s_device_registry[i].friendly_name, friendly_name) == 0) {
            found = &s_device_registry[i];
            break;
        }
    }

    mutex_give_debug();

    return found;
}

zb_device_t* zb_device_get_by_index(size_t index)
{
    if (!s_device_handler_initialized) {
        return NULL;
    }

    if (!mutex_take_debug("zb_device_get_by_index", pdMS_TO_TICKS(5000))) {
        return NULL;
    }

    zb_device_t *device = NULL;
    if (index < s_device_count) {
        device = &s_device_registry[index];
    }

    mutex_give_debug();

    return device;
}

esp_err_t zb_device_copy_by_index(size_t index, zb_device_t *dest)
{
    if (!s_device_handler_initialized || dest == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!mutex_take_debug("zb_device_copy_by_index", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (index < s_device_count) {
        memcpy(dest, &s_device_registry[index], sizeof(zb_device_t));
        ret = ESP_OK;
    }

    mutex_give_debug();

    return ret;
}

esp_err_t zb_device_set_friendly_name(uint16_t short_addr, const char *name)
{
    if (!s_device_handler_initialized || name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_zb_ieee_addr_t ieee_addr_copy;
    if (!mutex_take_debug("zb_device_set_friendly_name", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(device->friendly_name, name, ZB_DEVICE_FRIENDLY_NAME_LEN - 1);
    device->friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN - 1] = '\0';
    memcpy(ieee_addr_copy, device->ieee_addr, sizeof(esp_zb_ieee_addr_t));

    mutex_give_debug();

    ESP_LOGI(TAG, "Device 0x%04X renamed to '%s'", short_addr, name);

    /* Mark for NVS persistence */
    zb_device_mark_dirty(ieee_addr_copy);

    return ESP_OK;
}

esp_err_t zb_device_update_state(uint16_t short_addr, uint8_t endpoint,
                                  uint16_t cluster_id, uint16_t attr_id,
                                  void *value, size_t value_len)
{
    if (!s_device_handler_initialized || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!mutex_take_debug("zb_device_update_state", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    /* Update last seen */
    device->last_seen = time(NULL);
    device->online = true;

    /* Log attribute update */
    ESP_LOGD(TAG, "Device 0x%04X: EP=%d, Cluster=0x%04X, Attr=0x%04X, Len=%d",
             short_addr, endpoint, cluster_id, attr_id, value_len);

    mutex_give_debug();
    return ESP_OK;
}

esp_err_t zb_device_update_last_seen(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_update_last_seen", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    device->last_seen = time(NULL);
    device->online = true;

    /* Update link quality from coordinator's neighbor table */
    esp_zb_nwk_info_iterator_t iterator = 0;
    esp_zb_nwk_neighbor_info_t nbr_info;
    while (esp_zb_nwk_get_next_neighbor(&iterator, &nbr_info) == ESP_OK) {
        if (nbr_info.short_addr == short_addr) {
            device->link_quality = nbr_info.lqi;
            device->rssi = nbr_info.rssi;
            break;
        }
    }

    mutex_give_debug();
    return ESP_OK;
}

esp_err_t zb_device_set_online(uint16_t short_addr, bool online)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_set_online", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    device->online = online;
    if (online) {
        device->last_seen = time(NULL);
    }

    mutex_give_debug();

    ESP_LOGI(TAG, "Device 0x%04X: %s", short_addr, online ? "ONLINE" : "OFFLINE");
    return ESP_OK;
}

esp_err_t zb_device_update_info(uint16_t short_addr, const char *manufacturer,
                                 const char *model)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_update_info", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    if (manufacturer != NULL) {
        strncpy(device->manufacturer, manufacturer, ZB_DEVICE_MANUFACTURER_LEN - 1);
        device->manufacturer[ZB_DEVICE_MANUFACTURER_LEN - 1] = '\0';
    }

    if (model != NULL) {
        strncpy(device->model, model, ZB_DEVICE_MODEL_LEN - 1);
        device->model[ZB_DEVICE_MODEL_LEN - 1] = '\0';
    }

    mutex_give_debug();

    ESP_LOGI(TAG, "Device 0x%04X: Manufacturer='%s', Model='%s'",
             short_addr, manufacturer ? manufacturer : "N/A",
             model ? model : "N/A");
    return ESP_OK;
}

esp_err_t zb_device_add_cluster(uint16_t short_addr, uint16_t cluster_id)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_add_cluster", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if cluster already exists */
    for (uint16_t i = 0; i < device->cluster_count; i++) {
        if (device->clusters[i] == cluster_id) {
            mutex_give_debug();
            return ESP_OK; /* Already exists */
        }
    }

    /* Add cluster if space available */
    if (device->cluster_count < ZB_MAX_CLUSTERS_PER_ENDPOINT) {
        device->clusters[device->cluster_count++] = cluster_id;
        mutex_give_debug();
        ESP_LOGD(TAG, "Device 0x%04X: Added cluster 0x%04X", short_addr, cluster_id);
        return ESP_OK;
    }

    mutex_give_debug();
    return ESP_ERR_NO_MEM;
}

esp_err_t zb_device_determine_type(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_determine_type", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    /* Analyze clusters to determine device type */
    bool has_onoff = false;
    bool has_level = false;
    bool has_color = false;
    bool has_temp = false;
    bool has_humidity = false;
    bool has_occupancy = false;
    bool has_window_covering = false;
    bool has_door_lock = false;
    bool has_illuminance = false;
    bool has_pressure = false;
    bool has_pm25 = false;
    bool has_thermostat = false;
    bool has_fan_control = false;
    bool has_dehumid_control = false;
    bool has_electrical = false;
    bool has_ias_zone = false;
    bool has_metering = false;
    bool has_tuya = false;

    for (uint16_t i = 0; i < device->cluster_count; i++) {
        switch (device->clusters[i]) {
            case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                has_onoff = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                has_level = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
                has_color = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
                has_temp = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
                has_humidity = true;
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING:
                has_occupancy = true;
                break;
            case ZB_ZCL_CLUSTER_ID_WINDOW_COVERING:
                has_window_covering = true;
                break;
            case ZB_ZCL_CLUSTER_ID_DOOR_LOCK:
                has_door_lock = true;
                break;
            case ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT:
                has_illuminance = true;
                break;
            case ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT:
                has_pressure = true;
                break;
            case ZB_ZCL_CLUSTER_ID_PM25_MEASUREMENT:
                has_pm25 = true;
                break;
            case ZB_ZCL_CLUSTER_ID_THERMOSTAT:
                has_thermostat = true;
                break;
            case ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
                has_fan_control = true;
                break;
            case ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL:
                has_dehumid_control = true;
                break;
            case ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT:
                has_electrical = true;
                break;
            case ZB_ZCL_CLUSTER_ID_IAS_ZONE:
                has_ias_zone = true;
                break;
            case ZB_ZCL_CLUSTER_ID_METERING:
                has_metering = true;
                break;
            case ZB_ZCL_CLUSTER_ID_BINARY_OUTPUT:
                /* Binary Output devices are handled as relays/actuators */
                break;
            case ZB_ZCL_CLUSTER_ID_BINARY_VALUE:
                /* Binary Value devices are handled as generic switches */
                break;
            case ZB_TUYA_CLUSTER_ID:
                /* Tuya private cluster - device uses Tuya DP protocol */
                has_tuya = true;
                break;
        }
    }

    /* Determine device type based on clusters */
    /* HVAC devices (thermostat, fan, dehumidifier) have highest priority */
    if (has_thermostat) {
        device->device_type = ZB_DEVICE_TYPE_THERMOSTAT;
    } else if (has_fan_control) {
        device->device_type = ZB_DEVICE_TYPE_FAN;
    } else if (has_dehumid_control) {
        device->device_type = ZB_DEVICE_TYPE_DEHUMIDIFIER;
    } else if (has_door_lock) {
        device->device_type = ZB_DEVICE_TYPE_DOOR_LOCK;
    } else if (has_window_covering) {
        device->device_type = ZB_DEVICE_TYPE_WINDOW_COVERING;
    } else if (has_color && has_level && has_onoff) {
        device->device_type = ZB_DEVICE_TYPE_COLOR_LIGHT;
    } else if (has_level && has_onoff) {
        device->device_type = ZB_DEVICE_TYPE_DIMMABLE_LIGHT;
    } else if (has_tuya && has_onoff && !has_level) {
        /* Tuya devices with On/Off but no Level are switches/actuators (e.g., Fingerbot) */
        device->device_type = ZB_DEVICE_TYPE_ON_OFF_SWITCH;
    } else if (has_onoff && !has_level) {
        /* Generic On/Off only devices are typically switches/relays */
        device->device_type = ZB_DEVICE_TYPE_ON_OFF_SWITCH;
    } else if (has_onoff) {
        /* On/Off with level but no color is a dimmable light (fallback) */
        device->device_type = ZB_DEVICE_TYPE_DIMMABLE_LIGHT;
    } else if (has_metering) {
        /* Metering cluster - default to energy meter, specific type determined by device_type attr */
        device->device_type = ZB_DEVICE_TYPE_ENERGY_METER;
    } else if (has_electrical) {
        /* Electrical measurement - power monitor */
        device->device_type = ZB_DEVICE_TYPE_POWER_MONITOR;
    } else if (has_pm25) {
        /* PM2.5 sensor is a specific air quality sensor type */
        device->device_type = ZB_DEVICE_TYPE_AIR_QUALITY_SENSOR;
    } else if (has_illuminance) {
        device->device_type = ZB_DEVICE_TYPE_ILLUMINANCE_SENSOR;
    } else if (has_pressure) {
        device->device_type = ZB_DEVICE_TYPE_PRESSURE_SENSOR;
    } else if (has_temp) {
        device->device_type = ZB_DEVICE_TYPE_TEMP_SENSOR;
    } else if (has_humidity) {
        device->device_type = ZB_DEVICE_TYPE_HUMIDITY_SENSOR;
    } else if (has_occupancy) {
        device->device_type = ZB_DEVICE_TYPE_MOTION_SENSOR;
    } else if (has_ias_zone) {
        /* IAS Zone - specific type will be determined by zone_type attribute later */
        device->device_type = ZB_DEVICE_TYPE_IAS_ZONE;
    } else {
        device->device_type = ZB_DEVICE_TYPE_OTHER;
    }

    mutex_give_debug();

    ESP_LOGI(TAG, "Device 0x%04X type: %d", short_addr, device->device_type);
    return ESP_OK;
}

esp_err_t zb_device_test(void)
{
    ESP_LOGI(TAG, "Running device handler self-test...");

    if (!s_device_handler_initialized) {
        ESP_LOGE(TAG, "Device handler not initialized");
        return ESP_FAIL;
    }

    /* Test device add */
    esp_zb_ieee_addr_t test_ieee = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint16_t test_short = 0x1234;

    esp_err_t ret = zb_device_add(test_ieee, test_short);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add test device");
        return ESP_FAIL;
    }

    /* Test device get */
    zb_device_t *device = zb_device_get(test_short);
    if (device == NULL) {
        ESP_LOGE(TAG, "Failed to get test device");
        return ESP_FAIL;
    }

    /* Test device update */
    ret = zb_device_set_friendly_name(test_short, "Test Device");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set friendly name");
        return ESP_FAIL;
    }

    /* Test device remove */
    ret = zb_device_remove(test_short);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove test device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Device handler self-test PASSED");
    return ESP_OK;
}

/* Internal helper functions */
static zb_device_t* find_device_by_short_addr(uint16_t short_addr)
{
    for (size_t i = 0; i < s_device_count; i++) {
        if (s_device_registry[i].short_addr == short_addr) {
            return &s_device_registry[i];
        }
    }
    return NULL;
}

/**
 * @brief Check if device has a specific cluster
 *
 * Generic helper to reduce code duplication in zb_device_has_X() functions.
 * This consolidates the common pattern of checking cluster presence.
 *
 * @param[in] short_addr Device short address
 * @param[in] cluster_id Cluster ID to check for
 * @return true if device has the cluster, false otherwise
 */
bool zb_device_has_cluster(uint16_t short_addr, uint16_t cluster_id)
{
    if (!s_device_handler_initialized) {
        return false;
    }

    if (!mutex_take_debug("zb_device_has_cluster", pdMS_TO_TICKS(5000))) {
        return false;
    }

    zb_device_t *device = find_device_by_short_addr(short_addr);
    if (device == NULL) {
        mutex_give_debug();
        return false;
    }

    bool has_cluster = false;
    for (uint16_t i = 0; i < device->cluster_count; i++) {
        if (device->clusters[i] == cluster_id) {
            has_cluster = true;
            break;
        }
    }

    mutex_give_debug();
    return has_cluster;
}

static zb_device_t* find_device_by_ieee_addr(esp_zb_ieee_addr_t ieee_addr)
{
    for (size_t i = 0; i < s_device_count; i++) {
        if (ieee_addr_equal(s_device_registry[i].ieee_addr, ieee_addr)) {
            return &s_device_registry[i];
        }
    }
    return NULL;
}

static bool ieee_addr_equal(esp_zb_ieee_addr_t addr1, esp_zb_ieee_addr_t addr2)
{
    return memcmp(addr1, addr2, sizeof(esp_zb_ieee_addr_t)) == 0;
}

/* ============================================================================
 * Accessor Functions for Cluster Modules (zb_cluster_internal.h)
 * ============================================================================ */

SemaphoreHandle_t zb_device_get_mutex(void)
{
    return s_device_mutex;
}

bool zb_device_handler_is_initialized(void)
{
    return s_device_handler_initialized;
}

bool zb_device_has_cluster_internal(uint16_t short_addr, uint16_t cluster_id)
{
    return zb_device_has_cluster(short_addr, cluster_id);
}

zb_device_t* zb_device_find_by_short_addr_unlocked(uint16_t short_addr)
{
    return find_device_by_short_addr(short_addr);
}

/* ============================================================================
 * Persistent Device Names (NVS Storage) Implementation
 * ============================================================================ */

esp_err_t zb_device_ieee_to_nvs_key(const uint8_t *ieee_addr, char *key_buf, size_t buf_len)
{
    if (ieee_addr == NULL || key_buf == NULL || buf_len < ZB_IEEE_ADDR_KEY_BUFFER_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Convert IEEE address to hex string key (16 chars + null) */
    /* NVS key max is 15 chars, so we use last 7 bytes (14 hex chars) */
    snprintf(key_buf, buf_len, "%02X%02X%02X%02X%02X%02X%02X",
             ieee_addr[6], ieee_addr[5], ieee_addr[4], ieee_addr[3],
             ieee_addr[2], ieee_addr[1], ieee_addr[0]);

    return ESP_OK;
}

esp_err_t zb_device_save_friendly_name(const uint8_t *ieee_addr, const char *name)
{
    if (ieee_addr == NULL || name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_DEVICE_NAMES_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s",
                 ZB_DEVICE_NAMES_NVS_NAMESPACE, esp_err_to_name(ret));
        return ret;
    }

    /* Generate NVS key from IEEE address */
    char key[16];
    zb_device_ieee_to_nvs_key(ieee_addr, key, sizeof(key));

    /* Save the friendly name */
    ret = nvs_set_str(nvs_handle, key, name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save friendly name for key '%s': %s",
                 key, esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    /* Commit the change */
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Saved friendly name '%s' for device %s", name, key);
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_device_load_friendly_name(const uint8_t *ieee_addr, char *name, size_t len)
{
    if (ieee_addr == NULL || name == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_DEVICE_NAMES_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        /* NVS namespace doesn't exist yet - no names stored */
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Generate NVS key from IEEE address */
    char key[16];
    zb_device_ieee_to_nvs_key(ieee_addr, key, sizeof(key));

    /* Load the friendly name */
    size_t required_size = len;
    ret = nvs_get_str(nvs_handle, key, name, &required_size);
    if (ret != ESP_OK) {
        if (ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGD(TAG, "No friendly name stored for device %s", key);
        }
    } else {
        ESP_LOGD(TAG, "Loaded friendly name '%s' for device %s", name, key);
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_device_delete_friendly_name(const uint8_t *ieee_addr)
{
    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_DEVICE_NAMES_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            return ESP_ERR_NOT_FOUND;
        }
        return ret;
    }

    /* Generate NVS key from IEEE address */
    char key[16];
    zb_device_ieee_to_nvs_key(ieee_addr, key, sizeof(key));

    /* Delete the key */
    ret = nvs_erase_key(nvs_handle, key);
    if (ret == ESP_OK) {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Deleted friendly name for device %s", key);
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to delete friendly name for device %s: %s",
                 key, esp_err_to_name(ret));
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t zb_device_load_all_friendly_names(void)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Loading persistent friendly names from NVS...");

    if (!mutex_take_debug("zb_device_load_all_friendly_names", pdMS_TO_TICKS(10000))) {
        return ESP_ERR_TIMEOUT;
    }

    int loaded_count = 0;

    /* Iterate through all devices and try to load their friendly names */
    for (size_t i = 0; i < s_device_count; i++) {
        zb_device_t *device = &s_device_registry[i];

        char stored_name[ZB_DEVICE_FRIENDLY_NAME_LEN];
        esp_err_t ret = zb_device_load_friendly_name(device->ieee_addr,
                                                      stored_name,
                                                      sizeof(stored_name));
        if (ret == ESP_OK) {
            /* Apply the stored name to the device */
            strncpy(device->friendly_name, stored_name, ZB_DEVICE_FRIENDLY_NAME_LEN - 1);
            device->friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN - 1] = '\0';
            loaded_count++;

            char ieee_str[24];
            snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                     device->ieee_addr[7], device->ieee_addr[6],
                     device->ieee_addr[5], device->ieee_addr[4],
                     device->ieee_addr[3], device->ieee_addr[2],
                     device->ieee_addr[1], device->ieee_addr[0]);
            ESP_LOGI(TAG, "Loaded friendly name '%s' for %s",
                     device->friendly_name, ieee_str);
        }
    }

    mutex_give_debug();

    ESP_LOGI(TAG, "Loaded %d persistent friendly names", loaded_count);
    return ESP_OK;
}

/* ============================================================================
 * Device Persistence (NVS) Implementation
 * ============================================================================ */

/**
 * @brief Convert device registry entry to NVS record
 */
static void device_to_nvs_record(const zb_device_t *device, zb_device_nvs_record_t *record)
{
    memset(record, 0, sizeof(zb_device_nvs_record_t));
    memcpy(record->ieee_addr, device->ieee_addr, 8);
    record->endpoint = device->endpoint;
    record->device_type = (uint8_t)device->device_type;
    record->cluster_count = (device->cluster_count > ZB_NVS_MAX_CLUSTERS)
                            ? ZB_NVS_MAX_CLUSTERS : device->cluster_count;
    for (uint16_t i = 0; i < record->cluster_count; i++) {
        record->clusters[i] = device->clusters[i];
    }
    strncpy(record->manufacturer, device->manufacturer, ZB_DEVICE_MANUFACTURER_LEN - 1);
    strncpy(record->model, device->model, ZB_DEVICE_MODEL_LEN - 1);
    strncpy(record->friendly_name, device->friendly_name, ZB_DEVICE_FRIENDLY_NAME_LEN - 1);

    /* Persist power info */
    record->power_mode = device->power_info.current_power_mode;
    record->available_power_sources = device->power_info.available_power_sources;
    record->current_power_source = device->power_info.current_power_source;
    record->current_power_source_level = device->power_info.current_power_source_level;
    record->power_info_valid = device->power_info.power_info_valid ? 1 : 0;
}

/**
 * @brief Convert NVS record to device registry entry
 */
static void nvs_record_to_device(const zb_device_nvs_record_t *record, zb_device_t *device)
{
    memcpy(device->ieee_addr, record->ieee_addr, 8);
    device->short_addr = ZB_SHORT_ADDR_PENDING;  /* Will be updated on communication */
    device->endpoint = record->endpoint;
    device->device_type = (zb_device_type_t)record->device_type;
    device->online = false;  /* Will be updated when device communicates */
    device->last_seen = 0;
    device->link_quality = 0;
    device->rssi = 0;
    device->cluster_count = (record->cluster_count > ZB_NVS_MAX_CLUSTERS)
                            ? ZB_NVS_MAX_CLUSTERS : record->cluster_count;
    for (uint16_t i = 0; i < device->cluster_count; i++) {
        device->clusters[i] = record->clusters[i];
    }
    strncpy(device->manufacturer, record->manufacturer, ZB_DEVICE_MANUFACTURER_LEN - 1);
    device->manufacturer[ZB_DEVICE_MANUFACTURER_LEN - 1] = '\0';
    strncpy(device->model, record->model, ZB_DEVICE_MODEL_LEN - 1);
    device->model[ZB_DEVICE_MODEL_LEN - 1] = '\0';
    strncpy(device->friendly_name, record->friendly_name, ZB_DEVICE_FRIENDLY_NAME_LEN - 1);
    device->friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN - 1] = '\0';

    /* Restore power info (zero for old records without these fields) */
    device->power_info.current_power_mode = record->power_mode;
    device->power_info.available_power_sources = record->available_power_sources;
    device->power_info.current_power_source = record->current_power_source;
    device->power_info.current_power_source_level = record->current_power_source_level;
    device->power_info.power_info_valid = (record->power_info_valid != 0);
}

/**
 * @brief Generate NVS key for device index
 */
static void generate_device_key(uint16_t index, char *key_buf, size_t buf_len)
{
    snprintf(key_buf, buf_len, "%s%04u", ZB_DEVICE_NVS_KEY_PREFIX, index);
}

esp_err_t zb_device_save_to_nvs(const uint8_t *ieee_addr)
{
    if (!s_device_handler_initialized || ieee_addr == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle = 0;

    /* Allocate record on heap to avoid stack overflow in timer task */
    zb_device_nvs_record_t *record = malloc(sizeof(zb_device_nvs_record_t));
    if (record == NULL) {
        ESP_LOGE(TAG, "Failed to allocate NVS record");
        return ESP_ERR_NO_MEM;
    }

    /* Find device in registry */
    if (!mutex_take_debug("zb_device_save_to_nvs", pdMS_TO_TICKS(5000))) {
        free(record);
        return ESP_ERR_TIMEOUT;
    }
    zb_device_t *device = find_device_by_ieee_addr((uint8_t *)ieee_addr);
    if (device == NULL) {
        mutex_give_debug();
        ESP_LOGW(TAG, "Device not found in registry for NVS save");
        free(record);
        return ESP_ERR_NOT_FOUND;
    }

    /* Convert to NVS record */
    device_to_nvs_record(device, record);
    mutex_give_debug();

    /* Open NVS */
    ret = nvs_open(ZB_DEVICE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s",
                 ZB_DEVICE_NVS_NAMESPACE, esp_err_to_name(ret));
        free(record);
        return ret;
    }

    /* Find existing entry or get next index */
    uint16_t device_count = 0;
    ret = nvs_get_u16(nvs_handle, ZB_DEVICE_NVS_KEY_COUNT, &device_count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        device_count = 0;
        ret = ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device count: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Check if device already exists in NVS by scanning IEEE addresses.
     * We must read the full blob because nvs_get_blob fails with
     * ESP_ERR_NVS_INVALID_LENGTH if the buffer is smaller than the stored data. */
    int16_t existing_index = -1;
    zb_device_nvs_record_t *scan_record = malloc(sizeof(zb_device_nvs_record_t));
    if (scan_record != NULL) {
        for (uint16_t i = 0; i < device_count; i++) {
            char scan_key[16];
            generate_device_key(i, scan_key, sizeof(scan_key));

            size_t scan_size = sizeof(zb_device_nvs_record_t);
            ret = nvs_get_blob(nvs_handle, scan_key, scan_record, &scan_size);
            if (ret == ESP_OK && memcmp(scan_record->ieee_addr, ieee_addr, 8) == 0) {
                existing_index = (int16_t)i;
                break;
            }
        }
        free(scan_record);
        ret = ESP_OK;  /* Reset ret from scan loop */
    } else {
        ESP_LOGW(TAG, "Failed to alloc scan buffer, will append as new device");
    }

    /* Generate key for this device */
    char key[16];
    if (existing_index >= 0) {
        generate_device_key((uint16_t)existing_index, key, sizeof(key));
        ESP_LOGD(TAG, "Updating existing NVS record at index %d", existing_index);
    } else {
        /* New device */
        if (device_count >= ZB_MAX_DEVICES) {
            ESP_LOGE(TAG, "NVS device storage full");
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        generate_device_key(device_count, key, sizeof(key));
        device_count++;

        /* Update count */
        ret = nvs_set_u16(nvs_handle, ZB_DEVICE_NVS_KEY_COUNT, device_count);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update device count: %s", esp_err_to_name(ret));
            goto cleanup;
        }
    }

    /* Save the record */
    ret = nvs_set_blob(nvs_handle, key, record, sizeof(zb_device_nvs_record_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save device record: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* Commit */
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        char ieee_str[24];
        snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                 ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
        ESP_LOGI(TAG, "Saved device %s to NVS (key=%s)", ieee_str, key);
    }

cleanup:
    nvs_close(nvs_handle);
    free(record);
    return ret;
}

size_t zb_device_load_all_from_nvs(void)
{
    if (!s_device_handler_initialized) {
        return 0;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_DEVICE_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No persisted devices found in NVS");
        } else {
            ESP_LOGE(TAG, "Failed to open NVS namespace '%s': %s",
                     ZB_DEVICE_NVS_NAMESPACE, esp_err_to_name(ret));
        }
        return 0;
    }

    /* Get device count */
    uint16_t device_count = 0;
    ret = nvs_get_u16(nvs_handle, ZB_DEVICE_NVS_KEY_COUNT, &device_count);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "No device count in NVS");
        nvs_close(nvs_handle);
        return 0;
    }

    ESP_LOGI(TAG, "Loading %u devices from NVS...", device_count);

    size_t loaded = 0;

    for (uint16_t i = 0; i < device_count; i++) {
        char key[16];
        generate_device_key(i, key, sizeof(key));

        /* Read NVS without holding mutex */
        zb_device_nvs_record_t record;
        size_t record_size = sizeof(record);
        ret = nvs_get_blob(nvs_handle, key, &record, &record_size);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read device record %s: %s", key, esp_err_to_name(ret));
            continue;
        }

        /* Now acquire mutex briefly to update registry */
        if (!mutex_take_debug("zb_device_load_all_from_nvs", pdMS_TO_TICKS(5000))) {
            ESP_LOGE(TAG, "Mutex timeout during NVS load");
            continue;
        }

        /* Check capacity */
        if (s_device_count >= ZB_MAX_DEVICES) {
            mutex_give_debug();
            ESP_LOGW(TAG, "Device registry full, stopping NVS load");
            break;
        }

        /* Check if device already exists (by IEEE addr) */
        zb_device_t *existing = find_device_by_ieee_addr(record.ieee_addr);
        if (existing != NULL) {
            /* Device already in registry (from stack reconnect) - update with stored data */
            ESP_LOGD(TAG, "Device already in registry, updating with NVS data");
            strncpy(existing->manufacturer, record.manufacturer, ZB_DEVICE_MANUFACTURER_LEN - 1);
            strncpy(existing->model, record.model, ZB_DEVICE_MODEL_LEN - 1);
            strncpy(existing->friendly_name, record.friendly_name, ZB_DEVICE_FRIENDLY_NAME_LEN - 1);
            existing->device_type = (zb_device_type_t)record.device_type;
            if (existing->cluster_count == 0) {
                existing->cluster_count = record.cluster_count;
                memcpy(existing->clusters, record.clusters, record.cluster_count * sizeof(uint16_t));
            }
            loaded++;
            mutex_give_debug();
            continue;
        }

        /* Add new device to registry */
        zb_device_t *device = &s_device_registry[s_device_count];
        memset(device, 0, sizeof(zb_device_t));
        nvs_record_to_device(&record, device);
        s_device_count++;
        loaded++;

        mutex_give_debug();

        char ieee_str[24];
        snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 record.ieee_addr[7], record.ieee_addr[6],
                 record.ieee_addr[5], record.ieee_addr[4],
                 record.ieee_addr[3], record.ieee_addr[2],
                 record.ieee_addr[1], record.ieee_addr[0]);
        ESP_LOGI(TAG, "Loaded device %s from NVS: '%s' (type=%d, clusters=%d)",
                 ieee_str, device->friendly_name, device->device_type, device->cluster_count);
    }
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Loaded %zu devices from NVS", loaded);
    return loaded;
}

esp_err_t zb_device_delete_from_nvs(const uint8_t *ieee_addr)
{
    if (ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_DEVICE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return (ret == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : ret;
    }

    /* Get device count */
    uint16_t device_count = 0;
    ret = nvs_get_u16(nvs_handle, ZB_DEVICE_NVS_KEY_COUNT, &device_count);
    if (ret != ESP_OK || device_count == 0) {
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /* Find the device in NVS */
    int16_t found_index = -1;
    for (uint16_t i = 0; i < device_count; i++) {
        char key[16];
        generate_device_key(i, key, sizeof(key));

        zb_device_nvs_record_t record;
        size_t record_size = sizeof(record);
        ret = nvs_get_blob(nvs_handle, key, &record, &record_size);
        if (ret == ESP_OK && memcmp(record.ieee_addr, ieee_addr, 8) == 0) {
            found_index = (int16_t)i;
            break;
        }
    }

    if (found_index < 0) {
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /* Move last entry to the deleted slot (if not already last) */
    if ((uint16_t)found_index < device_count - 1) {
        char last_key[16];
        char found_key[16];
        generate_device_key(device_count - 1, last_key, sizeof(last_key));
        generate_device_key((uint16_t)found_index, found_key, sizeof(found_key));

        zb_device_nvs_record_t last_record;
        size_t record_size = sizeof(last_record);
        ret = nvs_get_blob(nvs_handle, last_key, &last_record, &record_size);
        if (ret == ESP_OK) {
            nvs_set_blob(nvs_handle, found_key, &last_record, sizeof(last_record));
        }
        nvs_erase_key(nvs_handle, last_key);
    } else {
        /* Erase the last key */
        char key[16];
        generate_device_key((uint16_t)found_index, key, sizeof(key));
        nvs_erase_key(nvs_handle, key);
    }

    /* Update count */
    device_count--;
    nvs_set_u16(nvs_handle, ZB_DEVICE_NVS_KEY_COUNT, device_count);
    nvs_commit(nvs_handle);

    char ieee_str[24];
    snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
    ESP_LOGI(TAG, "Deleted device %s from NVS", ieee_str);

    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t zb_device_update_short_addr(const uint8_t *ieee_addr, uint16_t new_short_addr)
{
    if (!s_device_handler_initialized || ieee_addr == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!mutex_take_debug("zb_device_update_short_addr", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }
    zb_device_t *device = find_device_by_ieee_addr((uint8_t *)ieee_addr);
    if (device == NULL) {
        mutex_give_debug();
        return ESP_ERR_NOT_FOUND;
    }

    uint16_t old_short_addr = device->short_addr;
    device->short_addr = new_short_addr;
    device->online = true;
    device->last_seen = time(NULL);
    mutex_give_debug();

    if (old_short_addr != new_short_addr) {
        char ieee_str[24];
        snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                 ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
        ESP_LOGI(TAG, "Device %s: short addr updated 0x%04X -> 0x%04X",
                 ieee_str, old_short_addr, new_short_addr);
    }

    return ESP_OK;
}

/**
 * @brief Timer callback to flush dirty devices to NVS
 */
static void nvs_save_timer_callback(TimerHandle_t timer)
{
    (void)timer;
    zb_device_flush_nvs();
}

void zb_device_mark_dirty(const uint8_t *ieee_addr)
{
    if (!s_device_handler_initialized || ieee_addr == NULL) {
        return;
    }

    if (!mutex_take_debug("zb_device_mark_dirty", pdMS_TO_TICKS(5000))) {
        return;
    }

    /* Find device index */
    for (size_t i = 0; i < s_device_count; i++) {
        if (memcmp(s_device_registry[i].ieee_addr, ieee_addr, 8) == 0) {
            s_nvs_dirty_flags[i] = true;
            break;
        }
    }

    mutex_give_debug();

    /* Start or restart debounce timer */
    if (s_nvs_save_timer == NULL) {
        s_nvs_save_timer = xTimerCreate("nvs_save", pdMS_TO_TICKS(NVS_SAVE_DEBOUNCE_MS),
                                        pdFALSE, NULL, nvs_save_timer_callback);
    }
    if (s_nvs_save_timer != NULL) {
        xTimerStart(s_nvs_save_timer, 0);
    }
}

void zb_device_flush_nvs(void)
{
    if (!s_device_handler_initialized) {
        return;
    }

    if (!mutex_take_debug("zb_device_flush_nvs", pdMS_TO_TICKS(5000))) {
        return;
    }

    size_t saved = 0;
    for (size_t i = 0; i < s_device_count; i++) {
        if (s_nvs_dirty_flags[i]) {
            s_nvs_dirty_flags[i] = false;
            /* Release mutex during NVS write to avoid blocking */
            mutex_give_debug();

            esp_err_t ret = zb_device_save_to_nvs(s_device_registry[i].ieee_addr);
            if (ret == ESP_OK) {
                saved++;
            }

            if (!mutex_take_debug("zb_device_flush_nvs_loop", pdMS_TO_TICKS(5000))) {
                ESP_LOGE(TAG, "Mutex timeout during NVS flush loop");
                return;
            }
        }
    }

    mutex_give_debug();

    if (saved > 0) {
        ESP_LOGI(TAG, "Flushed %zu devices to NVS", saved);
    }
}

/* ============================================================================
 * Power Descriptor APIs (API-007)
 * ============================================================================ */

/* Static callback for power descriptor responses */
static zb_power_desc_cb_t s_power_desc_callback = NULL;

/**
 * @brief Internal ZDO power descriptor callback
 *
 * Called by the Zigbee stack when a power descriptor response is received.
 */
static void power_desc_zdo_callback(esp_zb_zdo_power_desc_rsp_t *power_desc, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;

    if (power_desc == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X: Power descriptor response is NULL", short_addr);
        if (s_power_desc_callback != NULL) {
            s_power_desc_callback(short_addr, false, NULL);
        }
        return;
    }

    if (power_desc->status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Device 0x%04X power descriptor: mode=%d, available=0x%02X, "
                 "current=0x%02X, level=%d",
                 short_addr,
                 power_desc->desc.current_power_mode,
                 power_desc->desc.available_power_sources,
                 power_desc->desc.current_power_source,
                 power_desc->desc.current_power_source_level);

        /* Update device power info in device handler */
        if (!mutex_take_debug("power_desc_zdo_callback", pdMS_TO_TICKS(5000))) {
            return;
        }
        zb_device_t *device = find_device_by_short_addr(short_addr);
        if (device != NULL) {
            device->power_info.current_power_mode = power_desc->desc.current_power_mode;
            device->power_info.available_power_sources = power_desc->desc.available_power_sources;
            device->power_info.current_power_source = power_desc->desc.current_power_source;
            device->power_info.current_power_source_level = power_desc->desc.current_power_source_level;
            device->power_info.power_info_valid = true;

            /* Log human-readable info */
            const char *power_str = zb_power_source_to_string(&device->power_info);
            uint8_t battery_pct = zb_power_level_to_percent(device->power_info.current_power_source_level);
            ESP_LOGI(TAG, "Device 0x%04X: Power source = %s, Level = %d%%",
                     short_addr, power_str, battery_pct);

            /* Callback with updated info */
            if (s_power_desc_callback != NULL) {
                s_power_desc_callback(short_addr, true, &device->power_info);
            }
        }
        mutex_give_debug();
    } else {
        ESP_LOGW(TAG, "Device 0x%04X: Power descriptor request failed, status=%d",
                 short_addr, power_desc->status);

        /* Mark power info as invalid */
        if (!mutex_take_debug("power_desc_zdo_callback_fail", pdMS_TO_TICKS(5000))) {
            return;
        }
        zb_device_t *device = find_device_by_short_addr(short_addr);
        if (device != NULL) {
            device->power_info.power_info_valid = false;
        }
        mutex_give_debug();

        if (s_power_desc_callback != NULL) {
            s_power_desc_callback(short_addr, false, NULL);
        }
    }
}

esp_err_t zb_device_request_power_descriptor(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        ESP_LOGE(TAG, "Device handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Verify device exists */
    if (!mutex_take_debug("zb_device_request_power_descriptor", pdMS_TO_TICKS(5000))) {
        return ESP_ERR_TIMEOUT;
    }
    zb_device_t *device = find_device_by_short_addr(short_addr);
    mutex_give_debug();

    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X not found", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Requesting power descriptor from 0x%04X", short_addr);

    /* Send ZDO power descriptor request */
    esp_zb_zdo_power_desc_req_param_t req = {
        .dst_nwk_addr = short_addr
    };

    esp_zb_zdo_power_desc_req(&req, power_desc_zdo_callback,
                               (void *)(uintptr_t)short_addr);

    return ESP_OK;
}

esp_err_t zb_power_desc_register_callback(zb_power_desc_cb_t callback)
{
    s_power_desc_callback = callback;
    return ESP_OK;
}

bool zb_device_is_battery_powered(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return false;
    }

    if (!mutex_take_debug("zb_device_is_battery_powered", pdMS_TO_TICKS(5000))) {
        return false;
    }
    zb_device_t *device = find_device_by_short_addr(short_addr);
    bool is_battery = false;

    if (device != NULL && device->power_info.power_info_valid) {
        /* Check if current power source is a battery type */
        uint8_t source = device->power_info.current_power_source;
        is_battery = (source & ZB_POWER_SOURCE_RECHARGEABLE_BATTERY) ||
                     (source & ZB_POWER_SOURCE_DISPOSABLE_BATTERY);
    }

    mutex_give_debug();
    return is_battery;
}

int zb_device_get_battery_percent(uint16_t short_addr)
{
    if (!s_device_handler_initialized) {
        return -1;
    }

    if (!mutex_take_debug("zb_device_get_battery_percent", pdMS_TO_TICKS(5000))) {
        return -1;
    }
    zb_device_t *device = find_device_by_short_addr(short_addr);
    int percent = -1;

    if (device != NULL && device->power_info.power_info_valid) {
        /* Only return battery percentage if device is battery powered */
        uint8_t source = device->power_info.current_power_source;
        if ((source & ZB_POWER_SOURCE_RECHARGEABLE_BATTERY) ||
            (source & ZB_POWER_SOURCE_DISPOSABLE_BATTERY)) {
            percent = zb_power_level_to_percent(device->power_info.current_power_source_level);
        }
    }

    mutex_give_debug();
    return percent;
}

const char* zb_power_source_to_string(const zb_power_info_t *power_info)
{
    if (power_info == NULL || !power_info->power_info_valid) {
        return "Unknown";
    }

    uint8_t source = power_info->current_power_source;

    if (source & ZB_POWER_SOURCE_MAINS) {
        return "Mains";
    } else if (source & ZB_POWER_SOURCE_RECHARGEABLE_BATTERY) {
        return "Rechargeable Battery";
    } else if (source & ZB_POWER_SOURCE_DISPOSABLE_BATTERY) {
        return "Disposable Battery";
    }

    return "Unknown";
}

uint8_t zb_power_level_to_percent(uint8_t power_level)
{
    /* Power level is bits 0-3 of the current_power_source_level field */
    switch (power_level & 0x0F) {
        case ZB_POWER_LEVEL_CRITICAL:
            return 5;   /* Critical - approximately 5% */
        case ZB_POWER_LEVEL_33_PERCENT:
            return 33;
        case ZB_POWER_LEVEL_66_PERCENT:
            return 66;
        case ZB_POWER_LEVEL_100_PERCENT:
            return 100;
        default:
            /* For intermediate values, interpolate */
            if (power_level < ZB_POWER_LEVEL_33_PERCENT) {
                return 5 + ((power_level * 28) / ZB_POWER_LEVEL_33_PERCENT);
            } else if (power_level < ZB_POWER_LEVEL_66_PERCENT) {
                return 33 + (((power_level - ZB_POWER_LEVEL_33_PERCENT) * 33) /
                            (ZB_POWER_LEVEL_66_PERCENT - ZB_POWER_LEVEL_33_PERCENT));
            } else if (power_level < ZB_POWER_LEVEL_100_PERCENT) {
                return 66 + (((power_level - ZB_POWER_LEVEL_66_PERCENT) * 34) /
                            (ZB_POWER_LEVEL_100_PERCENT - ZB_POWER_LEVEL_66_PERCENT));
            }
            return 0;
    }
}

size_t zb_device_refresh_battery_status(void)
{
    if (!s_device_handler_initialized) {
        return 0;
    }

    size_t request_count = 0;

    if (!mutex_take_debug("zb_device_refresh_battery_status", pdMS_TO_TICKS(5000))) {
        return 0;
    }

    /* Iterate through all devices and refresh battery powered ones */
    for (size_t i = 0; i < s_device_count; i++) {
        zb_device_t *device = &s_device_registry[i];

        /* Check if device is known to be battery powered */
        if (device->power_info.power_info_valid) {
            uint8_t source = device->power_info.current_power_source;
            if ((source & ZB_POWER_SOURCE_RECHARGEABLE_BATTERY) ||
                (source & ZB_POWER_SOURCE_DISPOSABLE_BATTERY)) {
                /* Store short address for request (outside mutex) */
                uint16_t addr = device->short_addr;
                mutex_give_debug();

                /* Send power descriptor request */
                zb_device_request_power_descriptor(addr);
                request_count++;

                /* Add small delay between requests to avoid flooding */
                vTaskDelay(pdMS_TO_TICKS(100));

                if (!mutex_take_debug("zb_device_refresh_battery_status_loop", pdMS_TO_TICKS(5000))) {
                    ESP_LOGE(TAG, "Mutex timeout during battery status refresh");
                    return request_count;
                }
            }
        }
    }

    mutex_give_debug();

    ESP_LOGI(TAG, "Sent power descriptor requests to %d battery devices", request_count);
    return request_count;
}

/* ============================================================================
 * Multistate Input/Output/Value Clusters (0x0012, 0x0013, 0x0014) Implementation
 * ============================================================================ */

/**
 * @brief Find multistate state index for device/endpoint
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint (0 = first match)
 * @return Index in state array or -1 if not found
 */
static int find_multistate_state_index(uint16_t short_addr, uint8_t endpoint)
{
    for (size_t i = 0; i < s_multistate_count; i++) {
        if (s_multistate_addrs[i] == short_addr) {
            /* If endpoint is 0, return first match for this address */
            if (endpoint == 0 || s_multistate_states[i].endpoint == endpoint) {
                return (int)i;
            }
        }
    }
    return -1;
}

/**
 * @brief Get or create multistate state entry
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] type Multistate cluster type
 * @return Pointer to state structure or NULL if full
 */
static zb_multistate_state_t* get_or_create_multistate_state(uint16_t short_addr,
                                                               uint8_t endpoint,
                                                               zb_multistate_type_t type)
{
    /* Check if entry already exists for this device/endpoint */
    int idx = find_multistate_state_index(short_addr, endpoint);
    if (idx >= 0) {
        return &s_multistate_states[idx];
    }

    /* Create new entry */
    if (s_multistate_count >= ZB_STATE_MAX_MULTISTATE) {
        ESP_LOGW(TAG, "Multistate state storage full");
        return NULL;
    }

    idx = s_multistate_count++;
    s_multistate_addrs[idx] = short_addr;
    memset(&s_multistate_states[idx], 0, sizeof(zb_multistate_state_t));
    s_multistate_states[idx].type = type;
    s_multistate_states[idx].endpoint = endpoint;
    s_multistate_states[idx].number_of_states = 0;  /* Unknown until read */
    s_multistate_states[idx].present_value = 0;     /* Unknown until read */

    ESP_LOGI(TAG, "Created multistate state for device 0x%04X EP%d type=%s",
             short_addr, endpoint, zb_multistate_type_to_string(type));
    return &s_multistate_states[idx];
}

esp_err_t zb_multistate_register_callback(zb_multistate_state_cb_t callback)
{
    s_multistate_callback = callback;
    ESP_LOGI(TAG, "Multistate state callback registered");
    return ESP_OK;
}

bool zb_device_has_multistate_input(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT);
}

bool zb_device_has_multistate_output(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_MULTISTATE_OUTPUT);
}

bool zb_device_has_multistate_value(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE);
}

bool zb_device_has_multistate(uint16_t short_addr)
{
    return zb_device_has_multistate_input(short_addr) ||
           zb_device_has_multistate_output(short_addr) ||
           zb_device_has_multistate_value(short_addr);
}

int zb_multistate_get_type_from_cluster(uint16_t cluster_id)
{
    switch (cluster_id) {
        case ZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT:
            return ZB_MULTISTATE_TYPE_INPUT;
        case ZB_ZCL_CLUSTER_ID_MULTISTATE_OUTPUT:
            return ZB_MULTISTATE_TYPE_OUTPUT;
        case ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE:
            return ZB_MULTISTATE_TYPE_VALUE;
        default:
            return -1;
    }
}

const char* zb_multistate_type_to_string(zb_multistate_type_t type)
{
    switch (type) {
        case ZB_MULTISTATE_TYPE_INPUT:
            return "input";
        case ZB_MULTISTATE_TYPE_OUTPUT:
            return "output";
        case ZB_MULTISTATE_TYPE_VALUE:
            return "value";
        default:
            return "unknown";
    }
}

bool zb_multistate_is_in_alarm(const zb_multistate_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    return (state->status_flags & ZB_MULTISTATE_STATUS_IN_ALARM) != 0;
}

bool zb_multistate_has_fault(const zb_multistate_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    return (state->status_flags & ZB_MULTISTATE_STATUS_FAULT) != 0;
}

esp_err_t zb_multistate_read_state(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t cluster_id)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Verify this is a multistate cluster */
    int type = zb_multistate_get_type_from_cluster(cluster_id);
    if (type < 0) {
        ESP_LOGE(TAG, "Invalid multistate cluster ID: 0x%04X", cluster_id);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Reading multistate state from 0x%04X EP%d cluster=0x%04X",
             short_addr, endpoint, cluster_id);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_MULTISTATE_NUMBER_OF_STATES,
        ZB_ZCL_ATTR_MULTISTATE_PRESENT_VALUE,
        ZB_ZCL_ATTR_MULTISTATE_OUT_OF_SERVICE,
        ZB_ZCL_ATTR_MULTISTATE_STATUS_FLAGS
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(attr_ids[0]);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send multistate read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_multistate_set_value(uint16_t short_addr, uint8_t endpoint,
                                   uint16_t cluster_id, uint16_t value)
{
    if (!s_device_handler_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Verify this is a writable multistate cluster (output or value) */
    int type = zb_multistate_get_type_from_cluster(cluster_id);
    if (type < 0) {
        ESP_LOGE(TAG, "Invalid multistate cluster ID: 0x%04X", cluster_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (type == ZB_MULTISTATE_TYPE_INPUT) {
        ESP_LOGE(TAG, "Cannot write to Multistate Input cluster (read-only)");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Setting multistate value on 0x%04X EP%d to %u",
             short_addr, endpoint, value);

    /* Build write attribute request */
    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
    };

    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_MULTISTATE_PRESENT_VALUE,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_U16,
            .size = sizeof(uint16_t),
            .value = (void *)&value,
        },
    };
    cmd_req.attr_number = 1;
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send multistate write request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_multistate_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t cluster_id, uint16_t attr_id,
                                       void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Determine cluster type */
    int type = zb_multistate_get_type_from_cluster(cluster_id);
    if (type < 0) {
        ESP_LOGW(TAG, "Unknown multistate cluster ID: 0x%04X", cluster_id);
        return ESP_ERR_INVALID_ARG;
    }

    /* Get or create state entry */
    zb_multistate_state_t *state = get_or_create_multistate_state(short_addr, endpoint,
                                                                    (zb_multistate_type_t)type);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Process attribute */
    switch (attr_id) {
        case ZB_ZCL_ATTR_MULTISTATE_NUMBER_OF_STATES:
            if (value_len >= 2) {
                state->number_of_states = *(uint16_t *)value;
                ESP_LOGI(TAG, "Multistate 0x%04X EP%d: NumberOfStates=%u",
                         short_addr, endpoint, state->number_of_states);
            }
            break;

        case ZB_ZCL_ATTR_MULTISTATE_PRESENT_VALUE:
            if (value_len >= 2) {
                uint16_t new_value = *(uint16_t *)value;
                if (new_value != state->present_value) {
                    ESP_LOGI(TAG, "Multistate 0x%04X EP%d: PresentValue changed %u -> %u",
                             short_addr, endpoint, state->present_value, new_value);
                }
                state->present_value = new_value;
            }
            break;

        case ZB_ZCL_ATTR_MULTISTATE_OUT_OF_SERVICE:
            if (value_len >= 1) {
                state->out_of_service = (*(uint8_t *)value != 0);
                ESP_LOGD(TAG, "Multistate 0x%04X EP%d: OutOfService=%s",
                         short_addr, endpoint, state->out_of_service ? "true" : "false");
            }
            break;

        case ZB_ZCL_ATTR_MULTISTATE_STATUS_FLAGS:
            if (value_len >= 1) {
                state->status_flags = *(uint8_t *)value;
                ESP_LOGD(TAG, "Multistate 0x%04X EP%d: StatusFlags=0x%02X (alarm=%d fault=%d)",
                         short_addr, endpoint, state->status_flags,
                         zb_multistate_is_in_alarm(state),
                         zb_multistate_has_fault(state));
            }
            break;

        default:
            ESP_LOGD(TAG, "Multistate 0x%04X EP%d: unhandled attr 0x%04X",
                     short_addr, endpoint, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_multistate_callback != NULL) {
        s_multistate_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_multistate_get_state(uint16_t short_addr, uint8_t endpoint,
                                   zb_multistate_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_multistate_state_index(short_addr, endpoint);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_multistate_states[idx], sizeof(zb_multistate_state_t));
    return ESP_OK;
}

/* ============================================================================
 * Binary Output Cluster (0x0010) Implementation
 *
 * Note: find_binary_output_state_index() and get_or_create_binary_output_state()
 * are generated by IMPL_FIND_STATE_INDEX_SIMPLE and IMPL_GET_OR_CREATE_STATE_SIMPLE
 * macros at the top of this file.
 * ============================================================================ */

esp_err_t zb_binary_output_register_callback(zb_binary_output_state_cb_t callback)
{
    s_binary_output_callback = callback;
    ESP_LOGI(TAG, "Binary Output state callback registered");
    return ESP_OK;
}

bool zb_device_has_binary_output(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_BINARY_OUTPUT);
}

esp_err_t zb_binary_output_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading Binary Output state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_BINARY_OUTPUT,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_BINARY_OUT_OF_SERVICE_ID,
        ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID,
        ZB_ZCL_ATTR_BINARY_STATUS_FLAGS_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Binary Output read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_binary_output_set_value(uint16_t short_addr, uint8_t endpoint, bool value)
{
    ESP_LOGI(TAG, "Setting Binary Output to %s for 0x%04X EP%d",
             value ? "ON" : "OFF", short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_BINARY_OUTPUT,
        .attr_number = 1,
    };

    uint8_t bool_value = value ? 1 : 0;
    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            .size = sizeof(uint8_t),
            .value = (void *)&bool_value,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Binary Output value: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_binary_output_handle_report(uint16_t short_addr, uint8_t endpoint,
                                          uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_binary_output_state_t *state = get_or_create_binary_output_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID:
            if (value_len >= 1) {
                state->present_value = *(uint8_t *)value != 0;
                ESP_LOGI(TAG, "Binary Output 0x%04X: state=%s",
                         short_addr, state->present_value ? "ON" : "OFF");
            }
            break;

        case ZB_ZCL_ATTR_BINARY_OUT_OF_SERVICE_ID:
            if (value_len >= 1) {
                state->out_of_service = *(uint8_t *)value != 0;
                ESP_LOGD(TAG, "Binary Output 0x%04X: out_of_service=%d",
                         short_addr, state->out_of_service);
            }
            break;

        case ZB_ZCL_ATTR_BINARY_STATUS_FLAGS_ID:
            if (value_len >= 1) {
                state->status_flags = *(uint8_t *)value;
                parse_binary_status_flags_generic(state->status_flags,
                                                  &state->in_alarm,
                                                  &state->fault,
                                                  &state->overridden,
                                                  &state->out_of_service);
                ESP_LOGD(TAG, "Binary Output 0x%04X: status_flags=0x%02X",
                         short_addr, state->status_flags);
            }
            break;

        default:
            ESP_LOGD(TAG, "Binary Output 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_binary_output_callback != NULL) {
        s_binary_output_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_binary_output_get_state(uint16_t short_addr, zb_binary_output_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_binary_output_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_binary_output_states[idx], sizeof(zb_binary_output_state_t));
    return ESP_OK;
}

/* ============================================================================
 * Binary Value Cluster (0x0011) Implementation
 *
 * Note: find_binary_value_state_index() and get_or_create_binary_value_state()
 * are generated by IMPL_FIND_STATE_INDEX_SIMPLE and IMPL_GET_OR_CREATE_STATE_SIMPLE
 * macros at the top of this file.
 * ============================================================================ */

esp_err_t zb_binary_value_register_callback(zb_binary_value_state_cb_t callback)
{
    s_binary_value_callback = callback;
    ESP_LOGI(TAG, "Binary Value state callback registered");
    return ESP_OK;
}

bool zb_device_has_binary_value(uint16_t short_addr)
{
    return zb_device_has_cluster(short_addr, ZB_ZCL_CLUSTER_ID_BINARY_VALUE);
}

esp_err_t zb_binary_value_read_state(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Reading Binary Value state from 0x%04X EP%d", short_addr, endpoint);

    /* Build read attributes request */
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_BINARY_VALUE,
    };

    uint16_t attr_ids[] = {
        ZB_ZCL_ATTR_BINARY_OUT_OF_SERVICE_ID,
        ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID,
        ZB_ZCL_ATTR_BINARY_STATUS_FLAGS_ID
    };
    cmd_req.attr_number = sizeof(attr_ids) / sizeof(uint16_t);
    cmd_req.attr_field = attr_ids;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Binary Value read request: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_binary_value_set_value(uint16_t short_addr, uint8_t endpoint, bool value)
{
    ESP_LOGI(TAG, "Setting Binary Value to %s for 0x%04X EP%d",
             value ? "ON" : "OFF", short_addr, endpoint);

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        ZCL_BASIC_CMD_INIT(short_addr, endpoint),
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ZB_ZCL_CLUSTER_ID_BINARY_VALUE,
        .attr_number = 1,
    };

    uint8_t bool_value = value ? 1 : 0;
    esp_zb_zcl_attribute_t attr = {
        .id = ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            .size = sizeof(uint8_t),
            .value = (void *)&bool_value,
        },
    };
    cmd_req.attr_field = &attr;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Binary Value: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_binary_value_handle_report(uint16_t short_addr, uint8_t endpoint,
                                         uint16_t attr_id, void *value, size_t value_len)
{
    if (value == NULL || value_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_binary_value_state_t *state = get_or_create_binary_value_state(short_addr);
    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    switch (attr_id) {
        case ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID:
            if (value_len >= 1) {
                state->present_value = *(uint8_t *)value != 0;
                ESP_LOGI(TAG, "Binary Value 0x%04X: value=%s",
                         short_addr, state->present_value ? "ON" : "OFF");
            }
            break;

        case ZB_ZCL_ATTR_BINARY_OUT_OF_SERVICE_ID:
            if (value_len >= 1) {
                state->out_of_service = *(uint8_t *)value != 0;
                ESP_LOGD(TAG, "Binary Value 0x%04X: out_of_service=%d",
                         short_addr, state->out_of_service);
            }
            break;

        case ZB_ZCL_ATTR_BINARY_STATUS_FLAGS_ID:
            if (value_len >= 1) {
                state->status_flags = *(uint8_t *)value;
                parse_binary_status_flags_generic(state->status_flags,
                                                  &state->in_alarm,
                                                  &state->fault,
                                                  &state->overridden,
                                                  &state->out_of_service);
                ESP_LOGD(TAG, "Binary Value 0x%04X: status_flags=0x%02X",
                         short_addr, state->status_flags);
            }
            break;

        default:
            ESP_LOGD(TAG, "Binary Value 0x%04X: unhandled attr 0x%04X", short_addr, attr_id);
            return ESP_OK;
    }

    /* Invoke callback if registered */
    if (s_binary_value_callback != NULL) {
        s_binary_value_callback(short_addr, endpoint, state);
    }

    return ESP_OK;
}

esp_err_t zb_binary_value_get_state(uint16_t short_addr, zb_binary_value_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_binary_value_state_index(short_addr);
    if (idx < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(state, &s_binary_value_states[idx], sizeof(zb_binary_value_state_t));
    return ESP_OK;
}
