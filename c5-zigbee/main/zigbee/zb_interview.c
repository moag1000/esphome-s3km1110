/**
 * @file zb_interview.c
 * @brief Zigbee Device Interview Implementation (ZG-001)
 *
 * This module implements the device interview process which discovers
 * endpoints, clusters, and attributes of newly joined Zigbee devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_interview.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "zb_zcl_helpers.h"
#include "core/compat_stubs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

#if CONFIG_BT_SCANNER_ENABLED
#endif

static const char *TAG = "ZB_INTERVIEW";

/** @brief Track whether BLE was paused for an interview */
#if CONFIG_BT_SCANNER_ENABLED
static bool s_ble_paused_for_interview = false;
#endif

/** @brief Retries for entire interview on timeout */
#define ZB_INTERVIEW_FULL_RETRY_COUNT  2

/** @brief Basic cluster attribute IDs for manufacturer/model reading */
#define ZB_BASIC_ATTR_MANUFACTURER_NAME  0x0004
#define ZB_BASIC_ATTR_MODEL_IDENTIFIER   0x0005

/** @brief Timeout for Basic cluster read response in milliseconds */
#define ZB_BASIC_CLUSTER_READ_TIMEOUT_MS  5000

/** @brief Flags for tracking which Basic cluster attributes we've received */
#define ZB_BASIC_READ_MANUFACTURER_RECEIVED  0x01
#define ZB_BASIC_READ_MODEL_RECEIVED         0x02
#define ZB_BASIC_READ_ALL_RECEIVED          (ZB_BASIC_READ_MANUFACTURER_RECEIVED | ZB_BASIC_READ_MODEL_RECEIVED)

/**
 * @brief Interview context structure for tracking active interviews
 */
typedef struct {
    uint64_t ieee_addr;                     /**< Device IEEE address */
    uint16_t short_addr;                    /**< Device short address */
    zb_interview_status_t status;           /**< Current status */
    zb_interview_result_t result;           /**< Interview result */
    uint8_t current_endpoint_idx;           /**< Current endpoint being processed */
    uint8_t retry_count;                    /**< Remaining retries for current request */
    uint8_t interview_retry_count;          /**< Remaining retries for entire interview */
    uint8_t endpoints[ZB_INTERVIEW_MAX_ENDPOINTS]; /**< Discovered endpoints */
    uint8_t endpoint_count;                 /**< Number of discovered endpoints */
    int64_t start_time;                     /**< Interview start time (us) */
    TimerHandle_t timeout_timer;            /**< Timeout timer handle */
    TimerHandle_t basic_read_timer;         /**< Timer for Basic cluster read timeout */
    uint8_t basic_read_flags;               /**< Flags tracking received Basic cluster attrs */
    bool in_use;                            /**< Context slot in use flag */
} zb_interview_context_t;

/* Static variables with s_ prefix */
static bool s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;
static zb_interview_config_t s_config;
static zb_interview_context_t s_contexts[ZB_INTERVIEW_MAX_CONCURRENT];
static zb_interview_result_t *s_cached_results = NULL;
static uint8_t s_cached_result_count = 0;
static uint8_t s_max_cached_results = 50;

/* Forward declarations */
static zb_interview_context_t* find_context_by_ieee(uint64_t ieee_addr);
static zb_interview_context_t* find_context_by_short(uint16_t short_addr);
static zb_interview_context_t* allocate_context(void);
static void free_context(zb_interview_context_t *ctx);
static void free_endpoint_info(zb_endpoint_info_t *ep_info);
static void free_interview_result_data(zb_interview_result_t *result);
static esp_err_t request_active_endpoints(zb_interview_context_t *ctx);
static esp_err_t request_simple_descriptor(zb_interview_context_t *ctx, uint8_t endpoint);
static esp_err_t request_power_descriptor(zb_interview_context_t *ctx);
static esp_err_t request_basic_cluster_attributes(zb_interview_context_t *ctx);
static void interview_complete(zb_interview_context_t *ctx, zb_interview_status_t status);
static void timeout_callback(TimerHandle_t timer);
static void basic_read_timeout_callback(TimerHandle_t timer);
static void report_progress(zb_interview_context_t *ctx, uint8_t progress);
static esp_err_t cache_result(zb_interview_result_t *result);
static zb_interview_result_t* find_cached_result(uint64_t ieee_addr);
static void complete_basic_cluster_read(zb_interview_context_t *ctx);

/**
 * @brief Active endpoints response callback for ZDO
 */
static void active_ep_callback(esp_zb_zdp_status_t status, uint8_t ep_count,
                                uint8_t *ep_list, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    zb_interview_handle_active_ep_resp(short_addr, status, ep_list, ep_count);
}

/**
 * @brief Simple descriptor response callback for ZDO
 */
static void simple_desc_callback(esp_zb_zdp_status_t status,
                                  esp_zb_af_simple_desc_1_1_t *simple_desc,
                                  void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;
    zb_interview_handle_simple_desc_resp(short_addr, status, simple_desc);
}

/**
 * @brief Power descriptor response callback for ZDO (API-007)
 *
 * Thread-safe callback that handles the power descriptor response
 * and updates the device's power information.
 */
static void power_desc_callback(esp_zb_zdo_power_desc_rsp_t *power_desc, void *user_ctx)
{
    uint16_t short_addr = (uint16_t)(uintptr_t)user_ctx;

    if (power_desc == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X: Power descriptor response is NULL", short_addr);
        /* Continue interview even if power descriptor is NULL */
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        zb_interview_context_t *ctx = find_context_by_short(short_addr);
        if (ctx != NULL && ctx->in_use) {
            ctx->status = ZB_INTERVIEW_STATUS_BASIC_INFO;
            ctx->result.status = ZB_INTERVIEW_STATUS_BASIC_INFO;
            report_progress(ctx, 90);
            xSemaphoreGive(s_mutex);
            request_basic_cluster_attributes(ctx);
        } else {
            xSemaphoreGive(s_mutex);
        }
        return;
    }

    if (power_desc->status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        /* Log power descriptor information */
        ESP_LOGI(TAG, "Device 0x%04X power descriptor: mode=%d, available=0x%02X, "
                 "current=0x%02X, level=%d",
                 short_addr,
                 power_desc->desc.current_power_mode,
                 power_desc->desc.available_power_sources,
                 power_desc->desc.current_power_source,
                 power_desc->desc.current_power_source_level);

        /* Log human-readable power source info */
        const char *power_source_str = "Unknown";
        if (power_desc->desc.current_power_source & ESP_ZB_AF_NODE_POWER_SOURCE_CONSTANT_POWER) {
            power_source_str = "Mains";
        } else if (power_desc->desc.current_power_source & ESP_ZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY) {
            power_source_str = "Rechargeable Battery";
        } else if (power_desc->desc.current_power_source & ESP_ZB_AF_NODE_POWER_SOURCE_DISPOSABLE_BATTERY) {
            power_source_str = "Disposable Battery";
        }

        uint8_t battery_percent = 0;
        switch (power_desc->desc.current_power_source_level) {
            case ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_CRITICAL:
                battery_percent = 5;  /* Critical level */
                break;
            case ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_33_PERCENT:
                battery_percent = 33;
                break;
            case ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_66_PERCENT:
                battery_percent = 66;
                break;
            case ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT:
                battery_percent = 100;
                break;
            default:
                battery_percent = 0;
                break;
        }

        ESP_LOGI(TAG, "Device 0x%04X: Power source = %s, Battery level = %d%%",
                 short_addr, power_source_str, battery_percent);

        /* Update device power info in device handler */
        zb_device_t *device = zb_device_get(short_addr);
        if (device != NULL) {
            device->power_info.current_power_mode = power_desc->desc.current_power_mode;
            device->power_info.available_power_sources = power_desc->desc.available_power_sources;
            device->power_info.current_power_source = power_desc->desc.current_power_source;
            device->power_info.current_power_source_level = power_desc->desc.current_power_source_level;
            device->power_info.power_info_valid = true;
            ESP_LOGI(TAG, "Device 0x%04X: Power info updated in device registry", short_addr);
        }
    } else {
        ESP_LOGW(TAG, "Device 0x%04X: Power descriptor request failed, status=%d",
                 short_addr, power_desc->status);

        /* Mark power info as invalid but continue interview */
        zb_device_t *device = zb_device_get(short_addr);
        if (device != NULL) {
            device->power_info.power_info_valid = false;
        }
    }

    /* Continue to basic cluster attributes phase */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    zb_interview_context_t *ctx = find_context_by_short(short_addr);
    if (ctx != NULL && ctx->in_use) {
        /* Move to basic info phase and complete */
        ctx->status = ZB_INTERVIEW_STATUS_BASIC_INFO;
        ctx->result.status = ZB_INTERVIEW_STATUS_BASIC_INFO;
        report_progress(ctx, 90);
        xSemaphoreGive(s_mutex);

        /* Continue with basic cluster attributes */
        request_basic_cluster_attributes(ctx);
    } else {
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t zb_interview_init(const zb_interview_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Interview module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create mutex */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create interview mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize configuration with defaults or provided config */
    if (config != NULL) {
        memcpy(&s_config, config, sizeof(zb_interview_config_t));
    } else {
        memset(&s_config, 0, sizeof(zb_interview_config_t));
        s_config.timeout_ms = ZB_INTERVIEW_TIMEOUT_MS;
        s_config.retry_count = ZB_INTERVIEW_RETRY_COUNT;
        s_config.read_attributes = false;
        s_config.auto_interview_on_join = true;
    }

    /* Apply defaults for zero values */
    if (s_config.timeout_ms == 0) {
        s_config.timeout_ms = ZB_INTERVIEW_TIMEOUT_MS;
    }
    /* Cap maximum timeout to prevent excessively long waits */
#define ZB_INTERVIEW_MAX_TIMEOUT_MS  (5 * 60 * 1000)  /* 5 minutes */
    if (s_config.timeout_ms > ZB_INTERVIEW_MAX_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Timeout %lu ms exceeds max, capping to %d ms",
                 (unsigned long)s_config.timeout_ms, ZB_INTERVIEW_MAX_TIMEOUT_MS);
        s_config.timeout_ms = ZB_INTERVIEW_MAX_TIMEOUT_MS;
    }
    if (s_config.retry_count == 0) {
        s_config.retry_count = ZB_INTERVIEW_RETRY_COUNT;
    }

    /* Initialize context array */
    memset(s_contexts, 0, sizeof(s_contexts));

    /* Allocate cached results array - prefer PSRAM to free internal RAM for WiFi */
    s_cached_results = heap_caps_calloc(s_max_cached_results, sizeof(zb_interview_result_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_cached_results == NULL) {
        /* Fallback to internal RAM if PSRAM not available */
        s_cached_results = calloc(s_max_cached_results, sizeof(zb_interview_result_t));
    }
    if (s_cached_results == NULL) {
        ESP_LOGE(TAG, "Failed to allocate cached results array");
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Interview cache allocated: %zu bytes",
             s_max_cached_results * sizeof(zb_interview_result_t));
    s_cached_result_count = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "Interview module initialized (timeout=%lums, retries=%d)",
             (unsigned long)s_config.timeout_ms, s_config.retry_count);

    return ESP_OK;
}

esp_err_t zb_interview_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Cancel all active interviews */
    for (int i = 0; i < ZB_INTERVIEW_MAX_CONCURRENT; i++) {
        if (s_contexts[i].in_use) {
            if (s_contexts[i].timeout_timer != NULL) {
                xTimerStop(s_contexts[i].timeout_timer, 0);
                xTimerDelete(s_contexts[i].timeout_timer, 0);
            }
            free_interview_result_data(&s_contexts[i].result);
        }
    }
    memset(s_contexts, 0, sizeof(s_contexts));

    /* Free cached results */
    if (s_cached_results != NULL) {
        for (uint8_t i = 0; i < s_cached_result_count; i++) {
            free_interview_result_data(&s_cached_results[i]);
        }
        free(s_cached_results);
        s_cached_results = NULL;
    }
    s_cached_result_count = 0;

    s_initialized = false;

    xSemaphoreGive(s_mutex);
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;

    ESP_LOGI(TAG, "Interview module deinitialized");
    return ESP_OK;
}

esp_err_t zb_interview_start(uint64_t ieee_addr, uint16_t short_addr)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Interview module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (short_addr == 0x0000 || short_addr == 0xFFFF) {
        ESP_LOGE(TAG, "Invalid short address: 0x%04X", short_addr);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Check if interview already in progress for this device */
    zb_interview_context_t *existing = find_context_by_ieee(ieee_addr);
    if (existing != NULL && existing->in_use) {
        ESP_LOGW(TAG, "Interview already in progress for device 0x%04X", short_addr);
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Allocate context */
    zb_interview_context_t *ctx = allocate_context();
    if (ctx == NULL) {
        ESP_LOGE(TAG, "Max concurrent interviews reached (%d/%d)",
                 ZB_INTERVIEW_MAX_CONCURRENT, ZB_INTERVIEW_MAX_CONCURRENT);
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Initialize context */
    ctx->ieee_addr = ieee_addr;
    ctx->short_addr = short_addr;
    ctx->status = ZB_INTERVIEW_STATUS_ACTIVE_ENDPOINTS;
    ctx->current_endpoint_idx = 0;
    ctx->retry_count = s_config.retry_count;
    ctx->interview_retry_count = ZB_INTERVIEW_FULL_RETRY_COUNT;
    ctx->endpoint_count = 0;
    ctx->start_time = esp_timer_get_time();
    ctx->in_use = true;

    /* Initialize result */
    memset(&ctx->result, 0, sizeof(zb_interview_result_t));
    ctx->result.ieee_addr = ieee_addr;
    ctx->result.short_addr = short_addr;
    ctx->result.status = ZB_INTERVIEW_STATUS_ACTIVE_ENDPOINTS;
    ctx->result.interview_complete = false;

    /* Create timeout timer */
    char timer_name[32];
    snprintf(timer_name, sizeof(timer_name), "iv_tmr_%04X", short_addr);
    ctx->timeout_timer = xTimerCreate(timer_name,
                                       pdMS_TO_TICKS(s_config.timeout_ms),
                                       pdFALSE,  /* One-shot */
                                       ctx,
                                       timeout_callback);
    if (ctx->timeout_timer != NULL) {
        xTimerStart(ctx->timeout_timer, 0);
    }

    ESP_LOGI(TAG, "Starting interview for device 0x%04X (IEEE: 0x%016llX)",
             short_addr, (unsigned long long)ieee_addr);

    xSemaphoreGive(s_mutex);

    /* Pause BLE scanning during interview to reduce radio contention */
#if CONFIG_BT_SCANNER_ENABLED
    if (ble_scanner_is_running()) {
        esp_err_t ble_ret = ble_scanner_stop();
        if (ble_ret == ESP_OK) {
            s_ble_paused_for_interview = true;
            ESP_LOGI(TAG, "BLE scanning paused for device interview");
        }
    }
#endif

    /* Publish bridge event: interview started */
    bridge_event_device_interview(ieee_addr, BRIDGE_INTERVIEW_STARTED);

    /* Start with active endpoints request */
    esp_err_t ret = request_active_endpoints(ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to request active endpoints: %s", esp_err_to_name(ret));
        /* Publish bridge event: interview failed */
        bridge_event_device_interview(ieee_addr, BRIDGE_INTERVIEW_FAILED);
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        interview_complete(ctx, ZB_INTERVIEW_STATUS_FAILED);
        xSemaphoreGive(s_mutex);
        return ret;
    }

    /* Report initial progress */
    report_progress(ctx, 5);

    return ESP_OK;
}

esp_err_t zb_interview_cancel(uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_ieee(ieee_addr);
    if (ctx == NULL || !ctx->in_use) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Canceling interview for device 0x%04X", ctx->short_addr);
    interview_complete(ctx, ZB_INTERVIEW_STATUS_FAILED);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

const zb_interview_result_t* zb_interview_get_result(uint64_t ieee_addr)
{
    if (!s_initialized) {
        return NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* First check active interviews */
    zb_interview_context_t *ctx = find_context_by_ieee(ieee_addr);
    if (ctx != NULL && ctx->in_use && ctx->result.interview_complete) {
        xSemaphoreGive(s_mutex);
        return &ctx->result;
    }

    /* Then check cached results */
    zb_interview_result_t *cached = find_cached_result(ieee_addr);
    if (cached != NULL) {
        xSemaphoreGive(s_mutex);
        return cached;
    }

    xSemaphoreGive(s_mutex);
    return NULL;
}

bool zb_interview_is_active(uint64_t ieee_addr)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Interview module not initialized");
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_ieee(ieee_addr);
    bool active = (ctx != NULL && ctx->in_use &&
                   ctx->status != ZB_INTERVIEW_STATUS_COMPLETE &&
                   ctx->status != ZB_INTERVIEW_STATUS_FAILED &&
                   ctx->status != ZB_INTERVIEW_STATUS_TIMEOUT);

    xSemaphoreGive(s_mutex);
    return active;
}

zb_interview_status_t zb_interview_get_status(uint64_t ieee_addr)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Interview module not initialized");
        return ZB_INTERVIEW_STATUS_IDLE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_ieee(ieee_addr);
    zb_interview_status_t status = ZB_INTERVIEW_STATUS_IDLE;
    if (ctx != NULL && ctx->in_use) {
        status = ctx->status;
    } else {
        /* Check cached results */
        zb_interview_result_t *cached = find_cached_result(ieee_addr);
        if (cached != NULL) {
            status = cached->status;
        }
    }

    xSemaphoreGive(s_mutex);
    return status;
}

esp_err_t zb_interview_set_complete_callback(zb_interview_complete_cb_t callback)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config.complete_cb = callback;
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t zb_interview_set_progress_callback(zb_interview_progress_cb_t callback)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_config.progress_cb = callback;
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t zb_interview_free_result(uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Find and remove from cached results */
    for (uint8_t i = 0; i < s_cached_result_count; i++) {
        if (s_cached_results[i].ieee_addr == ieee_addr) {
            free_interview_result_data(&s_cached_results[i]);

            /* Shift remaining results */
            if (i < s_cached_result_count - 1) {
                memmove(&s_cached_results[i], &s_cached_results[i + 1],
                       (s_cached_result_count - i - 1) * sizeof(zb_interview_result_t));
            }
            s_cached_result_count--;

            xSemaphoreGive(s_mutex);
            ESP_LOGI(TAG, "Freed cached result for IEEE 0x%016llX",
                     (unsigned long long)ieee_addr);
            return ESP_OK;
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

uint8_t zb_interview_get_active_count(void)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Interview module not initialized");
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uint8_t count = 0;
    for (int i = 0; i < ZB_INTERVIEW_MAX_CONCURRENT; i++) {
        if (s_contexts[i].in_use &&
            s_contexts[i].status != ZB_INTERVIEW_STATUS_COMPLETE &&
            s_contexts[i].status != ZB_INTERVIEW_STATUS_FAILED &&
            s_contexts[i].status != ZB_INTERVIEW_STATUS_TIMEOUT) {
            count++;
        }
    }

    xSemaphoreGive(s_mutex);
    return count;
}

esp_err_t zb_interview_handle_active_ep_resp(uint16_t short_addr,
                                              esp_zb_zdp_status_t status,
                                              uint8_t *endpoints,
                                              uint8_t ep_count)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_short(short_addr);
    if (ctx == NULL || !ctx->in_use) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    if (status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Active endpoints request failed for 0x%04X: status=%d",
                 short_addr, status);

        if (ctx->retry_count > 0) {
            ctx->retry_count--;
            xSemaphoreGive(s_mutex);
            return request_active_endpoints(ctx);
        }

        interview_complete(ctx, ZB_INTERVIEW_STATUS_FAILED);
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Device 0x%04X has %d endpoints", short_addr, ep_count);

    /* Store discovered endpoints */
    ctx->endpoint_count = (ep_count > ZB_INTERVIEW_MAX_ENDPOINTS) ?
                           ZB_INTERVIEW_MAX_ENDPOINTS : ep_count;
    if (endpoints != NULL && ctx->endpoint_count > 0) {
        memcpy(ctx->endpoints, endpoints, ctx->endpoint_count);
    }

    /* Allocate endpoint info array */
    if (ctx->endpoint_count > 0) {
        ctx->result.endpoints = calloc(ctx->endpoint_count, sizeof(zb_endpoint_info_t));
        if (ctx->result.endpoints == NULL) {
            ESP_LOGE(TAG, "Failed to allocate endpoint info array");
            interview_complete(ctx, ZB_INTERVIEW_STATUS_FAILED);
            xSemaphoreGive(s_mutex);
            return ESP_ERR_NO_MEM;
        }
        ctx->result.endpoint_count = ctx->endpoint_count;
    }

    /* Move to simple descriptor phase */
    ctx->status = ZB_INTERVIEW_STATUS_SIMPLE_DESC;
    ctx->result.status = ZB_INTERVIEW_STATUS_SIMPLE_DESC;
    ctx->current_endpoint_idx = 0;
    ctx->retry_count = s_config.retry_count;

    report_progress(ctx, 20);

    xSemaphoreGive(s_mutex);

    /* Request simple descriptor for first endpoint */
    if (ctx->endpoint_count > 0) {
        return request_simple_descriptor(ctx, ctx->endpoints[0]);
    }

    /* No endpoints - complete interview */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    interview_complete(ctx, ZB_INTERVIEW_STATUS_COMPLETE);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t zb_interview_handle_simple_desc_resp(uint16_t short_addr,
                                                esp_zb_zdp_status_t status,
                                                esp_zb_af_simple_desc_1_1_t *simple_desc)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_short(short_addr);
    if (ctx == NULL || !ctx->in_use) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t ep_idx = ctx->current_endpoint_idx;

    if (status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Simple descriptor request failed for 0x%04X EP %d: status=%d",
                 short_addr, ctx->endpoints[ep_idx], status);

        if (ctx->retry_count > 0) {
            ctx->retry_count--;
            xSemaphoreGive(s_mutex);
            return request_simple_descriptor(ctx, ctx->endpoints[ep_idx]);
        }

        /* Skip this endpoint, continue with next */
        ESP_LOGW(TAG, "Skipping endpoint %d after retries exhausted", ctx->endpoints[ep_idx]);
    } else if (simple_desc != NULL) {
        /* Store endpoint information */
        zb_endpoint_info_t *ep_info = &ctx->result.endpoints[ep_idx];
        ep_info->endpoint = simple_desc->endpoint;
        ep_info->profile_id = simple_desc->app_profile_id;
        ep_info->device_id = simple_desc->app_device_id;
        ep_info->device_version = simple_desc->app_device_version;

        ESP_LOGI(TAG, "EP %d: Profile=0x%04X, DeviceID=0x%04X, Version=%d",
                 ep_info->endpoint, ep_info->profile_id,
                 ep_info->device_id, ep_info->device_version);

        /* Store server clusters */
        uint8_t in_count = simple_desc->app_input_cluster_count;
        /* Bound check - prevent excessive memory allocation from malformed descriptors */
        if (in_count > ZB_INTERVIEW_MAX_CLUSTERS) {
            ESP_LOGW(TAG, "Capping server cluster count from %d to %d",
                     in_count, ZB_INTERVIEW_MAX_CLUSTERS);
            in_count = ZB_INTERVIEW_MAX_CLUSTERS;
        }
        /* Validate cluster list pointer before accessing */
        if (in_count > 0 && simple_desc->app_cluster_list == NULL) {
            ESP_LOGW(TAG, "Simple descriptor has cluster count but NULL cluster list");
            in_count = 0;
        }
        if (in_count > 0) {
            ep_info->server_clusters = calloc(in_count, sizeof(uint16_t));
            if (ep_info->server_clusters != NULL) {
                /* Copy cluster IDs from simple descriptor */
                uint16_t *cluster_list = (uint16_t *)simple_desc->app_cluster_list;
                memcpy(ep_info->server_clusters, cluster_list, in_count * sizeof(uint16_t));
                ep_info->server_cluster_count = in_count;

                ESP_LOGI(TAG, "  Server clusters (%d):", in_count);
                for (uint8_t i = 0; i < in_count; i++) {
                    ESP_LOGI(TAG, "    - 0x%04X", ep_info->server_clusters[i]);
                }
            }
        }

        /* Store client clusters */
        uint8_t out_count = simple_desc->app_output_cluster_count;
        /* Bound check - prevent excessive memory allocation from malformed descriptors */
        if (out_count > ZB_INTERVIEW_MAX_CLUSTERS) {
            ESP_LOGW(TAG, "Capping client cluster count from %d to %d",
                     out_count, ZB_INTERVIEW_MAX_CLUSTERS);
            out_count = ZB_INTERVIEW_MAX_CLUSTERS;
        }
        /* Validate cluster list pointer before accessing */
        if (out_count > 0 && simple_desc->app_cluster_list == NULL) {
            ESP_LOGW(TAG, "Simple descriptor has cluster count but NULL cluster list");
            out_count = 0;
        }
        if (out_count > 0) {
            ep_info->client_clusters = calloc(out_count, sizeof(uint16_t));
            if (ep_info->client_clusters != NULL) {
                /* Client clusters come after server clusters in the list */
                uint16_t *cluster_list = (uint16_t *)simple_desc->app_cluster_list;
                memcpy(ep_info->client_clusters, &cluster_list[in_count],
                       out_count * sizeof(uint16_t));
                ep_info->client_cluster_count = out_count;

                ESP_LOGI(TAG, "  Client clusters (%d):", out_count);
                for (uint8_t i = 0; i < out_count; i++) {
                    ESP_LOGI(TAG, "    - 0x%04X", ep_info->client_clusters[i]);
                }
            }
        }
    }

    /* Move to next endpoint */
    ctx->current_endpoint_idx++;
    ctx->retry_count = s_config.retry_count;

    /* Calculate progress */
    uint8_t progress = 20 + ((ctx->current_endpoint_idx * 60) / ctx->endpoint_count);
    report_progress(ctx, progress);

    if (ctx->current_endpoint_idx < ctx->endpoint_count) {
        /* Request next endpoint */
        uint8_t next_ep = ctx->endpoints[ctx->current_endpoint_idx];
        xSemaphoreGive(s_mutex);
        return request_simple_descriptor(ctx, next_ep);
    }

    /* All endpoints processed - request power descriptor (API-007) */
    report_progress(ctx, 85);

    xSemaphoreGive(s_mutex);

    /* Request power descriptor before basic cluster attributes */
    return request_power_descriptor(ctx);
}

esp_err_t zb_interview_handle_zdo_response(uint16_t short_addr,
                                            esp_zb_zdp_status_t status,
                                            void *data, size_t data_len)
{
    /* This is a generic handler - specific responses are handled by their callbacks */
    (void)short_addr;
    (void)status;
    (void)data;
    (void)data_len;
    return ESP_OK;
}

esp_err_t zb_interview_handle_read_attr_resp(uint16_t short_addr,
                                               uint16_t cluster_id,
                                               uint16_t attr_id,
                                               uint8_t status,
                                               uint8_t attr_type,
                                               const void *value,
                                               size_t value_len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Only handle Basic cluster responses */
    if (cluster_id != ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_interview_context_t *ctx = find_context_by_short(short_addr);
    if (ctx == NULL || !ctx->in_use ||
        ctx->status != ZB_INTERVIEW_STATUS_BASIC_INFO) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if read was successful */
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Basic cluster attr 0x%04X read failed with status 0x%02X",
                 attr_id, status);
        /* Mark as received even on failure to avoid timeout */
        if (attr_id == ZB_BASIC_ATTR_MANUFACTURER_NAME) {
            ctx->basic_read_flags |= ZB_BASIC_READ_MANUFACTURER_RECEIVED;
        } else if (attr_id == ZB_BASIC_ATTR_MODEL_IDENTIFIER) {
            ctx->basic_read_flags |= ZB_BASIC_READ_MODEL_RECEIVED;
        }
    } else if (value != NULL && value_len > 0) {
        /* Parse ZCL string attributes (first byte is length) */
        zb_device_t *device = zb_device_get(short_addr);
        if (device != NULL) {
            uint8_t str_len = ((const uint8_t *)value)[0];
            const char *str_data = (const char *)value + 1;

            /* Ensure we don't exceed buffer or value bounds */
            if (str_len > value_len - 1) {
                str_len = value_len - 1;
            }

            if (attr_id == ZB_BASIC_ATTR_MANUFACTURER_NAME) {
                size_t copy_len = (str_len < sizeof(device->manufacturer) - 1) ?
                                   str_len : sizeof(device->manufacturer) - 1;
                memcpy(device->manufacturer, str_data, copy_len);
                device->manufacturer[copy_len] = '\0';
                ctx->basic_read_flags |= ZB_BASIC_READ_MANUFACTURER_RECEIVED;
                ESP_LOGI(TAG, "Device 0x%04X manufacturer: %s",
                         short_addr, device->manufacturer);
            } else if (attr_id == ZB_BASIC_ATTR_MODEL_IDENTIFIER) {
                size_t copy_len = (str_len < sizeof(device->model) - 1) ?
                                   str_len : sizeof(device->model) - 1;
                memcpy(device->model, str_data, copy_len);
                device->model[copy_len] = '\0';
                ctx->basic_read_flags |= ZB_BASIC_READ_MODEL_RECEIVED;
                ESP_LOGI(TAG, "Device 0x%04X model: %s",
                         short_addr, device->model);
            }
        }
    }

    /* Check if we've received all expected attributes */
    if ((ctx->basic_read_flags & ZB_BASIC_READ_ALL_RECEIVED) == ZB_BASIC_READ_ALL_RECEIVED) {
        ESP_LOGI(TAG, "All Basic cluster attributes received for 0x%04X", short_addr);
        complete_basic_cluster_read(ctx);
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t zb_interview_test(void)
{
    ESP_LOGI(TAG, "Running interview module self-test...");

    /* Test initialization */
    if (!s_initialized) {
        zb_interview_config_t config = {
            .complete_cb = NULL,
            .progress_cb = NULL,
            .timeout_ms = ZB_INTERVIEW_TIMEOUT_MS,
            .retry_count = 1,
            .read_attributes = false,
            .auto_interview_on_join = false
        };

        esp_err_t ret = zb_interview_init(&config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Init test failed: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }
    }

    /* Test status query for non-existent device */
    zb_interview_status_t status = zb_interview_get_status(0x123456789ABCDEF0ULL);
    if (status != ZB_INTERVIEW_STATUS_IDLE) {
        ESP_LOGE(TAG, "Status query test failed");
        return ESP_FAIL;
    }

    /* Test active count */
    uint8_t count = zb_interview_get_active_count();
    ESP_LOGI(TAG, "Active interviews: %d", count);

    /* Test result query for non-existent device */
    const zb_interview_result_t *result = zb_interview_get_result(0x123456789ABCDEF0ULL);
    if (result != NULL) {
        ESP_LOGE(TAG, "Result query test failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Interview module self-test PASSED");
    return ESP_OK;
}

/* ==================== Internal Helper Functions ==================== */

/**
 * @brief Find interview context by IEEE address
 */
static zb_interview_context_t* find_context_by_ieee(uint64_t ieee_addr)
{
    for (int i = 0; i < ZB_INTERVIEW_MAX_CONCURRENT; i++) {
        if (s_contexts[i].in_use && s_contexts[i].ieee_addr == ieee_addr) {
            return &s_contexts[i];
        }
    }
    return NULL;
}

/**
 * @brief Find interview context by short address
 */
static zb_interview_context_t* find_context_by_short(uint16_t short_addr)
{
    for (int i = 0; i < ZB_INTERVIEW_MAX_CONCURRENT; i++) {
        if (s_contexts[i].in_use && s_contexts[i].short_addr == short_addr) {
            return &s_contexts[i];
        }
    }
    return NULL;
}

/**
 * @brief Allocate a free interview context
 */
static zb_interview_context_t* allocate_context(void)
{
    for (int i = 0; i < ZB_INTERVIEW_MAX_CONCURRENT; i++) {
        if (!s_contexts[i].in_use) {
            memset(&s_contexts[i], 0, sizeof(zb_interview_context_t));
            return &s_contexts[i];
        }
    }
    return NULL;
}

/**
 * @brief Free an interview context
 */
static void free_context(zb_interview_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->timeout_timer != NULL) {
        xTimerStop(ctx->timeout_timer, 0);
        xTimerDelete(ctx->timeout_timer, 0);
        ctx->timeout_timer = NULL;
    }

    if (ctx->basic_read_timer != NULL) {
        xTimerStop(ctx->basic_read_timer, 0);
        xTimerDelete(ctx->basic_read_timer, 0);
        ctx->basic_read_timer = NULL;
    }

    /* Note: Result data ownership transfers to cache on success */
    ctx->in_use = false;
}

/**
 * @brief Free endpoint info memory
 */
static void free_endpoint_info(zb_endpoint_info_t *ep_info)
{
    if (ep_info == NULL) {
        return;
    }

    if (ep_info->server_clusters != NULL) {
        free(ep_info->server_clusters);
        ep_info->server_clusters = NULL;
    }
    if (ep_info->client_clusters != NULL) {
        free(ep_info->client_clusters);
        ep_info->client_clusters = NULL;
    }
}

/**
 * @brief Free interview result data
 */
static void free_interview_result_data(zb_interview_result_t *result)
{
    if (result == NULL) {
        return;
    }

    if (result->endpoints != NULL) {
        for (uint8_t i = 0; i < result->endpoint_count; i++) {
            free_endpoint_info(&result->endpoints[i]);
        }
        free(result->endpoints);
        result->endpoints = NULL;
    }
    result->endpoint_count = 0;
}

/**
 * @brief Request active endpoints from device
 */
static esp_err_t request_active_endpoints(zb_interview_context_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Requesting active endpoints from 0x%04X", ctx->short_addr);

    esp_zb_zdo_active_ep_req_param_t req = {
        .addr_of_interest = ctx->short_addr
    };

    esp_zb_zdo_active_ep_req(&req, active_ep_callback,
                             (void *)(uintptr_t)ctx->short_addr);

    return ESP_OK;
}

/**
 * @brief Request simple descriptor for an endpoint
 */
static esp_err_t request_simple_descriptor(zb_interview_context_t *ctx, uint8_t endpoint)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Requesting simple descriptor for 0x%04X EP %d",
             ctx->short_addr, endpoint);

    esp_zb_zdo_simple_desc_req_param_t req = {
        .addr_of_interest = ctx->short_addr,
        .endpoint = endpoint
    };

    esp_zb_zdo_simple_desc_req(&req, simple_desc_callback,
                               (void *)(uintptr_t)ctx->short_addr);

    return ESP_OK;
}

/**
 * @brief Request power descriptor from device (API-007)
 *
 * Requests the power descriptor from the device to determine
 * power source (mains/battery) and battery level.
 */
static esp_err_t request_power_descriptor(zb_interview_context_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Requesting power descriptor from 0x%04X", ctx->short_addr);

    esp_zb_zdo_power_desc_req_param_t req = {
        .dst_nwk_addr = ctx->short_addr
    };

    esp_zb_zdo_power_desc_req(&req, power_desc_callback,
                               (void *)(uintptr_t)ctx->short_addr);

    return ESP_OK;
}

/**
 * @brief Request basic cluster attributes (manufacturer, model)
 *
 * Sends ZCL read attribute requests for Basic cluster (0x0000) to get
 * the manufacturer name (0x0004) and model identifier (0x0005).
 */
static esp_err_t request_basic_cluster_attributes(zb_interview_context_t *ctx)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Requesting Basic cluster attributes from 0x%04X", ctx->short_addr);

    /* Reset Basic cluster read state */
    ctx->basic_read_flags = 0;

    /* Find the endpoint to read from - use first endpoint or endpoint 1 */
    uint8_t endpoint = 1;
    if (ctx->endpoint_count > 0) {
        endpoint = ctx->endpoints[0];
    }

    /* Attribute IDs to read: Manufacturer Name (0x0004) and Model Identifier (0x0005) */
    uint16_t attr_ids[2] = {
        ZB_BASIC_ATTR_MANUFACTURER_NAME,
        ZB_BASIC_ATTR_MODEL_IDENTIFIER
    };

    /* Send read attributes request using the ZCL helper */
    esp_err_t ret = zb_zcl_read_attr_with_lock(
        ctx->short_addr,
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        attr_ids,
        2
    );

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Basic cluster read request: %s",
                 esp_err_to_name(ret));
        /* Continue with interview completion even if read fails */
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        interview_complete(ctx, ZB_INTERVIEW_STATUS_COMPLETE);
        xSemaphoreGive(s_mutex);
        return ret;
    }

    /* Start a timeout timer for the Basic cluster read response */
    char timer_name[32];
    snprintf(timer_name, sizeof(timer_name), "basic_tmr_%04X", ctx->short_addr);
    ctx->basic_read_timer = xTimerCreate(timer_name,
                                          pdMS_TO_TICKS(ZB_BASIC_CLUSTER_READ_TIMEOUT_MS),
                                          pdFALSE,  /* One-shot */
                                          ctx,
                                          basic_read_timeout_callback);
    if (ctx->basic_read_timer != NULL) {
        xTimerStart(ctx->basic_read_timer, 0);
    } else {
        ESP_LOGW(TAG, "Failed to create Basic cluster read timer, completing immediately");
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        interview_complete(ctx, ZB_INTERVIEW_STATUS_COMPLETE);
        xSemaphoreGive(s_mutex);
    }

    return ESP_OK;
}

/**
 * @brief Timeout callback for Basic cluster read
 *
 * If we don't receive a response within the timeout, complete the interview
 * without manufacturer/model information.
 */
static void basic_read_timeout_callback(TimerHandle_t timer)
{
    zb_interview_context_t *ctx = (zb_interview_context_t *)pvTimerGetTimerID(timer);
    if (ctx == NULL || !ctx->in_use) {
        return;
    }

    ESP_LOGW(TAG, "Basic cluster read timeout for device 0x%04X, completing interview",
             ctx->short_addr);

    /* Complete the interview without the Basic cluster attributes */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (ctx->in_use && ctx->status == ZB_INTERVIEW_STATUS_BASIC_INFO) {
        interview_complete(ctx, ZB_INTERVIEW_STATUS_COMPLETE);
    }
    xSemaphoreGive(s_mutex);
}

/**
 * @brief Complete Basic cluster read and finish interview
 *
 * Called when all Basic cluster attributes have been received or on timeout.
 */
static void complete_basic_cluster_read(zb_interview_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    /* Stop the Basic cluster read timer */
    if (ctx->basic_read_timer != NULL) {
        xTimerStop(ctx->basic_read_timer, 0);
        xTimerDelete(ctx->basic_read_timer, 0);
        ctx->basic_read_timer = NULL;
    }

    /* Copy manufacturer/model to result */
    zb_device_t *device = zb_device_get(ctx->short_addr);
    if (device != NULL) {
        strncpy(ctx->result.manufacturer, device->manufacturer,
                sizeof(ctx->result.manufacturer) - 1);
        ctx->result.manufacturer[sizeof(ctx->result.manufacturer) - 1] = '\0';
        strncpy(ctx->result.model, device->model,
                sizeof(ctx->result.model) - 1);
        ctx->result.model[sizeof(ctx->result.model) - 1] = '\0';
    }

    ESP_LOGI(TAG, "Basic cluster read complete for 0x%04X: manufacturer='%s', model='%s'",
             ctx->short_addr, ctx->result.manufacturer, ctx->result.model);

    interview_complete(ctx, ZB_INTERVIEW_STATUS_COMPLETE);
}

/**
 * @brief Complete the interview process
 */
static void interview_complete(zb_interview_context_t *ctx, zb_interview_status_t status)
{
    if (ctx == NULL) {
        return;
    }

    /* Calculate duration */
    int64_t end_time = esp_timer_get_time();
    ctx->result.duration_ms = (uint32_t)((end_time - ctx->start_time) / 1000);

    /* Set final status */
    ctx->status = status;
    ctx->result.status = status;
    ctx->result.interview_complete = (status == ZB_INTERVIEW_STATUS_COMPLETE);

    ESP_LOGI(TAG, "Interview %s for device 0x%04X (duration: %lums)",
             (status == ZB_INTERVIEW_STATUS_COMPLETE) ? "COMPLETE" : "FAILED",
             ctx->short_addr, (unsigned long)ctx->result.duration_ms);

    /* Log summary */
    if (status == ZB_INTERVIEW_STATUS_COMPLETE) {
        ESP_LOGI(TAG, "  Endpoints: %d", ctx->result.endpoint_count);
        for (uint8_t i = 0; i < ctx->result.endpoint_count; i++) {
            zb_endpoint_info_t *ep = &ctx->result.endpoints[i];
            ESP_LOGI(TAG, "  EP %d: Server=%d, Client=%d clusters",
                     ep->endpoint, ep->server_cluster_count, ep->client_cluster_count);
        }
    }

    /* Publish bridge event: interview result */
    if (status == ZB_INTERVIEW_STATUS_COMPLETE) {
        bridge_event_device_interview(ctx->ieee_addr, BRIDGE_INTERVIEW_SUCCESSFUL);

        /* Copy discovered clusters to device registry (critical for type determination!) */
        for (uint8_t ep_idx = 0; ep_idx < ctx->result.endpoint_count; ep_idx++) {
            zb_endpoint_info_t *ep = &ctx->result.endpoints[ep_idx];
            if (ep->server_clusters != NULL) {
                for (uint8_t c = 0; c < ep->server_cluster_count; c++) {
                    zb_device_add_cluster(ctx->short_addr, ep->server_clusters[c]);
                }
            }
        }

        /* Determine device type from discovered clusters (critical for HA discovery!) */
        esp_err_t type_ret = zb_device_determine_type(ctx->short_addr);
        if (type_ret == ESP_OK) {
            /* Re-publish HA discovery now that device type is known */
            const zb_device_t *device = zb_device_get(ctx->short_addr);
            if (device != NULL) {
                ESP_LOGI(TAG, "Re-publishing HA discovery for 0x%04X (type=%d)",
                         ctx->short_addr, device->device_type);
                ha_discovery_publish_device(device);

                /* Publish battery state if device is battery-powered (API-007) */
                if (device->power_info.power_info_valid) {
                    esp_err_t bat_ret = device_state_publish_battery(ctx->short_addr);
                    if (bat_ret == ESP_OK) {
                        ESP_LOGI(TAG, "Published battery state for 0x%04X", ctx->short_addr);
                    }
                }

                /* Publish initial device state and availability */
                device_state_publish(device);
                device_state_publish_availability(device, true);

                /* Save device to NVS for persistence across reboots */
                zb_device_mark_dirty(device->ieee_addr);
                ESP_LOGD(TAG, "Marked device 0x%04X for NVS persistence", ctx->short_addr);
            }
        } else {
            ESP_LOGW(TAG, "Failed to determine device type for 0x%04X: %s",
                     ctx->short_addr, esp_err_to_name(type_ret));
        }
    } else {
        bridge_event_device_interview(ctx->ieee_addr, BRIDGE_INTERVIEW_FAILED);
    }

    /* Invoke completion callback BEFORE caching (callback sees valid data) */
    if (s_config.complete_cb != NULL) {
        s_config.complete_cb(&ctx->result);
    }

    /* Report final progress */
    report_progress(ctx, 100);

    /* Handle memory cleanup based on interview status */
    if (status == ZB_INTERVIEW_STATUS_COMPLETE) {
        /* Cache result - transfers ownership of heap pointers.
         * After this call, ctx->result.endpoints is NULL. */
        cache_result(&ctx->result);
    } else {
        /* Free result data on failure */
        free_interview_result_data(&ctx->result);
    }

    /* Resume BLE scanning after interview completes */
#if CONFIG_BT_SCANNER_ENABLED
    if (s_ble_paused_for_interview) {
        s_ble_paused_for_interview = false;
        esp_err_t ble_ret = ble_scanner_start();
        ESP_LOGI(TAG, "BLE scanning resumed after interview: %s", esp_err_to_name(ble_ret));
    }
#endif

    free_context(ctx);
}

/**
 * @brief Timeout callback
 */
static void timeout_callback(TimerHandle_t timer)
{
    zb_interview_context_t *ctx = (zb_interview_context_t *)pvTimerGetTimerID(timer);
    if (ctx == NULL || !ctx->in_use) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Check if we can retry the entire interview */
    if (ctx->interview_retry_count > 0) {
        ctx->interview_retry_count--;
        ESP_LOGW(TAG, "Interview timeout for 0x%04X, retrying (%d retries left)",
                 ctx->short_addr, ctx->interview_retry_count);

        /* Reset interview state for retry */
        ctx->status = ZB_INTERVIEW_STATUS_ACTIVE_ENDPOINTS;
        ctx->result.status = ZB_INTERVIEW_STATUS_ACTIVE_ENDPOINTS;
        ctx->current_endpoint_idx = 0;
        ctx->retry_count = s_config.retry_count;
        ctx->start_time = esp_timer_get_time();

        /* Free any partial endpoint data from previous attempt */
        free_interview_result_data(&ctx->result);
        ctx->endpoint_count = 0;

        /* Restart timeout timer */
        xTimerReset(ctx->timeout_timer, 0);

        xSemaphoreGive(s_mutex);

        /* Restart interview from active endpoints */
        request_active_endpoints(ctx);
        return;
    }

    ESP_LOGW(TAG, "Interview timeout for device 0x%04X, no retries left", ctx->short_addr);
    interview_complete(ctx, ZB_INTERVIEW_STATUS_TIMEOUT);
    xSemaphoreGive(s_mutex);
}

/**
 * @brief Report interview progress
 */
static void report_progress(zb_interview_context_t *ctx, uint8_t progress)
{
    if (ctx == NULL) {
        return;
    }

    if (s_config.progress_cb != NULL) {
        s_config.progress_cb(ctx->ieee_addr, ctx->status, progress);
    }
}

/**
 * @brief Cache interview result with ownership transfer
 *
 * Caches the interview result and takes ownership of all heap-allocated
 * pointers (endpoints, cluster arrays). After this call, the source
 * result's pointers are cleared to prevent double-free.
 *
 * @param result Result to cache (pointers will be cleared after transfer)
 * @return ESP_OK on success
 */
static esp_err_t cache_result(zb_interview_result_t *result)
{
    if (result == NULL || s_cached_results == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_interview_result_t *dest = NULL;

    /* Check if already cached (update existing) */
    for (uint8_t i = 0; i < s_cached_result_count; i++) {
        if (s_cached_results[i].ieee_addr == result->ieee_addr) {
            /* Free old data before replacing */
            free_interview_result_data(&s_cached_results[i]);
            dest = &s_cached_results[i];
            ESP_LOGI(TAG, "Updated cached result for device 0x%04X", result->short_addr);
            break;
        }
    }

    /* Add new entry if space available */
    if (dest == NULL && s_cached_result_count < s_max_cached_results) {
        dest = &s_cached_results[s_cached_result_count];
        s_cached_result_count++;
        ESP_LOGI(TAG, "Cached result for device 0x%04X (total cached: %d)",
                 result->short_addr, s_cached_result_count);
    }

    /* Cache full - remove oldest entry (LRU eviction) */
    if (dest == NULL) {
        free_interview_result_data(&s_cached_results[0]);
        memmove(&s_cached_results[0], &s_cached_results[1],
               (s_max_cached_results - 1) * sizeof(zb_interview_result_t));
        dest = &s_cached_results[s_max_cached_results - 1];
        ESP_LOGW(TAG, "Cache full, removed oldest result");
    }

    /* Copy structure (shallow copy transfers pointer ownership) */
    memcpy(dest, result, sizeof(zb_interview_result_t));

    /* Clear source pointers to make ownership transfer explicit
     * and prevent accidental double-free by caller */
    result->endpoints = NULL;
    result->endpoint_count = 0;

    return ESP_OK;
}

/**
 * @brief Find cached result by IEEE address
 */
static zb_interview_result_t* find_cached_result(uint64_t ieee_addr)
{
    for (uint8_t i = 0; i < s_cached_result_count; i++) {
        if (s_cached_results[i].ieee_addr == ieee_addr) {
            return &s_cached_results[i];
        }
    }
    return NULL;
}
