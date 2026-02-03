/**
 * @file tuya_driver_registry.c
 * @brief Tuya Device Driver Registry Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "tuya_driver_registry.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "TUYA_REGISTRY";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** @brief Registered drivers */
static const tuya_device_driver_t *s_drivers[TUYA_MAX_DRIVERS];
static size_t s_driver_count = 0;

/** @brief Device-to-driver binding cache */
typedef struct {
    uint16_t short_addr;
    const tuya_device_driver_t *driver;
} tuya_device_binding_t;

static tuya_device_binding_t s_bindings[TUYA_MAX_DEVICE_BINDINGS];
static size_t s_binding_count = 0;

/** @brief Mutex for thread-safe access */
static SemaphoreHandle_t s_mutex = NULL;

/** @brief Initialization flag */
static bool s_initialized = false;

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t tuya_driver_registry_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create registry mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(s_drivers, 0, sizeof(s_drivers));
    s_driver_count = 0;
    memset(s_bindings, 0, sizeof(s_bindings));
    s_binding_count = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "Tuya driver registry initialized (max %d drivers, %d bindings)",
             TUYA_MAX_DRIVERS, TUYA_MAX_DEVICE_BINDINGS);
    return ESP_OK;
}

esp_err_t tuya_driver_register(const tuya_device_driver_t *driver)
{
    if (driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_driver_count >= TUYA_MAX_DRIVERS) {
        ESP_LOGE(TAG, "Driver registry full (%d/%d)", (int)s_driver_count, TUYA_MAX_DRIVERS);
        return ESP_ERR_NO_MEM;
    }

    s_drivers[s_driver_count++] = driver;
    ESP_LOGI(TAG, "Registered Tuya driver: '%s' (%d/%d)",
             driver->name, (int)s_driver_count, TUYA_MAX_DRIVERS);
    return ESP_OK;
}

const tuya_device_driver_t *tuya_driver_find(const char *manufacturer, const char *model)
{
    for (size_t i = 0; i < s_driver_count; i++) {
        if (s_drivers[i]->match != NULL &&
            s_drivers[i]->match(manufacturer, model)) {
            return s_drivers[i];
        }
    }
    return NULL;
}

const tuya_device_driver_t *tuya_driver_get(uint16_t short_addr)
{
    const tuya_device_driver_t *result = NULL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_binding_count; i++) {
        if (s_bindings[i].short_addr == short_addr) {
            result = s_bindings[i].driver;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t tuya_driver_bind(uint16_t short_addr, const tuya_device_driver_t *driver)
{
    if (driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Check if already bound — update driver */
    for (size_t i = 0; i < s_binding_count; i++) {
        if (s_bindings[i].short_addr == short_addr) {
            s_bindings[i].driver = driver;
            xSemaphoreGive(s_mutex);
            ESP_LOGD(TAG, "Updated binding: 0x%04X -> '%s'", short_addr, driver->name);
            return ESP_OK;
        }
    }

    /* New binding */
    if (s_binding_count >= TUYA_MAX_DEVICE_BINDINGS) {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "Binding table full (%d/%d)", (int)s_binding_count, TUYA_MAX_DEVICE_BINDINGS);
        return ESP_ERR_NO_MEM;
    }

    s_bindings[s_binding_count].short_addr = short_addr;
    s_bindings[s_binding_count].driver = driver;
    s_binding_count++;

    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Bound device 0x%04X to driver '%s' (%d/%d)",
             short_addr, driver->name, (int)s_binding_count, TUYA_MAX_DEVICE_BINDINGS);
    return ESP_OK;
}

void tuya_driver_unbind(uint16_t short_addr)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_binding_count; i++) {
        if (s_bindings[i].short_addr == short_addr) {
            ESP_LOGI(TAG, "Unbound device 0x%04X from driver '%s'",
                     short_addr, s_bindings[i].driver->name);

            /* Swap with last entry */
            if (i < s_binding_count - 1) {
                s_bindings[i] = s_bindings[s_binding_count - 1];
            }
            s_binding_count--;
            xSemaphoreGive(s_mutex);
            return;
        }
    }
    xSemaphoreGive(s_mutex);
}
