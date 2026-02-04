/**
 * @file main.c
 * @brief ESP32-C5+ Zigbee UART Coordinator — Main Entry Point
 *
 * Full-featured firmware for ESP32-C5 with WiFi:
 * - Zigbee 3.0 Coordinator
 * - UART1 bridge to ESP32-S3 (JSON Lines protocol)
 * - WiFi 6 connectivity with mDNS
 * - WiFi/Zigbee coexistence enabled
 *
 * UART0 (TXD/RXD via USB bridge) is used for debug console.
 * UART1 (IO6=RX, IO7=TX) communicates with the ESP32-S3.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

/* Project modules - from shared esp_zb_coordinator component */
#include "uart_bridge.h"
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "zb_network.h"
#include "zb_callbacks.h"
#include "zb_interview.h"
#include "zb_reporting.h"
#include "zb_availability.h"
#include "zb_tuya.h"
#include "tuya/tuya_driver_registry.h"
#include "xiaomi/aqara_vibration.h"
#include "zb_topology.h"
#include "zb_groups.h"
#include "zb_cmd_retry.h"
#include "zb_scenes.h"
#include "zb_binding.h"
#include "zb_backup.h"
#include "zb_ota.h"
#include "zb_green_power.h"
#include "zb_touchlink.h"
#include "zb_multi_pan.h"
#include "zb_hal.h"
#include "led_controller.h"
#include "wifi_manager.h"
#include "utils/version.h"

static const char *TAG = "C5+_MAIN";

/* ============================================================================
 * PSRAM-optimized Task Creation
 * ============================================================================ */

#if CONFIG_SPIRAM
/**
 * @brief Create task with stack allocated in PSRAM
 * Falls back to internal RAM if PSRAM allocation fails.
 */
static BaseType_t create_task_psram(TaskFunction_t pxTaskCode,
                                    const char *pcName,
                                    uint32_t usStackDepth,
                                    void *pvParameters,
                                    UBaseType_t uxPriority,
                                    TaskHandle_t *pxCreatedTask)
{
    /* Try PSRAM first (external memory) */
    StackType_t *stack = heap_caps_malloc(usStackDepth * sizeof(StackType_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stack != NULL) {
        StaticTask_t *tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
        if (tcb != NULL) {
            TaskHandle_t handle = xTaskCreateStatic(pxTaskCode, pcName, usStackDepth,
                                                    pvParameters, uxPriority, stack, tcb);
            if (handle != NULL) {
                ESP_LOGI(TAG, "Task '%s' created with PSRAM stack (%lu bytes)", pcName, usStackDepth * sizeof(StackType_t));
                if (pxCreatedTask) *pxCreatedTask = handle;
                return pdPASS;
            }
            heap_caps_free(tcb);
        }
        heap_caps_free(stack);
    }

    /* Fallback to regular task creation (internal RAM) */
    ESP_LOGW(TAG, "Task '%s' falling back to internal RAM", pcName);
    return xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask);
}
#else
#define create_task_psram xTaskCreate
#endif

/* ============================================================================
 * LED Callback Wrappers for HAL
 * Maps HAL enum types to LED controller types
 * ============================================================================ */

static void led_hal_blink_wrapper(zb_hal_led_notify_t notify)
{
    /* HAL enum values match LED controller values */
    led_blink_notify((led_notify_t)notify);
}

static void led_hal_status_wrapper(zb_hal_led_status_t status)
{
    /* HAL enum values match LED controller values */
    led_set_status((led_status_t)status);
}

/* ============================================================================
 * NVS Initialization
 * ============================================================================ */

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erasing, reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/* ============================================================================
 * Main Application Entry
 * ============================================================================ */

void app_main(void)
{
    /* Wait for USB-Serial-JTAG console to be ready */
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-C5+ Zigbee WiFi Coordinator");
    ESP_LOGI(TAG, " %s", CONFIG_DEVICE_NAME);
    ESP_LOGI(TAG, " Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, " Build: %s %s", BUILD_DATE, BUILD_TIME);
#if CONFIG_SPIRAM
    ESP_LOGI(TAG, " PSRAM: %lu KB free",
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
#endif
    ESP_LOGI(TAG, " Internal: %lu KB free",
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    ESP_LOGI(TAG, "========================================");

    /* Phase 1: NVS */
    ESP_LOGI(TAG, "[1/7] Initializing NVS...");
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 2: LED (optional feedback) */
    ESP_LOGI(TAG, "[2/7] Initializing LED controller...");
    esp_err_t led_ret = led_controller_init();
    if (led_ret == ESP_OK) {
        led_set_status(LED_STATUS_BOOT);
        /* Register LED callbacks with HAL for use by shared component */
        zb_hal_register_led_callbacks(led_hal_blink_wrapper, led_hal_status_wrapper);
        ESP_LOGI(TAG, "LED callbacks registered with HAL");
    } else {
        ESP_LOGW(TAG, "LED init failed (non-critical): %s", esp_err_to_name(led_ret));
    }

    /* Phase 3: WiFi + mDNS */
    ESP_LOGI(TAG, "[3/7] Initializing WiFi...");
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed (non-critical): %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Continuing without WiFi connectivity");
    } else {
        /* Start mDNS after WiFi is connected */
        ret = wifi_manager_start_mdns();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "mDNS init failed (non-critical): %s", esp_err_to_name(ret));
        }

        char ip_buf[16];
        if (wifi_manager_get_ip(ip_buf, sizeof(ip_buf)) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi connected, IP: %s", ip_buf);
            ESP_LOGI(TAG, "mDNS: %s.local", CONFIG_MDNS_HOSTNAME);
        }
    }

    /* Phase 4: UART bridge */
    ESP_LOGI(TAG, "[4/7] Initializing UART bridge...");
    ret = uart_bridge_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART bridge init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 5: Zigbee stack */
    ESP_LOGI(TAG, "[5/7] Initializing Zigbee device handler...");
    ret = zb_device_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device handler init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[5/7] Initializing Zigbee callbacks...");
    ret = zb_callbacks_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Callbacks init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[5/7] Initializing Tuya module...");
    ret = zb_tuya_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tuya init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[5/7] Initializing Tuya driver registry...");
    ret = tuya_driver_registry_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tuya driver registry init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Register device drivers */
    ret = aqara_vibration_register();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Aqara vibration register failed (non-critical): %s", esp_err_to_name(ret));

    /* Initialize all remaining modules with semaphores (non-fatal) */
    ESP_LOGI(TAG, "[5/7] Initializing remaining Zigbee modules...");

    ret = zb_reporting_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Reporting init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_binding_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Binding init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_groups_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Groups init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_scenes_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Scenes init failed (non-critical): %s", esp_err_to_name(ret));

    ret = cmd_retry_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Cmd retry init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_topology_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Topology init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_backup_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Backup init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_ota_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "OTA init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_gp_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Green Power init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_touchlink_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Touchlink init failed (non-critical): %s", esp_err_to_name(ret));

    ret = zb_multi_pan_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Multi-PAN init failed (non-critical): %s", esp_err_to_name(ret));

    ESP_LOGI(TAG, "[5/7] Initializing Zigbee coordinator...");
    ret = zb_coordinator_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Coordinator init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 5: Load persisted device data */
    ESP_LOGI(TAG, "[6/7] Loading devices from NVS...");
    size_t loaded = zb_device_load_all_from_nvs();
    ESP_LOGI(TAG, "Loaded %zu devices from NVS", loaded);

    /* Phase 6: Start everything */
    ESP_LOGI(TAG, "[7/7] Starting coordinator and UART bridge...");

    /* Start Zigbee coordinator (creates its own task) */
    ret = zb_coordinator_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Coordinator start failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Wait briefly for network to form before sending ready */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Start UART bridge (sends ready message) */
    ret = uart_bridge_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART bridge start failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Create UART RX task to receive commands from S3 (PSRAM stack) */
    create_task_psram(uart_bridge_rx_task, "uart_rx", 4096, NULL, 5, NULL);

    /* Auto permit join so known battery devices can rejoin after C5 reboot */
    ESP_LOGI(TAG, "Opening network for device rejoin (180s)...");
    ret = zb_coordinator_permit_join(180);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Auto permit_join failed (non-critical): %s", esp_err_to_name(ret));
    }

    /* Update LED to normal operation */
    if (led_ret == ESP_OK) {
        led_set_status(LED_STATUS_NORMAL);
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " C5+ Coordinator ready!");
    ESP_LOGI(TAG, " UART1: IO7(TX) → S3, IO6(RX) ← S3");
    ESP_LOGI(TAG, " %zu devices in registry", zb_device_get_count());
    if (wifi_manager_is_connected()) {
        char ip_buf[16];
        wifi_manager_get_ip(ip_buf, sizeof(ip_buf));
        ESP_LOGI(TAG, " WiFi: %s (%d dBm)", ip_buf, wifi_manager_get_rssi());
        ESP_LOGI(TAG, " mDNS: %s.local", CONFIG_MDNS_HOSTNAME);
    } else {
        ESP_LOGI(TAG, " WiFi: Not connected");
    }
#if CONFIG_SPIRAM
    ESP_LOGI(TAG, " Memory: %lu KB PSRAM, %lu KB internal free",
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
#else
    ESP_LOGI(TAG, " Memory: %lu KB internal free",
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
#endif
    ESP_LOGI(TAG, "========================================");

    /* Main task done — FreeRTOS tasks run the show */
}
