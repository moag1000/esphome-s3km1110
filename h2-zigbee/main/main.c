/**
 * @file main.c
 * @brief ESP32-H2 Zigbee UART Coordinator — Main Entry Point
 *
 * Memory-optimized firmware for ESP32-H2:
 * - Zigbee 3.0 Coordinator
 * - UART1 bridge to ESP32-S3 (JSON Lines protocol)
 * - No WiFi (H2 doesn't have it), No BLE, No OTA
 *
 * UART0 is used for debug console.
 * UART1 (GPIO23=RX, GPIO24=TX) communicates with the ESP32-S3.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

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
#include "zb_green_power.h"
#include "zb_hal.h"
#include "led_controller.h"
#include "utils/version.h"

static const char *TAG = "H2_MAIN";

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

    /* Log HAL configuration */
    zb_hal_log_config();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " %s Zigbee UART Coordinator", zb_hal_get_chip_name());
    ESP_LOGI(TAG, " Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, " Build: %s %s", BUILD_DATE, BUILD_TIME);
    ESP_LOGI(TAG, " Memory-optimized build");
    ESP_LOGI(TAG, "========================================");

    /* Phase 1: NVS */
    ESP_LOGI(TAG, "[1/6] Initializing NVS...");
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 2: LED (optional - H2-Zero typically has no LED) */
    ESP_LOGI(TAG, "[2/6] Initializing LED controller...");
    esp_err_t led_ret = led_controller_init();
    if (led_ret == ESP_OK) {
        led_set_status(LED_STATUS_BOOT);
    } else {
        ESP_LOGW(TAG, "LED init skipped (H2-Zero has no LED): %s", esp_err_to_name(led_ret));
    }

    /* Phase 3: UART bridge */
    ESP_LOGI(TAG, "[3/6] Initializing UART bridge...");
    ret = uart_bridge_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART bridge init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 4: Zigbee stack */
    ESP_LOGI(TAG, "[4/6] Initializing Zigbee device handler...");
    ret = zb_device_handler_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device handler init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[4/6] Initializing Zigbee callbacks...");
    ret = zb_callbacks_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Callbacks init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[4/6] Initializing Tuya module...");
    ret = zb_tuya_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tuya init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "[4/6] Initializing Tuya driver registry...");
    ret = tuya_driver_registry_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Tuya driver registry init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Register device drivers */
    ret = aqara_vibration_register();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Aqara vibration register failed (non-critical): %s", esp_err_to_name(ret));

    /* Initialize remaining modules (memory-optimized selection) */
    ESP_LOGI(TAG, "[4/6] Initializing remaining Zigbee modules...");

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

    /* Green Power kept for battery device support */
    if (zb_hal_feature_green_power_enabled()) {
        ret = zb_gp_init();
        if (ret != ESP_OK) ESP_LOGW(TAG, "Green Power init failed (non-critical): %s", esp_err_to_name(ret));
    }

    /* Skip OTA, Touchlink, Multi-PAN on H2 to save memory */
    if (!zb_hal_feature_ota_enabled()) {
        ESP_LOGI(TAG, "OTA disabled (memory optimization)");
    }
    if (!zb_hal_feature_touchlink_enabled()) {
        ESP_LOGI(TAG, "Touchlink disabled (memory optimization)");
    }

    ESP_LOGI(TAG, "[4/6] Initializing Zigbee coordinator...");
    ret = zb_coordinator_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Coordinator init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Phase 5: Load persisted device data */
    ESP_LOGI(TAG, "[5/6] Loading devices from NVS...");
    size_t loaded = zb_device_load_all_from_nvs();
    ESP_LOGI(TAG, "Loaded %zu devices from NVS", loaded);

    /* Phase 6: Start everything */
    ESP_LOGI(TAG, "[6/6] Starting coordinator and UART bridge...");

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

    /* Create UART RX task to receive commands from S3 */
    size_t uart_stack = zb_hal_get_uart_rx_task_stack_size();
    xTaskCreate(uart_bridge_rx_task, "uart_rx", uart_stack, NULL, 5, NULL);

    /* Auto permit join so known battery devices can rejoin after H2 reboot */
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
    ESP_LOGI(TAG, " Coordinator ready!");
    ESP_LOGI(TAG, " UART1: TX=%d, RX=%d",
             zb_hal_get_uart_tx_pin(), zb_hal_get_uart_rx_pin());
    ESP_LOGI(TAG, " %zu devices in registry", zb_device_get_count());
    ESP_LOGI(TAG, " Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    /* Main task done — FreeRTOS tasks run the show */
}
