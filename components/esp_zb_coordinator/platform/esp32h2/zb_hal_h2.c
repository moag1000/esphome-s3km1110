/**
 * @file zb_hal_h2.c
 * @brief HAL Implementation for ESP32-H2
 *
 * ESP32-H2 specifications:
 * - CPU: 96 MHz RISC-V single-core
 * - RAM: 320 KB internal SRAM
 * - PSRAM: Not available
 * - WiFi: Not available
 * - Zigbee SDK: Identical to C5
 *
 * Memory-constrained chip — stack sizes and buffers are reduced.
 */

#include "zb_hal.h"
#include "esp_mac.h"
#include "esp_log.h"

static const char *TAG = "ZB_HAL_H2";

/* ============================================================================
 * Chip Identification
 * ============================================================================ */

const char* zb_hal_get_chip_name(void)
{
    return "ESP32-H2";
}

zb_hal_chip_t zb_hal_get_chip_type(void)
{
    return ZB_HAL_CHIP_ESP32_H2;
}

/* ============================================================================
 * MAC Address
 * ============================================================================ */

esp_err_t zb_hal_read_mac(uint8_t *mac_addr)
{
    if (mac_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_read_mac(mac_addr, ESP_MAC_IEEE802154);
}

/* ============================================================================
 * GPIO Pin Configuration
 * ============================================================================
 * H2-Zero pinout (based on common devkit layouts):
 * - UART TX: GPIO24
 * - UART RX: GPIO23
 * - LED: GPIO8 (or none on Zero module)
 */

int zb_hal_get_uart_tx_pin(void)
{
#if defined(CONFIG_ZB_UART_TX_PIN)
    return CONFIG_ZB_UART_TX_PIN;
#else
    return 24;  /* GPIO24 → S3 RX (default) */
#endif
}

int zb_hal_get_uart_rx_pin(void)
{
#if defined(CONFIG_ZB_UART_RX_PIN)
    return CONFIG_ZB_UART_RX_PIN;
#else
    return 23;  /* GPIO23 ← S3 TX (default) */
#endif
}

int zb_hal_get_led_gpio(void)
{
#if defined(CONFIG_ZB_LED_GPIO)
    return CONFIG_ZB_LED_GPIO;
#else
    return -1;  /* H2-Zero typically has no LED (default) */
#endif
}

/* ============================================================================
 * Memory Configuration
 * ============================================================================
 * H2 has limited RAM — use smaller stacks and buffers
 */

size_t zb_hal_get_zb_task_stack_size(void)
{
    return 4 * 1024;  /* 4KB - reduced for H2 */
}

size_t zb_hal_get_avail_task_stack_size(void)
{
    return 3 * 1024;  /* 3KB - reduced for H2 */
}

size_t zb_hal_get_uart_rx_task_stack_size(void)
{
    return 3 * 1024;  /* 3KB - reduced for H2 */
}

bool zb_hal_has_psram(void)
{
    return false;  /* H2 has no PSRAM support */
}

uint32_t zb_hal_get_ram_size_kb(void)
{
    return 320;  /* 320 KB internal SRAM */
}

/* ============================================================================
 * Network Configuration Defaults
 * ============================================================================
 * Reduced for memory-constrained H2
 */

uint8_t zb_hal_get_default_channel(void)
{
    return 11;  /* Channel 11 - best device compatibility */
}

uint16_t zb_hal_get_max_network_devices(void)
{
#if defined(CONFIG_ZB_MAX_DEVICES)
    return CONFIG_ZB_MAX_DEVICES;
#else
    return 32;  /* Reduced for H2 memory constraints */
#endif
}

uint8_t zb_hal_get_max_children(void)
{
    return 16;  /* Reduced for H2 */
}

uint16_t zb_hal_get_io_buffer_size(void)
{
    return 192;  /* Slightly reduced for H2 */
}

uint8_t zb_hal_get_binding_table_size(void)
{
    return 32;  /* Reduced for H2 */
}

uint8_t zb_hal_get_scene_table_size(void)
{
    return 8;  /* Reduced for H2 */
}

/* ============================================================================
 * Feature Availability
 * ============================================================================
 * Some features disabled by default on H2 to save memory
 */

bool zb_hal_feature_ota_enabled(void)
{
#if defined(CONFIG_ZB_ENABLE_OTA)
    return CONFIG_ZB_ENABLE_OTA;
#else
    return false;  /* OTA disabled by default on H2 to save ~20KB */
#endif
}

bool zb_hal_feature_touchlink_enabled(void)
{
    return false;  /* Touchlink disabled to save memory */
}

bool zb_hal_feature_green_power_enabled(void)
{
    return true;  /* Green Power kept for battery device support */
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

void zb_hal_log_config(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " HAL Configuration: %s", zb_hal_get_chip_name());
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " UART: TX=%d, RX=%d", zb_hal_get_uart_tx_pin(), zb_hal_get_uart_rx_pin());
    ESP_LOGI(TAG, " LED GPIO: %d", zb_hal_get_led_gpio());
    ESP_LOGI(TAG, " RAM: %lu KB, PSRAM: %s",
             (unsigned long)zb_hal_get_ram_size_kb(),
             zb_hal_has_psram() ? "yes" : "no");
    ESP_LOGI(TAG, " Max devices: %u, Max children: %u",
             zb_hal_get_max_network_devices(), zb_hal_get_max_children());
    ESP_LOGI(TAG, " Stack sizes: ZB=%zu, Avail=%zu, UART=%zu",
             zb_hal_get_zb_task_stack_size(),
             zb_hal_get_avail_task_stack_size(),
             zb_hal_get_uart_rx_task_stack_size());
    ESP_LOGI(TAG, " Features: OTA=%d, Touchlink=%d, GreenPower=%d",
             zb_hal_feature_ota_enabled(),
             zb_hal_feature_touchlink_enabled(),
             zb_hal_feature_green_power_enabled());
    ESP_LOGW(TAG, " NOTE: Memory-constrained configuration");
    ESP_LOGI(TAG, "========================================");
}
