/**
 * @file zb_hal_c5.c
 * @brief HAL Implementation for ESP32-C5
 *
 * ESP32-C5 specifications:
 * - CPU: 240 MHz RISC-V dual-core
 * - RAM: 512 KB internal SRAM
 * - PSRAM: Optional 8MB (Waveshare N16R8)
 * - WiFi: WiFi 6 (not used in Zigbee-only mode)
 * - Zigbee SDK: Identical to H2
 */

#include "zb_hal.h"
#include "esp_mac.h"
#include "esp_log.h"

static const char *TAG = "ZB_HAL_C5";

/* ============================================================================
 * Chip Identification
 * ============================================================================ */

const char* zb_hal_get_chip_name(void)
{
    return "ESP32-C5";
}

zb_hal_chip_t zb_hal_get_chip_type(void)
{
    return ZB_HAL_CHIP_ESP32_C5;
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
 * ============================================================================ */

int zb_hal_get_uart_tx_pin(void)
{
#if defined(CONFIG_ZB_UART_TX_PIN)
    return CONFIG_ZB_UART_TX_PIN;
#else
    return 7;  /* IO7 → S3 RX (default) */
#endif
}

int zb_hal_get_uart_rx_pin(void)
{
#if defined(CONFIG_ZB_UART_RX_PIN)
    return CONFIG_ZB_UART_RX_PIN;
#else
    return 6;  /* IO6 ← S3 TX (default) */
#endif
}

int zb_hal_get_led_gpio(void)
{
#if defined(CONFIG_ZB_LED_GPIO)
    return CONFIG_ZB_LED_GPIO;
#else
    return 8;  /* WS2812 on GPIO8 (Waveshare C5 board) */
#endif
}

/* ============================================================================
 * Memory Configuration
 * ============================================================================ */

size_t zb_hal_get_zb_task_stack_size(void)
{
    return 6 * 1024;  /* 6KB - C5 has plenty of RAM */
}

size_t zb_hal_get_avail_task_stack_size(void)
{
    return 4 * 1024;  /* 4KB */
}

size_t zb_hal_get_uart_rx_task_stack_size(void)
{
    return 4 * 1024;  /* 4KB */
}

bool zb_hal_has_psram(void)
{
#if CONFIG_SPIRAM
    return true;
#else
    return false;
#endif
}

uint32_t zb_hal_get_ram_size_kb(void)
{
    return 512;  /* 512 KB internal SRAM */
}

/* ============================================================================
 * Network Configuration Defaults
 * ============================================================================ */

uint8_t zb_hal_get_default_channel(void)
{
    return 11;  /* Channel 11 - best device compatibility */
}

uint16_t zb_hal_get_max_network_devices(void)
{
#if defined(CONFIG_ZB_MAX_DEVICES)
    return CONFIG_ZB_MAX_DEVICES;
#else
    return 64;  /* C5 can handle larger networks */
#endif
}

uint8_t zb_hal_get_max_children(void)
{
    return 32;  /* Generous default for C5 */
}

uint16_t zb_hal_get_io_buffer_size(void)
{
    return 256;  /* Standard buffer size */
}

uint8_t zb_hal_get_binding_table_size(void)
{
    return 64;  /* Larger binding table for C5 */
}

uint8_t zb_hal_get_scene_table_size(void)
{
    return 16;  /* Standard scene table */
}

/* ============================================================================
 * Feature Availability
 * ============================================================================ */

bool zb_hal_feature_ota_enabled(void)
{
#if defined(CONFIG_ZB_ENABLE_OTA)
    return CONFIG_ZB_ENABLE_OTA;
#else
    return true;  /* OTA enabled by default on C5 */
#endif
}

bool zb_hal_feature_touchlink_enabled(void)
{
    return true;  /* Touchlink available on C5 */
}

bool zb_hal_feature_green_power_enabled(void)
{
    return true;  /* Green Power available on C5 */
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
    ESP_LOGI(TAG, "========================================");
}
