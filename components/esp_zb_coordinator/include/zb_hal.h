/**
 * @file zb_hal.h
 * @brief Hardware Abstraction Layer for Multi-Chip Zigbee Coordinator
 *
 * Provides chip-agnostic interface for platform-specific functionality.
 * Implementations provided in platform/esp32c5/ and platform/esp32h2/.
 */

#ifndef ZB_HAL_H
#define ZB_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Chip Identification
 * ============================================================================ */

/**
 * @brief Get the chip name string
 * @return Chip name (e.g., "ESP32-C5", "ESP32-H2")
 */
const char* zb_hal_get_chip_name(void);

/**
 * @brief Get the chip type enumeration
 */
typedef enum {
    ZB_HAL_CHIP_UNKNOWN = 0,
    ZB_HAL_CHIP_ESP32_C5,
    ZB_HAL_CHIP_ESP32_H2,
} zb_hal_chip_t;

zb_hal_chip_t zb_hal_get_chip_type(void);

/* ============================================================================
 * MAC Address
 * ============================================================================ */

/**
 * @brief Read the IEEE 802.15.4 MAC address
 * @param mac_addr Output buffer (8 bytes)
 * @return ESP_OK on success
 */
esp_err_t zb_hal_read_mac(uint8_t *mac_addr);

/* ============================================================================
 * GPIO Pin Configuration (Defaults)
 * ============================================================================ */

/**
 * @brief Get default UART TX pin for bridge communication
 * @return GPIO number
 */
int zb_hal_get_uart_tx_pin(void);

/**
 * @brief Get default UART RX pin for bridge communication
 * @return GPIO number
 */
int zb_hal_get_uart_rx_pin(void);

/**
 * @brief Get default LED GPIO pin
 * @return GPIO number, or -1 if no LED available
 */
int zb_hal_get_led_gpio(void);

/* ============================================================================
 * Memory Configuration
 * ============================================================================ */

/**
 * @brief Get recommended Zigbee task stack size
 * @return Stack size in bytes
 */
size_t zb_hal_get_zb_task_stack_size(void);

/**
 * @brief Get recommended availability task stack size
 * @return Stack size in bytes
 */
size_t zb_hal_get_avail_task_stack_size(void);

/**
 * @brief Get recommended UART RX task stack size
 * @return Stack size in bytes
 */
size_t zb_hal_get_uart_rx_task_stack_size(void);

/**
 * @brief Check if PSRAM is available on this chip
 * @return true if PSRAM is available
 */
bool zb_hal_has_psram(void);

/**
 * @brief Get total internal RAM size
 * @return RAM size in KB
 */
uint32_t zb_hal_get_ram_size_kb(void);

/* ============================================================================
 * Network Configuration Defaults
 * ============================================================================ */

/**
 * @brief Get default maximum network devices
 * @return Max device count
 */
uint16_t zb_hal_get_max_network_devices(void);

/**
 * @brief Get default maximum children
 * @return Max children count
 */
uint8_t zb_hal_get_max_children(void);

/**
 * @brief Get default IO buffer size
 * @return Buffer size in bytes
 */
uint16_t zb_hal_get_io_buffer_size(void);

/**
 * @brief Get default binding table size
 * @return Table size
 */
uint8_t zb_hal_get_binding_table_size(void);

/**
 * @brief Get default scene table size
 * @return Table size
 */
uint8_t zb_hal_get_scene_table_size(void);

/* ============================================================================
 * Feature Availability
 * ============================================================================ */

/**
 * @brief Check if OTA support should be enabled
 * @return true if OTA is recommended for this chip
 */
bool zb_hal_feature_ota_enabled(void);

/**
 * @brief Check if Touchlink should be enabled
 * @return true if Touchlink is recommended
 */
bool zb_hal_feature_touchlink_enabled(void);

/**
 * @brief Check if Green Power should be enabled
 * @return true if Green Power is recommended
 */
bool zb_hal_feature_green_power_enabled(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Log HAL configuration summary
 */
void zb_hal_log_config(void);

/* ============================================================================
 * LED Callback Interface
 *
 * The LED controller is chip-specific (different GPIO, optional on some boards).
 * The main component registers its LED functions here to avoid circular deps.
 * ============================================================================ */

/**
 * @brief LED notification types (mirror of led_notify_t)
 */
typedef enum {
    ZB_HAL_LED_NOTIFY_DEVICE_NEW = 0,    /**< New device joined */
    ZB_HAL_LED_NOTIFY_DEVICE_KNOWN,       /**< Known device reconnected */
    ZB_HAL_LED_NOTIFY_DEVICE_FAILED,      /**< Device interview failed */
} zb_hal_led_notify_t;

/**
 * @brief LED status types (mirror of led_status_t)
 */
typedef enum {
    ZB_HAL_LED_STATUS_BOOT = 0,
    ZB_HAL_LED_STATUS_WIFI_CONNECTING,
    ZB_HAL_LED_STATUS_WIFI_CONNECTED,
    ZB_HAL_LED_STATUS_NORMAL,
    ZB_HAL_LED_STATUS_PAIRING,
    ZB_HAL_LED_STATUS_WARNING,
    ZB_HAL_LED_STATUS_ERROR,
    ZB_HAL_LED_STATUS_OTA,
} zb_hal_led_status_t;

/**
 * @brief Callback function type for LED blink notifications
 */
typedef void (*zb_hal_led_blink_fn)(zb_hal_led_notify_t notify);

/**
 * @brief Callback function type for LED status changes
 */
typedef void (*zb_hal_led_status_fn)(zb_hal_led_status_t status);

/**
 * @brief Register LED callback functions
 *
 * Call this from main component during initialization to enable LED feedback.
 * If not called, LED functions are no-ops.
 *
 * @param blink_fn Callback for led_blink_notify (can be NULL)
 * @param status_fn Callback for led_set_status (can be NULL)
 */
void zb_hal_register_led_callbacks(zb_hal_led_blink_fn blink_fn,
                                   zb_hal_led_status_fn status_fn);

/**
 * @brief Trigger LED blink notification (calls registered callback)
 * @param notify Notification type
 */
void zb_hal_led_blink_notify(zb_hal_led_notify_t notify);

/**
 * @brief Set LED status (calls registered callback)
 * @param status Status type
 */
void zb_hal_led_set_status(zb_hal_led_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZB_HAL_H */
