/**
 * @file led_controller.h
 * @brief LED Controller for ESP32-H2-Zero with WS2812 RGB LED
 *
 * The Waveshare ESP32-H2-Zero has a WS2812 RGB LED on GPIO8.
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED Status Definitions
 * NOTE: Order must match zb_hal_led_status_t in zb_hal.h!
 */
typedef enum {
    LED_STATUS_BOOT = 0,  /**< Blue - system booting */
    LED_STATUS_WIFI_CONNECTING, /**< Yellow - unused on H2 */
    LED_STATUS_WIFI_CONNECTED,  /**< Green flash - unused on H2 */
    LED_STATUS_NORMAL,    /**< Green - normal operation */
    LED_STATUS_PAIRING,   /**< Orange - pairing/permit join active */
    LED_STATUS_WARNING,   /**< Orange - warning */
    LED_STATUS_ERROR,     /**< Red - error condition */
    LED_STATUS_OTA,       /**< Purple - OTA update */
    LED_STATUS_OFF,       /**< LED off (manual) */
    LED_STATUS_MAX
} led_status_t;

/**
 * @brief LED Notification Types for device events
 * NOTE: Order must match zb_hal_led_notify_t in zb_hal.h!
 */
typedef enum {
    LED_NOTIFY_DEVICE_NEW = 0,  /**< New device joined - blue blink */
    LED_NOTIFY_DEVICE_KNOWN,    /**< Known device reconnected - green blink */
    LED_NOTIFY_DEVICE_FAILED,   /**< Device interview failed - red blink */
} led_notify_t;

/**
 * @brief Initialize LED controller
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if no LED configured
 */
esp_err_t led_controller_init(void);

/**
 * @brief De-initialize LED controller
 */
void led_controller_deinit(void);

/**
 * @brief Set LED status (predefined colors)
 * @param status LED status to set
 */
void led_set_status(led_status_t status);

/**
 * @brief Get current LED status
 * @return Current LED status
 */
led_status_t led_get_status(void);

/**
 * @brief Set LED color directly (RGB)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Turn LED on/off while keeping color
 * @param on true to turn on, false to turn off
 */
void led_set_on(bool on);

/**
 * @brief Check if LED is on
 * @return true if LED is on
 */
bool led_is_on(void);

/**
 * @brief Blink LED once (for notifications)
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param duration_ms Blink duration in milliseconds
 */
void led_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);

/**
 * @brief Trigger notification blink (for device events)
 * @param notify Notification type
 */
void led_blink_notify(led_notify_t notify);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROLLER_H */
