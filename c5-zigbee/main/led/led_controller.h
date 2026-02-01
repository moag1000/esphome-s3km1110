/**
 * @file led_controller.h
 * @brief LED Controller with Effects for Addressable RGB LEDs
 *
 * Features:
 * - RGB color control with brightness
 * - Multiple effect modes (breathing, blink, rainbow, etc.)
 * - Status indicator effects (boot, WiFi, error, OTA)
 * - ESPHome Light entity compatible
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants and Definitions
 * ============================================================================ */

/* ============================================================================
 * LED Effect Timing Constants (CQ-017)
 * ============================================================================ */

/** @brief Default LED effect timer period in microseconds (20ms) */
#define LED_EFFECT_DEFAULT_PERIOD_US        20000

/** @brief Minimum strobe effect period in microseconds (10ms) */
#define LED_STROBE_MIN_PERIOD_US            10000

/**
 * @brief LED effect modes
 */
typedef enum {
    LED_EFFECT_NONE = 0,        /**< No effect, solid color */
    LED_EFFECT_BREATHING,       /**< Breathing/pulse effect */
    LED_EFFECT_BLINK,           /**< Simple on/off blink */
    LED_EFFECT_BLINK_FAST,      /**< Fast blink (error indicator) */
    LED_EFFECT_RAINBOW,         /**< Rainbow color cycle */
    LED_EFFECT_CANDLE,          /**< Candle flicker effect */
    LED_EFFECT_STROBE,          /**< Strobe flash effect */
    LED_EFFECT_MAX
} led_effect_t;

/**
 * @brief System status for automatic LED indication
 */
typedef enum {
    LED_STATUS_BOOT,            /**< System booting - breathing blue */
    LED_STATUS_WIFI_CONNECTING, /**< WiFi connecting - slow blink yellow */
    LED_STATUS_WIFI_CONNECTED,  /**< WiFi connected - brief green flash */
    LED_STATUS_NORMAL,          /**< Normal operation - user controlled */
    LED_STATUS_PAIRING,         /**< Pairing mode - breathing cyan */
    LED_STATUS_WARNING,         /**< Warning - slow blink orange */
    LED_STATUS_ERROR,           /**< Error - fast blink red */
    LED_STATUS_OTA,             /**< OTA update - breathing purple */
} led_status_t;

/**
 * @brief Notification blink types
 */
typedef enum {
    LED_NOTIFY_DEVICE_NEW,      /**< New device joined - 1x blue blink */
    LED_NOTIFY_DEVICE_KNOWN,    /**< Known device reconnected - 2x blue blink */
    LED_NOTIFY_DEVICE_FAILED,   /**< Device interview failed - 1x red blink */
} led_notify_t;

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief LED state structure
 */
typedef struct {
    bool on;                    /**< LED on/off state */
    uint8_t brightness;         /**< Brightness 0-255 */
    uint8_t red;                /**< Red component 0-255 */
    uint8_t green;              /**< Green component 0-255 */
    uint8_t blue;               /**< Blue component 0-255 */
    led_effect_t effect;        /**< Current effect */
    uint16_t effect_speed;      /**< Effect speed in ms (period) */
} led_state_t;

/**
 * @brief Effect definition for ESPHome
 */
typedef struct {
    const char *name;           /**< Effect name */
    led_effect_t effect;        /**< Effect type */
    uint16_t default_speed;     /**< Default speed in ms */
} led_effect_def_t;

/** Number of available effects */
#define LED_EFFECT_COUNT        (LED_EFFECT_MAX - 1)

/** Effect definitions for ESPHome */
extern const led_effect_def_t led_effects[];

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize LED controller
 *
 * @return ESP_OK on success
 */
esp_err_t led_controller_init(void);

/**
 * @brief Deinitialize LED controller
 */
void led_controller_deinit(void);

/**
 * @brief Set LED on/off state
 *
 * @param[in] on true to turn on, false to turn off
 */
void led_set_on(bool on);

/**
 * @brief Get LED on/off state
 *
 * @return true if LED is on
 */
bool led_get_on(void);

/**
 * @brief Set LED brightness
 *
 * @param[in] brightness Brightness 0-255
 */
void led_set_brightness(uint8_t brightness);

/**
 * @brief Get LED brightness
 *
 * @return Current brightness 0-255
 */
uint8_t led_get_brightness(void);

/**
 * @brief Set LED color (RGB)
 *
 * @param[in] r Red component 0-255
 * @param[in] g Green component 0-255
 * @param[in] b Blue component 0-255
 */
void led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Get LED color (RGB)
 *
 * @param[out] r Pointer to receive red component (can be NULL)
 * @param[out] g Pointer to receive green component (can be NULL)
 * @param[out] b Pointer to receive blue component (can be NULL)
 */
void led_get_color(uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief Set LED effect
 *
 * @param[in] effect Effect type
 * @param[in] speed Effect speed in ms (0 = use default)
 */
void led_set_effect(led_effect_t effect, uint16_t speed);

/**
 * @brief Set LED effect by name
 *
 * @param[in] name Effect name (case insensitive)
 * @return ESP_OK if effect found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t led_set_effect_by_name(const char *name);

/**
 * @brief Get current effect
 *
 * @return Current effect type
 */
led_effect_t led_get_effect(void);

/**
 * @brief Get current effect name
 *
 * @return Effect name string
 */
const char *led_get_effect_name(void);

/**
 * @brief Get full LED state
 *
 * @param[out] state Pointer to receive state
 */
void led_get_state(led_state_t *state);

/**
 * @brief Set full LED state
 *
 * @param[in] state Pointer to state
 */
void led_set_state(const led_state_t *state);

/**
 * @brief Set system status for automatic LED indication
 *
 * Only has effect if CONFIG_GW_LED_STATUS_EFFECTS is enabled.
 *
 * @param[in] status System status
 */
void led_set_status(led_status_t status);

/**
 * @brief Get current system status
 *
 * @return Current status
 */
led_status_t led_get_status(void);

/**
 * @brief Check if LED is in manual control mode
 *
 * @return true if user has set a manual effect/color
 */
bool led_is_manual_mode(void);

/**
 * @brief Release manual control, return to status indication
 */
void led_release_manual(void);

/**
 * @brief Trigger a notification blink sequence
 *
 * The LED will blink the specified pattern (color + count) and then
 * automatically return to the previous state/color.
 *
 * @param[in] notify Notification type
 */
void led_blink_notify(led_notify_t notify);

/**
 * @brief Self-test function
 *
 * @return ESP_OK if all tests pass
 */
esp_err_t led_controller_test(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROLLER_H */
