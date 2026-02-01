/**
 * @file led_effects.h
 * @brief LED effect timing and configuration constants
 *
 * This header defines constants for LED effect durations, status indicator
 * timings, PWM configuration, and HSV color conversion to eliminate magic
 * numbers throughout the LED controller module.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef LED_EFFECTS_H
#define LED_EFFECTS_H

/* ============================================================================
 * Effect Default Durations (milliseconds)
 * ============================================================================ */

/** @brief Default breathing effect duration (complete cycle) in milliseconds */
#define LED_EFFECT_FADE_DURATION_MS         3000

/** @brief Default blink effect duration (on+off cycle) in milliseconds */
#define LED_EFFECT_BLINK_DURATION_MS        1000

/** @brief Fast blink effect duration (for error indication) in milliseconds */
#define LED_EFFECT_FAST_BLINK_DURATION_MS   200

/** @brief Rainbow/breathe effect duration (complete cycle) in milliseconds */
#define LED_EFFECT_BREATHE_DURATION_MS      5000

/** @brief Strobe effect on-phase duration in milliseconds */
#define LED_EFFECT_STROBE_ON_MS             100

/** @brief Strobe effect off-phase duration in milliseconds */
#define LED_EFFECT_STROBE_OFF_MS            50

/** @brief Candle effect default period in milliseconds */
#define LED_EFFECT_CANDLE_PERIOD_MS         100

/* ============================================================================
 * Status Indicator Timings
 * ============================================================================ */

/** @brief WiFi connecting breathing effect period in milliseconds */
#define LED_STATUS_WIFI_BREATHING_MS        2000

/** @brief WiFi connected flash duration in microseconds */
#define LED_STATUS_FLASH_DURATION_US        500000

/** @brief OTA update blink period in milliseconds */
#define LED_STATUS_OTA_BLINK_MS             1000

/** @brief Error status fast blink period in milliseconds */
#define LED_STATUS_ERROR_BLINK_FAST_MS      200

/** @brief Zigbee pairing breathing effect period in milliseconds */
#define LED_STATUS_ZIGBEE_BREATHING_MS      1500

/** @brief Notification blink period in microseconds */
#define LED_NOTIFY_BLINK_PERIOD_US          150000

/** @brief Pairing mode breathing effect period in milliseconds */
#define LED_STATUS_PAIRING_BREATHING_MS     1000

/** @brief Warning status blink period in milliseconds */
#define LED_STATUS_WARNING_BLINK_MS         1000

/** @brief Boot status breathing period in milliseconds */
#define LED_STATUS_BOOT_BREATHING_MS        2000

/* ============================================================================
 * PWM and Hardware Configuration
 * ============================================================================ */

/** @brief RMT resolution frequency in Hz (10 MHz) */
#define LED_PWM_FREQUENCY_HZ                (10 * 1000 * 1000)

/** @brief Candle effect random flicker maximum variation in microseconds */
#define LED_EFFECT_CANDLE_RANDOM_MAX_US     50000

/** @brief Candle effect base period in microseconds */
#define LED_EFFECT_CANDLE_BASE_PERIOD_US    50000

/* ============================================================================
 * HSV Color Conversion
 * ============================================================================ */

/** @brief HSV hue region divisor for color wheel calculation */
#define HSV_HUE_REGION_DIVISOR              60

/** @brief Full hue circle in degrees */
#define HSV_HUE_FULL_CIRCLE                 360

/* ============================================================================
 * Effect Calculation Constants
 * ============================================================================ */

/** @brief Effect timer interval in milliseconds (50 Hz) */
#define LED_EFFECT_TIMER_INTERVAL_MS        20

/** @brief Default effect period in microseconds */
#define LED_EFFECT_DEFAULT_PERIOD_US        (LED_EFFECT_TIMER_INTERVAL_MS * 1000)

/** @brief Minimum strobe period in microseconds */
#define LED_STROBE_MIN_PERIOD_US            10000

/* ============================================================================
 * Effect Fallback Step Counts
 * ============================================================================ */

/** @brief Sine wave phase quadrant 1 boundary (0-90 degrees) */
#define LED_PHASE_QUADRANT_1                32

/** @brief Sine wave phase quadrant 2 boundary (90-180 degrees) */
#define LED_PHASE_QUADRANT_2                64

/** @brief Default breathing effect total steps */
#define LED_BREATHING_DEFAULT_STEPS         150

/** @brief Default blink effect half-period steps */
#define LED_BLINK_DEFAULT_HALF_PERIOD       25

/** @brief Default rainbow effect total steps */
#define LED_RAINBOW_DEFAULT_STEPS           250

/** @brief Maximum brightness value */
#define LED_BRIGHTNESS_MAX                  255

/** @brief Candle effect base brightness */
#define LED_CANDLE_BASE_BRIGHTNESS          175

/** @brief Candle effect brightness variation */
#define LED_CANDLE_BRIGHTNESS_VARIATION     80

/** @brief Candle effect red variation */
#define LED_CANDLE_RED_VARIATION            30

/** @brief Candle effect green base value */
#define LED_CANDLE_GREEN_BASE               100

/** @brief Candle effect green variation */
#define LED_CANDLE_GREEN_VARIATION          55

/** @brief Candle effect blue base value */
#define LED_CANDLE_BLUE_BASE                20

#endif /* LED_EFFECTS_H */
