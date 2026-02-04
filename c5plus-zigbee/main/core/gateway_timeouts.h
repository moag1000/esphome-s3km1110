/**
 * @file gateway_timeouts.h
 * @brief Gateway Timeout and Timing Constants
 *
 * Centralized definitions for timeouts, delays, and timing-related constants.
 * Includes both millisecond values and FreeRTOS tick conversions.
 *
 * Categories:
 * - GW_TIMEOUT_*_MS     : Operation timeouts in milliseconds
 * - GW_TIMEOUT_*_TICKS  : Operation timeouts in FreeRTOS ticks
 * - GW_DELAY_*_MS       : Non-timeout delays in milliseconds
 * - GW_DELAY_*_TICKS    : Non-timeout delays in FreeRTOS ticks
 * - GW_RETRY_*          : Retry counts for operations
 * - GW_INTERVAL_*_MS    : Periodic intervals in milliseconds
 * - GW_INTERVAL_*_TICKS : Periodic intervals in FreeRTOS ticks
 *
 * Related headers:
 * - gateway_defaults.h      - Domain-specific timeouts (mutex, WiFi, MQTT)
 * - gateway_buffer_sizes.h  - Buffer size constants
 * - gateway_config.h        - Umbrella header including all
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef GATEWAY_TIMEOUTS_H
#define GATEWAY_TIMEOUTS_H

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Standard Timeouts (milliseconds)
 * ============================================================================
 * Common timeout values for various operations.
 * ============================================================================ */

/** @brief Minimal timeout (10ms) - very fast operations */
#define GW_TIMEOUT_MINIMAL_MS               10

/** @brief Short timeout (50ms) - quick polling */
#define GW_TIMEOUT_SHORT_MS                 50

/** @brief Fast timeout (100ms) - mutex, semaphore acquisition */
#define GW_TIMEOUT_FAST_MS                  100

/** @brief Medium timeout (500ms) - inter-task communication */
#define GW_TIMEOUT_MEDIUM_MS                500

/** @brief Standard timeout (1 second) - general operations */
#define GW_TIMEOUT_STANDARD_MS              1000

/** @brief Extended timeout (2 seconds) - network operations */
#define GW_TIMEOUT_EXTENDED_MS              2000

/** @brief Long timeout (5 seconds) - slow operations */
#define GW_TIMEOUT_LONG_MS                  5000

/** @brief Very long timeout (10 seconds) - initialization */
#define GW_TIMEOUT_VERY_LONG_MS             10000

/** @brief Connection timeout (30 seconds) - WiFi/MQTT connect */
#define GW_TIMEOUT_CONNECTION_MS            30000

/** @brief Startup timeout (60 seconds) - full system startup */
#define GW_TIMEOUT_STARTUP_MS               60000

/* ============================================================================
 * FreeRTOS Tick Conversions
 * ============================================================================
 * Pre-computed tick values for common timeouts.
 * ============================================================================ */

/** @brief Minimal timeout in ticks (10ms) */
#define GW_TIMEOUT_MINIMAL_TICKS            pdMS_TO_TICKS(GW_TIMEOUT_MINIMAL_MS)

/** @brief Short timeout in ticks (50ms) */
#define GW_TIMEOUT_SHORT_TICKS              pdMS_TO_TICKS(GW_TIMEOUT_SHORT_MS)

/** @brief Fast timeout in ticks (100ms) */
#define GW_TIMEOUT_FAST_TICKS               pdMS_TO_TICKS(GW_TIMEOUT_FAST_MS)

/** @brief Medium timeout in ticks (500ms) */
#define GW_TIMEOUT_MEDIUM_TICKS             pdMS_TO_TICKS(GW_TIMEOUT_MEDIUM_MS)

/** @brief Standard timeout in ticks (1 second) */
#define GW_TIMEOUT_STANDARD_TICKS           pdMS_TO_TICKS(GW_TIMEOUT_STANDARD_MS)

/** @brief Extended timeout in ticks (2 seconds) */
#define GW_TIMEOUT_EXTENDED_TICKS           pdMS_TO_TICKS(GW_TIMEOUT_EXTENDED_MS)

/** @brief Long timeout in ticks (5 seconds) */
#define GW_TIMEOUT_LONG_TICKS               pdMS_TO_TICKS(GW_TIMEOUT_LONG_MS)

/** @brief Very long timeout in ticks (10 seconds) */
#define GW_TIMEOUT_VERY_LONG_TICKS          pdMS_TO_TICKS(GW_TIMEOUT_VERY_LONG_MS)

/** @brief Connection timeout in ticks (30 seconds) */
#define GW_TIMEOUT_CONNECTION_TICKS         pdMS_TO_TICKS(GW_TIMEOUT_CONNECTION_MS)

/** @brief Startup timeout in ticks (60 seconds) */
#define GW_TIMEOUT_STARTUP_TICKS            pdMS_TO_TICKS(GW_TIMEOUT_STARTUP_MS)

/* ============================================================================
 * Delay Values (milliseconds)
 * ============================================================================
 * Non-timeout delays for pacing and stabilization.
 * ============================================================================ */

/** @brief Tiny delay (10ms) - minimal pause */
#define GW_DELAY_TINY_MS                    10

/** @brief Short delay (50ms) - quick pause */
#define GW_DELAY_SHORT_MS                   50

/** @brief Poll delay (100ms) - polling loops */
#define GW_DELAY_POLL_MS                    100

/** @brief Medium delay (500ms) - rate limiting */
#define GW_DELAY_MEDIUM_MS                  500

/** @brief Long delay (1 second) - main loop, stabilization */
#define GW_DELAY_LONG_MS                    1000

/** @brief Extended delay (2 seconds) - recovery delays */
#define GW_DELAY_EXTENDED_MS                2000

/** @brief Very long delay (5 seconds) - startup sequences */
#define GW_DELAY_VERY_LONG_MS               5000

/* ============================================================================
 * FreeRTOS Delay Tick Conversions
 * ============================================================================ */

/** @brief Tiny delay in ticks (10ms) */
#define GW_DELAY_TINY_TICKS                 pdMS_TO_TICKS(GW_DELAY_TINY_MS)

/** @brief Short delay in ticks (50ms) */
#define GW_DELAY_SHORT_TICKS                pdMS_TO_TICKS(GW_DELAY_SHORT_MS)

/** @brief Poll delay in ticks (100ms) */
#define GW_DELAY_POLL_TICKS                 pdMS_TO_TICKS(GW_DELAY_POLL_MS)

/** @brief Medium delay in ticks (500ms) */
#define GW_DELAY_MEDIUM_TICKS               pdMS_TO_TICKS(GW_DELAY_MEDIUM_MS)

/** @brief Long delay in ticks (1 second) */
#define GW_DELAY_LONG_TICKS                 pdMS_TO_TICKS(GW_DELAY_LONG_MS)

/** @brief Extended delay in ticks (2 seconds) */
#define GW_DELAY_EXTENDED_TICKS             pdMS_TO_TICKS(GW_DELAY_EXTENDED_MS)

/** @brief Very long delay in ticks (5 seconds) */
#define GW_DELAY_VERY_LONG_TICKS            pdMS_TO_TICKS(GW_DELAY_VERY_LONG_MS)

/* ============================================================================
 * Retry Counts
 * ============================================================================
 * Standard retry counts for various operations.
 * ============================================================================ */

/** @brief Minimal retry count (quick fail) */
#define GW_RETRY_MINIMAL                    3

/** @brief Standard retry count */
#define GW_RETRY_STANDARD                   5

/** @brief Extended retry count (resilient operations) */
#define GW_RETRY_EXTENDED                   10

/** @brief Maximum retry count (critical operations) */
#define GW_RETRY_MAXIMUM                    20

/** @brief Infinite retry (use with caution) */
#define GW_RETRY_INFINITE                   0xFFFFFFFF

/* ============================================================================
 * Interval Values (milliseconds)
 * ============================================================================
 * Periodic task intervals and polling rates.
 * ============================================================================ */

/** @brief Fast polling interval (100ms) */
#define GW_INTERVAL_FAST_MS                 100

/** @brief Medium polling interval (500ms) */
#define GW_INTERVAL_MEDIUM_MS               500

/** @brief Standard polling interval (1 second) */
#define GW_INTERVAL_STANDARD_MS             1000

/** @brief Slow polling interval (5 seconds) */
#define GW_INTERVAL_SLOW_MS                 5000

/** @brief Very slow polling interval (10 seconds) */
#define GW_INTERVAL_VERY_SLOW_MS            10000

/** @brief Minute interval (60 seconds) */
#define GW_INTERVAL_MINUTE_MS               60000

/* ============================================================================
 * FreeRTOS Interval Tick Conversions
 * ============================================================================ */

/** @brief Fast interval in ticks (100ms) */
#define GW_INTERVAL_FAST_TICKS              pdMS_TO_TICKS(GW_INTERVAL_FAST_MS)

/** @brief Medium interval in ticks (500ms) */
#define GW_INTERVAL_MEDIUM_TICKS            pdMS_TO_TICKS(GW_INTERVAL_MEDIUM_MS)

/** @brief Standard interval in ticks (1 second) */
#define GW_INTERVAL_STANDARD_TICKS          pdMS_TO_TICKS(GW_INTERVAL_STANDARD_MS)

/** @brief Slow interval in ticks (5 seconds) */
#define GW_INTERVAL_SLOW_TICKS              pdMS_TO_TICKS(GW_INTERVAL_SLOW_MS)

/** @brief Very slow interval in ticks (10 seconds) */
#define GW_INTERVAL_VERY_SLOW_TICKS         pdMS_TO_TICKS(GW_INTERVAL_VERY_SLOW_MS)

/** @brief Minute interval in ticks (60 seconds) */
#define GW_INTERVAL_MINUTE_TICKS            pdMS_TO_TICKS(GW_INTERVAL_MINUTE_MS)

/* ============================================================================
 * Time Unit Conversion Constants
 * ============================================================================
 * Standard time unit conversion factors.
 * ============================================================================ */

/** @brief Seconds per hour (60 * 60 = 3600) */
#define GW_SECONDS_PER_HOUR                 3600

/** @brief Milliseconds per second */
#define GW_MS_PER_SECOND                    1000

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_TIMEOUTS_H */
