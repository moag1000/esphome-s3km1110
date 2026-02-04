/**
 * @file gateway_defaults.h
 * @brief Gateway Default Configuration Constants
 *
 * Centralized definitions for gateway-wide defaults including:
 *   - FreeRTOS task priorities
 *   - Domain-specific timeouts (WiFi, MQTT, mutex)
 *   - Unit conversions
 *   - System monitoring thresholds
 *   - OTA configuration
 *
 * For comprehensive timeout/delay constants, see: gateway_timeouts.h
 * For comprehensive buffer size constants, see: gateway_buffer_sizes.h
 * For all constants via single include, see: gateway_config.h
 *
 * Naming convention: GW_{MODULE}_{ENTITY}_{PROPERTY}
 * Example: GW_DEFAULT_WIFI_TIMEOUT_MS
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef GATEWAY_DEFAULTS_H
#define GATEWAY_DEFAULTS_H

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Domain-Specific Timeouts (milliseconds)
 * ============================================================================
 * These are specific default values for particular subsystems.
 * For general-purpose timeouts, use gateway_timeouts.h
 * ============================================================================ */

/** @brief Default mutex acquisition timeout */
#define GW_DEFAULT_MUTEX_TIMEOUT_MS         100

/** @brief Extended mutex acquisition timeout (1 second) */
#define GW_DEFAULT_MUTEX_TIMEOUT_1S_MS      1000

/** @brief Default mutex acquisition timeout in FreeRTOS ticks */
#define GW_DEFAULT_MUTEX_TIMEOUT_TICKS      pdMS_TO_TICKS(GW_DEFAULT_MUTEX_TIMEOUT_MS)

/** @brief Extended mutex acquisition timeout in FreeRTOS ticks (1 second) */
#define GW_DEFAULT_MUTEX_TIMEOUT_1S_TICKS   pdMS_TO_TICKS(GW_DEFAULT_MUTEX_TIMEOUT_1S_MS)

/** @brief WiFi connection wait timeout (45s for tri-radio coexistence) */
#define GW_DEFAULT_WIFI_TIMEOUT_MS          45000

/** @brief MQTT connection wait timeout */
#define GW_DEFAULT_MQTT_TIMEOUT_MS          30000

/** @brief Task stop/cleanup timeout */
#define GW_DEFAULT_TASK_STOP_TIMEOUT_MS     5000

/** @brief Task stop polling interval */
#define GW_DEFAULT_TASK_STOP_POLL_MS        100

/** @brief WiFi stabilization delay after connect */
#define GW_WIFI_STABILIZE_DELAY_MS          1000

/** @brief Main loop delay */
#define GW_MAIN_LOOP_DELAY_MS               1000

/* ============================================================================
 * FreeRTOS Task Priorities
 * ============================================================================
 * ESP-IDF/FreeRTOS priority range: 0 (lowest) to configMAX_PRIORITIES-1
 * Higher number = higher priority
 * ============================================================================ */

/** @brief Low priority tasks (monitoring, logging) */
#define GW_TASK_PRIORITY_LOW                3

/** @brief Medium priority tasks (MQTT, WiFi) */
#define GW_TASK_PRIORITY_MEDIUM             5

/** @brief High priority tasks (command handlers) */
#define GW_TASK_PRIORITY_HIGH               6

/** @brief Critical priority tasks (Zigbee stack) */
#define GW_TASK_PRIORITY_CRITICAL           20

/* ============================================================================
 * Unit Conversions
 * ============================================================================ */

/** @brief Bytes per megabyte */
#define GW_BYTES_PER_MB                     (1024 * 1024)

/** @brief Seconds per minute */
#define GW_SECONDS_PER_MINUTE               60

/** @brief Seconds per hour */
#define GW_SECONDS_PER_HOUR                 3600

/** @brief Seconds per day */
#define GW_SECONDS_PER_DAY                  86400

/** @brief Microseconds per second */
#define GW_MICROSECONDS_PER_SECOND          1000000

/* ============================================================================
 * System Monitoring
 * ============================================================================ */

/** @brief Minimum monitoring interval (seconds) */
#define GW_SYSMON_INTERVAL_MIN_SEC          10

/** @brief Maximum monitoring interval (seconds) */
#define GW_SYSMON_INTERVAL_MAX_SEC          3600

/** @brief Default monitoring interval (seconds) */
#define GW_SYSMON_INTERVAL_DEFAULT_SEC      60

/* ============================================================================
 * Task Manager (CQ-017)
 * ============================================================================ */

/** @brief Maximum number of tasks to track in task manager */
#define GW_TASK_MAX_COUNT                   32

/** @brief Task stack warning threshold (bytes) */
#define GW_TASK_STACK_WARNING_THRESHOLD     512

/* ============================================================================
 * Progress and Percentage Scales
 * ============================================================================ */

/** @brief Progress percentage scale (0-100) */
#define GW_PROGRESS_PERCENTAGE_SCALE        100

/** @brief Maximum percentage value */
#define GW_PERCENTAGE_MAX                   100

/* ============================================================================
 * Timer Conversions
 * ============================================================================ */

/** @brief Microseconds to milliseconds scale factor */
#define GW_TIMER_US_TO_MS_SCALE             1000

/** @brief Microseconds per second */
#define GW_TIMER_US_PER_SECOND              1000000ULL

/** @brief Hour to milliseconds scale factor */
#define GW_HOUR_TO_MS_SCALE                 (3600 * 1000)

/* ============================================================================
 * OTA Specific Constants
 * ============================================================================ */

/** @brief OTA update check timeout in milliseconds */
#define GW_OTA_CHECK_TIMEOUT_MS             30000

/** @brief OTA restart delay in milliseconds */
#define GW_OTA_RESTART_DELAY_MS             3000

/** @brief OTA transfer buffer size */
#define GW_OTA_TRANSFER_BUFFER_SIZE         4096

/* ============================================================================
 * Memory Thresholds
 * ============================================================================ */

/** @brief Maximum memory fragmentation percentage */
#define GW_MEMORY_FRAGMENTATION_MAX_PERCENT 100

/* ============================================================================
 * MQTT Reconnect Discovery Delays
 * ============================================================================ */

/** @brief Delay before/between discovery batches during reconnect (ms)
 *  Longer than initial discovery to avoid queue pressure after reconnect */
#define GW_RECONNECT_DISCOVERY_DELAY_MS     500

/* ============================================================================
 * Metering and Sensor Precision
 * ============================================================================ */

/** @brief Energy metering precision scale (kWh with 3 decimal places) */
#define GW_METERING_ENERGY_PRECISION        1000.0

/** @brief Power metering precision scale (W with 1 decimal place) */
#define GW_METERING_POWER_PRECISION         10.0

/** @brief Maximum battery percentage value */
#define GW_BATTERY_PERCENTAGE_MAX           100

/** @brief Default link quality value for testing */
#define GW_DEFAULT_LINK_QUALITY_TEST        150

/* ============================================================================
 * Include Specialized Headers for Backwards Compatibility
 * ============================================================================
 * These includes ensure that existing code which only includes gateway_defaults.h
 * continues to have access to all timeout and buffer size constants.
 * For new code, consider using gateway_config.h as the umbrella header.
 * ============================================================================ */

#include "gateway_timeouts.h"
#include "gateway_buffer_sizes.h"

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_DEFAULTS_H */
