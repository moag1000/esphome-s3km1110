/**
 * @file zb_constants.h
 * @brief Zigbee Protocol Constants and Utility Macros
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CONSTANTS_H
#define ZB_CONSTANTS_H

#include <stdint.h>
#include "esp_zigbee_type.h"

/* ============================================================================
 * ZCL Unit Conversion Scales
 * ============================================================================ */

/** @brief Temperature scale: ZCL uses centidegrees (value/100 = °C) */
#define ZCL_TEMP_SCALE              100

/** @brief Humidity scale: ZCL uses centipercent (value/100 = %) */
#define ZCL_HUMIDITY_SCALE          100

/** @brief Pressure scale: ZCL uses 10 Pa units (value/10 = hPa) */
#define ZCL_PRESSURE_SCALE          10

/** @brief Illuminance scale: ZCL uses 10000*log10(lux)+1 */
#define ZCL_ILLUMINANCE_SCALE       10000

/** @brief Voltage scale: ZCL uses decivolts (value/10 = V) */
#define ZCL_VOLTAGE_SCALE           10

/** @brief Current scale: ZCL uses milliamps (value/1000 = A) */
#define ZCL_CURRENT_SCALE           1000

/** @brief Power scale: ZCL uses watts or deciwatts depending on cluster */
#define ZCL_POWER_SCALE             10

/** @brief Color temperature scale: ZCL uses mireds */
#define ZCL_COLOR_TEMP_MIRED_MIN    153     /**< ~6500K */
#define ZCL_COLOR_TEMP_MIRED_MAX    500     /**< ~2000K */

/** @brief Level control max value */
#define ZCL_LEVEL_MAX               254

/** @brief On/Off cluster values */
#define ZCL_ON_OFF_ON               1
#define ZCL_ON_OFF_OFF              0

/** @brief Transition time scale factor (seconds to 1/10 seconds) */
#define ZCL_TRANSITION_TIME_SCALE   10

/** @brief Maximum brightness value for Level Control cluster */
#define ZCL_BRIGHTNESS_MAX          254

/** @brief Maximum attribute value for 8-bit attributes */
#define ZCL_ATTR_VALUE_U8_MAX       255

/* ============================================================================
 * Zigbee Channel Constants
 * ============================================================================ */

/** @brief Minimum valid Zigbee channel (2.4GHz band) */
#define ZB_CHANNEL_MIN                  11

/** @brief Maximum valid Zigbee channel (2.4GHz band) */
#define ZB_CHANNEL_MAX                  26

/** @brief Default primary channel for coordinator */
#define ZB_DEFAULT_PRIMARY_CHANNEL      15

/** @brief Default secondary channel for multi-PAN */
#define ZB_DEFAULT_SECONDARY_CHANNEL    20

/** @brief Channel mask for all channels (11-26) */
#define ZB_CHANNEL_MASK_ALL             0x07FFF800

/* Note: ZB_DEFAULT_MAX_CHILDREN (32) is defined by ESP-Zigbee-SDK in zb_config_common.h */

/* ============================================================================
 * Multi-PAN Constants (CQ-017)
 * ============================================================================ */

/** @brief Default Multi-PAN switch timeout in milliseconds */
#define ZB_MULTI_PAN_SWITCH_TIMEOUT_MS      500

/* ============================================================================
 * Thermostat Defaults (centidegrees)
 * ============================================================================ */

/** @brief Default heating setpoint (20.00C) */
#define ZB_THERMOSTAT_DEFAULT_HEAT_SETPOINT     2000

/** @brief Default cooling setpoint (26.00C) */
#define ZB_THERMOSTAT_DEFAULT_COOL_SETPOINT     2600

/** @brief Minimum heating setpoint limit (7.00C) */
#define ZB_THERMOSTAT_MIN_HEAT_SETPOINT         700

/** @brief Maximum heating setpoint limit (30.00C) */
#define ZB_THERMOSTAT_MAX_HEAT_SETPOINT         3000

/** @brief Minimum cooling setpoint limit (16.00C) */
#define ZB_THERMOSTAT_MIN_COOL_SETPOINT         1600

/** @brief Maximum cooling setpoint limit (32.00C) */
#define ZB_THERMOSTAT_MAX_COOL_SETPOINT         3200

/** @brief Invalid temperature value */
#define ZB_THERMOSTAT_TEMP_INVALID              0x8000

/* ============================================================================
 * Reporting Thresholds
 * ============================================================================ */

/** @brief Humidity reporting threshold (1% = 100 centipercent) */
#define ZB_HUMIDITY_REPORTING_THRESHOLD         100

/** @brief Illuminance reporting threshold */
#define ZB_ILLUMINANCE_REPORTING_THRESHOLD      5000

/* ============================================================================
 * Interview and Network Timeouts
 * ============================================================================ */

/** @brief Device interview timeout in milliseconds (30s for complex devices) */
#define ZB_INTERVIEW_TIMEOUT_MS                 30000

/** @brief Channel change operation timeout in milliseconds */
#define ZB_CHANNEL_CHANGE_TIMEOUT_MS            30000

/* ============================================================================
 * Device and Cluster Limits
 * ============================================================================ */

/** @brief Maximum clusters per endpoint */
#define ZB_MAX_CLUSTERS_PER_ENDPOINT            32

/** @brief Maximum clusters in a binding request */
#define ZB_BINDING_MAX_CLUSTERS                 16

/** @brief Maximum Green Power commands per device */
#define ZB_GREEN_POWER_MAX_CMDS                 16

/* ============================================================================
 * Buffer Size Constants
 * ============================================================================ */

/** @brief Buffer size for IEEE address as NVS key (14 hex + null) */
#define ZB_IEEE_ADDR_KEY_BUFFER_SIZE            17

/** @brief Buffer size for IEEE address string with 0x prefix (2 + 16 + null) */
#define ZB_IEEE_ADDR_STR_BUFFER_SIZE            19

/** @brief Length of IEEE address as hex string (16 chars, no prefix) */
#define ZB_IEEE_ADDR_HEX_STRING_LEN             16

/** @brief Buffer size for Extended PAN ID string (2 + 16 + null) */
#define ZB_EXT_PAN_ID_BUFFER_SIZE               19

/** @brief Length of Extended PAN ID as hex string (16 chars) */
#define ZB_EXT_PAN_ID_HEX_LEN                   16

/** @brief Minimum size for OTA version string buffer */
#define ZB_OTA_VERSION_STRING_MIN_SIZE          24

/** @brief Minimum data size for Touchlink Scan Response */
#define ZB_TOUCHLINK_DATA_MIN_SIZE              20

/** @brief Minimum message size for Zigbee event data */
#define ZB_EVENT_MESSAGE_MIN_SIZE               23

/** @brief Safety margin for JSON buffer operations */
#define ZB_JSON_BUFFER_SAFETY_MARGIN            100

/* ============================================================================
 * Reporting Configuration
 * ============================================================================ */

/** @brief Default minimum reporting interval (seconds) */
#define ZB_REPORT_MIN_INTERVAL_SEC              10

/** @brief Default maximum reporting interval (seconds) */
#define ZB_REPORT_MAX_INTERVAL_SEC              3600

/* ============================================================================
 * Scene Test Values
 * ============================================================================ */

/** @brief Test level value for scene testing */
#define ZB_SCENE_TEST_LEVEL_VALUE               200

/** @brief Default XY color value (0.5 normalized = 32768) */
#define ZB_COLOR_XY_DEFAULT                     32768

/** @brief Test transition time (2.0 seconds in 1/10th second units) */
#define ZB_SCENE_TRANSITION_TIME_TEST           20

/* ============================================================================
 * Cluster-Specific Constants
 * ============================================================================ */

/** @brief Window covering percentage maximum */
#define ZB_WINDOW_COVERING_PERCENT_MAX          100

/** @brief Electrical measurement current divisor (milliamps to amps) */
#define ZB_ELECTRICAL_CURRENT_DIVISOR_MA        1000

/** @brief Dehumidification percentage maximum */
#define ZB_DEHUMID_PERCENT_MAX                  100

/** @brief Test humidity percentage for dehumidification testing */
#define ZB_DEHUMID_TEST_HUMIDITY_PERCENT        65

/** @brief OTA query jitter for broadcast (100% jitter) */
#define ZB_OTA_BROADCAST_QUERY_JITTER           100

/* ============================================================================
 * Poll Control Time Conversion
 * ============================================================================ */

/** @brief Quarter-seconds per hour (3600 * 4 = 14400) */
#define ZB_POLL_QUARTER_SECONDS_PER_HOUR        14400

/* ============================================================================
 * IEEE Address Conversion Helpers
 * ============================================================================ */

/**
 * @brief Convert esp_zb_ieee_addr_t array to uint64_t
 *
 * Converts an 8-byte IEEE address array (little-endian) to a 64-bit integer.
 * This is useful for logging, comparison, and storage operations.
 *
 * @param ieee_array IEEE address as 8-byte array (little-endian)
 * @return IEEE address as uint64_t
 */
static inline uint64_t zb_ieee_to_u64(const esp_zb_ieee_addr_t ieee_array)
{
    uint64_t ieee64 = 0;
    for (int i = 0; i < 8; i++) {
        ieee64 |= ((uint64_t)ieee_array[i]) << (i * 8);
    }
    return ieee64;
}

/**
 * @brief Convert uint64_t to esp_zb_ieee_addr_t array
 *
 * Converts a 64-bit IEEE address to an 8-byte array (little-endian).
 * This is useful when interfacing with ESP-Zigbee-SDK APIs that require
 * the array format.
 *
 * @param ieee64 IEEE address as uint64_t
 * @param ieee_array Output array (8 bytes, little-endian)
 */
static inline void zb_u64_to_ieee(uint64_t ieee64, esp_zb_ieee_addr_t ieee_array)
{
    for (int i = 0; i < 8; i++) {
        ieee_array[i] = (uint8_t)(ieee64 >> (i * 8));
    }
}

#endif /* ZB_CONSTANTS_H */
