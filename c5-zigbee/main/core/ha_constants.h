/**
 * @file ha_constants.h
 * @brief Home Assistant Integration Constants
 *
 * Device classes, units, and state values for Home Assistant MQTT Discovery.
 * See: https://www.home-assistant.io/integrations/sensor/#device-class
 *
 * Categories:
 * - HA_DEVICE_CLASS_*    : Device class strings for sensors/binary sensors
 * - HA_UNIT_*            : Unit of measurement strings
 * - HA_STATE_CLASS_*     : State class strings
 * - HA_ENTITY_CATEGORY_* : Entity category strings
 * - HA_COVER_*           : Cover position values
 * - HA_CLIMATE_*         : Climate/thermostat defaults
 *
 * Related headers:
 * - gateway_defaults.h  - Task priorities, general defaults
 * - gateway_timeouts.h  - Timeout and delay constants
 * - gateway_config.h    - Umbrella header for all gateway constants
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef HA_CONSTANTS_H
#define HA_CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Device Classes - Sensors
 * ============================================================================ */

#define HA_DEVICE_CLASS_TEMPERATURE         "temperature"
#define HA_DEVICE_CLASS_HUMIDITY            "humidity"
#define HA_DEVICE_CLASS_BATTERY             "battery"
#define HA_DEVICE_CLASS_ILLUMINANCE         "illuminance"
#define HA_DEVICE_CLASS_PRESSURE            "atmospheric_pressure"
#define HA_DEVICE_CLASS_PM25                "pm25"
#define HA_DEVICE_CLASS_VOLTAGE             "voltage"
#define HA_DEVICE_CLASS_CURRENT             "current"
#define HA_DEVICE_CLASS_POWER               "power"
#define HA_DEVICE_CLASS_ENERGY              "energy"
#define HA_DEVICE_CLASS_POWER_FACTOR        "power_factor"
#define HA_DEVICE_CLASS_SIGNAL_STRENGTH     "signal_strength"

/* ============================================================================
 * Device Classes - Binary Sensors
 * ============================================================================ */

#define HA_DEVICE_CLASS_MOTION              "motion"
#define HA_DEVICE_CLASS_DOOR                "door"
#define HA_DEVICE_CLASS_WINDOW              "window"
#define HA_DEVICE_CLASS_OCCUPANCY           "occupancy"
#define HA_DEVICE_CLASS_SMOKE               "smoke"
#define HA_DEVICE_CLASS_GAS                 "gas"
#define HA_DEVICE_CLASS_MOISTURE            "moisture"
#define HA_DEVICE_CLASS_VIBRATION           "vibration"
#define HA_DEVICE_CLASS_PROBLEM             "problem"
#define HA_DEVICE_CLASS_SAFETY              "safety"

/* ============================================================================
 * Units of Measurement
 * ============================================================================ */

/** @brief Temperature unit (Celsius) - Unicode degree symbol */
#define HA_UNIT_CELSIUS                     "\u00B0C"

/** @brief Temperature unit (Fahrenheit) */
#define HA_UNIT_FAHRENHEIT                  "\u00B0F"

/** @brief Percentage */
#define HA_UNIT_PERCENT                     "%"

/** @brief Illuminance (lux) */
#define HA_UNIT_LUX                         "lx"

/** @brief Pressure (hectopascals) */
#define HA_UNIT_HPA                         "hPa"

/** @brief Concentration (micrograms per cubic meter) */
#define HA_UNIT_UG_M3                       "\u00b5g/m\u00b3"

/** @brief Voltage (volts) */
#define HA_UNIT_VOLT                        "V"

/** @brief Current (amperes) */
#define HA_UNIT_AMPERE                      "A"

/** @brief Power (watts) */
#define HA_UNIT_WATT                        "W"

/** @brief Energy (kilowatt-hours) */
#define HA_UNIT_KWH                         "kWh"

/** @brief Signal strength (dBm) */
#define HA_UNIT_DBM                         "dBm"

/** @brief Frequency (hertz) */
#define HA_UNIT_HERTZ                       "Hz"

/* ============================================================================
 * State Classes
 * ============================================================================ */

#define HA_STATE_CLASS_MEASUREMENT          "measurement"
#define HA_STATE_CLASS_TOTAL                "total"
#define HA_STATE_CLASS_TOTAL_INCREASING     "total_increasing"

/* ============================================================================
 * Entity Categories
 * ============================================================================ */

#define HA_ENTITY_CATEGORY_CONFIG           "config"
#define HA_ENTITY_CATEGORY_DIAGNOSTIC       "diagnostic"

/* ============================================================================
 * Cover (Window Covering) Position Values
 * ============================================================================ */

/** @brief Cover fully open position (100%) */
#define HA_COVER_POSITION_OPEN              100

/** @brief Cover fully closed position (0%) */
#define HA_COVER_POSITION_CLOSED            0

/* ============================================================================
 * Climate (Thermostat) Default Values
 * ============================================================================ */

/** @brief Default minimum temperature for thermostat (Celsius) */
#define HA_CLIMATE_MIN_TEMP_DEFAULT         7

/** @brief Default maximum temperature for thermostat (Celsius) */
#define HA_CLIMATE_MAX_TEMP_DEFAULT         35

/** @brief Default temperature step for thermostat (Celsius) */
#define HA_CLIMATE_TEMP_STEP_DEFAULT        0.5f

/* ============================================================================
 * Discovery Timing
 * ============================================================================ */

/** @brief Delay between discovery publishes in milliseconds
 *  Small delay to avoid overwhelming the MQTT broker with burst traffic.
 *  Direct publish architecture allows shorter delays since there's no queue. */
#define HA_DISCOVERY_PUBLISH_DELAY_MS       50

#ifdef __cplusplus
}
#endif

#endif /* HA_CONSTANTS_H */
