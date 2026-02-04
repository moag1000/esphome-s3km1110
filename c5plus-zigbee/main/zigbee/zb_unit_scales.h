/**
 * @file zb_unit_scales.h
 * @brief Zigbee Unit Conversion and Scaling Constants
 *
 * Centralized definitions for Zigbee-specific unit conversions and scaling
 * factors. These constants are used when converting between Zigbee attribute
 * values and human-readable units.
 *
 * Naming convention: ZB_SCALE_{MEASUREMENT_TYPE}_{DESCRIPTION}
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_UNIT_SCALES_H
#define ZB_UNIT_SCALES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Percentage Scaling
 * ============================================================================
 * Zigbee uses 0-254 for many percentage values (255 = invalid).
 * Some clusters use 0-100 directly.
 * ============================================================================ */

/** @brief Standard percentage divisor (0-100 scale) */
#define ZB_SCALE_PERCENT_DIVISOR            100

/** @brief Level cluster maximum (0-254, 255=invalid) */
#define ZB_SCALE_LEVEL_MAX                  254

/** @brief Percentage to level multiplier (254/100) */
#define ZB_SCALE_PERCENT_TO_LEVEL           2.54f

/** @brief Level to percentage divisor */
#define ZB_SCALE_LEVEL_TO_PERCENT           2.54f

/* ============================================================================
 * Temperature Scaling
 * ============================================================================
 * Zigbee temperature is in units of 0.01 degrees Celsius (centidegrees).
 * Value 0x8000 indicates invalid/unknown temperature.
 * ============================================================================ */

/** @brief Temperature divisor (centidegrees to degrees) */
#define ZB_SCALE_TEMPERATURE_DIVISOR        100

/** @brief Temperature scale factor (same as divisor, for clarity) */
#define ZB_SCALE_TEMPERATURE_FACTOR         100

/** @brief Invalid temperature marker */
#define ZB_SCALE_TEMPERATURE_INVALID        0x8000

/** @brief Minimum valid temperature (-273.15C in centidegrees) */
#define ZB_SCALE_TEMPERATURE_MIN            -27315

/** @brief Maximum valid temperature (327.67C in centidegrees) */
#define ZB_SCALE_TEMPERATURE_MAX            32767

/* ============================================================================
 * Humidity Scaling
 * ============================================================================
 * Zigbee humidity is in units of 0.01% RH (centi-percent).
 * Value 0xFFFF indicates invalid/unknown humidity.
 * ============================================================================ */

/** @brief Humidity divisor (centi-percent to percent) */
#define ZB_SCALE_HUMIDITY_DIVISOR           100

/** @brief Humidity scale factor */
#define ZB_SCALE_HUMIDITY_FACTOR            100

/** @brief Invalid humidity marker */
#define ZB_SCALE_HUMIDITY_INVALID           0xFFFF

/** @brief Minimum humidity (0%) */
#define ZB_SCALE_HUMIDITY_MIN               0

/** @brief Maximum humidity (100% = 10000 in centi-percent) */
#define ZB_SCALE_HUMIDITY_MAX               10000

/* ============================================================================
 * Pressure Scaling
 * ============================================================================
 * Zigbee pressure is typically in units of 0.1 hPa (decipascals) or kPa.
 * Value 0xFFFF indicates invalid/unknown pressure.
 * ============================================================================ */

/** @brief Pressure divisor (deci-hPa to hPa) */
#define ZB_SCALE_PRESSURE_DIVISOR           10

/** @brief Pressure scale factor */
#define ZB_SCALE_PRESSURE_FACTOR            10

/** @brief kPa to hPa conversion factor */
#define ZB_SCALE_KPA_TO_HPA                 10

/** @brief Invalid pressure marker */
#define ZB_SCALE_PRESSURE_INVALID           0xFFFF

/* ============================================================================
 * Illuminance Scaling
 * ============================================================================
 * Zigbee illuminance uses a logarithmic scale: lux = 10^((value-1)/10000)
 * Value 0 = too dark to measure, 0xFFFF = invalid.
 * ============================================================================ */

/** @brief Illuminance log scale divisor */
#define ZB_SCALE_ILLUMINANCE_DIVISOR        10000

/** @brief Illuminance log scale offset */
#define ZB_SCALE_ILLUMINANCE_OFFSET         1

/** @brief Invalid illuminance marker */
#define ZB_SCALE_ILLUMINANCE_INVALID        0xFFFF

/** @brief Minimum illuminance (too dark) */
#define ZB_SCALE_ILLUMINANCE_MIN            0

/* ============================================================================
 * Color Scaling
 * ============================================================================
 * Zigbee color uses 16-bit values (0-65535) for x/y coordinates and hue/sat.
 * ============================================================================ */

/** @brief Color XY maximum value (16-bit) */
#define ZB_SCALE_COLOR_XY_MAX               65535

/** @brief Color XY divisor for normalization (0.0-1.0) */
#define ZB_SCALE_COLOR_XY_DIVISOR           65535

/** @brief Color XY divisor as float */
#define ZB_SCALE_COLOR_XY_DIVISOR_F         65535.0f

/** @brief Hue maximum value (0-254, mapped to 0-360 degrees) */
#define ZB_SCALE_HUE_MAX                    254

/** @brief Hue to degrees multiplier (360/254) */
#define ZB_SCALE_HUE_TO_DEGREES             1.417322835f

/** @brief Degrees to hue divisor */
#define ZB_SCALE_DEGREES_TO_HUE             1.417322835f

/** @brief Saturation maximum value (0-254) */
#define ZB_SCALE_SATURATION_MAX             254

/** @brief Saturation to percent multiplier (100/254) */
#define ZB_SCALE_SAT_TO_PERCENT             0.393700787f

/** @brief Color temperature mireds minimum (typically 153 = 6500K) */
#define ZB_SCALE_MIREDS_MIN                 153

/** @brief Color temperature mireds maximum (typically 500 = 2000K) */
#define ZB_SCALE_MIREDS_MAX                 500

/** @brief Kelvin to mireds conversion factor */
#define ZB_SCALE_KELVIN_TO_MIREDS_FACTOR    1000000

/* ============================================================================
 * Power and Energy Scaling
 * ============================================================================
 * Electrical measurement cluster uses various scales.
 * ============================================================================ */

/** @brief Power divisor (milliwatts to watts) */
#define ZB_SCALE_POWER_MW_TO_W              1000

/** @brief Voltage divisor (decivolts to volts) */
#define ZB_SCALE_VOLTAGE_DV_TO_V            10

/** @brief Current divisor (milliamps to amps) */
#define ZB_SCALE_CURRENT_MA_TO_A            1000

/** @brief Energy divisor (watt-hours, often x1000) */
#define ZB_SCALE_ENERGY_WH_DIVISOR          1000

/** @brief Power factor scale (0-100 or 0-1000) */
#define ZB_SCALE_POWER_FACTOR               100

/* ============================================================================
 * Battery Scaling
 * ============================================================================
 * Battery percentage remaining uses 0-200 scale (0.5% resolution).
 * Voltage is in units of 100mV.
 * ============================================================================ */

/** @brief Battery percentage divisor (0-200 to 0-100%) */
#define ZB_SCALE_BATTERY_PERCENT_DIVISOR    2

/** @brief Battery voltage divisor (100mV to V) */
#define ZB_SCALE_BATTERY_VOLTAGE_DIVISOR    10

/** @brief Invalid battery percentage */
#define ZB_SCALE_BATTERY_INVALID            0xFF

/** @brief Battery not used/reporting marker */
#define ZB_SCALE_BATTERY_NOT_USED           0x00

/* ============================================================================
 * Time Scaling
 * ============================================================================
 * Zigbee uses various time units for different purposes.
 * ============================================================================ */

/** @brief Transition time units (1/10 second) */
#define ZB_SCALE_TRANSITION_TIME_DIVISOR    10

/** @brief On-time units (1/10 second for some clusters) */
#define ZB_SCALE_ON_TIME_DIVISOR            10

/** @brief Seconds to Zigbee time epoch offset (Jan 1, 2000) */
#define ZB_SCALE_TIME_EPOCH_OFFSET          946684800

/* ============================================================================
 * Angle and Position Scaling
 * ============================================================================
 * Window covering and similar clusters use percentage or angle.
 * ============================================================================ */

/** @brief Window covering position max (0-100%) */
#define ZB_SCALE_POSITION_MAX               100

/** @brief Angle divisor (0.1 degree resolution) */
#define ZB_SCALE_ANGLE_DIVISOR              10

/* ============================================================================
 * Flow and Volume Scaling
 * ============================================================================
 * Metering cluster uses various units.
 * ============================================================================ */

/** @brief Flow rate divisor (typical: liters/hour) */
#define ZB_SCALE_FLOW_DIVISOR               10

/** @brief Volume divisor (typical: liters or m3) */
#define ZB_SCALE_VOLUME_DIVISOR             1000

#ifdef __cplusplus
}
#endif

#endif /* ZB_UNIT_SCALES_H */
