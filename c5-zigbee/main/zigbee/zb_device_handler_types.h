/**
 * @file zb_device_handler_types.h
 * @brief Zigbee Device Handler Type Definitions
 *
 * This header contains all type definitions (structs, enums, typedefs, and
 * type-related constants) for the Zigbee device handler module. It is included
 * by zb_device_handler.h and can also be included separately when only type
 * definitions are needed (for faster compilation).
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_DEVICE_HANDLER_TYPES_H
#define ZB_DEVICE_HANDLER_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Device Limits
 * ============================================================================ */

/**
 * @brief Maximum number of devices supported
 */
#ifndef CONFIG_MAX_ZIGBEE_DEVICES
#define ZB_MAX_DEVICES 50
#else
#define ZB_MAX_DEVICES CONFIG_MAX_ZIGBEE_DEVICES
#endif

/* ============================================================================
 * Device State Array Limits
 * ============================================================================ */

/** @brief Maximum window covering devices tracked */
#define ZB_STATE_MAX_WINDOW_COVERING        16

/** @brief Maximum door lock devices tracked */
#define ZB_STATE_MAX_DOOR_LOCK              16

/** @brief Maximum illuminance sensor devices tracked */
#define ZB_STATE_MAX_ILLUMINANCE            16

/** @brief Maximum pressure sensor devices tracked */
#define ZB_STATE_MAX_PRESSURE               16

/** @brief Maximum PM2.5 sensor devices tracked */
#define ZB_STATE_MAX_PM25                   16

/** @brief Maximum thermostat devices tracked */
#define ZB_STATE_MAX_THERMOSTAT             16

/** @brief Maximum fan control devices tracked */
#define ZB_STATE_MAX_FAN_CONTROL            16

/** @brief Maximum electrical measurement devices tracked */
#define ZB_STATE_MAX_ELECTRICAL             16

/** @brief Maximum IAS Zone (alarm) devices tracked */
#define ZB_STATE_MAX_IAS_ZONE               32

/** @brief Maximum metering (smart meter) devices tracked */
#define ZB_STATE_MAX_METERING               16

/** @brief Maximum binary output devices tracked */
#define ZB_STATE_MAX_BINARY_OUTPUT          16

/** @brief Maximum binary value devices tracked */
#define ZB_STATE_MAX_BINARY_VALUE           16

/** @brief Maximum multistate (input/output/value) devices tracked */
#define ZB_STATE_MAX_MULTISTATE             32

/* ============================================================================
 * String Length Constants
 * ============================================================================ */

/**
 * @brief Maximum friendly name length
 */
#define ZB_DEVICE_FRIENDLY_NAME_LEN 32

/**
 * @brief Maximum model name length
 */
#define ZB_DEVICE_MODEL_LEN 32

/**
 * @brief Maximum manufacturer name length
 */
#define ZB_DEVICE_MANUFACTURER_LEN 32

/* ============================================================================
 * Device Types
 * ============================================================================ */

/**
 * @brief Zigbee device type enumeration
 */
typedef enum {
    ZB_DEVICE_TYPE_UNKNOWN = 0,
    ZB_DEVICE_TYPE_ON_OFF_LIGHT,       /**< On/Off Light */
    ZB_DEVICE_TYPE_DIMMABLE_LIGHT,     /**< Dimmable Light */
    ZB_DEVICE_TYPE_COLOR_LIGHT,        /**< Color Light */
    ZB_DEVICE_TYPE_ON_OFF_SWITCH,      /**< On/Off Switch */
    ZB_DEVICE_TYPE_TEMP_SENSOR,        /**< Temperature Sensor */
    ZB_DEVICE_TYPE_HUMIDITY_SENSOR,    /**< Humidity Sensor */
    ZB_DEVICE_TYPE_MOTION_SENSOR,      /**< Motion Sensor */
    ZB_DEVICE_TYPE_DOOR_SENSOR,        /**< Door/Window Sensor */
    ZB_DEVICE_TYPE_PLUG,               /**< Smart Plug */
    ZB_DEVICE_TYPE_WINDOW_COVERING,    /**< Window Covering (Blinds/Shades) */
    ZB_DEVICE_TYPE_DOOR_LOCK,          /**< Door Lock */
    ZB_DEVICE_TYPE_THERMOSTAT,         /**< Thermostat/HVAC */
    ZB_DEVICE_TYPE_FAN,                /**< Fan Control */
    ZB_DEVICE_TYPE_DEHUMIDIFIER,       /**< Dehumidifier/HVAC */
    ZB_DEVICE_TYPE_ILLUMINANCE_SENSOR, /**< Illuminance Sensor */
    ZB_DEVICE_TYPE_PRESSURE_SENSOR,    /**< Pressure Sensor */
    ZB_DEVICE_TYPE_AIR_QUALITY_SENSOR, /**< Air Quality Sensor (PM2.5) */
    ZB_DEVICE_TYPE_ENERGY_METER,       /**< Energy Meter (kWh) */
    ZB_DEVICE_TYPE_GAS_METER,          /**< Gas Meter (m3) */
    ZB_DEVICE_TYPE_WATER_METER,        /**< Water Meter (L/gal) */
    ZB_DEVICE_TYPE_POWER_MONITOR,      /**< Power Monitor (Electrical Measurement) */
    ZB_DEVICE_TYPE_IAS_ZONE,           /**< IAS Zone (Generic Alarm Sensor) */
    ZB_DEVICE_TYPE_FIRE_SENSOR,        /**< Fire/Smoke Sensor (IAS Zone) */
    ZB_DEVICE_TYPE_WATER_LEAK_SENSOR,  /**< Water Leak Sensor (IAS Zone) */
    ZB_DEVICE_TYPE_GAS_SENSOR,         /**< Gas Sensor (IAS Zone) */
    ZB_DEVICE_TYPE_VIBRATION_SENSOR,   /**< Vibration/Movement Sensor (IAS Zone) */
    ZB_DEVICE_TYPE_OTHER               /**< Other Device Type */
} zb_device_type_t;

/* ============================================================================
 * Power Information
 * ============================================================================ */

/**
 * @brief Power source information structure
 */
typedef struct {
    uint8_t current_power_mode;         /**< Current power mode (0=sync, 1=periodic, 2=stimulated) */
    uint8_t available_power_sources;    /**< Available power sources bitmask */
    uint8_t current_power_source;       /**< Current power source bitmask */
    uint8_t current_power_source_level; /**< Power level (0=critical, 4=33%, 8=66%, 12=100%) */
    bool power_info_valid;              /**< True if power descriptor was successfully read */
} zb_power_info_t;

/* ============================================================================
 * Device Structure
 * ============================================================================ */

/**
 * @brief Zigbee device structure
 */
typedef struct {
    esp_zb_ieee_addr_t ieee_addr;                       /**< IEEE 64-bit address */
    uint16_t short_addr;                                /**< Network short address */
    uint8_t endpoint;                                   /**< Primary endpoint */
    char friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN];    /**< User-friendly name */
    char model[ZB_DEVICE_MODEL_LEN];                    /**< Model identifier */
    char manufacturer[ZB_DEVICE_MANUFACTURER_LEN];      /**< Manufacturer name */
    zb_device_type_t device_type;                       /**< Device type */
    bool online;                                        /**< Online status */
    time_t last_seen;                                   /**< Last activity timestamp */
    uint8_t link_quality;                               /**< Link quality (LQI) */
    uint8_t rssi;                                       /**< Signal strength */
    uint16_t cluster_count;                             /**< Number of supported clusters */
    uint16_t clusters[32];                              /**< Supported cluster IDs */
    zb_power_info_t power_info;                         /**< Power source information (API-007) */
} zb_device_t;

/* ============================================================================
 * Binary Output Cluster (0x0010) Types
 * ============================================================================ */

/**
 * @brief Binary Output Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_BINARY_OUTPUT             0x0010

/**
 * @brief Binary Output Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_BINARY_OUT_OF_SERVICE_ID        0x0051  /**< OutOfService (bool) */
#define ZB_ZCL_ATTR_BINARY_PRESENT_VALUE_ID         0x0055  /**< PresentValue (bool) */
#define ZB_ZCL_ATTR_BINARY_STATUS_FLAGS_ID          0x006F  /**< StatusFlags (bitmap8) */

/**
 * @brief Binary Status Flags bits (bitmap8)
 */
#define ZB_BINARY_STATUS_IN_ALARM       (1 << 0)   /**< Bit 0: In Alarm */
#define ZB_BINARY_STATUS_FAULT          (1 << 1)   /**< Bit 1: Fault */
#define ZB_BINARY_STATUS_OVERRIDDEN     (1 << 2)   /**< Bit 2: Overridden */
#define ZB_BINARY_STATUS_OUT_OF_SERVICE (1 << 3)   /**< Bit 3: Out of Service */

/**
 * @brief Binary Output state structure
 */
typedef struct {
    bool present_value;         /**< Current output state (ON/OFF) */
    bool out_of_service;        /**< Device out of service */
    uint8_t status_flags;       /**< Raw status flags bitmap */
    bool in_alarm;              /**< In alarm state */
    bool fault;                 /**< Fault detected */
    bool overridden;            /**< Overridden by local control */
} zb_binary_output_state_t;

/**
 * @brief Binary Output state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current binary output state
 */
typedef void (*zb_binary_output_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                             const zb_binary_output_state_t *state);

/* ============================================================================
 * Binary Value Cluster (0x0011) Types
 * ============================================================================ */

/**
 * @brief Binary Value Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_BINARY_VALUE              0x0011

/**
 * @brief Binary Value state structure
 *
 * Uses same attribute IDs as Binary Output (0x0051, 0x0055, 0x006F)
 */
typedef struct {
    bool present_value;         /**< Current value (ON/OFF) */
    bool out_of_service;        /**< Value out of service */
    uint8_t status_flags;       /**< Raw status flags bitmap */
    bool in_alarm;              /**< In alarm state */
    bool fault;                 /**< Fault detected */
    bool overridden;            /**< Overridden by local control */
} zb_binary_value_state_t;

/**
 * @brief Binary Value state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current binary value state
 */
typedef void (*zb_binary_value_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                            const zb_binary_value_state_t *state);

/* ============================================================================
 * Window Covering Cluster (0x0102) Types
 * ============================================================================ */

/**
 * @brief Window Covering Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_WINDOW_COVERING           0x0102

/**
 * @brief Window Covering Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ID                     0x0000
#define ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID            0x0007
#define ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_LIFT_PERCENT_ID 0x0008
#define ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POS_TILT_PERCENT_ID 0x0009
#define ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_LIFT   0x0010
#define ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_LIFT 0x0011
#define ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID                     0x0017

/**
 * @brief Window Covering Cluster Command IDs
 */
#define ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN              0x00
#define ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE           0x01
#define ZB_ZCL_CMD_WINDOW_COVERING_STOP                 0x02
#define ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE     0x04
#define ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENT   0x05
#define ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE     0x07
#define ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENT   0x08

/**
 * @brief Window Covering state structure
 */
typedef struct {
    uint8_t current_position_lift;  /**< Current lift position (0-100%) */
    uint8_t current_position_tilt;  /**< Current tilt position (0-100%) */
    uint8_t covering_type;          /**< Covering type (roller, venetian, etc.) */
    bool is_moving;                 /**< True if cover is currently moving */
} zb_window_covering_state_t;

/**
 * @brief Window Covering state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current window covering state
 */
typedef void (*zb_window_covering_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                               const zb_window_covering_state_t *state);

/* ============================================================================
 * Door Lock Cluster (0x0101) Types
 * ============================================================================ */

/**
 * @brief Door Lock Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_DOOR_LOCK                 0x0101

/**
 * @brief Door Lock Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID         0x0000
#define ZB_ZCL_ATTR_DOOR_LOCK_LOCK_TYPE_ID          0x0001
#define ZB_ZCL_ATTR_DOOR_LOCK_ACTUATOR_ENABLED_ID   0x0002

/**
 * @brief Door Lock Cluster Command IDs
 */
#define ZB_ZCL_CMD_DOOR_LOCK_LOCK_DOOR_ID           0x00
#define ZB_ZCL_CMD_DOOR_LOCK_UNLOCK_DOOR_ID         0x01

/**
 * @brief Door Lock State values (enum8)
 */
typedef enum {
    ZB_DOOR_LOCK_STATE_NOT_FULLY_LOCKED = 0,  /**< Not fully locked */
    ZB_DOOR_LOCK_STATE_LOCKED = 1,            /**< Locked */
    ZB_DOOR_LOCK_STATE_UNLOCKED = 2           /**< Unlocked */
} zb_door_lock_state_t;

/**
 * @brief Door Lock state structure
 */
typedef struct {
    zb_door_lock_state_t lock_state;    /**< Current lock state */
    uint8_t lock_type;                   /**< Lock type */
    bool actuator_enabled;               /**< Actuator enabled status */
} zb_door_lock_state_struct_t;

/**
 * @brief Door Lock state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current door lock state
 */
typedef void (*zb_door_lock_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                         const zb_door_lock_state_struct_t *state);

/* ============================================================================
 * Thermostat Cluster (0x0201) Types
 * ============================================================================ */

/**
 * @brief Thermostat Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_THERMOSTAT                    0x0201

/**
 * @brief Thermostat Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID             0x0000  /**< Local temperature (int16, 0.01C units) */
#define ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID           0x0001  /**< Outdoor temperature (int16, 0.01C units) */
#define ZB_ZCL_ATTR_THERMOSTAT_OCCUPANCY_ID                     0x0002  /**< Occupancy (bitmap8) */
#define ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_ID         0x0003  /**< Abs min heat setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_ID         0x0004  /**< Abs max heat setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_ID         0x0005  /**< Abs min cool setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_ID         0x0006  /**< Abs max cool setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_PI_COOLING_DEMAND_ID             0x0007  /**< PI cooling demand */
#define ZB_ZCL_ATTR_THERMOSTAT_PI_HEATING_DEMAND_ID             0x0008  /**< PI heating demand */
#define ZB_ZCL_ATTR_THERMOSTAT_HVAC_SYSTEM_TYPE_ID              0x0009  /**< HVAC system type config */
#define ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMP_CALIBRATION_ID        0x0010  /**< Local temperature calibration */
#define ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID     0x0011  /**< Occupied cooling setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID     0x0012  /**< Occupied heating setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_COOLING_SETPOINT_ID   0x0013  /**< Unoccupied cooling setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_UNOCCUPIED_HEATING_SETPOINT_ID   0x0014  /**< Unoccupied heating setpoint */
#define ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID       0x0015  /**< Min heat setpoint limit */
#define ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID       0x0016  /**< Max heat setpoint limit */
#define ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID       0x0017  /**< Min cool setpoint limit */
#define ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID       0x0018  /**< Max cool setpoint limit */
#define ZB_ZCL_ATTR_THERMOSTAT_MIN_SETPOINT_DEAD_BAND_ID        0x0019  /**< Min setpoint dead band */
#define ZB_ZCL_ATTR_THERMOSTAT_REMOTE_SENSING_ID                0x001A  /**< Remote sensing */
#define ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID 0x001B  /**< Control sequence of operation */
#define ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID                   0x001C  /**< System mode */
#define ZB_ZCL_ATTR_THERMOSTAT_ALARM_MASK_ID                    0x001D  /**< Alarm mask */
#define ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID                  0x001E  /**< Thermostat running mode */

/**
 * @brief Thermostat Cluster Command IDs
 */
#define ZB_ZCL_CMD_THERMOSTAT_SETPOINT_RAISE_LOWER_ID           0x00    /**< Setpoint raise/lower */
#define ZB_ZCL_CMD_THERMOSTAT_SET_WEEKLY_SCHEDULE_ID            0x01    /**< Set weekly schedule */
#define ZB_ZCL_CMD_THERMOSTAT_GET_WEEKLY_SCHEDULE_ID            0x02    /**< Get weekly schedule */
#define ZB_ZCL_CMD_THERMOSTAT_CLEAR_WEEKLY_SCHEDULE_ID          0x03    /**< Clear weekly schedule */
#define ZB_ZCL_CMD_THERMOSTAT_GET_RELAY_STATUS_LOG_ID           0x04    /**< Get relay status log */

/**
 * @brief Thermostat System Mode values (enum8)
 */
typedef enum {
    ZB_THERMOSTAT_SYSTEM_MODE_OFF = 0x00,           /**< Off */
    ZB_THERMOSTAT_SYSTEM_MODE_AUTO = 0x01,          /**< Auto */
    ZB_THERMOSTAT_SYSTEM_MODE_COOL = 0x03,          /**< Cool */
    ZB_THERMOSTAT_SYSTEM_MODE_HEAT = 0x04,          /**< Heat */
    ZB_THERMOSTAT_SYSTEM_MODE_EMERGENCY_HEAT = 0x05, /**< Emergency heating */
    ZB_THERMOSTAT_SYSTEM_MODE_PRECOOLING = 0x06,    /**< Precooling */
    ZB_THERMOSTAT_SYSTEM_MODE_FAN_ONLY = 0x07,      /**< Fan only */
    ZB_THERMOSTAT_SYSTEM_MODE_DRY = 0x08,           /**< Dry */
    ZB_THERMOSTAT_SYSTEM_MODE_SLEEP = 0x09          /**< Sleep */
} zb_thermostat_system_mode_t;

/**
 * @brief Thermostat Running Mode values (enum8)
 */
typedef enum {
    ZB_THERMOSTAT_RUNNING_MODE_OFF = 0x00,          /**< Off */
    ZB_THERMOSTAT_RUNNING_MODE_COOL = 0x03,         /**< Cool */
    ZB_THERMOSTAT_RUNNING_MODE_HEAT = 0x04          /**< Heat */
} zb_thermostat_running_mode_t;

/**
 * @brief Thermostat state structure
 */
typedef struct {
    int16_t local_temperature;          /**< Local temperature (0.01C units) */
    int16_t occupied_cooling_setpoint;  /**< Occupied cooling setpoint (0.01C units) */
    int16_t occupied_heating_setpoint;  /**< Occupied heating setpoint (0.01C units) */
    int16_t min_heat_setpoint_limit;    /**< Min heat setpoint limit (0.01C units) */
    int16_t max_heat_setpoint_limit;    /**< Max heat setpoint limit (0.01C units) */
    int16_t min_cool_setpoint_limit;    /**< Min cool setpoint limit (0.01C units) */
    int16_t max_cool_setpoint_limit;    /**< Max cool setpoint limit (0.01C units) */
    zb_thermostat_system_mode_t system_mode;   /**< System mode */
    zb_thermostat_running_mode_t running_mode; /**< Running mode */
    uint8_t pi_heating_demand;          /**< PI heating demand (0-100%) */
    uint8_t pi_cooling_demand;          /**< PI cooling demand (0-100%) */
} zb_thermostat_state_t;

/**
 * @brief Thermostat state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current thermostat state
 */
typedef void (*zb_thermostat_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                          const zb_thermostat_state_t *state);

/* ============================================================================
 * Fan Control Cluster (0x0202) Types
 * ============================================================================ */

/**
 * @brief Fan Control Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_FAN_CONTROL                   0x0202

/**
 * @brief Fan Control Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID             0x0000  /**< Fan mode (enum8) */
#define ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID    0x0001  /**< Fan mode sequence (enum8) */

/**
 * @brief Fan Mode values (enum8)
 */
typedef enum {
    ZB_FAN_MODE_OFF = 0x00,         /**< Off */
    ZB_FAN_MODE_LOW = 0x01,         /**< Low */
    ZB_FAN_MODE_MEDIUM = 0x02,      /**< Medium */
    ZB_FAN_MODE_HIGH = 0x03,        /**< High */
    ZB_FAN_MODE_ON = 0x04,          /**< On */
    ZB_FAN_MODE_AUTO = 0x05,        /**< Auto */
    ZB_FAN_MODE_SMART = 0x06        /**< Smart */
} zb_fan_mode_t;

/**
 * @brief Fan Mode Sequence values (enum8)
 * Defines which fan modes are supported
 */
typedef enum {
    ZB_FAN_MODE_SEQ_LOW_MED_HIGH = 0x00,      /**< Low/Med/High */
    ZB_FAN_MODE_SEQ_LOW_HIGH = 0x01,          /**< Low/High */
    ZB_FAN_MODE_SEQ_LOW_MED_HIGH_AUTO = 0x02, /**< Low/Med/High/Auto */
    ZB_FAN_MODE_SEQ_LOW_HIGH_AUTO = 0x03,     /**< Low/High/Auto */
    ZB_FAN_MODE_SEQ_ON_AUTO = 0x04            /**< On/Auto */
} zb_fan_mode_sequence_t;

/**
 * @brief Fan Control state structure
 */
typedef struct {
    zb_fan_mode_t fan_mode;                 /**< Current fan mode */
    zb_fan_mode_sequence_t fan_mode_sequence; /**< Supported fan mode sequence */
} zb_fan_control_state_t;

/**
 * @brief Fan Control state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current fan control state
 */
typedef void (*zb_fan_control_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                           const zb_fan_control_state_t *state);

/* ============================================================================
 * Illuminance Measurement Cluster (0x0400) Types
 * ============================================================================ */

/**
 * @brief Illuminance Measurement Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT   0x0400

/**
 * @brief Illuminance Measurement Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_ILLUMINANCE_MEASURED_VALUE_ID   0x0000  /**< MeasuredValue (uint16) */
#define ZB_ZCL_ATTR_ILLUMINANCE_MIN_MEASURED_ID     0x0001  /**< MinMeasuredValue (uint16) */
#define ZB_ZCL_ATTR_ILLUMINANCE_MAX_MEASURED_ID     0x0002  /**< MaxMeasuredValue (uint16) */
#define ZB_ZCL_ATTR_ILLUMINANCE_TOLERANCE_ID        0x0003  /**< Tolerance (uint16) */
#define ZB_ZCL_ATTR_ILLUMINANCE_LIGHT_SENSOR_TYPE_ID 0x0004 /**< LightSensorType (enum8) */

/**
 * @brief Illuminance measurement invalid value
 *
 * Value 0x0000 indicates that illuminance is too low to be measured.
 * Value 0xFFFF indicates that illuminance is invalid/not configured.
 */
#define ZB_ZCL_ILLUMINANCE_MEASURED_VALUE_INVALID   0xFFFF
#define ZB_ZCL_ILLUMINANCE_MEASURED_VALUE_TOO_LOW   0x0000

/**
 * @brief Illuminance measurement state structure
 */
typedef struct {
    uint16_t measured_value;    /**< Measured illuminance value (log10(lux) * 10000 + 1) */
    uint16_t min_measured;      /**< Minimum measurable value */
    uint16_t max_measured;      /**< Maximum measurable value */
    uint16_t tolerance;         /**< Tolerance (optional) */
    uint8_t light_sensor_type;  /**< Light sensor type (0=photodiode, 1=CMOS) */
} zb_illuminance_state_t;

/**
 * @brief Illuminance state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current illuminance measurement state
 */
typedef void (*zb_illuminance_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                          const zb_illuminance_state_t *state);

/* ============================================================================
 * Pressure Measurement Cluster (0x0403) Types
 * ============================================================================ */

/**
 * @brief Pressure Measurement Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT      0x0403

/**
 * @brief Pressure Measurement Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_PRESSURE_MEASURED_VALUE_ID      0x0000  /**< MeasuredValue (int16) in 10 Pa units */
#define ZB_ZCL_ATTR_PRESSURE_MIN_MEASURED_ID        0x0001  /**< MinMeasuredValue (int16) */
#define ZB_ZCL_ATTR_PRESSURE_MAX_MEASURED_ID        0x0002  /**< MaxMeasuredValue (int16) */
#define ZB_ZCL_ATTR_PRESSURE_TOLERANCE_ID           0x0003  /**< Tolerance (uint16) */
#define ZB_ZCL_ATTR_PRESSURE_SCALED_VALUE_ID        0x0010  /**< ScaledValue (int16) */
#define ZB_ZCL_ATTR_PRESSURE_MIN_SCALED_VALUE_ID    0x0011  /**< MinScaledValue (int16) */
#define ZB_ZCL_ATTR_PRESSURE_MAX_SCALED_VALUE_ID    0x0012  /**< MaxScaledValue (int16) */
#define ZB_ZCL_ATTR_PRESSURE_SCALED_TOLERANCE_ID    0x0013  /**< ScaledTolerance (uint16) */
#define ZB_ZCL_ATTR_PRESSURE_SCALE_ID               0x0014  /**< Scale (int8) - 10^Scale multiplier */

/**
 * @brief Pressure measurement invalid value
 * @note 0x8000 in int16_t representation is -32768
 */
#define ZB_ZCL_PRESSURE_MEASURED_VALUE_INVALID      ((int16_t)0x8000)

/**
 * @brief Pressure measurement state structure
 */
typedef struct {
    int16_t measured_value;     /**< Measured pressure in 10 Pa (1 kPa) units */
    int16_t min_measured;       /**< Minimum measurable value */
    int16_t max_measured;       /**< Maximum measurable value */
    uint16_t tolerance;         /**< Tolerance (optional) */
    int16_t scaled_value;       /**< Scaled value (optional, higher resolution) */
    int16_t min_scaled_value;   /**< Minimum scaled value */
    int16_t max_scaled_value;   /**< Maximum scaled value */
    uint16_t scaled_tolerance;  /**< Scaled tolerance */
    int8_t scale;               /**< Scale factor (10^scale) */
    bool has_scaled;            /**< True if scaled attributes are valid */
} zb_pressure_state_t;

/**
 * @brief Pressure state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current pressure measurement state
 */
typedef void (*zb_pressure_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                        const zb_pressure_state_t *state);

/* ============================================================================
 * PM2.5 Measurement Cluster (0x042A) Types
 * ============================================================================ */

/**
 * @brief PM2.5 Measurement Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_PM25_MEASUREMENT          0x042A

/**
 * @brief PM2.5 Measurement Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_PM25_MEASURED_VALUE_ID          0x0000  /**< MeasuredValue (float/single) */
#define ZB_ZCL_ATTR_PM25_MIN_MEASURED_ID            0x0001  /**< MinMeasuredValue (float/single) */
#define ZB_ZCL_ATTR_PM25_MAX_MEASURED_ID            0x0002  /**< MaxMeasuredValue (float/single) */
#define ZB_ZCL_ATTR_PM25_TOLERANCE_ID               0x0003  /**< Tolerance (float/single) */

/**
 * @brief PM2.5 measurement invalid value (NaN)
 */
#define ZB_ZCL_PM25_MEASURED_VALUE_INVALID          0x7FC00000  /**< IEEE 754 quiet NaN */

/**
 * @brief PM2.5 measurement state structure
 */
typedef struct {
    float measured_value;   /**< Measured PM2.5 concentration in ug/m3 */
    float min_measured;     /**< Minimum measurable value */
    float max_measured;     /**< Maximum measurable value */
    float tolerance;        /**< Measurement tolerance */
    bool is_valid;          /**< True if measurement is valid (not NaN) */
} zb_pm25_state_t;

/**
 * @brief PM2.5 state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current PM2.5 measurement state
 */
typedef void (*zb_pm25_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                    const zb_pm25_state_t *state);

/* ============================================================================
 * Electrical Measurement Cluster (0x0B04) Types
 * ============================================================================ */

/**
 * @brief Electrical Measurement Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT    0x0B04

/**
 * @brief Electrical Measurement Cluster Attribute IDs
 */
/* Basic Information */
#define ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_TYPE_ID              0x0000  /**< MeasurementType (bitmap32) */

/* AC (Non-phase specific) */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_FREQUENCY_ID                  0x0300  /**< AC Frequency (uint16, Hz * 10) */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_FREQUENCY_MIN_ID              0x0301  /**< AC Frequency Min */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_FREQUENCY_MAX_ID              0x0302  /**< AC Frequency Max */

/* AC Single Phase or Phase A */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_VOLTAGE_ID                   0x0505  /**< RMS Voltage (uint16, V * 10) */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_VOLTAGE_MIN_ID               0x0506  /**< RMS Voltage Min */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_VOLTAGE_MAX_ID               0x0507  /**< RMS Voltage Max */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_CURRENT_ID                   0x0508  /**< RMS Current (uint16, A * 1000 = mA) */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_CURRENT_MIN_ID               0x0509  /**< RMS Current Min */
#define ZB_ZCL_ATTR_ELECTRICAL_RMS_CURRENT_MAX_ID               0x050A  /**< RMS Current Max */
#define ZB_ZCL_ATTR_ELECTRICAL_ACTIVE_POWER_ID                  0x050B  /**< Active Power (int16, W * 10) */
#define ZB_ZCL_ATTR_ELECTRICAL_ACTIVE_POWER_MIN_ID              0x050C  /**< Active Power Min */
#define ZB_ZCL_ATTR_ELECTRICAL_ACTIVE_POWER_MAX_ID              0x050D  /**< Active Power Max */
#define ZB_ZCL_ATTR_ELECTRICAL_REACTIVE_POWER_ID                0x050E  /**< Reactive Power (int16, VAR * 10) */
#define ZB_ZCL_ATTR_ELECTRICAL_APPARENT_POWER_ID                0x050F  /**< Apparent Power (uint16, VA * 10) */
#define ZB_ZCL_ATTR_ELECTRICAL_POWER_FACTOR_ID                  0x0510  /**< Power Factor (int8, -100 to 100) */

/* Divisors/Multipliers for correct scaling */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_MULTIPLIER_ID         0x0600  /**< AC Voltage Multiplier */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_VOLTAGE_DIVISOR_ID            0x0601  /**< AC Voltage Divisor */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_MULTIPLIER_ID         0x0602  /**< AC Current Multiplier */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_CURRENT_DIVISOR_ID            0x0603  /**< AC Current Divisor */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_MULTIPLIER_ID           0x0604  /**< AC Power Multiplier */
#define ZB_ZCL_ATTR_ELECTRICAL_AC_POWER_DIVISOR_ID              0x0605  /**< AC Power Divisor */

/**
 * @brief Electrical Measurement state structure
 *
 * Contains raw values from the device. Use helper functions to get
 * properly scaled values in standard units.
 */
typedef struct {
    /* Raw measurement values */
    uint16_t rms_voltage;           /**< RMS Voltage raw value (V * 10 typically) */
    uint16_t rms_current;           /**< RMS Current raw value (mA typically) */
    int16_t active_power;           /**< Active Power raw value (W * 10 typically) */
    int16_t reactive_power;         /**< Reactive Power raw value (VAR * 10 typically) */
    uint16_t apparent_power;        /**< Apparent Power raw value (VA * 10 typically) */
    int8_t power_factor;            /**< Power Factor (-100 to 100, percentage) */
    uint16_t ac_frequency;          /**< AC Frequency raw value (Hz * 10 typically) */

    /* Scaling factors (read once during device interview) */
    uint16_t voltage_multiplier;    /**< Voltage multiplier (default 1) */
    uint16_t voltage_divisor;       /**< Voltage divisor (default 10) */
    uint16_t current_multiplier;    /**< Current multiplier (default 1) */
    uint16_t current_divisor;       /**< Current divisor (default 1000) */
    uint16_t power_multiplier;      /**< Power multiplier (default 1) */
    uint16_t power_divisor;         /**< Power divisor (default 10) */

    /* Status flags */
    bool scaling_factors_read;      /**< True if scaling factors have been read */
    bool is_valid;                  /**< True if measurements are valid */
} zb_electrical_state_t;

/**
 * @brief Electrical Measurement state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current electrical measurement state
 */
typedef void (*zb_electrical_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                          const zb_electrical_state_t *state);

/* ============================================================================
 * Metering Cluster (0x0702) Types - Smart Energy
 * ============================================================================ */

/**
 * @brief Metering Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_METERING                      0x0702

/**
 * @brief Metering Cluster Attribute IDs - Reading Information Set
 */
#define ZB_ZCL_ATTR_METERING_CURRENT_SUMM_DELIVERED_ID      0x0000  /**< CurrentSummationDelivered (uint48) */
#define ZB_ZCL_ATTR_METERING_CURRENT_SUMM_RECEIVED_ID       0x0001  /**< CurrentSummationReceived (uint48) */
#define ZB_ZCL_ATTR_METERING_CURRENT_MAX_DEMAND_DELIV_ID    0x0002  /**< CurrentMaxDemandDelivered (uint48) */
#define ZB_ZCL_ATTR_METERING_CURRENT_MAX_DEMAND_RECV_ID     0x0003  /**< CurrentMaxDemandReceived (uint48) */

/**
 * @brief Metering Cluster Attribute IDs - TOU (Time of Use) Information
 */
#define ZB_ZCL_ATTR_METERING_CURRENT_TIER1_SUMM_DELIV_ID    0x0100  /**< CurrentTier1SummationDelivered (uint48) */
#define ZB_ZCL_ATTR_METERING_CURRENT_TIER2_SUMM_DELIV_ID    0x0102  /**< CurrentTier2SummationDelivered (uint48) */

/**
 * @brief Metering Cluster Attribute IDs - Meter Status
 */
#define ZB_ZCL_ATTR_METERING_STATUS_ID                      0x0200  /**< Status (bitmap8) */
#define ZB_ZCL_ATTR_METERING_REMAINING_BATTERY_LIFE_ID      0x0201  /**< RemainingBatteryLife (uint8, %) */
#define ZB_ZCL_ATTR_METERING_HOURS_IN_OPERATION_ID          0x0202  /**< HoursInOperation (uint24) */

/**
 * @brief Metering Cluster Attribute IDs - Formatting
 */
#define ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID             0x0300  /**< UnitOfMeasure (enum8) */
#define ZB_ZCL_ATTR_METERING_MULTIPLIER_ID                  0x0301  /**< Multiplier (uint24) */
#define ZB_ZCL_ATTR_METERING_DIVISOR_ID                     0x0302  /**< Divisor (uint24) */
#define ZB_ZCL_ATTR_METERING_SUMMATION_FORMATTING_ID        0x0303  /**< SummationFormatting (bitmap8) */
#define ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID           0x0304  /**< DemandFormatting (bitmap8) */
#define ZB_ZCL_ATTR_METERING_METERING_DEVICE_TYPE_ID        0x0306  /**< MeteringDeviceType (bitmap8) */

/**
 * @brief Metering Cluster Attribute IDs - Historical Consumption
 */
#define ZB_ZCL_ATTR_METERING_INSTANTANEOUS_DEMAND_ID        0x0400  /**< InstantaneousDemand (int24) - current power */
#define ZB_ZCL_ATTR_METERING_CURRENT_DAY_CONSUMPTION_ID     0x0401  /**< CurrentDayConsumption (uint24) */
#define ZB_ZCL_ATTR_METERING_PREVIOUS_DAY_CONSUMPTION_ID    0x0403  /**< PreviousDayConsumption (uint24) */

/**
 * @brief Metering Unit of Measure values (enum8)
 */
typedef enum {
    ZB_METERING_UNIT_KWH = 0x00,            /**< kWh (Kilowatt Hours) - Electricity */
    ZB_METERING_UNIT_CUBIC_METERS = 0x01,   /**< m3 (Cubic Meters) - Gas */
    ZB_METERING_UNIT_CUBIC_FEET = 0x02,     /**< ft3 (Cubic Feet) - Gas */
    ZB_METERING_UNIT_CCF = 0x03,            /**< ccf (100 Cubic Feet) */
    ZB_METERING_UNIT_US_GAL = 0x04,         /**< US gal (US Gallons) - Water */
    ZB_METERING_UNIT_IMP_GAL = 0x05,        /**< IMP gal (Imperial Gallons) - Water */
    ZB_METERING_UNIT_BTU = 0x06,            /**< BTU (British Thermal Units) */
    ZB_METERING_UNIT_LITERS = 0x07,         /**< L (Liters) - Water */
    ZB_METERING_UNIT_KPA_GAUGE = 0x08,      /**< kPA (Gauge) */
    ZB_METERING_UNIT_KPA_ABSOLUTE = 0x09,   /**< kPA (Absolute) */
    ZB_METERING_UNIT_MCF = 0x0A,            /**< mcf (1000 Cubic Feet) */
    ZB_METERING_UNIT_UNITLESS = 0x0B,       /**< Unitless */
    ZB_METERING_UNIT_MJ = 0x0C,             /**< MJ (Mega Joules) */
    ZB_METERING_UNIT_KVAR = 0x0D,           /**< kVAr (Kilo Volt Ampere Reactive) */
} zb_metering_unit_t;

/**
 * @brief Metering Device Type values (bitmap8)
 */
typedef enum {
    ZB_METERING_DEVICE_ELECTRIC = 0x00,     /**< Electric Metering */
    ZB_METERING_DEVICE_GAS = 0x01,          /**< Gas Metering */
    ZB_METERING_DEVICE_WATER = 0x02,        /**< Water Metering */
    ZB_METERING_DEVICE_THERMAL = 0x03,      /**< Thermal Metering (heat) */
    ZB_METERING_DEVICE_PRESSURE = 0x04,     /**< Pressure Metering */
    ZB_METERING_DEVICE_HEAT = 0x05,         /**< Heat Metering */
    ZB_METERING_DEVICE_COOLING = 0x06,      /**< Cooling Metering */
} zb_metering_device_type_t;

/**
 * @brief Metering state structure
 *
 * Stores raw values from the meter. Use helper functions to get formatted values.
 */
typedef struct {
    uint64_t current_summation_delivered;   /**< Total consumption delivered (raw uint48) */
    uint64_t current_summation_received;    /**< Total energy fed back (raw uint48) */
    int32_t instantaneous_demand;           /**< Current power/demand (raw int24) */
    uint32_t current_day_consumption;       /**< Today's consumption (raw uint24) */
    uint32_t previous_day_consumption;      /**< Yesterday's consumption (raw uint24) */
    uint8_t unit_of_measure;                /**< Unit of measure (enum8) */
    uint32_t multiplier;                    /**< Multiplier for formatting (uint24) */
    uint32_t divisor;                       /**< Divisor for formatting (uint24) */
    uint8_t summation_formatting;           /**< Summation formatting bits (bitmap8) */
    uint8_t demand_formatting;              /**< Demand formatting bits (bitmap8) */
    uint8_t metering_device_type;           /**< Device type (electric/gas/water) */
    uint8_t status;                         /**< Meter status (bitmap8) */
    uint8_t battery_percentage;             /**< Remaining battery life (0-100%) */
} zb_metering_state_t;

/**
 * @brief Metering state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current metering state
 */
typedef void (*zb_metering_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                        const zb_metering_state_t *state);

/* ============================================================================
 * IAS Zone Cluster (0x0500) Types
 * ============================================================================ */

/**
 * @brief IAS Zone Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_IAS_ZONE                  0x0500

/**
 * @brief IAS Zone Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_IAS_ZONE_STATE_ID               0x0000  /**< ZoneState (enum8) */
#define ZB_ZCL_ATTR_IAS_ZONE_TYPE_ID                0x0001  /**< ZoneType (enum16) */
#define ZB_ZCL_ATTR_IAS_ZONE_STATUS_ID              0x0002  /**< ZoneStatus (bitmap16) */
#define ZB_ZCL_ATTR_IAS_ZONE_CIE_ADDRESS_ID         0x0010  /**< IAS_CIE_Address (IEEE) */
#define ZB_ZCL_ATTR_IAS_ZONE_ID_ID                  0x0011  /**< ZoneID (uint8) */

/**
 * @brief IAS Zone Cluster Command IDs (Server to Client)
 */
#define ZB_ZCL_CMD_IAS_ZONE_STATUS_CHANGE_NOTIFICATION_ID   0x00  /**< Zone Status Change Notification */
#define ZB_ZCL_CMD_IAS_ZONE_ENROLL_REQUEST_ID               0x01  /**< Zone Enroll Request */

/**
 * @brief IAS Zone Cluster Command IDs (Client to Server)
 */
#define ZB_ZCL_CMD_IAS_ZONE_ENROLL_RESPONSE_ID              0x00  /**< Zone Enroll Response */
#define ZB_ZCL_CMD_IAS_ZONE_INIT_NORMAL_OP_MODE_ID          0x01  /**< Initiate Normal Operation Mode */
#define ZB_ZCL_CMD_IAS_ZONE_INIT_TEST_MODE_ID               0x02  /**< Initiate Test Mode */

/**
 * @brief IAS Zone State values (enum8)
 */
typedef enum {
    ZB_IAS_ZONE_STATE_NOT_ENROLLED = 0x00,  /**< Not enrolled */
    ZB_IAS_ZONE_STATE_ENROLLED = 0x01       /**< Enrolled */
} zb_ias_zone_state_t;

/**
 * @brief IAS Zone Type values (enum16)
 */
typedef enum {
    ZB_IAS_ZONE_TYPE_STANDARD_CIE       = 0x0000,  /**< Standard CIE */
    ZB_IAS_ZONE_TYPE_MOTION_SENSOR      = 0x000D,  /**< Motion Sensor */
    ZB_IAS_ZONE_TYPE_CONTACT_SWITCH     = 0x0015,  /**< Contact Switch (Door/Window) */
    ZB_IAS_ZONE_TYPE_FIRE_SENSOR        = 0x0028,  /**< Fire Sensor */
    ZB_IAS_ZONE_TYPE_WATER_SENSOR       = 0x002A,  /**< Water Sensor */
    ZB_IAS_ZONE_TYPE_GAS_SENSOR         = 0x002B,  /**< Gas Sensor */
    ZB_IAS_ZONE_TYPE_PERSONAL_EMERGENCY = 0x002C,  /**< Personal Emergency Device */
    ZB_IAS_ZONE_TYPE_VIBRATION_SENSOR   = 0x002D,  /**< Vibration/Movement Sensor */
    ZB_IAS_ZONE_TYPE_REMOTE_CONTROL     = 0x010F,  /**< Remote Control */
    ZB_IAS_ZONE_TYPE_KEY_FOB            = 0x0115,  /**< Key Fob */
    ZB_IAS_ZONE_TYPE_KEYPAD             = 0x021D,  /**< Keypad */
    ZB_IAS_ZONE_TYPE_STANDARD_WARNING   = 0x0225,  /**< Standard Warning Device */
    ZB_IAS_ZONE_TYPE_GLASS_BREAK_SENSOR = 0x0226,  /**< Glass Break Sensor */
    ZB_IAS_ZONE_TYPE_CARBON_MONOXIDE    = 0x0227,  /**< Carbon Monoxide Sensor */
    ZB_IAS_ZONE_TYPE_SECURITY_REPEATER  = 0x0229,  /**< Security Repeater */
    ZB_IAS_ZONE_TYPE_INVALID            = 0xFFFF   /**< Invalid Zone Type */
} zb_ias_zone_type_t;

/**
 * @brief IAS Zone Status bits (bitmap16)
 */
#define ZB_IAS_ZONE_STATUS_ALARM1           (1 << 0)   /**< Bit 0: Alarm1 (primary alarm) */
#define ZB_IAS_ZONE_STATUS_ALARM2           (1 << 1)   /**< Bit 1: Alarm2 (secondary alarm) */
#define ZB_IAS_ZONE_STATUS_TAMPER           (1 << 2)   /**< Bit 2: Tamper */
#define ZB_IAS_ZONE_STATUS_BATTERY_LOW      (1 << 3)   /**< Bit 3: Battery Low */
#define ZB_IAS_ZONE_STATUS_SUPERVISION      (1 << 4)   /**< Bit 4: Supervision Reports */
#define ZB_IAS_ZONE_STATUS_RESTORE_REPORTS  (1 << 5)   /**< Bit 5: Restore Reports */
#define ZB_IAS_ZONE_STATUS_TROUBLE          (1 << 6)   /**< Bit 6: Trouble */
#define ZB_IAS_ZONE_STATUS_AC_MAINS_FAULT   (1 << 7)   /**< Bit 7: AC Mains Fault */
#define ZB_IAS_ZONE_STATUS_TEST             (1 << 8)   /**< Bit 8: Test */
#define ZB_IAS_ZONE_STATUS_BATTERY_DEFECT   (1 << 9)   /**< Bit 9: Battery Defect */

/**
 * @brief IAS Zone Enroll Response codes
 */
typedef enum {
    ZB_IAS_ZONE_ENROLL_SUCCESS          = 0x00,  /**< Success */
    ZB_IAS_ZONE_ENROLL_NOT_SUPPORTED    = 0x01,  /**< Not Supported */
    ZB_IAS_ZONE_ENROLL_NO_ENROLL_PERMIT = 0x02,  /**< No Enroll Permit */
    ZB_IAS_ZONE_ENROLL_TOO_MANY_ZONES   = 0x03   /**< Too Many Zones */
} zb_ias_zone_enroll_response_code_t;

/**
 * @brief IAS Zone state structure
 */
typedef struct {
    zb_ias_zone_state_t zone_state;     /**< Zone enrollment state */
    zb_ias_zone_type_t zone_type;       /**< Zone type */
    uint16_t zone_status;               /**< Raw zone status bitmap */
    uint8_t zone_id;                    /**< Assigned Zone ID */
    esp_zb_ieee_addr_t cie_address;     /**< IAS CIE Address */
    bool alarm1;                        /**< Primary alarm active */
    bool alarm2;                        /**< Secondary alarm active */
    bool tamper;                        /**< Tamper detected */
    bool battery_low;                   /**< Battery low */
    bool supervision_reports;           /**< Supervision reports enabled */
    bool restore_reports;               /**< Restore reports enabled */
    bool trouble;                       /**< Trouble detected */
    bool ac_mains_fault;                /**< AC mains fault */
    bool test_mode;                     /**< Test mode active */
    bool battery_defect;                /**< Battery defect */
} zb_ias_zone_state_struct_t;

/**
 * @brief IAS Zone state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current IAS Zone state
 */
typedef void (*zb_ias_zone_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                        const zb_ias_zone_state_struct_t *state);

/* ============================================================================
 * NVS Storage Constants
 * ============================================================================ */

/**
 * @brief NVS namespace for device friendly names
 */
#define ZB_DEVICE_NAMES_NVS_NAMESPACE   "dev_names"

/* ============================================================================
 * Device Persistence NVS Constants
 * ============================================================================ */

/**
 * @brief NVS namespace for device registry persistence
 */
#define ZB_DEVICE_NVS_NAMESPACE         "zb_devices"

/**
 * @brief NVS key prefix for device records
 */
#define ZB_DEVICE_NVS_KEY_PREFIX        "dev_"

/**
 * @brief NVS key for device count
 */
#define ZB_DEVICE_NVS_KEY_COUNT         "dev_count"

/**
 * @brief Marker for pending/unknown short address
 *
 * Used when device is loaded from NVS but hasn't communicated yet
 */
#define ZB_SHORT_ADDR_PENDING           0xFFFF

/**
 * @brief Maximum clusters stored in NVS record
 */
#define ZB_NVS_MAX_CLUSTERS             32

/**
 * @brief NVS record structure for device persistence
 *
 * This structure is serialized as a binary blob in NVS.
 * All essential device information is stored to survive reboots.
 * Size: ~172 bytes per device
 */
typedef struct __attribute__((packed)) {
    uint8_t ieee_addr[8];                           /**< IEEE 64-bit address (primary key) */
    uint8_t endpoint;                               /**< Primary endpoint */
    uint8_t device_type;                            /**< Device type enum (zb_device_type_t) */
    uint16_t cluster_count;                         /**< Number of supported clusters */
    uint16_t clusters[ZB_NVS_MAX_CLUSTERS];         /**< Supported cluster IDs */
    char manufacturer[ZB_DEVICE_MANUFACTURER_LEN];  /**< Manufacturer name */
    char model[ZB_DEVICE_MODEL_LEN];                /**< Model identifier */
    char friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN]; /**< User-friendly name */
    /* Power info (added v2 — zero-initialized for old records) */
    uint8_t power_mode;                             /**< Current power mode */
    uint8_t available_power_sources;                /**< Available power source bitmask */
    uint8_t current_power_source;                   /**< Current power source bitmask */
    uint8_t current_power_source_level;             /**< Power source level */
    uint8_t power_info_valid;                       /**< Non-zero if power info is valid */
} zb_device_nvs_record_t;

/* ============================================================================
 * Power Descriptor Types (API-007)
 * ============================================================================ */

/**
 * @brief Power Source Bitmask values
 *
 * These aliases map to the ESP-Zigbee-SDK constants for convenience.
 * From Zigbee Specification: Table 2.37 Power Source Field
 */
#define ZB_POWER_SOURCE_MAINS                   ESP_ZB_AF_NODE_POWER_SOURCE_CONSTANT_POWER       /**< Constant (mains) power */
#define ZB_POWER_SOURCE_RECHARGEABLE_BATTERY    ESP_ZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY /**< Rechargeable battery */
#define ZB_POWER_SOURCE_DISPOSABLE_BATTERY      ESP_ZB_AF_NODE_POWER_SOURCE_DISPOSABLE_BATTERY   /**< Disposable battery */

/**
 * @brief Power Level values
 *
 * These aliases map to the ESP-Zigbee-SDK constants for convenience.
 * From Zigbee Specification: Table 2.38 Current Power Source Level Field
 */
#define ZB_POWER_LEVEL_CRITICAL     ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_CRITICAL     /**< Critical (< 5%) */
#define ZB_POWER_LEVEL_33_PERCENT   ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_33_PERCENT   /**< 33% remaining */
#define ZB_POWER_LEVEL_66_PERCENT   ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_66_PERCENT   /**< 66% remaining */
#define ZB_POWER_LEVEL_100_PERCENT  ESP_ZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT  /**< 100% (full) */

/**
 * @brief Power Mode values
 *
 * From Zigbee Specification: Table 2.36 Current Power Mode Field
 */
#define ZB_POWER_MODE_SYNC_ON_WHEN_IDLE         0x00    /**< Receiver synchronized when idle */
#define ZB_POWER_MODE_PERIODIC_ON_WHEN_IDLE     0x01    /**< Receiver periodically on when idle */
#define ZB_POWER_MODE_STIMULATED_ON_WHEN_IDLE   0x02    /**< Receiver stimulated on when idle */

/**
 * @brief Power descriptor response callback type
 *
 * Called when a power descriptor response is received.
 *
 * @param[in] short_addr Device short address
 * @param[in] success true if request was successful
 * @param[in] power_info Power information (only valid if success is true)
 */
typedef void (*zb_power_desc_cb_t)(uint16_t short_addr, bool success,
                                    const zb_power_info_t *power_info);

/* ============================================================================
 * Multistate Input/Output/Value Clusters (0x0012, 0x0013, 0x0014) Types
 * ============================================================================ */

/**
 * @brief Multistate Input Cluster ID
 *
 * Used for multi-button switches (Aqara, IKEA), rotary switches, and
 * other devices that report discrete state values.
 */
#define ZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT      0x0012

/**
 * @brief Multistate Output Cluster ID
 *
 * Used for multi-mode outputs where PresentValue is writable.
 */
#define ZB_ZCL_CLUSTER_ID_MULTISTATE_OUTPUT     0x0013

/**
 * @brief Multistate Value Cluster ID
 *
 * Used for generic multi-state storage where PresentValue is writable.
 */
#define ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE      0x0014

/**
 * @brief Multistate Cluster Attribute IDs
 *
 * These attributes are common to all three multistate clusters.
 */
#define ZB_ZCL_ATTR_MULTISTATE_NUMBER_OF_STATES     0x004A  /**< NumberOfStates (uint16) */
#define ZB_ZCL_ATTR_MULTISTATE_OUT_OF_SERVICE       0x0051  /**< OutOfService (bool) */
#define ZB_ZCL_ATTR_MULTISTATE_PRESENT_VALUE        0x0055  /**< PresentValue (uint16) */
#define ZB_ZCL_ATTR_MULTISTATE_STATUS_FLAGS         0x006F  /**< StatusFlags (bitmap8) */

/**
 * @brief Multistate StatusFlags bits (bitmap8)
 *
 * Bit 0: InAlarm - Indicates an alarm condition
 * Bit 1: Fault - Indicates a fault condition
 * Bit 2: Overridden - Indicates value is overridden
 * Bit 3: OutOfService - Duplicate of OutOfService attribute
 */
#define ZB_MULTISTATE_STATUS_IN_ALARM       (1 << 0)
#define ZB_MULTISTATE_STATUS_FAULT          (1 << 1)
#define ZB_MULTISTATE_STATUS_OVERRIDDEN     (1 << 2)
#define ZB_MULTISTATE_STATUS_OUT_OF_SERVICE (1 << 3)

/**
 * @brief Multistate cluster type enumeration
 *
 * Identifies which type of multistate cluster is being used.
 */
typedef enum {
    ZB_MULTISTATE_TYPE_INPUT  = 0,  /**< Multistate Input (read-only) */
    ZB_MULTISTATE_TYPE_OUTPUT = 1,  /**< Multistate Output (writable) */
    ZB_MULTISTATE_TYPE_VALUE  = 2   /**< Multistate Value (writable) */
} zb_multistate_type_t;

/**
 * @brief Multistate state structure
 *
 * Stores the current state for a multistate input/output/value cluster.
 * This structure is shared by all three cluster types.
 */
typedef struct {
    uint16_t number_of_states;      /**< Number of valid states (1 to N) */
    uint16_t present_value;         /**< Current state value (1 to number_of_states) */
    bool out_of_service;            /**< True if device is out of service */
    uint8_t status_flags;           /**< Status flags bitmap */
    zb_multistate_type_t type;      /**< Cluster type (input/output/value) */
    uint8_t endpoint;               /**< Endpoint this state is associated with */
} zb_multistate_state_t;

/**
 * @brief Multistate state change callback type
 *
 * Called when a multistate cluster attribute changes.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current multistate state
 */
typedef void (*zb_multistate_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                          const zb_multistate_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ZB_DEVICE_HANDLER_TYPES_H */
