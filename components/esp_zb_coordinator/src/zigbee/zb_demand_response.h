/**
 * @file zb_demand_response.h
 * @brief Zigbee Demand Response and Load Control Cluster (0x0701) API
 *
 * This module implements the ZCL Demand Response and Load Control (DRLC) Cluster
 * for ESP32-C5 Zigbee2MQTT Gateway. The DRLC cluster is part of the Smart Energy
 * profile and provides support for utility demand response programs.
 *
 * Key Features:
 * - Load control event management from utilities
 * - Event status reporting to utilities
 * - Scheduled events tracking
 * - Randomized start/stop times for grid stability
 * - Device class filtering
 *
 * ZCL Specification Reference: ZCL8, Smart Energy Profile, Cluster 0x0701
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/demand_response/event - Active load control events
 * - zigbee2mqtt/bridge/demand_response/status - Module status
 * - zigbee2mqtt/{device}/demand_response - Device-specific events
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_DEMAND_RESPONSE_H
#define ZB_DEMAND_RESPONSE_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Demand Response and Load Control Cluster Constants
 * ============================================================================ */

/**
 * @brief Demand Response and Load Control Cluster ID (ZCL Spec: 0x0701)
 */
#define ZB_ZCL_CLUSTER_ID_DEMAND_RESPONSE               0x0701

/**
 * @brief DRLC Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_DRLC_UTILITY_ENROLLMENT_GROUP_ID    0x0000  /**< UtilityEnrollmentGroup (uint8) */
#define ZB_ZCL_ATTR_DRLC_START_RANDOMIZE_MINUTES_ID     0x0001  /**< StartRandomizeMinutes (uint8) */
#define ZB_ZCL_ATTR_DRLC_STOP_RANDOMIZE_MINUTES_ID      0x0002  /**< StopRandomizeMinutes (uint8) */
#define ZB_ZCL_ATTR_DRLC_DEVICE_CLASS_VALUE_ID          0x0003  /**< DeviceClassValue (uint16) */

/**
 * @brief DRLC Cluster Command IDs (Server to Client)
 */
#define ZB_ZCL_CMD_DRLC_LOAD_CONTROL_EVENT_ID           0x00    /**< LoadControlEvent */
#define ZB_ZCL_CMD_DRLC_CANCEL_LOAD_CONTROL_EVENT_ID    0x01    /**< CancelLoadControlEvent */
#define ZB_ZCL_CMD_DRLC_CANCEL_ALL_LOAD_CONTROL_ID      0x02    /**< CancelAllLoadControlEvents */

/**
 * @brief DRLC Cluster Command IDs (Client to Server)
 */
#define ZB_ZCL_CMD_DRLC_REPORT_EVENT_STATUS_ID          0x00    /**< ReportEventStatus */
#define ZB_ZCL_CMD_DRLC_GET_SCHEDULED_EVENTS_ID         0x01    /**< GetScheduledEvents */

/**
 * @brief Maximum active load control events
 */
#define ZB_DRLC_MAX_ACTIVE_EVENTS                       8

/**
 * @brief Maximum devices tracking DRLC events
 */
#define ZB_DRLC_MAX_DEVICES                             16

/**
 * @brief Invalid event ID sentinel
 */
#define ZB_DRLC_INVALID_EVENT_ID                        0xFFFFFFFF

/**
 * @brief Event duration indicating cancel
 */
#define ZB_DRLC_DURATION_CANCEL                         0

/* ============================================================================
 * Device Class Bitmap Definitions (per ZCL specification)
 * ============================================================================ */

/**
 * @brief Device class bitmap values
 *
 * Defines which device types should respond to a load control event.
 */
#define ZB_DRLC_DEVICE_CLASS_HVAC                       0x0001  /**< HVAC compressor or furnace */
#define ZB_DRLC_DEVICE_CLASS_STRIP_HEAT                 0x0002  /**< Strip heaters/baseboard */
#define ZB_DRLC_DEVICE_CLASS_WATER_HEATER               0x0004  /**< Water heater */
#define ZB_DRLC_DEVICE_CLASS_POOL_PUMP                  0x0008  /**< Pool pump/spa/jacuzzi */
#define ZB_DRLC_DEVICE_CLASS_SMART_APPLIANCE            0x0010  /**< Smart appliance */
#define ZB_DRLC_DEVICE_CLASS_IRRIGATION_PUMP            0x0020  /**< Irrigation pump */
#define ZB_DRLC_DEVICE_CLASS_MANAGED_COMM_LOAD          0x0040  /**< Managed commercial/industrial load */
#define ZB_DRLC_DEVICE_CLASS_SIMPLE_LOAD                0x0080  /**< Simple misc load */
#define ZB_DRLC_DEVICE_CLASS_EXTERIOR_LIGHTING          0x0100  /**< Exterior lighting */
#define ZB_DRLC_DEVICE_CLASS_INTERIOR_LIGHTING          0x0200  /**< Interior lighting */
#define ZB_DRLC_DEVICE_CLASS_ELECTRIC_VEHICLE           0x0400  /**< Electric vehicle */
#define ZB_DRLC_DEVICE_CLASS_GENERATION_SYSTEM          0x0800  /**< Generation system */
#define ZB_DRLC_DEVICE_CLASS_ALL                        0x0FFF  /**< All device classes */

/* ============================================================================
 * Criticality Level Definitions
 * ============================================================================ */

/**
 * @brief Event criticality levels
 */
typedef enum {
    ZB_DRLC_CRITICALITY_RESERVED = 0x00,
    ZB_DRLC_CRITICALITY_GREEN = 0x01,           /**< Green - Lowest urgency */
    ZB_DRLC_CRITICALITY_LEVEL_1 = 0x02,         /**< Level 1 */
    ZB_DRLC_CRITICALITY_LEVEL_2 = 0x03,         /**< Level 2 */
    ZB_DRLC_CRITICALITY_LEVEL_3 = 0x04,         /**< Level 3 */
    ZB_DRLC_CRITICALITY_LEVEL_4 = 0x05,         /**< Level 4 */
    ZB_DRLC_CRITICALITY_LEVEL_5 = 0x06,         /**< Level 5 */
    ZB_DRLC_CRITICALITY_EMERGENCY = 0x07,       /**< Emergency */
    ZB_DRLC_CRITICALITY_PLANNED_OUTAGE = 0x08,  /**< Planned outage */
    ZB_DRLC_CRITICALITY_SERVICE_DISCONNECT = 0x09, /**< Service disconnect */
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_1 = 0x0A,
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_2 = 0x0B,
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_3 = 0x0C,
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_4 = 0x0D,
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_5 = 0x0E,
    ZB_DRLC_CRITICALITY_UTILITY_DEFINED_6 = 0x0F,
} zb_drlc_criticality_t;

/* ============================================================================
 * Event Status Definitions
 * ============================================================================ */

/**
 * @brief Event status values for ReportEventStatus
 */
typedef enum {
    ZB_DRLC_STATUS_EVENT_RECEIVED = 0x01,                /**< Event received */
    ZB_DRLC_STATUS_EVENT_STARTED = 0x02,                 /**< Event started */
    ZB_DRLC_STATUS_EVENT_COMPLETED = 0x03,               /**< Event completed */
    ZB_DRLC_STATUS_USER_OPTED_OUT = 0x04,                /**< User opted out */
    ZB_DRLC_STATUS_USER_OPTED_IN = 0x05,                 /**< User opted in */
    ZB_DRLC_STATUS_EVENT_CANCELLED = 0x06,               /**< Event cancelled */
    ZB_DRLC_STATUS_EVENT_SUPERSEDED = 0x07,              /**< Event superseded */
    ZB_DRLC_STATUS_EVENT_PARTIAL_OPT_OUT = 0x08,         /**< Partial opt out */
    ZB_DRLC_STATUS_EVENT_PARTIAL_OPT_IN = 0x09,          /**< Partial opt in */
    ZB_DRLC_STATUS_EVENT_COMPLETE_NO_USER = 0x0A,        /**< Event complete - no user action */
    ZB_DRLC_STATUS_INVALID_CANCEL_UNDEFINED = 0xF8,      /**< Invalid cancel - undefined */
    ZB_DRLC_STATUS_INVALID_CANCEL_COMMAND = 0xF9,        /**< Invalid cancel command */
    ZB_DRLC_STATUS_INVALID_EFFECTIVE_TIME = 0xFA,        /**< Invalid effective time */
    ZB_DRLC_STATUS_INVALID_ISSUER_EVENT = 0xFB,          /**< Invalid issuer event ID */
    ZB_DRLC_STATUS_REJECTED = 0xFD,                      /**< Event rejected */
    ZB_DRLC_STATUS_UNDEFINED = 0xFE,                     /**< Undefined event */
} zb_drlc_event_status_t;

/* ============================================================================
 * Event Control Bitmap
 * ============================================================================ */

/**
 * @brief Event control bitmap fields
 */
#define ZB_DRLC_EVENT_CTRL_RANDOMIZED_START     0x01    /**< Randomize start time */
#define ZB_DRLC_EVENT_CTRL_RANDOMIZED_END       0x02    /**< Randomize end time */

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Load control event structure
 *
 * Represents a demand response load control event from a utility.
 */
typedef struct {
    uint32_t issuer_event_id;           /**< Unique event identifier from utility */
    uint16_t device_class;              /**< Device class bitmap (which devices affected) */
    uint8_t utility_enrollment_group;   /**< Enrollment group (0xFF = all) */
    uint32_t start_time;                /**< UTC start time (0 = now) */
    uint16_t duration_minutes;          /**< Event duration in minutes */
    uint8_t criticality_level;          /**< Criticality level (zb_drlc_criticality_t) */
    uint8_t cooling_temp_offset;        /**< Cooling temperature offset (0xFF = N/A) */
    uint8_t heating_temp_offset;        /**< Heating temperature offset (0xFF = N/A) */
    int16_t cooling_temp_setpoint;      /**< Cooling temperature setpoint (0x8000 = N/A) */
    int16_t heating_temp_setpoint;      /**< Heating temperature setpoint (0x8000 = N/A) */
    int8_t average_load_adjustment;     /**< Average load adjustment % (-100 to +100) */
    uint8_t duty_cycle;                 /**< Duty cycle % (0-100, 0xFF = N/A) */
    uint8_t event_control;              /**< Event control bitmap */
} zb_load_control_event_t;

/**
 * @brief Active event tracking structure
 */
typedef struct {
    zb_load_control_event_t event;      /**< The load control event */
    uint32_t actual_start_time;         /**< Actual start time (after randomization) */
    uint32_t actual_end_time;           /**< Actual end time (after randomization) */
    zb_drlc_event_status_t status;      /**< Current event status */
    bool active;                        /**< Event is currently active */
    bool user_opted_out;                /**< User has opted out of this event */
} zb_drlc_active_event_t;

/**
 * @brief DRLC module configuration
 */
typedef struct {
    uint8_t utility_enrollment_group;   /**< Default enrollment group (0xFF = respond to all) */
    uint8_t start_randomize_minutes;    /**< Start randomization window (minutes) */
    uint8_t stop_randomize_minutes;     /**< Stop randomization window (minutes) */
    uint16_t device_class;              /**< Device class this device belongs to */
    bool allow_user_opt_out;            /**< Allow user to opt out of events */
} zb_drlc_config_t;

/**
 * @brief DRLC statistics structure
 */
typedef struct {
    uint32_t events_received;           /**< Total events received */
    uint32_t events_started;            /**< Events that started */
    uint32_t events_completed;          /**< Events that completed normally */
    uint32_t events_cancelled;          /**< Events that were cancelled */
    uint32_t events_opted_out;          /**< Events where user opted out */
    uint32_t status_reports_sent;       /**< Status reports sent to utility */
} zb_drlc_stats_t;

/* ============================================================================
 * Callback Types
 * ============================================================================ */

/**
 * @brief Load control event callback type
 *
 * Called when a load control event is received from the utility.
 *
 * @param[in] event     The load control event
 * @param[in] is_cancel true if this is a cancel event
 */
typedef void (*zb_drlc_event_callback_t)(const zb_load_control_event_t *event, bool is_cancel);

/**
 * @brief Event status change callback type
 *
 * Called when an event's status changes (started, completed, etc.).
 *
 * @param[in] issuer_event_id Event identifier
 * @param[in] old_status      Previous status
 * @param[in] new_status      New status
 */
typedef void (*zb_drlc_status_callback_t)(uint32_t issuer_event_id,
                                           zb_drlc_event_status_t old_status,
                                           zb_drlc_event_status_t new_status);

/* ============================================================================
 * Module Initialization and Control
 * ============================================================================ */

/**
 * @brief Initialize the Demand Response module
 *
 * Initializes the DRLC module with default configuration.
 * Must be called before any other DRLC operations.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_demand_response_init(void);

/**
 * @brief Deinitialize the Demand Response module
 *
 * Stops event processing and frees resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_demand_response_deinit(void);

/**
 * @brief Check if DRLC module is initialized
 *
 * @return true if initialized
 * @return false if not initialized
 */
bool zb_demand_response_is_initialized(void);

/**
 * @brief Configure the DRLC module
 *
 * Sets the configuration for demand response handling.
 *
 * @param[in] config Configuration structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_demand_response_configure(const zb_drlc_config_t *config);

/**
 * @brief Get current DRLC configuration
 *
 * @param[out] config Output configuration structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_demand_response_get_config(zb_drlc_config_t *config);

/* ============================================================================
 * Event Handling
 * ============================================================================ */

/**
 * @brief Handle incoming load control event
 *
 * Processes a LoadControlEvent command received from a utility/server.
 * This is called by the ZCL callback handler.
 *
 * @param[in] event Load control event to process
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if event is NULL
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_demand_response_handle_event(const zb_load_control_event_t *event);

/**
 * @brief Handle cancel load control event command
 *
 * Processes a CancelLoadControlEvent command.
 *
 * @param[in] issuer_event_id   Event ID to cancel
 * @param[in] device_class      Device class bitmap
 * @param[in] enrollment_group  Enrollment group
 * @param[in] cancel_control    Cancel control flags
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if event not found
 */
esp_err_t zb_demand_response_handle_cancel_event(uint32_t issuer_event_id,
                                                   uint16_t device_class,
                                                   uint8_t enrollment_group,
                                                   uint8_t cancel_control);

/**
 * @brief Handle cancel all load control events command
 *
 * Cancels all active load control events.
 *
 * @param[in] cancel_control Cancel control flags
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_handle_cancel_all(uint8_t cancel_control);

/* ============================================================================
 * Event Status and Reporting
 * ============================================================================ */

/**
 * @brief Report event status to utility
 *
 * Sends a ReportEventStatus command to the utility for a specific event.
 *
 * @param[in] issuer_event_id   Event identifier
 * @param[in] status            Event status to report
 * @param[in] event_control     Event control from original event
 * @param[in] signature_type    Signature type (0 = no signature)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if event not found
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_demand_response_report_status(uint32_t issuer_event_id,
                                            zb_drlc_event_status_t status,
                                            uint8_t event_control,
                                            uint8_t signature_type);

/**
 * @brief Request scheduled events from server
 *
 * Sends a GetScheduledEvents command to request upcoming events.
 *
 * @param[in] dst_addr      Destination address
 * @param[in] dst_endpoint  Destination endpoint
 * @param[in] start_time    Start time for query (0 = now)
 * @param[in] num_events    Number of events to request (0 = all)
 * @return ESP_OK on success
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_demand_response_get_scheduled_events(uint16_t dst_addr,
                                                    uint8_t dst_endpoint,
                                                    uint32_t start_time,
                                                    uint8_t num_events);

/* ============================================================================
 * Active Events Management
 * ============================================================================ */

/**
 * @brief Get active load control events
 *
 * Returns all currently active (or scheduled) events.
 *
 * @param[out] events       Array to store events
 * @param[in]  max_count    Maximum events to return
 * @param[out] count        Actual number of events returned
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_demand_response_get_active_events(zb_drlc_active_event_t *events,
                                                 size_t max_count,
                                                 size_t *count);

/**
 * @brief Get active event count
 *
 * @return Number of active or scheduled events
 */
size_t zb_demand_response_get_active_event_count(void);

/**
 * @brief Get specific event by ID
 *
 * @param[in]  issuer_event_id  Event ID to look up
 * @param[out] event            Output event structure
 * @return ESP_OK if found
 * @return ESP_ERR_NOT_FOUND if not found
 */
esp_err_t zb_demand_response_get_event(uint32_t issuer_event_id,
                                         zb_drlc_active_event_t *event);

/**
 * @brief User opt-out of an event
 *
 * Allows the user to opt out of a specific event.
 *
 * @param[in] issuer_event_id Event ID to opt out of
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if event not found
 * @return ESP_ERR_NOT_SUPPORTED if opt-out not allowed
 */
esp_err_t zb_demand_response_user_opt_out(uint32_t issuer_event_id);

/**
 * @brief User opt-in to an event
 *
 * Re-enrolls in an event after opt-out.
 *
 * @param[in] issuer_event_id Event ID to opt in to
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if event not found
 */
esp_err_t zb_demand_response_user_opt_in(uint32_t issuer_event_id);

/* ============================================================================
 * Callbacks
 * ============================================================================ */

/**
 * @brief Register event callback
 *
 * Called when load control events are received.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_register_callback(zb_drlc_event_callback_t callback);

/**
 * @brief Register status change callback
 *
 * Called when event status changes.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_register_status_callback(zb_drlc_status_callback_t callback);

/* ============================================================================
 * Statistics and Diagnostics
 * ============================================================================ */

/**
 * @brief Get DRLC statistics
 *
 * @param[out] stats Output statistics structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if stats is NULL
 */
esp_err_t zb_demand_response_get_stats(zb_drlc_stats_t *stats);

/**
 * @brief Reset DRLC statistics
 *
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_reset_stats(void);

/* ============================================================================
 * MQTT Integration
 * ============================================================================ */

/**
 * @brief Publish active events to MQTT
 *
 * Publishes list of active load control events to MQTT.
 * Topic: zigbee2mqtt/bridge/demand_response/events
 *
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_publish_events(void);

/**
 * @brief Publish DRLC status to MQTT
 *
 * Publishes module status and statistics to MQTT.
 * Topic: zigbee2mqtt/bridge/demand_response/status
 *
 * @return ESP_OK on success
 */
esp_err_t zb_demand_response_publish_status(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Get criticality level name
 *
 * @param[in] level Criticality level
 * @return Human-readable string
 */
const char* zb_drlc_criticality_to_string(zb_drlc_criticality_t level);

/**
 * @brief Get event status name
 *
 * @param[in] status Event status
 * @return Human-readable string
 */
const char* zb_drlc_status_to_string(zb_drlc_event_status_t status);

/**
 * @brief Get device class description
 *
 * @param[in] device_class Device class bitmap
 * @param[out] buf         Output buffer
 * @param[in] buf_len      Buffer length
 * @return ESP_OK on success
 */
esp_err_t zb_drlc_device_class_to_string(uint16_t device_class, char *buf, size_t buf_len);

/* ============================================================================
 * Self-Test
 * ============================================================================ */

/**
 * @brief Self-test function for DRLC module
 *
 * Tests module initialization, event handling, and reporting.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_demand_response_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_DEMAND_RESPONSE_H */
