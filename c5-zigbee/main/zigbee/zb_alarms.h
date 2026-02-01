/**
 * @file zb_alarms.h
 * @brief Zigbee Alarms Cluster (0x0009) API
 *
 * This module implements the ZCL Alarms Cluster for ESP32-C5 Zigbee2MQTT Gateway.
 * The Alarms cluster provides alarm management capabilities including:
 * - Alarm table management (storing alarm history)
 * - Alarm notifications handling (receiving alarm events from devices)
 * - Reset alarm command (clear specific alarm)
 * - Get alarm command (retrieve alarm from table)
 * - Reset all alarms command (clear all alarms for a device)
 *
 * ZCL Specification Reference: ZCL8, Chapter 3.11 Alarms Cluster
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_ALARMS_H
#define ZB_ALARMS_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Alarms Cluster Constants
 * ============================================================================ */

/**
 * @brief Alarms Cluster ID (ZCL Spec: 0x0009)
 */
#define ZB_ZCL_CLUSTER_ID_ALARMS                    0x0009

/**
 * @brief Alarms Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_ALARMS_ALARM_COUNT_ID           0x0000  /**< AlarmCount (uint16) - R/O */

/**
 * @brief Alarms Cluster Command IDs (Client to Server)
 */
#define ZB_ZCL_CMD_ALARMS_RESET_ALARM_ID            0x00    /**< Reset Alarm */
#define ZB_ZCL_CMD_ALARMS_RESET_ALL_ALARMS_ID       0x01    /**< Reset All Alarms */
#define ZB_ZCL_CMD_ALARMS_GET_ALARM_ID              0x02    /**< Get Alarm */
#define ZB_ZCL_CMD_ALARMS_RESET_ALARM_LOG_ID        0x03    /**< Reset Alarm Log */

/**
 * @brief Alarms Cluster Command IDs (Server to Client)
 */
#define ZB_ZCL_CMD_ALARMS_ALARM_ID                  0x00    /**< Alarm notification */
#define ZB_ZCL_CMD_ALARMS_GET_ALARM_RESPONSE_ID     0x01    /**< Get Alarm Response */

/**
 * @brief Maximum alarms per device in the alarm table
 */
#define ZB_ALARMS_MAX_PER_DEVICE                    16

/**
 * @brief Maximum devices with alarm tracking
 */
#define ZB_ALARMS_MAX_DEVICES                       32

/**
 * @brief Invalid alarm code sentinel
 */
#define ZB_ALARM_CODE_INVALID                       0xFF

/* ============================================================================
 * Alarms Data Structures
 * ============================================================================ */

/**
 * @brief Single alarm entry structure
 *
 * Represents one alarm in the alarm table.
 */
typedef struct {
    uint8_t alarm_code;         /**< Alarm code (cluster-specific meaning) */
    uint16_t cluster_id;        /**< Cluster ID that generated the alarm */
    uint32_t timestamp;         /**< Timestamp when alarm occurred (seconds since epoch) */
    bool valid;                 /**< Entry is valid */
} zb_alarm_entry_t;

/**
 * @brief Device alarms structure
 *
 * Contains all alarms for a single device.
 */
typedef struct {
    uint16_t short_addr;                                /**< Device short address */
    uint8_t endpoint;                                   /**< Device endpoint */
    zb_alarm_entry_t alarms[ZB_ALARMS_MAX_PER_DEVICE];  /**< Alarm table */
    uint8_t alarm_count;                                /**< Number of active alarms */
    uint32_t last_alarm_time;                           /**< Timestamp of most recent alarm */
} zb_device_alarms_t;

/**
 * @brief Alarm notification callback type
 *
 * Called when an alarm notification is received from a device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster that generated the alarm
 * @param[in] timestamp Timestamp of the alarm
 */
typedef void (*zb_alarms_notification_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                             uint8_t alarm_code, uint16_t cluster_id,
                                             uint32_t timestamp);

/**
 * @brief Alarm cleared callback type
 *
 * Called when an alarm is cleared (reset).
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code that was cleared (0xFF for all)
 * @param[in] cluster_id Cluster ID (0xFFFF for all)
 */
typedef void (*zb_alarms_cleared_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                        uint8_t alarm_code, uint16_t cluster_id);

/* ============================================================================
 * Alarms Module API
 * ============================================================================ */

/**
 * @brief Initialize the alarms module
 *
 * Initializes internal state and alarm table storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_alarms_init(void);

/**
 * @brief Deinitialize the alarms module
 *
 * Frees all allocated resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_alarms_deinit(void);

/**
 * @brief Register alarm notification callback
 *
 * The callback is invoked when an alarm notification is received.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_alarms_register_notification_cb(zb_alarms_notification_cb_t callback);

/**
 * @brief Register alarm cleared callback
 *
 * The callback is invoked when an alarm is cleared.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_alarms_register_cleared_cb(zb_alarms_cleared_cb_t callback);

/* ============================================================================
 * Alarm Table Management
 * ============================================================================ */

/**
 * @brief Get all alarms for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] alarms Output array for alarm entries (must be ZB_ALARMS_MAX_PER_DEVICE elements)
 * @param[out] count Number of valid alarms returned
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no alarms for device
 * @return ESP_ERR_INVALID_ARG if parameters are NULL
 */
esp_err_t zb_alarms_get_device_alarms(uint16_t short_addr, zb_alarm_entry_t *alarms, uint8_t *count);

/**
 * @brief Get alarm count for a device
 *
 * @param[in] short_addr Device short address
 * @return Number of active alarms (0 if none or device not found)
 */
uint8_t zb_alarms_get_device_alarm_count(uint16_t short_addr);

/**
 * @brief Get total alarm count across all devices
 *
 * @return Total number of active alarms
 */
uint32_t zb_alarms_get_total_count(void);

/**
 * @brief Check if device has any active alarms
 *
 * @param[in] short_addr Device short address
 * @return true if device has active alarms
 */
bool zb_alarms_device_has_alarms(uint16_t short_addr);

/**
 * @brief Get the most recent alarm for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] alarm Output alarm entry
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no alarms for device
 * @return ESP_ERR_INVALID_ARG if alarm is NULL
 */
esp_err_t zb_alarms_get_latest(uint16_t short_addr, zb_alarm_entry_t *alarm);

/* ============================================================================
 * Alarm Commands (Client to Server)
 * ============================================================================ */

/**
 * @brief Send Reset Alarm command to device
 *
 * Resets (clears) a specific alarm on a device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code to reset
 * @param[in] cluster_id Cluster ID of the alarm
 * @return ESP_OK on success
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_alarms_cmd_reset_alarm(uint16_t short_addr, uint8_t endpoint,
                                     uint8_t alarm_code, uint16_t cluster_id);

/**
 * @brief Send Reset All Alarms command to device
 *
 * Resets all alarms on a device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_alarms_cmd_reset_all(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Get Alarm command to device
 *
 * Requests the device to send its first alarm entry.
 * The response is handled asynchronously via the callback.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_alarms_cmd_get_alarm(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Reset Alarm Log command to device
 *
 * Clears the alarm log on the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 * @return ESP_FAIL if command send fails
 */
esp_err_t zb_alarms_cmd_reset_alarm_log(uint16_t short_addr, uint8_t endpoint);

/* ============================================================================
 * Alarm Notifications Handling (Server to Client)
 * ============================================================================ */

/**
 * @brief Handle incoming Alarm notification
 *
 * Called by the action callback handler when an Alarm notification
 * is received from a device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster that generated the alarm
 * @return ESP_OK on success
 */
esp_err_t zb_alarms_handle_notification(uint16_t short_addr, uint8_t endpoint,
                                         uint8_t alarm_code, uint16_t cluster_id);

/**
 * @brief Handle Get Alarm Response
 *
 * Called when a Get Alarm Response is received from a device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] status Response status (0x00 = success, 0x01 = no alarms)
 * @param[in] alarm_code Alarm code (valid if status == 0x00)
 * @param[in] cluster_id Cluster ID (valid if status == 0x00)
 * @param[in] timestamp Alarm timestamp (valid if status == 0x00)
 * @return ESP_OK on success
 */
esp_err_t zb_alarms_handle_get_response(uint16_t short_addr, uint8_t endpoint,
                                         uint8_t status, uint8_t alarm_code,
                                         uint16_t cluster_id, uint32_t timestamp);

/* ============================================================================
 * Alarm Table Local Management
 * ============================================================================ */

/**
 * @brief Add alarm to local table
 *
 * Manually adds an alarm to the local alarm table.
 * Typically called internally when notifications are received.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster ID
 * @param[in] timestamp Timestamp (0 for current time)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if alarm table is full
 */
esp_err_t zb_alarms_add_to_table(uint16_t short_addr, uint8_t endpoint,
                                  uint8_t alarm_code, uint16_t cluster_id,
                                  uint32_t timestamp);

/**
 * @brief Remove alarm from local table
 *
 * Removes a specific alarm from the local alarm table.
 *
 * @param[in] short_addr Device short address
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if alarm not found
 */
esp_err_t zb_alarms_remove_from_table(uint16_t short_addr, uint8_t alarm_code,
                                       uint16_t cluster_id);

/**
 * @brief Clear all alarms for a device from local table
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success (even if no alarms existed)
 */
esp_err_t zb_alarms_clear_device(uint16_t short_addr);

/**
 * @brief Clear all alarms from local table
 *
 * @return ESP_OK on success
 */
esp_err_t zb_alarms_clear_all(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert alarm code to string representation
 *
 * Returns a human-readable string for common alarm codes.
 * Alarm codes are cluster-specific; this provides generic mappings.
 *
 * @param[in] alarm_code Alarm code
 * @param[in] cluster_id Cluster ID for context
 * @return String representation of alarm code
 */
const char* zb_alarms_code_to_string(uint8_t alarm_code, uint16_t cluster_id);

/**
 * @brief Get alarm cluster name for a cluster ID
 *
 * Returns the cluster name for context in alarm descriptions.
 *
 * @param[in] cluster_id Cluster ID
 * @return Cluster name string
 */
const char* zb_alarms_cluster_name(uint16_t cluster_id);

/**
 * @brief Self-test function for alarms module
 *
 * Tests alarm table operations and command building.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_alarms_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_ALARMS_H */
