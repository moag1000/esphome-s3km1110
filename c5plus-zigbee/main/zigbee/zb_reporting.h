/**
 * @file zb_reporting.h
 * @brief ZCL Reporting Configuration API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides ZCL Reporting Configuration functionality including:
 * - Configure Reporting Command (ZCL Frame 0x06) to set up automatic attribute reports
 * - Read Reporting Configuration (ZCL Frame 0x08) to query current settings
 * - Default reporting configurations for common sensor clusters
 * - NVS persistence for custom reporting configurations
 * - MQTT integration for Zigbee2MQTT compatibility
 *
 * Reporting allows devices to automatically send attribute updates:
 * - Periodically (based on min/max intervals)
 * - On change (when value changes by more than reportable_change)
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/device/reporting/configure - Configure reporting
 * - zigbee2mqtt/bridge/request/device/reporting/read - Read reporting config
 * - zigbee2mqtt/bridge/response/device/reporting/configure - Configure response
 * - zigbee2mqtt/bridge/response/device/reporting/read - Read response
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_REPORTING_H
#define ZB_REPORTING_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of reporting configuration entries
 */
#define ZB_REPORTING_MAX_ENTRIES 64

/**
 * @brief NVS namespace for reporting storage
 */
#define ZB_REPORTING_NVS_NAMESPACE "zb_report"

/**
 * @brief Maximum friendly name length for MQTT messages
 */
#define ZB_REPORTING_NAME_MAX_LEN 64

/**
 * @brief ZCL Configure Reporting Command ID
 */
#define ZCL_CMD_CONFIGURE_REPORTING         0x06

/**
 * @brief ZCL Configure Reporting Response Command ID
 */
#define ZCL_CMD_CONFIGURE_REPORTING_RSP     0x07

/**
 * @brief ZCL Read Reporting Configuration Command ID
 */
#define ZCL_CMD_READ_REPORTING_CONFIG       0x08

/**
 * @brief ZCL Read Reporting Configuration Response Command ID
 */
#define ZCL_CMD_READ_REPORTING_CONFIG_RSP   0x09

/**
 * @brief Reporting direction: attribute is reported
 */
#define ZCL_REPORTING_DIRECTION_REPORTED    0x00

/**
 * @brief Reporting direction: attribute reports are received
 */
#define ZCL_REPORTING_DIRECTION_RECEIVED    0x01

/**
 * @brief Special value for no minimum interval (report immediately on change)
 */
#define ZB_REPORTING_MIN_INTERVAL_NONE      0x0000

/**
 * @brief Special value for no maximum interval (no periodic reports)
 */
#define ZB_REPORTING_MAX_INTERVAL_NONE      0xFFFF

/**
 * @brief Special value for no reportable change (always report on interval)
 */
#define ZB_REPORTING_CHANGE_NONE            0x0000

/**
 * @brief Reporting status codes
 */
typedef enum {
    ZB_REPORTING_STATUS_SUCCESS = 0,              /**< Configuration successful */
    ZB_REPORTING_STATUS_NOT_SUPPORTED,            /**< Reporting not supported */
    ZB_REPORTING_STATUS_INVALID_ATTRIBUTE,        /**< Invalid attribute ID */
    ZB_REPORTING_STATUS_INVALID_DATATYPE,         /**< Invalid data type */
    ZB_REPORTING_STATUS_INVALID_CONFIG,           /**< Invalid configuration */
    ZB_REPORTING_STATUS_TIMEOUT,                  /**< Request timed out */
    ZB_REPORTING_STATUS_DEVICE_NOT_FOUND,         /**< Device not found */
    ZB_REPORTING_STATUS_ERROR                     /**< General error */
} zb_reporting_status_t;

/**
 * @brief Reporting configuration entry structure
 *
 * Defines the reporting parameters for a specific attribute on a device.
 */
typedef struct {
    uint16_t cluster_id;        /**< Cluster ID */
    uint16_t attr_id;           /**< Attribute ID */
    uint8_t attr_type;          /**< ZCL attribute data type */
    uint16_t min_interval;      /**< Minimum reporting interval in seconds (0x0000-0xFFFE) */
    uint16_t max_interval;      /**< Maximum reporting interval in seconds (0x0000-0xFFFF) */
    uint16_t reportable_change; /**< Minimum change to trigger report (type-dependent) */
} zb_reporting_config_t;

/**
 * @brief Stored reporting configuration entry with device info
 */
typedef struct {
    uint64_t device_ieee;              /**< Device IEEE address */
    uint8_t endpoint;                  /**< Device endpoint */
    zb_reporting_config_t config;      /**< Reporting configuration */
    bool active;                       /**< Entry is in use */
} zb_reporting_entry_t;

/**
 * @brief Reporting event types
 */
typedef enum {
    ZB_REPORTING_EVENT_CONFIGURED,     /**< Reporting was configured */
    ZB_REPORTING_EVENT_READ,           /**< Reporting config was read */
    ZB_REPORTING_EVENT_FAILED          /**< Operation failed */
} zb_reporting_event_type_t;

/**
 * @brief Reporting event callback type
 *
 * @param[in] event Event type
 * @param[in] device_ieee Device IEEE address
 * @param[in] endpoint Device endpoint
 * @param[in] config Reporting configuration (may be NULL for failures)
 * @param[in] status Status code
 */
typedef void (*zb_reporting_event_cb_t)(zb_reporting_event_type_t event,
                                         uint64_t device_ieee,
                                         uint8_t endpoint,
                                         const zb_reporting_config_t *config,
                                         zb_reporting_status_t status);

/**
 * @brief Reporting request result structure for MQTT responses
 */
typedef struct {
    bool success;                                  /**< Operation success status */
    zb_reporting_status_t status;                  /**< Detailed status code */
    char device_name[ZB_REPORTING_NAME_MAX_LEN];   /**< Device name/IEEE */
    uint8_t endpoint;                              /**< Endpoint */
    zb_reporting_config_t config;                  /**< Configuration */
    char error_message[128];                       /**< Error message if failed */
} zb_reporting_result_t;

/**
 * @brief Default reporting configuration for known clusters
 */
typedef struct {
    uint16_t cluster_id;        /**< Cluster ID */
    uint16_t attr_id;           /**< Attribute ID */
    uint8_t attr_type;          /**< Attribute type */
    uint16_t min_interval;      /**< Default min interval (seconds) */
    uint16_t max_interval;      /**< Default max interval (seconds) */
    uint16_t reportable_change; /**< Default reportable change */
    const char *cluster_name;   /**< Cluster name for MQTT */
    const char *attr_name;      /**< Attribute name for MQTT */
} zb_reporting_default_t;

/**
 * @brief Initialize reporting configuration module
 *
 * Initializes the reporting configuration module, allocates resources,
 * and loads existing configurations from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_reporting_init(void);

/**
 * @brief Deinitialize reporting configuration module
 *
 * Saves configurations to NVS and frees all resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_reporting_deinit(void);

/**
 * @brief Configure reporting for an attribute
 *
 * Sends a ZCL Configure Reporting command to the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] config Reporting configuration
 * @return ESP_OK on success (request sent, not yet confirmed)
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_reporting_configure(uint16_t short_addr, uint8_t endpoint,
                                  zb_reporting_config_t *config);

/**
 * @brief Read reporting configuration for an attribute
 *
 * Sends a ZCL Read Reporting Configuration command to the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] cluster_id Cluster ID
 * @param[in] attr_id Attribute ID
 * @return ESP_OK on success (request sent)
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_reporting_read(uint16_t short_addr, uint8_t endpoint,
                            uint16_t cluster_id, uint16_t attr_id);

/**
 * @brief Set default reporting configuration for a device
 *
 * Configures reporting for all known sensor clusters on the device
 * using predefined default values.
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_reporting_set_defaults(uint16_t short_addr);

/**
 * @brief Process MQTT reporting request
 *
 * Main entry point for handling reporting-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/device/reporting/configure
 * - zigbee2mqtt/bridge/request/device/reporting/read
 *
 * Expected JSON payload format for configure:
 * {
 *   "id": "0x00124B001234ABCD" or "friendly_name",
 *   "endpoint": 1,
 *   "cluster": "msTemperatureMeasurement",
 *   "attribute": "measuredValue",
 *   "minimum_report_interval": 10,
 *   "maximum_report_interval": 3600,
 *   "reportable_change": 10
 * }
 *
 * Expected JSON payload format for read:
 * {
 *   "id": "0x00124B001234ABCD" or "friendly_name",
 *   "endpoint": 1,
 *   "cluster": "msTemperatureMeasurement",
 *   "attribute": "measuredValue"
 * }
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request format
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_reporting_process_mqtt_request(const char *topic, const char *payload);

/**
 * @brief Handle Configure Reporting Response
 *
 * Processes the response from a Configure Reporting command.
 * Called by the Zigbee callback handler.
 *
 * @param[in] short_addr Source device short address
 * @param[in] endpoint Source endpoint
 * @param[in] cluster_id Cluster ID
 * @param[in] status ZCL status code
 * @param[in] attr_id Attribute ID (if available)
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_handle_configure_response(uint16_t short_addr,
                                                   uint8_t endpoint,
                                                   uint16_t cluster_id,
                                                   uint8_t status,
                                                   uint16_t attr_id);

/**
 * @brief Handle Read Reporting Configuration Response
 *
 * Processes the response from a Read Reporting Configuration command.
 * Called by the Zigbee callback handler.
 *
 * @param[in] short_addr Source device short address
 * @param[in] endpoint Source endpoint
 * @param[in] cluster_id Cluster ID
 * @param[in] status ZCL status code
 * @param[in] direction Reporting direction
 * @param[in] attr_id Attribute ID
 * @param[in] attr_type Attribute data type
 * @param[in] min_interval Minimum interval
 * @param[in] max_interval Maximum interval
 * @param[in] reportable_change Reportable change value
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_handle_read_response(uint16_t short_addr,
                                             uint8_t endpoint,
                                             uint16_t cluster_id,
                                             uint8_t status,
                                             uint8_t direction,
                                             uint16_t attr_id,
                                             uint8_t attr_type,
                                             uint16_t min_interval,
                                             uint16_t max_interval,
                                             uint16_t reportable_change);

/**
 * @brief Publish reporting configure response via MQTT
 *
 * Publishes the result of a configure operation to MQTT.
 * Topic: zigbee2mqtt/bridge/response/device/reporting/configure
 *
 * @param[in] result Reporting operation result
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_publish_configure_response(const zb_reporting_result_t *result);

/**
 * @brief Publish reporting read response via MQTT
 *
 * Publishes the result of a read operation to MQTT.
 * Topic: zigbee2mqtt/bridge/response/device/reporting/read
 *
 * @param[in] result Reporting operation result
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_publish_read_response(const zb_reporting_result_t *result);

/**
 * @brief Get stored reporting configuration
 *
 * Gets a stored reporting configuration entry by index.
 *
 * @param[in] index Index in the configuration table
 * @return Pointer to entry or NULL if not found/inactive
 */
const zb_reporting_entry_t* zb_reporting_get_entry(size_t index);

/**
 * @brief Get all reporting configurations for a device
 *
 * Copies all configurations for a specific device.
 *
 * @param[in] device_ieee Device IEEE address
 * @param[out] entries Destination array
 * @param[in] max_count Maximum entries to copy
 * @return Number of entries copied
 */
size_t zb_reporting_get_by_device(uint64_t device_ieee,
                                   zb_reporting_entry_t *entries,
                                   size_t max_count);

/**
 * @brief Get reporting configuration count
 *
 * @return Number of active configurations
 */
size_t zb_reporting_get_count(void);

/**
 * @brief Remove all reporting configurations for a device
 *
 * Removes all stored configurations when a device leaves.
 *
 * @param[in] device_ieee Device IEEE address
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_remove_device(uint64_t device_ieee);

/**
 * @brief Save reporting configurations to NVS
 *
 * Persists all configuration data to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_reporting_save_to_nvs(void);

/**
 * @brief Load reporting configurations from NVS
 *
 * Loads configuration data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NVS_NOT_FOUND if no saved data
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_reporting_load_from_nvs(void);

/**
 * @brief Clear all reporting configurations from NVS
 *
 * Removes all configuration data from non-volatile storage.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_clear_nvs(void);

/**
 * @brief Register reporting event callback
 *
 * Registers a callback function for reporting events.
 *
 * @param[in] callback Event callback function
 * @return ESP_OK on success
 */
esp_err_t zb_reporting_register_callback(zb_reporting_event_cb_t callback);

/**
 * @brief Get cluster name from ID
 *
 * Returns a human-readable cluster name for MQTT messages.
 *
 * @param[in] cluster_id Cluster ID
 * @return Cluster name string
 */
const char* zb_reporting_get_cluster_name(uint16_t cluster_id);

/**
 * @brief Get cluster ID from name
 *
 * Parses a cluster name string and returns the cluster ID.
 *
 * @param[in] name Cluster name
 * @return Cluster ID or 0xFFFF if not found
 */
uint16_t zb_reporting_get_cluster_id(const char *name);

/**
 * @brief Get attribute name from IDs
 *
 * Returns a human-readable attribute name.
 *
 * @param[in] cluster_id Cluster ID
 * @param[in] attr_id Attribute ID
 * @return Attribute name string
 */
const char* zb_reporting_get_attr_name(uint16_t cluster_id, uint16_t attr_id);

/**
 * @brief Get attribute ID from name
 *
 * Parses an attribute name and returns the ID.
 *
 * @param[in] cluster_id Cluster ID
 * @param[in] name Attribute name
 * @return Attribute ID or 0xFFFF if not found
 */
uint16_t zb_reporting_get_attr_id(uint16_t cluster_id, const char *name);

/**
 * @brief Get default reporting configuration for cluster/attribute
 *
 * Returns the default reporting settings for known attributes.
 *
 * @param[in] cluster_id Cluster ID
 * @param[in] attr_id Attribute ID
 * @return Pointer to default config or NULL if not found
 */
const zb_reporting_default_t* zb_reporting_get_default(uint16_t cluster_id,
                                                        uint16_t attr_id);

/**
 * @brief Self-test function for reporting configuration
 *
 * Tests configuration storage and retrieval.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_reporting_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_REPORTING_H */
