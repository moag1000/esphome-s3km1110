/**
 * @file zb_device_handler_internal.h
 * @brief Internal API for Zigbee Device Handler - DO NOT USE OUTSIDE ZIGBEE MODULE
 *
 * This header contains internal functions that are not part of the public API.
 * They are used by other files within the zigbee/ module (zb_callbacks.c,
 * zb_cluster_*.c, etc.) but should not be included by external modules.
 *
 * For public device management API, use zb_device_handler.h instead.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_DEVICE_HANDLER_INTERNAL_H
#define ZB_DEVICE_HANDLER_INTERNAL_H

#include "zb_device_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Device Handler Internal Access Functions
 *
 * These functions provide low-level access to the device registry for use
 * by other zigbee module files. External modules should use the public API.
 * ============================================================================ */

/**
 * @internal
 * @brief Get the device mutex for thread-safe access
 *
 * Used by cluster implementations that need to perform multiple operations
 * atomically on the device registry.
 *
 * @return SemaphoreHandle_t Device mutex handle
 */
SemaphoreHandle_t zb_device_get_mutex(void);

/**
 * @internal
 * @brief Check if device handler is initialized
 *
 * @return true if initialized, false otherwise
 */
bool zb_device_handler_is_initialized(void);

/**
 * @internal
 * @brief Check if a device has a specific cluster (thread-safe)
 *
 * This is the internal version that may be called with or without
 * the device mutex held.
 *
 * @param[in] short_addr Device short address
 * @param[in] cluster_id Cluster ID to check
 * @return true if device has the cluster
 */
bool zb_device_has_cluster_internal(uint16_t short_addr, uint16_t cluster_id);

/**
 * @internal
 * @brief Find device by short address (requires mutex held by caller)
 *
 * This function is NOT thread-safe and requires the caller to hold
 * the device mutex obtained via zb_device_get_mutex().
 *
 * @param[in] short_addr Device short address
 * @return Pointer to device or NULL if not found
 */
zb_device_t *zb_device_find_by_short_addr_unlocked(uint16_t short_addr);

/* ============================================================================
 * Attribute Report Handlers
 *
 * These functions are called by zb_callbacks.c to process incoming ZCL
 * attribute reports from devices. They update internal state and trigger
 * registered callbacks. Not intended for use outside the zigbee module.
 * ============================================================================ */

/**
 * @internal
 * @brief Handle Binary Output attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_binary_output_handle_report(uint16_t short_addr, uint8_t endpoint,
                                          uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Binary Value attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_binary_value_handle_report(uint16_t short_addr, uint8_t endpoint,
                                         uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Multistate Input/Output/Value attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] cluster_id Cluster ID (input, output, or value)
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_multistate_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t cluster_id, uint16_t attr_id,
                                       void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Window Covering attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_handle_report(uint16_t short_addr, uint8_t endpoint,
                                            uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Door Lock attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_door_lock_handle_report(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Thermostat attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Fan Control attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_fan_control_handle_report(uint16_t short_addr, uint8_t endpoint,
                                        uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Illuminance Measurement attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_illuminance_handle_report(uint16_t short_addr, uint8_t endpoint,
                                        uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Pressure Measurement attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_pressure_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle PM2.5 Measurement attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_pm25_handle_report(uint16_t short_addr, uint8_t endpoint,
                                 uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Electrical Measurement attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_electrical_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Metering (Smart Energy) attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_metering_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle IAS Zone attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_handle_report(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t attr_id, void *value, size_t value_len);

/**
 * @internal
 * @brief Handle Dehumidification Control attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_handle_report(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t attr_id, void *value, size_t value_len);

#ifdef __cplusplus
}
#endif

#endif /* ZB_DEVICE_HANDLER_INTERNAL_H */
