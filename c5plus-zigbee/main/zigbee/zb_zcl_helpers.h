/**
 * @file zb_zcl_helpers.h
 * @brief Zigbee ZCL Command Helper Functions
 *
 * Provides convenience functions to reduce code duplication in ZCL command handling.
 * All functions automatically manage Zigbee stack locks to ensure thread-safe API access.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_ZCL_HELPERS_H
#define ZB_ZCL_HELPERS_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute a ZCL on/off command with automatic lock handling
 *
 * Sends a ZCL on/off command to a target device endpoint. This function
 * acquires the Zigbee stack lock before calling the ESP-Zigbee API and
 * releases it after the command is queued.
 *
 * @param[in] short_addr    Target device short address
 * @param[in] endpoint      Target endpoint number
 * @param[in] on            true to turn ON, false to turn OFF
 * @return ESP_OK on success, or error code if command queuing failed
 */
esp_err_t zb_zcl_send_on_off_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                           bool on);

/**
 * @brief Execute a ZCL level control move-to-level command with automatic lock handling
 *
 * Sends a ZCL level control command to set brightness/level on a target device.
 * This function acquires the Zigbee stack lock before calling the ESP-Zigbee API.
 *
 * @param[in] short_addr     Target device short address
 * @param[in] endpoint       Target endpoint number
 * @param[in] level          Target level (0-254)
 * @param[in] transition_time Transition time in 1/10 second increments
 * @return ESP_OK on success, or error code if command queuing failed
 */
esp_err_t zb_zcl_send_level_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                          uint8_t level, uint16_t transition_time);

/**
 * @brief Execute a ZCL color move-to-color command with automatic lock handling
 *
 * Sends a ZCL color control command to set color (x, y) on a target device.
 * This function acquires the Zigbee stack lock before calling the ESP-Zigbee API.
 *
 * @param[in] short_addr     Target device short address
 * @param[in] endpoint       Target endpoint number
 * @param[in] color_x        CIE 1931 x coordinate (0-65535)
 * @param[in] color_y        CIE 1931 y coordinate (0-65535)
 * @param[in] transition_time Transition time in 1/10 second increments
 * @return ESP_OK on success, or error code if command queuing failed
 */
esp_err_t zb_zcl_send_color_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                          uint16_t color_x, uint16_t color_y,
                                          uint16_t transition_time);

/**
 * @brief Execute a ZCL read attributes command with automatic lock handling
 *
 * Sends a ZCL read attributes command to a target device endpoint to query
 * attribute values. This function acquires the Zigbee stack lock before
 * calling the ESP-Zigbee API.
 *
 * @param[in] short_addr     Target device short address
 * @param[in] endpoint       Target endpoint number
 * @param[in] cluster_id     Target cluster ID
 * @param[in] attr_ids       Array of attribute IDs to read
 * @param[in] attr_count     Number of attributes in the array
 * @return ESP_OK on success, or error code if command queuing failed
 */
esp_err_t zb_zcl_read_attr_with_lock(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t cluster_id, const uint16_t *attr_ids,
                                     uint16_t attr_count);

/**
 * @brief Execute a ZCL write attributes command with automatic lock handling
 *
 * Sends a ZCL write attributes command to a target device endpoint to set
 * attribute values. This function acquires the Zigbee stack lock before
 * calling the ESP-Zigbee API.
 *
 * @param[in] short_addr     Target device short address
 * @param[in] endpoint       Target endpoint number
 * @param[in] cluster_id     Target cluster ID
 * @param[in] attr_field     Array of attribute write records
 * @param[in] attr_count     Number of attributes to write
 * @return ESP_OK on success, or error code if command queuing failed
 */
esp_err_t zb_zcl_write_attr_with_lock(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t cluster_id,
                                      esp_zb_zcl_attribute_t *attr_field,
                                      uint16_t attr_count);

/**
 * @brief Execute a generic ZCL command with automatic lock handling
 *
 * Generic helper for executing any ZCL command through the standard request
 * path. This function acquires the Zigbee stack lock before calling the
 * ESP-Zigbee API.
 *
 * @param[in] cluster_id     Target cluster ID
 * @param[in] cmd_id         Command ID within the cluster
 * @param[in] payload        Command payload (cluster-specific)
 * @return ESP_OK on success, or error code if command queuing failed
 *
 * @note This is a lower-level function. Prefer specific command functions
 *       (e.g., zb_zcl_send_on_off_cmd_with_lock) when available.
 */
esp_err_t zb_zcl_send_cmd_with_lock(uint16_t cluster_id, uint8_t cmd_id,
                                    void *payload);

/**
 * @brief Initialize ZCL helper module
 *
 * Performs any initialization required for the helper functions.
 * Currently a placeholder for future extensibility.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_zcl_helpers_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_ZCL_HELPERS_H */
