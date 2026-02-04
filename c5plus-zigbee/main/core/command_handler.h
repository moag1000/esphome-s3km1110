/**
 * @file command_handler.h
 * @brief MQTT Command Handler for Zigbee Device Control
 *
 * Handles incoming MQTT commands and translates them to Zigbee ZCL commands.
 * Supports lights, switches, and other controllable devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize command handler
 *
 * Must be called before processing any commands.
 *
 * @return ESP_OK on success
 */
esp_err_t command_handler_init(void);

/**
 * @brief Process incoming MQTT command
 *
 * Main entry point for handling MQTT commands. Parses topic to extract
 * device friendly name, parses JSON payload, and sends appropriate ZCL commands.
 *
 * Expected topic format: "zigbee2mqtt/[friendly_name]/set"
 * Expected payload: JSON with command parameters
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if topic or payload invalid
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t command_handler_process(const char *topic, const char *payload, size_t len);

/**
 * @brief Send On/Off command to device
 *
 * Sends ZCL On/Off cluster command to a Zigbee device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] on true for ON, false for OFF
 * @return ESP_OK on success
 */
esp_err_t command_send_on_off(uint16_t short_addr, uint8_t endpoint, bool on);

/**
 * @brief Send Level (brightness) command to device
 *
 * Sends ZCL Level Control "Move to Level" command.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] level Brightness level (0-254)
 * @param[in] transition_time Transition time in 1/10 seconds (0 = instant)
 * @return ESP_OK on success
 */
esp_err_t command_send_level(uint16_t short_addr, uint8_t endpoint,
                             uint8_t level, uint16_t transition_time);

/**
 * @brief Send Color XY command to device
 *
 * Sends ZCL Color Control "Move to Color" command.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] x X chromaticity coordinate (0-65535)
 * @param[in] y Y chromaticity coordinate (0-65535)
 * @param[in] transition_time Transition time in 1/10 seconds (0 = instant)
 * @return ESP_OK on success
 */
esp_err_t command_send_color(uint16_t short_addr, uint8_t endpoint,
                             uint16_t x, uint16_t y, uint16_t transition_time);

/**
 * @brief Send toggle command to device
 *
 * Sends ZCL On/Off "Toggle" command.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t command_send_toggle(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Lock Door command to device
 *
 * Sends ZCL Door Lock "Lock Door" command (0x00).
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t command_send_lock(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Unlock Door command to device
 *
 * Sends ZCL Door Lock "Unlock Door" command (0x01).
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t command_send_unlock(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Reset Alarm command to device
 *
 * Sends ZCL Alarms cluster "Reset Alarm" command (0x00).
 * Clears a specific alarm from the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] alarm_code Alarm code to reset
 * @param[in] cluster_id Cluster ID that generated the alarm
 * @return ESP_OK on success
 */
esp_err_t command_send_alarm_reset(uint16_t short_addr, uint8_t endpoint,
                                    uint8_t alarm_code, uint16_t cluster_id);

/**
 * @brief Send Reset All Alarms command to device
 *
 * Sends ZCL Alarms cluster "Reset All Alarms" command (0x01).
 * Clears all alarms from the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t command_send_alarm_reset_all(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Get command handler statistics
 *
 * Returns number of commands processed and errors.
 *
 * @param[out] processed Total commands processed
 * @param[out] errors Total command errors
 * @return ESP_OK on success
 */
esp_err_t command_handler_get_stats(uint32_t *processed, uint32_t *errors);

/**
 * @brief Test command handler
 *
 * Performs basic command parsing tests.
 *
 * @return ESP_OK if tests pass, ESP_FAIL otherwise
 */
esp_err_t command_handler_test(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_HANDLER_H */
