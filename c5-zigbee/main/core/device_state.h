/**
 * @file device_state.h
 * @brief Device State to JSON Conversion
 *
 * Converts Zigbee device state to cJSON objects for UART transmission.
 * Simplified replacement for device_state_publisher (which used MQTT).
 */

#ifndef DEVICE_STATE_H
#define DEVICE_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"
#include "zigbee/zb_device_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert device to cJSON object
 *
 * Creates a cJSON object with all available device state fields.
 * Caller must free with cJSON_Delete().
 *
 * @param device Pointer to device structure
 * @return cJSON object or NULL on error
 */
cJSON *device_state_to_cjson(const zb_device_t *device);

/**
 * @brief Convert device to JSON string
 *
 * Creates a JSON string with all available device state fields.
 * Caller must free the returned string.
 *
 * @param device Pointer to device structure
 * @return JSON string or NULL on error
 */
char *device_state_to_json_str(const zb_device_t *device);

/**
 * @brief Publish device state by address (via UART bridge)
 *
 * Looks up device by short address and publishes state over UART.
 * This function replaces device_state_publish_by_addr() from the MQTT version.
 *
 * @param short_addr Device short address
 * @return ESP_OK on success
 */
esp_err_t device_state_publish_by_addr(uint16_t short_addr);

/**
 * @brief Publish multistate state (via UART bridge)
 */
esp_err_t device_state_publish_multistate(uint16_t short_addr, uint8_t endpoint,
                                           const zb_multistate_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_STATE_H */
