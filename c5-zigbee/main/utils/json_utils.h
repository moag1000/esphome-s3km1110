/**
 * @file json_utils.h
 * @brief JSON Building and Parsing Utilities
 *
 * Helper functions for creating and parsing JSON messages for Zigbee2MQTT.
 * Uses cJSON library from ESP-IDF.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "cJSON.h"
#include "zigbee/zb_device_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * IEEE Address Constants
 * ============================================================================ */

/** @brief IEEE address string length including "0x" prefix and null terminator (e.g., "0x00124B001234ABCD\0") */
#define IEEE_ADDR_STRING_LEN        19

/** @brief IEEE address hex digits length (without "0x" prefix, just the 16 hex chars) */
#define IEEE_ADDR_HEX_LEN           16

/** @brief IEEE address minimum buffer size (same as string length for safety) */
#define IEEE_ADDR_BUFFER_SIZE       19

/**
 * @brief Home Assistant component types
 */
typedef enum {
    HA_COMPONENT_LIGHT,
    HA_COMPONENT_SENSOR,
    HA_COMPONENT_BINARY_SENSOR,
    HA_COMPONENT_SWITCH,
    HA_COMPONENT_COVER,
    HA_COMPONENT_LOCK,
    HA_COMPONENT_CLIMATE,
    HA_COMPONENT_FAN
} ha_component_t;

/**
 * @brief Create device state JSON
 *
 * Builds a JSON object containing the current state of a Zigbee device.
 * Includes common attributes based on device type and clusters.
 *
 * @param[in] device Pointer to Zigbee device structure
 * @return cJSON object pointer (must be deleted by caller) or NULL on error
 */
cJSON* json_create_device_state(const zb_device_t *device);

/**
 * @brief Create bridge info JSON
 *
 * Builds a JSON object containing bridge information including:
 * - Version, commit ID
 * - Coordinator IEEE address
 * - Network info (PAN ID, channel, etc.)
 *
 * @return cJSON object pointer (must be deleted by caller) or NULL on error
 */
cJSON* json_create_bridge_info(void);

/**
 * @brief Create device list JSON
 *
 * Builds a JSON array containing all registered Zigbee devices
 * with their basic information.
 *
 * @return cJSON array pointer (must be deleted by caller) or NULL on error
 */
cJSON* json_create_device_list(void);

/**
 * @brief Create device availability JSON
 *
 * Creates a simple availability status JSON.
 *
 * @param[in] available true for "online", false for "offline"
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* json_create_availability(bool available);

/**
 * @brief Create bridge state JSON
 *
 * Creates bridge state message with status and timestamp.
 *
 * @param[in] state State string ("online", "offline", etc.)
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* json_create_bridge_state(const char *state);

/**
 * @brief Create Home Assistant discovery JSON
 *
 * Builds a complete Home Assistant MQTT discovery configuration message.
 *
 * @param[in] device Pointer to Zigbee device structure
 * @param[in] component HA component type
 * @return cJSON object pointer (must be deleted by caller) or NULL on error
 */
cJSON* json_create_ha_discovery(const zb_device_t *device, ha_component_t component);

/**
 * @brief Parse command JSON
 *
 * Parses incoming MQTT command JSON and extracts command parameters.
 *
 * @param[in] json_str JSON string to parse
 * @param[out] state Output: ON/OFF state (if present)
 * @param[out] brightness Output: Brightness level 0-254 (if present)
 * @param[out] color_x Output: Color X coordinate (if present)
 * @param[out] color_y Output: Color Y coordinate (if present)
 * @param[out] transition Output: Transition time in ms (if present)
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_command(const char *json_str,
                             bool *state,
                             uint8_t *brightness,
                             uint16_t *color_x,
                             uint16_t *color_y,
                             uint16_t *transition);

/**
 * @brief Parse permit join request
 *
 * Parses permit join request JSON.
 *
 * @param[in] json_str JSON string to parse
 * @param[out] duration Permit join duration in seconds
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_permit_join(const char *json_str, uint8_t *duration);

/**
 * @brief Extended permit join options structure
 */
typedef struct json_permit_join_options {
    bool value;                         /**< Enable/disable permit join */
    uint8_t time;                       /**< Duration in seconds (default 254) */
    bool has_device;                    /**< True if target device specified */
    uint8_t device_ieee[8];             /**< Target device IEEE address */
} json_permit_join_options_t;

/**
 * @brief Parse extended permit join request
 *
 * Parses permit join request JSON with optional time and device fields.
 * Supports formats:
 * - {"value": true}
 * - {"value": true, "time": 254}
 * - {"value": true, "device": "0x00124b001234abcd"}
 * - {"value": false}
 *
 * @param[in] json_str JSON string to parse
 * @param[out] options Parsed options
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_permit_join_extended(const char *json_str, json_permit_join_options_t *options);

/**
 * @brief Parse device remove request
 *
 * Parses device remove request JSON.
 *
 * @param[in] json_str JSON string to parse
 * @param[out] friendly_name Buffer for friendly name (min 64 bytes)
 * @param[in] buf_len Length of friendly_name buffer
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_device_remove(const char *json_str, char *friendly_name, size_t buf_len);

/**
 * @brief Parse device rename request
 *
 * Parses device rename request JSON.
 *
 * @param[in] json_str JSON string to parse
 * @param[out] old_name Buffer for old friendly name (min 64 bytes)
 * @param[out] new_name Buffer for new friendly name (min 64 bytes)
 * @param[in] buf_len Length of name buffers
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_device_rename(const char *json_str,
                                   char *old_name,
                                   char *new_name,
                                   size_t buf_len);

/**
 * @brief Convert cJSON object to string
 *
 * Helper to convert cJSON object to string and delete the object.
 * Caller must free the returned string.
 *
 * @param[in] json cJSON object (will be deleted)
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* json_to_string_and_delete(cJSON *json);

/**
 * @brief Get component type string
 *
 * Returns the Home Assistant component type as a string.
 *
 * @param[in] component Component type enum
 * @return Component type string (e.g., "light", "sensor")
 */
const char* json_get_component_string(ha_component_t component);

/**
 * @brief Create IEEE address string
 *
 * Formats IEEE address as hex string (e.g., "0x00124b001234abcd")
 *
 * @param[in] ieee_addr IEEE address array
 * @param[out] str_buf Buffer for output string (min 19 bytes)
 * @param[in] buf_len Length of output buffer
 * @return ESP_OK on success
 */
esp_err_t json_format_ieee_addr(const uint8_t ieee_addr[8], char *str_buf, size_t buf_len);

/**
 * @brief Parse door lock command JSON
 *
 * Parses incoming MQTT door lock command JSON.
 * Format: {"state": "LOCK"} or {"state": "UNLOCK"}
 *
 * @param[in] json_str JSON string to parse
 * @param[out] lock_cmd Output: true for LOCK, false for UNLOCK
 * @return ESP_OK on success, ESP_FAIL on parse error
 */
esp_err_t json_parse_lock_command(const char *json_str, bool *lock_cmd);

/**
 * @brief Create door lock state JSON string
 *
 * Creates JSON state message for door lock.
 * Format: {"state": "LOCK"} or {"state": "UNLOCK"}
 *
 * @param[in] locked true if locked, false if unlocked
 * @return JSON string (must be freed by caller) or NULL on error
 */
char* json_create_lock_state(bool locked);

/**
 * @brief Test JSON utilities
 *
 * Performs basic JSON creation and parsing tests.
 *
 * @return ESP_OK if tests pass, ESP_FAIL otherwise
 */
esp_err_t json_utils_test(void);

#ifdef __cplusplus
}
#endif

#endif /* JSON_UTILS_H */
