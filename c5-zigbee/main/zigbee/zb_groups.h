/**
 * @file zb_groups.h
 * @brief Zigbee Group Management API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee group management functionality including:
 * - Creating and removing groups
 * - Adding and removing devices from groups
 * - Sending commands to all devices in a group
 * - NVS persistence for group configuration
 * - MQTT integration for Zigbee2MQTT compatibility
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/group/add - Create group
 * - zigbee2mqtt/bridge/request/group/remove - Delete group
 * - zigbee2mqtt/bridge/request/group/members/add - Add device to group
 * - zigbee2mqtt/bridge/request/group/members/remove - Remove device from group
 * - zigbee2mqtt/group/[name]/set - Send command to group
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_GROUPS_H
#define ZB_GROUPS_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of groups supported
 */
#define ZB_GROUPS_MAX_COUNT 16

/**
 * @brief Maximum number of members per group
 */
#define ZB_GROUP_MAX_MEMBERS 32

/**
 * @brief Maximum length of group name (including null terminator)
 */
#define ZB_GROUP_NAME_MAX_LEN 32

/**
 * @brief Starting group ID (avoids reserved IDs)
 */
#define ZB_GROUP_ID_START 0x0001

/**
 * @brief Maximum group ID
 */
#define ZB_GROUP_ID_MAX 0xFFF7

/**
 * @brief Invalid group ID marker
 */
#define ZB_GROUP_ID_INVALID 0xFFFF

/**
 * @brief NVS namespace for group storage
 */
#define ZB_GROUPS_NVS_NAMESPACE "zb_groups"

/**
 * @brief Group command types
 */
typedef enum {
    ZB_GROUP_CMD_ON,           /**< Turn all devices in group ON */
    ZB_GROUP_CMD_OFF,          /**< Turn all devices in group OFF */
    ZB_GROUP_CMD_TOGGLE,       /**< Toggle all devices in group */
    ZB_GROUP_CMD_LEVEL,        /**< Set brightness level for group */
    ZB_GROUP_CMD_COLOR,        /**< Set color for group */
    ZB_GROUP_CMD_COLOR_TEMP,   /**< Set color temperature for group */
    ZB_GROUP_CMD_SCENE_RECALL, /**< Recall scene for group */
    ZB_GROUP_CMD_SCENE_STORE   /**< Store current state as scene */
} zb_group_cmd_type_t;

/**
 * @brief Group command parameters
 */
typedef struct {
    zb_group_cmd_type_t cmd_type;  /**< Command type */
    union {
        struct {
            uint8_t level;          /**< Brightness level (0-254) */
            uint16_t transition;    /**< Transition time (1/10 sec) */
        } level;
        struct {
            uint16_t color_x;       /**< Color X coordinate */
            uint16_t color_y;       /**< Color Y coordinate */
            uint16_t transition;    /**< Transition time (1/10 sec) */
        } color;
        struct {
            uint16_t color_temp;    /**< Color temperature (mireds) */
            uint16_t transition;    /**< Transition time (1/10 sec) */
        } color_temp;
        struct {
            uint8_t scene_id;       /**< Scene ID */
        } scene;
    } params;
} zb_group_cmd_t;

/**
 * @brief Zigbee group structure
 */
typedef struct {
    uint16_t group_id;                          /**< Zigbee group ID */
    char name[ZB_GROUP_NAME_MAX_LEN];           /**< Group friendly name */
    uint64_t members[ZB_GROUP_MAX_MEMBERS];     /**< IEEE addresses of members */
    uint8_t member_count;                       /**< Number of members in group */
    bool active;                                /**< Group is active/valid */
} zb_group_t;

/**
 * @brief Group event callback type
 */
typedef void (*zb_group_event_cb_t)(uint16_t group_id, const char *event_type);

/**
 * @brief Initialize group management
 *
 * Initializes the group management module, allocates resources,
 * and loads existing groups from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_groups_init(void);

/**
 * @brief Deinitialize group management
 *
 * Saves groups to NVS and frees all resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_groups_deinit(void);

/**
 * @brief Create a new group
 *
 * Creates a new Zigbee group with the specified name.
 * The group ID is automatically assigned.
 *
 * @param[in] name Group friendly name (max ZB_GROUP_NAME_MAX_LEN-1 chars)
 * @param[out] group_id Output: assigned group ID (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if maximum groups reached
 * @return ESP_ERR_INVALID_ARG if name is NULL or empty
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_groups_create(const char *name, uint16_t *group_id);

/**
 * @brief Remove a group
 *
 * Removes the group with the specified ID. All members are removed
 * from the group before deletion.
 *
 * @param[in] group_id Group ID to remove
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_groups_remove(uint16_t group_id);

/**
 * @brief Remove a group by name
 *
 * Removes the group with the specified name.
 *
 * @param[in] name Group name
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 * @return ESP_ERR_INVALID_ARG if name is NULL
 */
esp_err_t zb_groups_remove_by_name(const char *name);

/**
 * @brief Add a device to a group
 *
 * Adds the device with the specified IEEE address to the group.
 * Sends the ZCL Add Group command to the device.
 *
 * @param[in] group_id Group ID
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 * @return ESP_ERR_NO_MEM if group is full
 * @return ESP_ERR_INVALID_ARG if device already in group
 */
esp_err_t zb_groups_add_member(uint16_t group_id, uint64_t ieee_addr);

/**
 * @brief Remove a device from a group
 *
 * Removes the device with the specified IEEE address from the group.
 * Sends the ZCL Remove Group command to the device.
 *
 * @param[in] group_id Group ID
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group or member not found
 */
esp_err_t zb_groups_remove_member(uint16_t group_id, uint64_t ieee_addr);

/**
 * @brief Send command to group
 *
 * Sends a ZCL command to all devices in the group using group addressing.
 *
 * @param[in] group_id Group ID
 * @param[in] cmd Command to send
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 * @return ESP_ERR_INVALID_ARG if cmd is NULL
 */
esp_err_t zb_groups_send_command(uint16_t group_id, const zb_group_cmd_t *cmd);

/**
 * @brief Send On/Off command to group
 *
 * Convenience function to send On/Off command to a group.
 *
 * @param[in] group_id Group ID
 * @param[in] on true for ON, false for OFF
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 */
esp_err_t zb_groups_send_on_off(uint16_t group_id, bool on);

/**
 * @brief Send toggle command to group
 *
 * Convenience function to send toggle command to a group.
 *
 * @param[in] group_id Group ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 */
esp_err_t zb_groups_send_toggle(uint16_t group_id);

/**
 * @brief Send level command to group
 *
 * Convenience function to send brightness level to a group.
 *
 * @param[in] group_id Group ID
 * @param[in] level Brightness level (0-254)
 * @param[in] transition_time Transition time in 1/10 seconds
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 */
esp_err_t zb_groups_send_level(uint16_t group_id, uint8_t level, uint16_t transition_time);

/**
 * @brief Get group by ID
 *
 * Returns a pointer to the group structure. The pointer is valid
 * until the group is removed or module is deinitialized.
 *
 * @param[in] group_id Group ID
 * @return Pointer to group structure or NULL if not found
 */
const zb_group_t* zb_groups_get(uint16_t group_id);

/**
 * @brief Get group by name
 *
 * Returns a pointer to the group structure by name.
 *
 * @param[in] name Group name
 * @return Pointer to group structure or NULL if not found
 */
const zb_group_t* zb_groups_get_by_name(const char *name);

/**
 * @brief Get all groups
 *
 * Copies all active groups to the provided array.
 *
 * @param[out] groups Destination array (must have space for ZB_GROUPS_MAX_COUNT)
 * @param[in] max_count Maximum number of groups to copy
 * @return Number of groups copied
 */
size_t zb_groups_get_all(zb_group_t *groups, size_t max_count);

/**
 * @brief Get group count
 *
 * @return Number of active groups
 */
size_t zb_groups_get_count(void);

/**
 * @brief Check if device is member of group
 *
 * @param[in] group_id Group ID
 * @param[in] ieee_addr Device IEEE address
 * @return true if device is member, false otherwise
 */
bool zb_groups_is_member(uint16_t group_id, uint64_t ieee_addr);

/**
 * @brief Get all groups a device belongs to
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[out] group_ids Array to store group IDs
 * @param[in] max_count Maximum number of group IDs to return
 * @return Number of group IDs stored
 */
size_t zb_groups_get_device_groups(uint64_t ieee_addr, uint16_t *group_ids, size_t max_count);

/**
 * @brief Rename a group
 *
 * Changes the friendly name of a group.
 *
 * @param[in] group_id Group ID
 * @param[in] new_name New group name
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 * @return ESP_ERR_INVALID_ARG if name is NULL or too long
 */
esp_err_t zb_groups_rename(uint16_t group_id, const char *new_name);

/**
 * @brief Save groups to NVS
 *
 * Persists all group data to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_groups_save_to_nvs(void);

/**
 * @brief Load groups from NVS
 *
 * Loads group data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NVS_NOT_FOUND if no saved data
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_groups_load_from_nvs(void);

/**
 * @brief Clear all groups from NVS
 *
 * Removes all group data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_groups_clear_nvs(void);

/**
 * @brief Process MQTT group request
 *
 * Main entry point for handling group-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/group/add
 * - zigbee2mqtt/bridge/request/group/remove
 * - zigbee2mqtt/bridge/request/group/members/add
 * - zigbee2mqtt/bridge/request/group/members/remove
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request
 */
esp_err_t zb_groups_process_mqtt_request(const char *topic, const char *payload, size_t len);

/**
 * @brief Process MQTT group command
 *
 * Handles group set commands from topic: zigbee2mqtt/group/[name]/set
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 */
esp_err_t zb_groups_process_mqtt_command(const char *topic, const char *payload, size_t len);

/**
 * @brief Publish group state via MQTT
 *
 * Publishes the current state of a group to MQTT.
 * Topic: zigbee2mqtt/group/[name]
 *
 * @param[in] group_id Group ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if group not found
 */
esp_err_t zb_groups_publish_state(uint16_t group_id);

/**
 * @brief Publish all groups list via MQTT
 *
 * Publishes the list of all groups to MQTT.
 * Topic: zigbee2mqtt/bridge/groups
 *
 * @return ESP_OK on success
 */
esp_err_t zb_groups_publish_list(void);

/**
 * @brief Register group event callback
 *
 * Registers a callback function that is called when group events occur.
 *
 * @param[in] callback Event callback function
 * @return ESP_OK on success
 */
esp_err_t zb_groups_register_callback(zb_group_event_cb_t callback);

/**
 * @brief Self-test function for group management
 *
 * Tests group creation, member management, and command sending.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_groups_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_GROUPS_H */
