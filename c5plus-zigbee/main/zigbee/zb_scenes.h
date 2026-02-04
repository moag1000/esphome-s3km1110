/**
 * @file zb_scenes.h
 * @brief Zigbee Scene Management API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee scene management functionality including:
 * - Creating and removing scenes (stored device state combinations)
 * - Scene store (save current state) and recall (restore state)
 * - NVS persistence for scene configuration
 * - MQTT integration for Zigbee2MQTT compatibility
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/scene/add - Create scene
 * - zigbee2mqtt/bridge/request/scene/remove - Delete scene
 * - zigbee2mqtt/bridge/request/scene/store - Store current state in scene
 * - zigbee2mqtt/bridge/request/scene/recall - Recall scene
 * - zigbee2mqtt/[group_name]/scene_recall - Recall scene for group
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_SCENES_H
#define ZB_SCENES_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of scenes supported
 */
#define ZB_SCENES_MAX_COUNT 16

/**
 * @brief Maximum number of devices per scene
 */
#define ZB_SCENE_MAX_DEVICES 16

/**
 * @brief Maximum length of scene name (including null terminator)
 */
#define ZB_SCENE_NAME_MAX_LEN 32

/**
 * @brief Starting scene ID (avoids reserved IDs)
 */
#define ZB_SCENE_ID_START 0x01

/**
 * @brief Maximum scene ID
 */
#define ZB_SCENE_ID_MAX 0xFE

/**
 * @brief Invalid scene ID marker
 */
#define ZB_SCENE_ID_INVALID 0xFF

/**
 * @brief NVS namespace for scene storage
 */
#define ZB_SCENES_NVS_NAMESPACE "zb_scenes"

/**
 * @brief Default transition time in 1/10 seconds
 */
#define ZB_SCENE_DEFAULT_TRANSITION 10

/**
 * @brief Device state stored within a scene
 */
typedef struct {
    uint64_t ieee_addr;         /**< Device IEEE address */
    uint8_t endpoint;           /**< Device endpoint */
    bool on_off_state;          /**< On/Off state */
    uint8_t level;              /**< Level (brightness 0-254) */
    uint16_t color_x;           /**< Color X coordinate (0-65535) */
    uint16_t color_y;           /**< Color Y coordinate (0-65535) */
    uint16_t color_temp;        /**< Color temperature in mireds */
} zb_scene_device_state_t;

/**
 * @brief Zigbee scene structure
 */
typedef struct {
    uint8_t scene_id;                                   /**< Scene ID (1-254) */
    uint16_t group_id;                                  /**< Associated group ID */
    char name[ZB_SCENE_NAME_MAX_LEN];                   /**< Scene friendly name */
    zb_scene_device_state_t devices[ZB_SCENE_MAX_DEVICES]; /**< Device states */
    uint8_t device_count;                               /**< Number of devices in scene */
    uint16_t transition_time;                           /**< Transition time in 1/10 seconds */
    bool active;                                        /**< Entry in use */
} zb_scene_t;

/**
 * @brief Scene event types
 */
typedef enum {
    ZB_SCENE_EVENT_CREATED,     /**< Scene was created */
    ZB_SCENE_EVENT_REMOVED,     /**< Scene was removed */
    ZB_SCENE_EVENT_STORED,      /**< Device states stored in scene */
    ZB_SCENE_EVENT_RECALLED,    /**< Scene was recalled */
    ZB_SCENE_EVENT_UPDATED      /**< Scene was updated */
} zb_scene_event_type_t;

/**
 * @brief Scene event callback type
 */
typedef void (*zb_scene_event_cb_t)(uint8_t scene_id, zb_scene_event_type_t event_type);

/**
 * @brief Initialize scene management
 *
 * Initializes the scene management module, allocates resources,
 * and loads existing scenes from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_scenes_init(void);

/**
 * @brief Deinitialize scene management
 *
 * Saves scenes to NVS and frees all resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_scenes_deinit(void);

/**
 * @brief Create a new scene
 *
 * Creates a new Zigbee scene associated with the specified group.
 * The scene ID is automatically assigned.
 *
 * @param[in] group_id Group ID this scene belongs to (0 for no group)
 * @param[in] name Scene friendly name (max ZB_SCENE_NAME_MAX_LEN-1 chars)
 * @param[out] scene_id Output: assigned scene ID (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if maximum scenes reached
 * @return ESP_ERR_INVALID_ARG if name is NULL or empty
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_scenes_create(uint16_t group_id, const char *name, uint8_t *scene_id);

/**
 * @brief Remove a scene
 *
 * Removes the scene with the specified ID.
 *
 * @param[in] scene_id Scene ID to remove
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_scenes_remove(uint8_t scene_id);

/**
 * @brief Remove a scene by name
 *
 * Removes the scene with the specified name.
 *
 * @param[in] name Scene name
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_INVALID_ARG if name is NULL
 */
esp_err_t zb_scenes_remove_by_name(const char *name);

/**
 * @brief Store current device states in scene
 *
 * Captures the current state of all devices in the associated group
 * and stores them in the scene.
 *
 * @param[in] scene_id Scene ID to store states in
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_scenes_store(uint8_t scene_id);

/**
 * @brief Store specific device state in scene
 *
 * Stores the current state of a specific device in the scene.
 *
 * @param[in] scene_id Scene ID
 * @param[in] ieee_addr Device IEEE address
 * @param[in] endpoint Device endpoint
 * @param[in] state Device state to store
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_NO_MEM if scene device limit reached
 */
esp_err_t zb_scenes_store_device(uint8_t scene_id, uint64_t ieee_addr,
                                  uint8_t endpoint, const zb_scene_device_state_t *state);

/**
 * @brief Recall a scene
 *
 * Recalls the scene and applies stored states to all devices.
 * Sends ZCL commands to each device in the scene.
 *
 * @param[in] scene_id Scene ID to recall
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_scenes_recall(uint8_t scene_id);

/**
 * @brief Recall scene by name
 *
 * Recalls the scene with the specified name.
 *
 * @param[in] name Scene name
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 */
esp_err_t zb_scenes_recall_by_name(const char *name);

/**
 * @brief Get scene by ID
 *
 * Returns a pointer to the scene structure. The pointer is valid
 * until the scene is removed or module is deinitialized.
 *
 * @param[in] scene_id Scene ID
 * @return Pointer to scene structure or NULL if not found
 */
const zb_scene_t* zb_scenes_get(uint8_t scene_id);

/**
 * @brief Get scene by name
 *
 * Returns a pointer to the scene structure by name.
 *
 * @param[in] name Scene name
 * @return Pointer to scene structure or NULL if not found
 */
const zb_scene_t* zb_scenes_get_by_name(const char *name);

/**
 * @brief Get all scenes for a group
 *
 * Copies all scenes belonging to the specified group to the provided array.
 *
 * @param[in] group_id Group ID (0 for scenes without group)
 * @param[out] scenes Destination array
 * @param[in] max_count Maximum number of scenes to copy
 * @return Number of scenes copied
 */
size_t zb_scenes_get_by_group(uint16_t group_id, zb_scene_t *scenes, size_t max_count);

/**
 * @brief Get all scenes
 *
 * Copies all active scenes to the provided array.
 *
 * @param[out] scenes Destination array (must have space for ZB_SCENES_MAX_COUNT)
 * @param[in] max_count Maximum number of scenes to copy
 * @return Number of scenes copied
 */
size_t zb_scenes_get_all(zb_scene_t *scenes, size_t max_count);

/**
 * @brief Get scene count
 *
 * @return Number of active scenes
 */
size_t zb_scenes_get_count(void);

/**
 * @brief Rename a scene
 *
 * Changes the friendly name of a scene.
 *
 * @param[in] scene_id Scene ID
 * @param[in] new_name New scene name
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 * @return ESP_ERR_INVALID_ARG if name is NULL or too long
 */
esp_err_t zb_scenes_rename(uint8_t scene_id, const char *new_name);

/**
 * @brief Set scene transition time
 *
 * Sets the transition time used when recalling the scene.
 *
 * @param[in] scene_id Scene ID
 * @param[in] transition_time Transition time in 1/10 seconds
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 */
esp_err_t zb_scenes_set_transition(uint8_t scene_id, uint16_t transition_time);

/**
 * @brief Save scenes to NVS
 *
 * Persists all scene data to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_scenes_save_to_nvs(void);

/**
 * @brief Load scenes from NVS
 *
 * Loads scene data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NVS_NOT_FOUND if no saved data
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_scenes_load_from_nvs(void);

/**
 * @brief Clear all scenes from NVS
 *
 * Removes all scene data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_scenes_clear_nvs(void);

/**
 * @brief Process MQTT scene request
 *
 * Main entry point for handling scene-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/scene/add
 * - zigbee2mqtt/bridge/request/scene/remove
 * - zigbee2mqtt/bridge/request/scene/store
 * - zigbee2mqtt/bridge/request/scene/recall
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request
 */
esp_err_t zb_scenes_process_mqtt_request(const char *topic, const char *payload, size_t len);

/**
 * @brief Process MQTT scene recall for group
 *
 * Handles scene recall from topic: zigbee2mqtt/[group_name]/scene_recall
 *
 * @param[in] topic MQTT topic
 * @param[in] payload Scene name or ID
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 */
esp_err_t zb_scenes_process_mqtt_recall(const char *topic, const char *payload, size_t len);

/**
 * @brief Publish scene state via MQTT
 *
 * Publishes the current state of a scene to MQTT.
 * Topic: zigbee2mqtt/bridge/scene/[name]
 *
 * @param[in] scene_id Scene ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if scene not found
 */
esp_err_t zb_scenes_publish_state(uint8_t scene_id);

/**
 * @brief Publish all scenes list via MQTT
 *
 * Publishes the list of all scenes to MQTT.
 * Topic: zigbee2mqtt/bridge/scenes
 *
 * @return ESP_OK on success
 */
esp_err_t zb_scenes_publish_list(void);

/**
 * @brief Register scene event callback
 *
 * Registers a callback function that is called when scene events occur.
 *
 * @param[in] callback Event callback function
 * @return ESP_OK on success
 */
esp_err_t zb_scenes_register_callback(zb_scene_event_cb_t callback);

/**
 * @brief Self-test function for scene management
 *
 * Tests scene creation, store, recall, and removal.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_scenes_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_SCENES_H */
