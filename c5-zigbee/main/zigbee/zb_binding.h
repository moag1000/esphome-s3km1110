/**
 * @file zb_binding.h
 * @brief Zigbee Binding Management API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee binding management functionality including:
 * - Creating and removing device-to-device bindings
 * - ZCL Bind Request/Response handling
 * - Binding Table persistence in NVS
 * - MQTT integration for Zigbee2MQTT compatibility
 *
 * Bindings enable direct communication between Zigbee devices without
 * coordinator intervention (e.g., a switch directly controlling a light).
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/device/bind - Create binding
 * - zigbee2mqtt/bridge/request/device/unbind - Remove binding
 * - zigbee2mqtt/bridge/response/device/bind - Bind response
 * - zigbee2mqtt/bridge/response/device/unbind - Unbind response
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_BINDING_H
#define ZB_BINDING_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of binding entries supported
 */
#define ZB_BINDING_MAX_ENTRIES 32

/**
 * @brief NVS namespace for binding storage
 */
#define ZB_BINDING_NVS_NAMESPACE "zb_binding"

/**
 * @brief Maximum friendly name length for MQTT messages
 */
#define ZB_BINDING_NAME_MAX_LEN 64

/**
 * @brief Binding status codes
 */
typedef enum {
    ZB_BINDING_STATUS_SUCCESS = 0,              /**< Binding operation successful */
    ZB_BINDING_STATUS_NOT_SUPPORTED,            /**< Binding not supported by device */
    ZB_BINDING_STATUS_TABLE_FULL,               /**< Binding table full */
    ZB_BINDING_STATUS_NO_ENTRY,                 /**< No matching entry found */
    ZB_BINDING_STATUS_TIMEOUT,                  /**< Request timed out */
    ZB_BINDING_STATUS_INVALID_EP,               /**< Invalid endpoint */
    ZB_BINDING_STATUS_DEVICE_NOT_FOUND,         /**< Device not found in network */
    ZB_BINDING_STATUS_ERROR                     /**< General error */
} zb_binding_status_t;

/**
 * @brief Binding entry structure
 *
 * Represents a single binding relationship between two devices.
 * Bindings are unidirectional: source device sends to destination.
 */
typedef struct {
    uint64_t source_ieee;       /**< Source device IEEE address (64-bit) */
    uint8_t source_endpoint;    /**< Source device endpoint */
    uint16_t cluster_id;        /**< Cluster ID to bind */
    uint64_t dest_ieee;         /**< Destination device IEEE address (64-bit) */
    uint8_t dest_endpoint;      /**< Destination device endpoint */
    bool active;                /**< Entry is in use */
} zb_binding_entry_t;

/**
 * @brief Binding event types
 */
typedef enum {
    ZB_BINDING_EVENT_CREATED,   /**< Binding was created */
    ZB_BINDING_EVENT_REMOVED,   /**< Binding was removed */
    ZB_BINDING_EVENT_FAILED     /**< Binding operation failed */
} zb_binding_event_type_t;

/**
 * @brief Binding event callback type
 *
 * @param[in] event Event type
 * @param[in] entry Binding entry (may be NULL for failures)
 * @param[in] status Status code
 */
typedef void (*zb_binding_event_cb_t)(zb_binding_event_type_t event,
                                       const zb_binding_entry_t *entry,
                                       zb_binding_status_t status);

/**
 * @brief Binding request result structure for MQTT responses
 */
typedef struct {
    bool success;                               /**< Operation success status */
    zb_binding_status_t status;                 /**< Detailed status code */
    char source_name[ZB_BINDING_NAME_MAX_LEN];  /**< Source device name/IEEE */
    char dest_name[ZB_BINDING_NAME_MAX_LEN];    /**< Destination device name/IEEE */
    uint16_t cluster_id;                        /**< Cluster that was bound */
    char error_message[128];                    /**< Error message if failed */
} zb_binding_result_t;

/**
 * @brief Initialize binding management
 *
 * Initializes the binding management module, allocates resources,
 * and loads existing bindings from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_binding_init(void);

/**
 * @brief Deinitialize binding management
 *
 * Saves bindings to NVS and frees all resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_binding_deinit(void);

/**
 * @brief Create a new binding
 *
 * Creates a binding between source and destination devices for the
 * specified cluster. Sends ZCL Bind Request to the source device.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[in] source_ep Source device endpoint
 * @param[in] cluster_id Cluster ID to bind
 * @param[in] dest_ieee Destination device IEEE address
 * @param[in] dest_ep Destination device endpoint
 * @return ESP_OK on success (request sent, not yet confirmed)
 * @return ESP_ERR_NO_MEM if binding table is full
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_NOT_FOUND if source device not found
 */
esp_err_t zb_binding_create(uint64_t source_ieee, uint8_t source_ep,
                            uint16_t cluster_id,
                            uint64_t dest_ieee, uint8_t dest_ep);

/**
 * @brief Remove a binding
 *
 * Removes the binding matching the specified parameters.
 * Sends ZCL Unbind Request to the source device.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[in] source_ep Source device endpoint
 * @param[in] cluster_id Cluster ID
 * @param[in] dest_ieee Destination device IEEE address
 * @param[in] dest_ep Destination device endpoint
 * @return ESP_OK on success (request sent, not yet confirmed)
 * @return ESP_ERR_NOT_FOUND if binding not found
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_binding_remove(uint64_t source_ieee, uint8_t source_ep,
                            uint16_t cluster_id,
                            uint64_t dest_ieee, uint8_t dest_ep);

/**
 * @brief Remove all bindings for a device
 *
 * Removes all bindings where the device is either source or destination.
 * Useful when a device leaves the network.
 *
 * @param[in] ieee_addr Device IEEE address
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_binding_remove_device(uint64_t ieee_addr);

/**
 * @brief Get a binding entry by index
 *
 * @param[in] index Index in the binding table (0 to ZB_BINDING_MAX_ENTRIES-1)
 * @return Pointer to binding entry or NULL if not found/inactive
 */
const zb_binding_entry_t* zb_binding_get(size_t index);

/**
 * @brief Get all active bindings
 *
 * Copies all active bindings to the provided array.
 *
 * @param[out] entries Destination array for binding entries
 * @param[in] max_count Maximum number of entries to copy
 * @return Number of entries copied
 */
size_t zb_binding_get_all(zb_binding_entry_t *entries, size_t max_count);

/**
 * @brief Get binding count
 *
 * @return Number of active bindings
 */
size_t zb_binding_get_count(void);

/**
 * @brief Find binding entry
 *
 * Searches for a binding matching all specified parameters.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[in] source_ep Source device endpoint
 * @param[in] cluster_id Cluster ID
 * @param[in] dest_ieee Destination device IEEE address
 * @param[in] dest_ep Destination device endpoint
 * @return Pointer to binding entry or NULL if not found
 */
const zb_binding_entry_t* zb_binding_find(uint64_t source_ieee, uint8_t source_ep,
                                          uint16_t cluster_id,
                                          uint64_t dest_ieee, uint8_t dest_ep);

/**
 * @brief Get bindings for source device
 *
 * Gets all bindings where the specified device is the source.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[out] entries Destination array for binding entries
 * @param[in] max_count Maximum number of entries to copy
 * @return Number of entries copied
 */
size_t zb_binding_get_by_source(uint64_t source_ieee,
                                 zb_binding_entry_t *entries, size_t max_count);

/**
 * @brief Get bindings for destination device
 *
 * Gets all bindings where the specified device is the destination.
 *
 * @param[in] dest_ieee Destination device IEEE address
 * @param[out] entries Destination array for binding entries
 * @param[in] max_count Maximum number of entries to copy
 * @return Number of entries copied
 */
size_t zb_binding_get_by_dest(uint64_t dest_ieee,
                               zb_binding_entry_t *entries, size_t max_count);

/**
 * @brief Save bindings to NVS
 *
 * Persists all binding data to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_binding_save_to_nvs(void);

/**
 * @brief Load bindings from NVS
 *
 * Loads binding data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NVS_NOT_FOUND if no saved data
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_binding_load_from_nvs(void);

/**
 * @brief Clear all bindings from NVS
 *
 * Removes all binding data from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_binding_clear_nvs(void);

/**
 * @brief Process MQTT bind/unbind request
 *
 * Main entry point for handling binding-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/device/bind
 * - zigbee2mqtt/bridge/request/device/unbind
 *
 * Expected JSON payload format:
 * {
 *   "from": "device_name_or_ieee",
 *   "to": "device_name_or_ieee",
 *   "clusters": ["genOnOff", "genLevelCtrl"]  // optional, binds all if omitted
 * }
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request format
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_binding_process_mqtt_request(const char *topic, const char *payload, size_t len);

/**
 * @brief Publish binding response via MQTT
 *
 * Publishes the result of a binding operation to MQTT.
 * Topic: zigbee2mqtt/bridge/response/device/bind or unbind
 *
 * @param[in] is_bind true for bind response, false for unbind
 * @param[in] result Binding operation result
 * @return ESP_OK on success
 */
esp_err_t zb_binding_publish_response(bool is_bind, const zb_binding_result_t *result);

/**
 * @brief Publish binding list via MQTT
 *
 * Publishes the list of all bindings to MQTT.
 * Topic: zigbee2mqtt/bridge/bindings
 *
 * @return ESP_OK on success
 */
esp_err_t zb_binding_publish_list(void);

/**
 * @brief Handle ZCL Bind Response
 *
 * Processes the response from a ZCL Bind Request.
 * Called by the Zigbee callback handler.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[in] status ZCL status code
 * @return ESP_OK on success
 */
esp_err_t zb_binding_handle_bind_response(uint64_t source_ieee, uint8_t status);

/**
 * @brief Handle ZCL Unbind Response
 *
 * Processes the response from a ZCL Unbind Request.
 * Called by the Zigbee callback handler.
 *
 * @param[in] source_ieee Source device IEEE address
 * @param[in] status ZCL status code
 * @return ESP_OK on success
 */
esp_err_t zb_binding_handle_unbind_response(uint64_t source_ieee, uint8_t status);

/**
 * @brief Register binding event callback
 *
 * Registers a callback function that is called when binding events occur.
 *
 * @param[in] callback Event callback function
 * @return ESP_OK on success
 */
esp_err_t zb_binding_register_callback(zb_binding_event_cb_t callback);

/**
 * @brief Get cluster name from ID
 *
 * Returns a human-readable cluster name for MQTT messages.
 *
 * @param[in] cluster_id Cluster ID
 * @return Cluster name string (e.g., "genOnOff", "genLevelCtrl")
 */
const char* zb_binding_get_cluster_name(uint16_t cluster_id);

/**
 * @brief Get cluster ID from name
 *
 * Parses a cluster name string and returns the cluster ID.
 *
 * @param[in] name Cluster name (e.g., "genOnOff", "genLevelCtrl")
 * @return Cluster ID or 0xFFFF if not found
 */
uint16_t zb_binding_get_cluster_id(const char *name);

/**
 * @brief Self-test function for binding management
 *
 * Tests binding creation, removal, and persistence.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_binding_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_BINDING_H */
