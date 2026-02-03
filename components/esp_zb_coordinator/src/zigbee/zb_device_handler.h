/**
 * @file zb_device_handler.h
 * @brief Zigbee Device Management API
 *
 * This module manages the registry of Zigbee devices connected to the coordinator.
 * It tracks device state, handles join/leave events, and provides device lookup.
 *
 * Type definitions (structs, enums, constants) are in zb_device_handler_types.h.
 * Include this header for the full API; include only the types header when you
 * only need type definitions (for faster compilation).
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_DEVICE_HANDLER_H
#define ZB_DEVICE_HANDLER_H

#include "esp_err.h"
#include "esp_log.h"
#include "zb_device_handler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Utility Macros
 * ============================================================================ */

/**
 * @brief Initialize a ZCL basic command structure with short address
 *
 * Macro to reduce repeated struct initialization for ZCL commands.
 * Initializes the zcl_basic_cmd field with destination address, endpoint, and
 * source endpoint information for Zigbee Cluster Library commands.
 *
 * @param addr Short address of the destination device
 * @param ep Endpoint on the destination device
 *
 * Usage:
 *   esp_zb_zcl_read_attr_cmd_t cmd_req = {
 *       ZCL_BASIC_CMD_INIT(short_addr, endpoint),
 *       .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
 *       .clusterID = ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
 *   };
 */
#define ZCL_BASIC_CMD_INIT(addr, ep) \
    .zcl_basic_cmd = { \
        .dst_addr_u = { \
            .addr_short = (addr), \
        }, \
        .dst_endpoint = (ep), \
        .src_endpoint = 1, \
    }

/**
 * @brief Get device by short address or return error
 *
 * Usage:
 *   DEVICE_GET_OR_RETURN(0x1234, device, ESP_ERR_NOT_FOUND);
 *   // device is now guaranteed non-NULL
 *
 * @param short_addr Device short address
 * @param var Variable name for the device pointer
 * @param error_return Value to return if device not found
 */
#define DEVICE_GET_OR_RETURN(short_addr, var, error_return) \
    zb_device_t *var = zb_device_get(short_addr); \
    if ((var) == NULL) { \
        ESP_LOGW(TAG, "Device 0x%04X not found", (short_addr)); \
        return (error_return); \
    }

/**
 * @brief Get device by short address or goto cleanup
 *
 * @param short_addr Device short address
 * @param var Variable name for the device pointer
 * @param ret_var Error variable to set
 * @param cleanup_label Label to goto on failure
 */
#define DEVICE_GET_OR_CLEANUP(short_addr, var, ret_var, cleanup_label) \
    zb_device_t *var = zb_device_get(short_addr); \
    if ((var) == NULL) { \
        ESP_LOGW(TAG, "Device 0x%04X not found", (short_addr)); \
        ret_var = ESP_ERR_NOT_FOUND; \
        goto cleanup_label; \
    }

/* ============================================================================
 * Device Handler Core Functions
 * ============================================================================ */

/**
 * @brief Initialize device handler
 *
 * Initializes the device registry and associated resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t zb_device_handler_init(void);

/**
 * @brief Deinitialize device handler
 *
 * Frees all resources and clears the device registry.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_device_handler_deinit(void);

/**
 * @brief Add a new device to the registry
 *
 * Adds a device with the specified IEEE and short addresses.
 * If the device already exists, it updates the short address.
 *
 * @param ieee_addr IEEE 64-bit address
 * @param short_addr Network short address
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if device registry is full
 * @return ESP_ERR_INVALID_ARG if addresses are invalid
 */
esp_err_t zb_device_add(esp_zb_ieee_addr_t ieee_addr, uint16_t short_addr);

/**
 * @brief Remove a device from the registry
 *
 * Removes the device with the specified short address.
 *
 * @param short_addr Network short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_device_remove(uint16_t short_addr);

/**
 * @brief Get device by short address
 *
 * Returns a pointer to the device structure. The pointer is valid
 * until the device is removed or the handler is deinitialized.
 *
 * @param short_addr Network short address
 * @return Pointer to device structure or NULL if not found
 */
zb_device_t* zb_device_get(uint16_t short_addr);

/**
 * @brief Get device by IEEE address
 *
 * Returns a pointer to the device structure by IEEE address.
 *
 * @param ieee_addr IEEE 64-bit address
 * @return Pointer to device structure or NULL if not found
 */
zb_device_t* zb_device_get_by_ieee(esp_zb_ieee_addr_t ieee_addr);

/**
 * @brief Get device by short address with logging on failure
 *
 * Wrapper around zb_device_get() that logs a warning if the device
 * is not found. Useful for reducing boilerplate in callback handlers.
 *
 * @param short_addr Device short address
 * @param context Context string for error logging (e.g., function name)
 * @return Pointer to device or NULL if not found (logs warning)
 */
zb_device_t* zb_device_get_or_log(uint16_t short_addr, const char *context);

/**
 * @brief Check if device has a specific cluster
 *
 * Checks if the device at the given short address supports the specified cluster.
 *
 * @param short_addr Device short address
 * @param cluster_id Cluster ID to check for
 * @return true if device has the cluster, false otherwise
 */
bool zb_device_has_cluster(uint16_t short_addr, uint16_t cluster_id);

/**
 * @brief Get all devices
 *
 * Copies all devices to the provided array.
 *
 * @warning This function allocates ~10KB on the stack when called with a
 *          stack-allocated array of ZB_MAX_DEVICES. Consider using
 *          zb_device_iterate() or zb_device_find_by_name() instead.
 *
 * @param devices Destination array
 * @param max_count Maximum number of devices to copy
 * @return Number of devices copied
 */
size_t zb_device_get_all(zb_device_t *devices, size_t max_count);

/**
 * @brief Get device count
 *
 * @return Number of devices in the registry
 */
size_t zb_device_get_count(void);

/**
 * @brief Device iterator callback type
 *
 * @param device Pointer to device (valid only during callback)
 * @param user_data User-provided context
 * @return true to continue iteration, false to stop early
 */
typedef bool (*zb_device_iterator_cb_t)(const zb_device_t *device, void *user_data);

/**
 * @brief Iterate over all devices with a callback
 *
 * Thread-safe iteration over all devices without copying to stack.
 * The callback receives a const pointer to each device.
 *
 * @param callback Function to call for each device
 * @param user_data User data passed to callback
 * @return Number of devices iterated (may be less if callback returned false)
 */
size_t zb_device_iterate(zb_device_iterator_cb_t callback, void *user_data);

/**
 * @brief Find device by friendly name
 *
 * Thread-safe search for device by name without copying all devices.
 *
 * @param friendly_name Name to search for
 * @return Pointer to device or NULL if not found
 */
zb_device_t* zb_device_find_by_name(const char *friendly_name);

/**
 * @brief Get device by index (for sequential iteration)
 *
 * Thread-safe access to device by index. Use with zb_device_get_count()
 * for iterating without copying all devices.
 *
 * @warning The returned pointer is only valid while the mutex is held internally.
 *          For safe access after the call, use zb_device_copy_by_index() instead.
 *
 * @param index Device index (0 to count-1)
 * @return Pointer to device or NULL if index out of range
 */
zb_device_t* zb_device_get_by_index(size_t index);

/**
 * @brief Safely copy device data by index
 *
 * Thread-safe copy of device data. The mutex is held during the copy,
 * ensuring consistent data. Use this instead of zb_device_get_by_index()
 * when you need to access device data outside of mutex protection.
 *
 * @param index Device index (0 to count-1)
 * @param dest Destination buffer for device copy
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if index out of range
 */
esp_err_t zb_device_copy_by_index(size_t index, zb_device_t *dest);

/**
 * @brief Set device friendly name
 *
 * Sets a user-friendly name for the device.
 *
 * @param short_addr Network short address
 * @param name Friendly name (max ZB_DEVICE_FRIENDLY_NAME_LEN-1 chars)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_INVALID_ARG if name is NULL or too long
 */
esp_err_t zb_device_set_friendly_name(uint16_t short_addr, const char *name);

/**
 * @brief Update device state
 *
 * Updates device state based on attribute change.
 * This is called when an attribute report is received.
 *
 * @param short_addr Network short address
 * @param endpoint Endpoint number
 * @param cluster_id Cluster ID
 * @param attr_id Attribute ID
 * @param value Attribute value
 * @param value_len Value length in bytes
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_device_update_state(uint16_t short_addr, uint8_t endpoint,
                                  uint16_t cluster_id, uint16_t attr_id,
                                  void *value, size_t value_len);

/**
 * @brief Update device last seen timestamp
 *
 * Updates the last_seen timestamp and marks device as online.
 *
 * @param short_addr Network short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_device_update_last_seen(uint16_t short_addr);

/**
 * @brief Set device online status
 *
 * Manually sets the device online/offline status.
 *
 * @param short_addr Network short address
 * @param online Online status
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_device_set_online(uint16_t short_addr, bool online);

/**
 * @brief Update device information from Basic cluster
 *
 * Updates manufacturer, model, and other info from Basic cluster attributes.
 *
 * @param short_addr Network short address
 * @param manufacturer Manufacturer name (can be NULL)
 * @param model Model identifier (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_device_update_info(uint16_t short_addr, const char *manufacturer,
                                 const char *model);

/**
 * @brief Add cluster to device
 *
 * Adds a supported cluster ID to the device's cluster list.
 *
 * @param short_addr Network short address
 * @param cluster_id Cluster ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_NO_MEM if cluster list is full
 */
esp_err_t zb_device_add_cluster(uint16_t short_addr, uint16_t cluster_id);

/**
 * @brief Determine device type from clusters
 *
 * Analyzes the device's supported clusters and determines the device type.
 *
 * @param short_addr Network short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_device_determine_type(uint16_t short_addr);

/**
 * @brief Self-test function for device handler
 *
 * Tests device add, get, update, and remove operations.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_device_test(void);

/* ============================================================================
 * Binary Output Cluster Functions
 * ============================================================================ */

/**
 * @brief Register binary output state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_binary_output_register_callback(zb_binary_output_state_cb_t callback);

/**
 * @brief Check if device has Binary Output cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Binary Output cluster
 */
bool zb_device_has_binary_output(uint16_t short_addr);

/**
 * @brief Read binary output state from device
 *
 * Reads present value, out of service, and status flags.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_binary_output_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set binary output present value
 *
 * Writes the PresentValue attribute to control the output.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] value Output value (true=ON, false=OFF)
 * @return ESP_OK on success
 */
esp_err_t zb_binary_output_set_value(uint16_t short_addr, uint8_t endpoint, bool value);

/**
 * @brief Handle Binary Output attribute report
 *
 * Processes incoming attribute reports from Binary Output devices.
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
 * @brief Get current Binary Output state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_binary_output_get_state(uint16_t short_addr, zb_binary_output_state_t *state);

/* ============================================================================
 * Binary Value Cluster Functions
 * ============================================================================ */

/**
 * @brief Register binary value state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_binary_value_register_callback(zb_binary_value_state_cb_t callback);

/**
 * @brief Check if device has Binary Value cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Binary Value cluster
 */
bool zb_device_has_binary_value(uint16_t short_addr);

/**
 * @brief Read binary value state from device
 *
 * Reads present value, out of service, and status flags.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_binary_value_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set binary value present value
 *
 * Writes the PresentValue attribute.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] value Value to set (true=ON, false=OFF)
 * @return ESP_OK on success
 */
esp_err_t zb_binary_value_set_value(uint16_t short_addr, uint8_t endpoint, bool value);

/**
 * @brief Handle Binary Value attribute report
 *
 * Processes incoming attribute reports from Binary Value devices.
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
 * @brief Get current Binary Value state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_binary_value_get_state(uint16_t short_addr, zb_binary_value_state_t *state);

/* ============================================================================
 * Window Covering Cluster Functions
 * ============================================================================ */

/**
 * @brief Register window covering state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_register_callback(zb_window_covering_state_cb_t callback);

/**
 * @brief Check if device has Window Covering cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Window Covering cluster
 */
bool zb_device_has_window_covering(uint16_t short_addr);

/**
 * @brief Read current lift position from Window Covering device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_read_position(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Up/Open command to Window Covering device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_cmd_up(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Down/Close command to Window Covering device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_cmd_down(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Stop command to Window Covering device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_cmd_stop(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Go to Lift Percentage command to Window Covering device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] percentage Target position (0-100, where 100 = fully open)
 * @return ESP_OK on success
 */
esp_err_t zb_window_covering_cmd_goto_lift_percent(uint16_t short_addr, uint8_t endpoint,
                                                    uint8_t percentage);

/**
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
 * @brief Get current Window Covering state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_window_covering_get_state(uint16_t short_addr, zb_window_covering_state_t *state);

/* ============================================================================
 * Door Lock Cluster Functions
 * ============================================================================ */

/**
 * @brief Register door lock state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_door_lock_register_callback(zb_door_lock_state_cb_t callback);

/**
 * @brief Check if device has Door Lock cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Door Lock cluster
 */
bool zb_device_has_door_lock(uint16_t short_addr);

/**
 * @brief Read current lock state from Door Lock device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_door_lock_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Lock Door command to Door Lock device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_door_lock_cmd_lock(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Unlock Door command to Door Lock device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_door_lock_cmd_unlock(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Get current Door Lock state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_door_lock_get_state(uint16_t short_addr, zb_door_lock_state_struct_t *state);

/* ============================================================================
 * Thermostat Cluster Functions
 * ============================================================================ */

/**
 * @brief Register thermostat state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_register_callback(zb_thermostat_state_cb_t callback);

/**
 * @brief Check if device has Thermostat cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Thermostat cluster
 */
bool zb_device_has_thermostat(uint16_t short_addr);

/**
 * @brief Read thermostat state from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set thermostat occupied heating setpoint
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] temperature Temperature in 0.01C units (e.g., 2100 = 21.00C)
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_set_heating_setpoint(uint16_t short_addr, uint8_t endpoint,
                                              int16_t temperature);

/**
 * @brief Set thermostat occupied cooling setpoint
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] temperature Temperature in 0.01C units (e.g., 2500 = 25.00C)
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_set_cooling_setpoint(uint16_t short_addr, uint8_t endpoint,
                                              int16_t temperature);

/**
 * @brief Set thermostat system mode
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] mode System mode
 * @return ESP_OK on success
 */
esp_err_t zb_thermostat_set_system_mode(uint16_t short_addr, uint8_t endpoint,
                                         zb_thermostat_system_mode_t mode);

/**
 * @brief Handle thermostat attribute report
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
 * @brief Get current thermostat state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_thermostat_get_state(uint16_t short_addr, zb_thermostat_state_t *state);

/* ============================================================================
 * Fan Control Cluster Functions
 * ============================================================================ */

/**
 * @brief Register fan control state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_fan_control_register_callback(zb_fan_control_state_cb_t callback);

/**
 * @brief Check if device has Fan Control cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Fan Control cluster
 */
bool zb_device_has_fan_control(uint16_t short_addr);

/**
 * @brief Read fan control state from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_fan_control_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set fan mode
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] mode Fan mode to set
 * @return ESP_OK on success
 */
esp_err_t zb_fan_control_set_mode(uint16_t short_addr, uint8_t endpoint,
                                   zb_fan_mode_t mode);

/**
 * @brief Handle fan control attribute report
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
 * @brief Get current fan control state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_fan_control_get_state(uint16_t short_addr, zb_fan_control_state_t *state);

/* ============================================================================
 * Illuminance Measurement Cluster Functions
 * ============================================================================ */

/**
 * @brief Register illuminance state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_illuminance_register_callback(zb_illuminance_state_cb_t callback);

/**
 * @brief Check if device has Illuminance Measurement cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Illuminance Measurement cluster
 */
bool zb_device_has_illuminance(uint16_t short_addr);

/**
 * @brief Read illuminance value from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_illuminance_read_value(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Get current Illuminance state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_illuminance_get_state(uint16_t short_addr, zb_illuminance_state_t *state);

/**
 * @brief Convert raw illuminance value to lux
 *
 * @param[in] raw_value Raw ZCL illuminance value
 * @return Illuminance in lux (0.0 if too low or invalid)
 */
float zb_illuminance_to_lux(uint16_t raw_value);

/* ============================================================================
 * Pressure Measurement Cluster Functions
 * ============================================================================ */

/**
 * @brief Register pressure state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_pressure_register_callback(zb_pressure_state_cb_t callback);

/**
 * @brief Check if device has Pressure Measurement cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Pressure Measurement cluster
 */
bool zb_device_has_pressure(uint16_t short_addr);

/**
 * @brief Read pressure value from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_pressure_read_value(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Get current Pressure state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_pressure_get_state(uint16_t short_addr, zb_pressure_state_t *state);

/**
 * @brief Convert pressure measurement to hPa
 *
 * @param[in] state Pressure state structure
 * @return Pressure in hPa
 */
float zb_pressure_to_hpa(const zb_pressure_state_t *state);

/* ============================================================================
 * PM2.5 Measurement Cluster Functions
 * ============================================================================ */

/**
 * @brief Register PM2.5 state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_pm25_register_callback(zb_pm25_state_cb_t callback);

/**
 * @brief Check if device has PM2.5 Measurement cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has PM2.5 Measurement cluster
 */
bool zb_device_has_pm25(uint16_t short_addr);

/**
 * @brief Read PM2.5 value from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_pm25_read_value(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Get current PM2.5 state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_pm25_get_state(uint16_t short_addr, zb_pm25_state_t *state);

/* ============================================================================
 * Electrical Measurement Cluster Functions
 * ============================================================================ */

/**
 * @brief Register electrical measurement state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_electrical_register_callback(zb_electrical_state_cb_t callback);

/**
 * @brief Check if device has Electrical Measurement cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Electrical Measurement cluster
 */
bool zb_device_has_electrical_measurement(uint16_t short_addr);

/**
 * @brief Read electrical measurement values from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_electrical_read_values(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Read electrical measurement scaling factors from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_electrical_read_scaling(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Get current Electrical Measurement state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_electrical_get_state(uint16_t short_addr, zb_electrical_state_t *state);

/**
 * @brief Get voltage in Volts (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Voltage in Volts
 */
float zb_electrical_get_voltage_v(const zb_electrical_state_t *state);

/**
 * @brief Get current in Amperes (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Current in Amperes
 */
float zb_electrical_get_current_a(const zb_electrical_state_t *state);

/**
 * @brief Get active power in Watts (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Power in Watts
 */
float zb_electrical_get_power_w(const zb_electrical_state_t *state);

/**
 * @brief Get reactive power in VAR (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Reactive power in VAR
 */
float zb_electrical_get_reactive_power_var(const zb_electrical_state_t *state);

/**
 * @brief Get apparent power in VA (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Apparent power in VA
 */
float zb_electrical_get_apparent_power_va(const zb_electrical_state_t *state);

/**
 * @brief Get power factor as decimal (-1.0 to 1.0)
 *
 * @param[in] state Electrical measurement state
 * @return Power factor as decimal value
 */
float zb_electrical_get_power_factor(const zb_electrical_state_t *state);

/**
 * @brief Get AC frequency in Hz (after applying scaling)
 *
 * @param[in] state Electrical measurement state
 * @return Frequency in Hz
 */
float zb_electrical_get_frequency_hz(const zb_electrical_state_t *state);

/* ============================================================================
 * Metering Cluster Functions
 * ============================================================================ */

/**
 * @brief Register metering state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_metering_register_callback(zb_metering_state_cb_t callback);

/**
 * @brief Check if device has Metering cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Metering cluster
 */
bool zb_device_has_metering(uint16_t short_addr);

/**
 * @brief Read metering values from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_metering_read_values(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Handle Metering attribute report
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
 * @brief Get current Metering state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_metering_get_state(uint16_t short_addr, zb_metering_state_t *state);

/**
 * @brief Get total energy in kWh (or appropriate unit)
 *
 * @param[in] state Metering state structure
 * @return Total energy in kWh (or m3 for gas, L for water)
 */
double zb_metering_get_total_energy(const zb_metering_state_t *state);

/**
 * @brief Get current power in Watts
 *
 * @param[in] state Metering state structure
 * @return Current power in W (or flow rate)
 */
double zb_metering_get_power_w(const zb_metering_state_t *state);

/**
 * @brief Get today's consumption in formatted units
 *
 * @param[in] state Metering state structure
 * @return Today's consumption in kWh (or m3/L)
 */
double zb_metering_get_current_day_energy(const zb_metering_state_t *state);

/**
 * @brief Get yesterday's consumption in formatted units
 *
 * @param[in] state Metering state structure
 * @return Yesterday's consumption in kWh (or m3/L)
 */
double zb_metering_get_previous_day_energy(const zb_metering_state_t *state);

/**
 * @brief Get unit string for the metering device
 *
 * @param[in] unit Unit of measure enum value
 * @return String representation of the unit (e.g., "kWh", "m3", "L")
 */
const char* zb_metering_get_unit_string(uint8_t unit);

/**
 * @brief Parse uint48 value from Zigbee attribute data
 *
 * @param[in] data Pointer to 6-byte data
 * @param[in] len Data length (must be >= 6)
 * @return Parsed uint48 value as uint64_t, or 0 if len < 6
 */
uint64_t zb_metering_parse_uint48(const void *data, size_t len);

/**
 * @brief Parse int24 value from Zigbee attribute data
 *
 * @param[in] data Pointer to 3-byte data
 * @param[in] len Data length (must be >= 3)
 * @return Parsed int24 value as int32_t
 */
int32_t zb_metering_parse_int24(const void *data, size_t len);

/**
 * @brief Parse uint24 value from Zigbee attribute data
 *
 * @param[in] data Pointer to 3-byte data
 * @param[in] len Data length (must be >= 3)
 * @return Parsed uint24 value as uint32_t
 */
uint32_t zb_metering_parse_uint24(const void *data, size_t len);

/**
 * @brief Get number of decimal places from formatting byte
 *
 * @param[in] formatting Formatting byte
 * @return Number of decimal places (0-7)
 */
uint8_t zb_metering_get_decimal_places(uint8_t formatting);

/* ============================================================================
 * IAS Zone Cluster Functions
 * ============================================================================ */

/**
 * @brief Register IAS Zone state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_register_callback(zb_ias_zone_state_cb_t callback);

/**
 * @brief Check if device has IAS Zone cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has IAS Zone cluster
 */
bool zb_device_has_ias_zone(uint16_t short_addr);

/**
 * @brief Read IAS Zone state from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Send Zone Enroll Response to device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] response_code Enrollment response code
 * @param[in] zone_id Zone ID to assign (0x00-0xFE, 0xFF = auto)
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_enroll_response(uint16_t short_addr, uint8_t endpoint,
                                       zb_ias_zone_enroll_response_code_t response_code,
                                       uint8_t zone_id);

/**
 * @brief Write CIE address to IAS Zone device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_write_cie_address(uint16_t short_addr, uint8_t endpoint);

/**
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
 * @brief Handle IAS Zone Status Change Notification command
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] zone_status New zone status bitmap
 * @param[in] extended_status Extended status byte
 * @param[in] zone_id Zone ID
 * @param[in] delay Delay in quarter seconds
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_handle_status_change(uint16_t short_addr, uint8_t endpoint,
                                            uint16_t zone_status, uint8_t extended_status,
                                            uint8_t zone_id, uint16_t delay);

/**
 * @brief Handle IAS Zone Enroll Request command
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] zone_type Zone type
 * @param[in] manufacturer_code Manufacturer code
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_handle_enroll_request(uint16_t short_addr, uint8_t endpoint,
                                             uint16_t zone_type, uint16_t manufacturer_code);

/**
 * @brief Get current IAS Zone state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_ias_zone_get_state(uint16_t short_addr, zb_ias_zone_state_struct_t *state);

/**
 * @brief Get device class string for IAS Zone type
 *
 * @param[in] zone_type IAS Zone type
 * @return Device class string (e.g., "motion", "door", "smoke", "moisture")
 */
const char* zb_ias_zone_get_device_class(zb_ias_zone_type_t zone_type);

/**
 * @brief Parse zone status bitmap into boolean values
 *
 * @param[in] zone_status Raw zone status bitmap
 * @param[out] state State structure to populate
 */
void zb_ias_zone_parse_status(uint16_t zone_status, zb_ias_zone_state_struct_t *state);

/**
 * @brief Enable/disable auto-enrollment for IAS Zone devices
 *
 * @param[in] enable true to enable auto-enrollment
 * @return ESP_OK on success
 */
esp_err_t zb_ias_zone_set_auto_enroll(bool enable);

/**
 * @brief Check if auto-enrollment is enabled
 *
 * @return true if auto-enrollment is enabled
 */
bool zb_ias_zone_get_auto_enroll(void);

/* ============================================================================
 * Persistent Device Names (NVS Storage) Functions
 * ============================================================================ */

/**
 * @brief Save device friendly name to NVS
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @param[in] name Friendly name to save (max ZB_DEVICE_FRIENDLY_NAME_LEN-1 chars)
 * @return ESP_OK on success
 */
esp_err_t zb_device_save_friendly_name(const uint8_t *ieee_addr, const char *name);

/**
 * @brief Load device friendly name from NVS
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @param[out] name Output buffer for friendly name
 * @param[in] len Length of output buffer
 * @return ESP_OK on success
 */
esp_err_t zb_device_load_friendly_name(const uint8_t *ieee_addr, char *name, size_t len);

/**
 * @brief Delete device friendly name from NVS
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @return ESP_OK on success
 */
esp_err_t zb_device_delete_friendly_name(const uint8_t *ieee_addr);

/**
 * @brief Load all friendly names from NVS
 *
 * @return ESP_OK on success
 */
esp_err_t zb_device_load_all_friendly_names(void);

/**
 * @brief Convert IEEE address to NVS key string
 *
 * @param[in] ieee_addr IEEE address (8 bytes)
 * @param[out] key_buf Output buffer (minimum 17 bytes)
 * @param[in] buf_len Buffer length
 * @return ESP_OK on success
 */
esp_err_t zb_device_ieee_to_nvs_key(const uint8_t *ieee_addr, char *key_buf, size_t buf_len);

/* ============================================================================
 * Device Persistence (NVS) Functions
 * ============================================================================ */

/**
 * @brief Save device to NVS
 *
 * Persists essential device information (IEEE address, endpoint, device type,
 * clusters, manufacturer, model, friendly name) to NVS for recovery after reboot.
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not in registry
 * @return ESP_ERR_NVS_* on NVS errors
 */
esp_err_t zb_device_save_to_nvs(const uint8_t *ieee_addr);

/**
 * @brief Load all devices from NVS
 *
 * Called on startup to restore device registry from persistent storage.
 * Loaded devices will have short_addr = ZB_SHORT_ADDR_PENDING (0xFFFF) until
 * they communicate and their actual short address is resolved.
 *
 * @return Number of devices loaded from NVS
 */
size_t zb_device_load_all_from_nvs(void);

/**
 * @brief Delete device from NVS
 *
 * Removes device persistence record from NVS.
 * Called when a device is removed from the network.
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not in NVS
 */
esp_err_t zb_device_delete_from_nvs(const uint8_t *ieee_addr);

/**
 * @brief Update device short address by IEEE address
 *
 * Called when a device communicates after rejoin with a new short address.
 * Updates the short address in the registry for the matching IEEE address.
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @param[in] new_short_addr New short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not in registry
 */
esp_err_t zb_device_update_short_addr(const uint8_t *ieee_addr, uint16_t new_short_addr);

/**
 * @brief Mark device for NVS save
 *
 * Marks the device as dirty, requiring an NVS save. The actual save may be
 * debounced to batch multiple changes. Call this after modifying device
 * properties that should persist across reboots.
 *
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 */
void zb_device_mark_dirty(const uint8_t *ieee_addr);

/**
 * @brief Flush pending NVS saves
 *
 * Immediately writes all dirty devices to NVS. Call this before shutdown
 * or when immediate persistence is required.
 */
void zb_device_flush_nvs(void);

/* ============================================================================
 * Power Descriptor Functions (API-007)
 * ============================================================================ */

/**
 * @brief Request power descriptor from device (API-007)
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK if request was sent successfully
 */
esp_err_t zb_device_request_power_descriptor(uint16_t short_addr);

/**
 * @brief Register power descriptor callback
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_power_desc_register_callback(zb_power_desc_cb_t callback);

/**
 * @brief Check if device is battery powered
 *
 * @param[in] short_addr Device short address
 * @return true if device is battery powered
 */
bool zb_device_is_battery_powered(uint16_t short_addr);

/**
 * @brief Get device battery percentage
 *
 * @param[in] short_addr Device short address
 * @return Battery percentage (0-100), or -1 if not available
 */
int zb_device_get_battery_percent(uint16_t short_addr);

/**
 * @brief Get power source string
 *
 * @param[in] power_info Pointer to power info structure
 * @return Static string describing power source (e.g., "Mains", "Battery")
 */
const char* zb_power_source_to_string(const zb_power_info_t *power_info);

/**
 * @brief Convert power level to percentage
 *
 * @param[in] power_level Power level from power descriptor
 * @return Approximate percentage (0, 5, 33, 66, or 100)
 */
uint8_t zb_power_level_to_percent(uint8_t power_level);

/**
 * @brief Refresh power info for all battery devices
 *
 * @return Number of requests sent
 */
size_t zb_device_refresh_battery_status(void);

/* ============================================================================
 * Multistate Input/Output/Value Cluster Functions
 * ============================================================================ */

/**
 * @brief Register multistate state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_multistate_register_callback(zb_multistate_state_cb_t callback);

/**
 * @brief Check if device has Multistate Input cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Multistate Input cluster
 */
bool zb_device_has_multistate_input(uint16_t short_addr);

/**
 * @brief Check if device has Multistate Output cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Multistate Output cluster
 */
bool zb_device_has_multistate_output(uint16_t short_addr);

/**
 * @brief Check if device has Multistate Value cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Multistate Value cluster
 */
bool zb_device_has_multistate_value(uint16_t short_addr);

/**
 * @brief Check if device has any multistate cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has any multistate cluster (input, output, or value)
 */
bool zb_device_has_multistate(uint16_t short_addr);

/**
 * @brief Read multistate attributes from device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] cluster_id Cluster ID (0x0012, 0x0013, or 0x0014)
 * @return ESP_OK on success
 */
esp_err_t zb_multistate_read_state(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t cluster_id);

/**
 * @brief Set multistate PresentValue (for output/value clusters)
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] cluster_id Cluster ID (0x0013 or 0x0014)
 * @param[in] value New present value (1 to number_of_states)
 * @return ESP_OK on success
 */
esp_err_t zb_multistate_set_value(uint16_t short_addr, uint8_t endpoint,
                                   uint16_t cluster_id, uint16_t value);

/**
 * @brief Handle multistate attribute report
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] cluster_id Cluster ID (0x0012, 0x0013, or 0x0014)
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_multistate_handle_report(uint16_t short_addr, uint8_t endpoint,
                                       uint16_t cluster_id, uint16_t attr_id,
                                       void *value, size_t value_len);

/**
 * @brief Get current multistate state for a device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint (0 = first match)
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_multistate_get_state(uint16_t short_addr, uint8_t endpoint,
                                   zb_multistate_state_t *state);

/**
 * @brief Get cluster type from cluster ID
 *
 * @param[in] cluster_id Cluster ID
 * @return Multistate type, or -1 if not a multistate cluster
 */
int zb_multistate_get_type_from_cluster(uint16_t cluster_id);

/**
 * @brief Get string representation of multistate type
 *
 * @param[in] type Multistate type
 * @return Static string ("input", "output", or "value")
 */
const char* zb_multistate_type_to_string(zb_multistate_type_t type);

/**
 * @brief Check if present value is in alarm state
 *
 * @param[in] state Multistate state structure
 * @return true if IN_ALARM status flag is set
 */
bool zb_multistate_is_in_alarm(const zb_multistate_state_t *state);

/**
 * @brief Check if device has a fault
 *
 * @param[in] state Multistate state structure
 * @return true if FAULT status flag is set
 */
bool zb_multistate_has_fault(const zb_multistate_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ZB_DEVICE_HANDLER_H */
