/**
 * @file zb_availability.h
 * @brief Zigbee Device Availability Tracking API
 *
 * This module provides active availability checking for Zigbee devices.
 * - Router Devices: Active check via ZCL Read Basic Cluster (periodic ping)
 * - Battery Devices: Passive check based on last_seen timeout
 * - Publishes availability state to MQTT: zigbee2mqtt/[device]/availability
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_AVAILABILITY_H
#define ZB_AVAILABILITY_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration Defaults
 * ============================================================================ */

/**
 * @brief Default router check interval in seconds
 */
#ifndef CONFIG_ZB_AVAILABILITY_ROUTER_INTERVAL
#define ZB_AVAIL_DEFAULT_ROUTER_INTERVAL    300
#else
#define ZB_AVAIL_DEFAULT_ROUTER_INTERVAL    CONFIG_ZB_AVAILABILITY_ROUTER_INTERVAL
#endif

/**
 * @brief Default battery device timeout in seconds (24 hours)
 */
#ifndef CONFIG_ZB_AVAILABILITY_BATTERY_TIMEOUT
#define ZB_AVAIL_DEFAULT_BATTERY_TIMEOUT    86400
#else
#define ZB_AVAIL_DEFAULT_BATTERY_TIMEOUT    CONFIG_ZB_AVAILABILITY_BATTERY_TIMEOUT
#endif

/**
 * @brief Default router timeout after failed ping in seconds (15 minutes)
 */
#ifndef CONFIG_ZB_AVAILABILITY_ROUTER_TIMEOUT
#define ZB_AVAIL_DEFAULT_ROUTER_TIMEOUT     900
#else
#define ZB_AVAIL_DEFAULT_ROUTER_TIMEOUT     CONFIG_ZB_AVAILABILITY_ROUTER_TIMEOUT
#endif

/**
 * @brief Maximum number of tracked devices
 */
#ifndef CONFIG_MAX_ZIGBEE_DEVICES
#define ZB_AVAIL_MAX_DEVICES                50
#else
#define ZB_AVAIL_MAX_DEVICES                CONFIG_MAX_ZIGBEE_DEVICES
#endif

/**
 * @brief Maximum consecutive failures before marking offline
 */
#define ZB_AVAIL_MAX_FAILURES               3

/**
 * @brief Exponential backoff base multiplier
 */
#define ZB_AVAIL_BACKOFF_BASE               2

/**
 * @brief Maximum backoff multiplier
 */
#define ZB_AVAIL_MAX_BACKOFF                8

/* ============================================================================
 * Types and Structures
 * ============================================================================ */

/**
 * @brief Availability state enumeration
 */
typedef enum {
    ZB_AVAIL_ONLINE = 0,    /**< Device is online and responding */
    ZB_AVAIL_OFFLINE,       /**< Device is offline (not responding) */
    ZB_AVAIL_UNKNOWN        /**< Device availability is unknown (not yet checked) */
} zb_availability_state_t;

/**
 * @brief Device power source type for availability check strategy
 */
typedef enum {
    ZB_AVAIL_POWER_MAINS = 0,   /**< Mains powered (router) - active check */
    ZB_AVAIL_POWER_BATTERY,     /**< Battery powered (end device) - passive check */
    ZB_AVAIL_POWER_UNKNOWN      /**< Unknown power source - use passive check */
} zb_availability_power_type_t;

/**
 * @brief Availability configuration structure
 */
typedef struct {
    uint32_t router_check_interval;  /**< Seconds between active checks for routers */
    uint32_t battery_timeout;        /**< Seconds until battery device marked offline */
    uint32_t router_timeout;         /**< Seconds until router offline after failed ping */
    bool active_check_enabled;       /**< Enable active availability checking */
} zb_availability_config_t;

/**
 * @brief Per-device availability tracking data
 */
typedef struct {
    uint16_t short_addr;                    /**< Device short address */
    zb_availability_state_t state;          /**< Current availability state */
    zb_availability_power_type_t power_type; /**< Device power source type */
    time_t last_seen;                       /**< Last activity timestamp */
    time_t last_check;                      /**< Last active check timestamp */
    time_t next_check;                      /**< Next scheduled check timestamp */
    uint8_t consecutive_failures;           /**< Number of consecutive ping failures */
    uint8_t backoff_multiplier;             /**< Current backoff multiplier */
    uint32_t custom_timeout;                /**< Per-device custom timeout (0=use default) */
    bool pending_response;                  /**< Waiting for ping response */
} zb_availability_device_t;

/**
 * @brief Availability state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] old_state Previous availability state
 * @param[in] new_state New availability state
 */
typedef void (*zb_availability_state_cb_t)(uint16_t short_addr,
                                            zb_availability_state_t old_state,
                                            zb_availability_state_t new_state);

/* ============================================================================
 * Initialization and Control
 * ============================================================================ */

/**
 * @brief Initialize the availability tracking module
 *
 * Initializes internal data structures, loads configuration from NVS,
 * and prepares the module for operation.
 *
 * @param[in] config Configuration structure (NULL for defaults)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_availability_init(zb_availability_config_t *config);

/**
 * @brief Deinitialize the availability tracking module
 *
 * Stops the availability task and frees all resources.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_availability_deinit(void);

/**
 * @brief Start availability tracking
 *
 * Creates the FreeRTOS task for periodic availability checks.
 * The Zigbee coordinator must be running before calling this.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_FAIL if task creation fails
 */
esp_err_t zb_availability_start(void);

/**
 * @brief Stop availability tracking
 *
 * Stops the availability task but preserves state.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not running
 */
esp_err_t zb_availability_stop(void);

/**
 * @brief Check if availability tracking is running
 *
 * @return true if the availability task is running
 */
bool zb_availability_is_running(void);

/* ============================================================================
 * Device Tracking
 * ============================================================================ */

/**
 * @brief Add a device to availability tracking
 *
 * Adds a device to be tracked for availability. Called automatically
 * when a device joins the network.
 *
 * @param[in] short_addr Device short address
 * @param[in] power_type Device power source type
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if device list is full
 * @return ESP_ERR_INVALID_ARG if address is invalid
 */
esp_err_t zb_availability_add_device(uint16_t short_addr, zb_availability_power_type_t power_type);

/**
 * @brief Remove a device from availability tracking
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_availability_remove_device(uint16_t short_addr);

/**
 * @brief Update device last seen timestamp
 *
 * Should be called whenever any frame is received from a device.
 * This resets the timeout counter and marks the device as online.
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_availability_update_last_seen(uint16_t short_addr);

/**
 * @brief Trigger an immediate availability check for a device
 *
 * Sends a ZCL Read Attribute request to the device's Basic cluster.
 * Used for manual checks or after device rejoins.
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 * @return ESP_ERR_INVALID_STATE if check already pending
 */
esp_err_t zb_availability_check_device(uint16_t short_addr);

/**
 * @brief Get the current availability state of a device
 *
 * @param[in] short_addr Device short address
 * @return Current availability state (ZB_AVAIL_UNKNOWN if not found)
 */
zb_availability_state_t zb_availability_get_state(uint16_t short_addr);

/**
 * @brief Get availability tracking data for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] data Output availability data structure
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 * @return ESP_ERR_INVALID_ARG if data is NULL
 */
esp_err_t zb_availability_get_device_data(uint16_t short_addr, zb_availability_device_t *data);

/* ============================================================================
 * Configuration
 * ============================================================================ */

/**
 * @brief Set per-device custom timeout
 *
 * Allows configuring a custom timeout for specific devices.
 * Set to 0 to use the global default.
 *
 * @param[in] short_addr Device short address
 * @param[in] timeout_sec Custom timeout in seconds (0=use default)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_availability_set_device_timeout(uint16_t short_addr, uint32_t timeout_sec);

/**
 * @brief Set device power type
 *
 * Updates the power type for a device, affecting which check strategy is used.
 *
 * @param[in] short_addr Device short address
 * @param[in] power_type New power type
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_availability_set_device_power_type(uint16_t short_addr,
                                                  zb_availability_power_type_t power_type);

/**
 * @brief Update global configuration
 *
 * @param[in] config New configuration (fields with 0 are not updated)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_availability_set_config(const zb_availability_config_t *config);

/**
 * @brief Get current configuration
 *
 * @param[out] config Output configuration structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_availability_get_config(zb_availability_config_t *config);

/**
 * @brief Save configuration to NVS
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_availability_save_config(void);

/**
 * @brief Load configuration from NVS
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no saved config
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_availability_load_config(void);

/* ============================================================================
 * MQTT Publishing
 * ============================================================================ */

/**
 * @brief Publish availability state for a device
 *
 * Publishes to: zigbee2mqtt/[friendly_name]/availability
 * Payload: "online" or "offline"
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_INVALID_STATE if MQTT not connected
 */
esp_err_t zb_availability_publish_state(uint16_t short_addr);

/**
 * @brief Publish availability state for all tracked devices
 *
 * Useful on startup or reconnection to ensure consistent state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_availability_publish_all(void);

/* ============================================================================
 * Callbacks
 * ============================================================================ */

/**
 * @brief Register availability state change callback
 *
 * @param[in] callback Callback function
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if callback is NULL
 */
esp_err_t zb_availability_register_callback(zb_availability_state_cb_t callback);

/**
 * @brief Unregister availability state change callback
 *
 * @return ESP_OK on success
 */
esp_err_t zb_availability_unregister_callback(void);

/* ============================================================================
 * Response Handlers
 * ============================================================================ */

/**
 * @brief Handle ZCL read attribute response for availability ping
 *
 * Called by the ZCL callback handler when a read response is received.
 *
 * @param[in] short_addr Device short address
 * @param[in] status ZCL status code
 * @return ESP_OK on success
 */
esp_err_t zb_availability_handle_read_response(uint16_t short_addr, uint8_t status);

/**
 * @brief Handle ping timeout
 *
 * Called when a ping request times out.
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 */
esp_err_t zb_availability_handle_timeout(uint16_t short_addr);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert availability state to string
 *
 * @param[in] state Availability state
 * @return State string ("online", "offline", or "unknown")
 */
const char* zb_availability_state_to_str(zb_availability_state_t state);

/**
 * @brief Convert power type to string
 *
 * @param[in] power_type Power type
 * @return Power type string
 */
const char* zb_availability_power_type_to_str(zb_availability_power_type_t power_type);

/**
 * @brief Get availability statistics
 *
 * @param[out] online_count Number of online devices
 * @param[out] offline_count Number of offline devices
 * @param[out] unknown_count Number of devices with unknown state
 * @return ESP_OK on success
 */
esp_err_t zb_availability_get_stats(uint16_t *online_count, uint16_t *offline_count,
                                     uint16_t *unknown_count);

/**
 * @brief Self-test function for availability module
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_availability_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_AVAILABILITY_H */
