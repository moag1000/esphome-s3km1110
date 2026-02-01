/**
 * @file zb_coordinator.h
 * @brief Zigbee Coordinator API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides the core Zigbee coordinator functionality including
 * stack initialization, network formation, and device management.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_COORDINATOR_H
#define ZB_COORDINATOR_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Zigbee coordinator state enumeration
 */
typedef enum {
    ZB_COORD_STATE_UNINITIALIZED = 0,  /**< Coordinator not initialized */
    ZB_COORD_STATE_INITIALIZED,        /**< Coordinator initialized but not started */
    ZB_COORD_STATE_STARTING,           /**< Coordinator starting up */
    ZB_COORD_STATE_RUNNING,            /**< Coordinator running and network formed */
    ZB_COORD_STATE_ERROR,              /**< Coordinator in error state */
    ZB_COORD_STATE_STOPPED             /**< Coordinator stopped */
} zb_coordinator_state_t;

/**
 * @brief Default permit join duration in seconds
 */
#define ZB_PERMIT_JOIN_DEFAULT_DURATION     254

/**
 * @brief Default maximum number of child devices for coordinator
 */
#define ZB_COORDINATOR_MAX_CHILDREN         32

/**
 * @brief Permit join forever (no timeout)
 */
#define ZB_PERMIT_JOIN_FOREVER              255

/* ============================================================================
 * Network Configuration Constants (CQ-017)
 * ============================================================================
 * These constants configure the Zigbee stack network capacity.
 * Used in esp_zb_*_set() calls before esp_zb_init().
 */

/** @brief Maximum number of devices in the network (default: 100) */
#define ZB_COORDINATOR_MAX_NETWORK_DEVICES  100

/** @brief I/O buffer size for Zigbee stack (default: 128 bytes) */
#define ZB_COORDINATOR_IO_BUFFER_SIZE       128

/** @brief APS source binding table size (default: 32 entries) */
#define ZB_COORDINATOR_BINDING_TABLE_SIZE   32

/** @brief ZCL scenes table size (default: 32 entries) */
#define ZB_COORDINATOR_SCENE_TABLE_SIZE     32

/**
 * @brief Permit join timer update interval in seconds
 */
#define ZB_PERMIT_JOIN_UPDATE_INTERVAL      10

/**
 * @brief Permit join state structure
 *
 * Tracks the current permit join state including timer and target device.
 */
typedef struct {
    bool enabled;                       /**< Permit join is currently enabled */
    uint8_t initial_duration;           /**< Initial duration requested (seconds) */
    uint8_t remaining_time;             /**< Remaining time (seconds), 255=forever */
    bool has_target_device;             /**< True if permit join is for specific device */
    uint8_t target_ieee_addr[8];        /**< Target device IEEE address (if has_target_device) */
    int64_t start_time_us;              /**< Start time in microseconds (esp_timer_get_time) */
} zb_permit_join_state_t;

/**
 * @brief Permit join options for extended control
 */
typedef struct {
    uint8_t duration;                   /**< Duration in seconds (0=close, 254=default, 255=forever) */
    bool has_target_device;             /**< True to permit join for specific device only */
    uint8_t target_ieee_addr[8];        /**< Target device IEEE address */
} zb_permit_join_options_t;

/**
 * @brief Initialize the Zigbee coordinator
 *
 * Initializes the Zigbee stack, configures the coordinator role,
 * and prepares for network formation.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_coordinator_init(void);

/**
 * @brief Start the Zigbee coordinator
 *
 * Forms a Zigbee network and starts accepting device joins.
 * This function creates the coordinator task and begins the main event loop.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_coordinator_start(void);

/**
 * @brief Stop the Zigbee coordinator
 *
 * Stops the coordinator task and shuts down the Zigbee stack.
 * The network will be closed and all devices will be disconnected.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not running
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_coordinator_stop(void);

/**
 * @brief Check if coordinator is running
 *
 * @return true if coordinator is in RUNNING state
 * @return false otherwise
 */
bool zb_coordinator_is_running(void);

/**
 * @brief Get current coordinator state
 *
 * @return Current coordinator state
 */
zb_coordinator_state_t zb_coordinator_get_state(void);

/**
 * @brief Open the network for new device joins
 *
 * Allows new Zigbee devices to join the network for the specified duration.
 * This is thread-safe and can be called from any task.
 *
 * @param duration Duration in seconds (0=close, 1-254=timeout, 255=forever)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if coordinator not running
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_coordinator_permit_join(uint8_t duration);

/**
 * @brief Open the network with extended options
 *
 * Enhanced permit join with timer management and optional target device.
 * The timer will automatically close the network when expired and publish
 * periodic updates to MQTT.
 *
 * @param options Permit join options (duration, target device)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if coordinator not running
 * @return ESP_ERR_INVALID_ARG if options is NULL
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_coordinator_permit_join_with_options(const zb_permit_join_options_t *options);

/**
 * @brief Get current permit join state
 *
 * Returns the current permit join state including remaining time.
 *
 * @param state Output: Current permit join state
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if state is NULL
 */
esp_err_t zb_coordinator_get_permit_join_state(zb_permit_join_state_t *state);

/**
 * @brief Check if permit join is enabled
 *
 * @return true if permit join is currently enabled
 * @return false if permit join is disabled
 */
bool zb_coordinator_is_permit_join_enabled(void);

/**
 * @brief Get remaining permit join time
 *
 * @return Remaining time in seconds, 0 if closed, 255 if forever
 */
uint8_t zb_coordinator_get_permit_join_remaining(void);

/**
 * @brief Main Zigbee coordinator task
 *
 * This is the main FreeRTOS task that runs the Zigbee stack event loop.
 * Do not call this directly - use zb_coordinator_start() instead.
 *
 * @param pvParameters Task parameters (unused)
 */
void zb_coordinator_task(void *pvParameters);

/**
 * @brief Get the coordinator mutex for thread-safe API access
 *
 * @return Mutex handle or NULL if not initialized
 */
SemaphoreHandle_t zb_coordinator_get_mutex(void);

/**
 * @brief Self-test function for coordinator initialization
 *
 * Verifies that the coordinator has been properly initialized.
 * Useful for debugging and validation.
 *
 * @return ESP_OK if all checks pass
 * @return ESP_FAIL if any check fails
 */
esp_err_t zb_coordinator_self_test(void);

/* ============================================================================
 * Coordinator Settings and Info (ZG-012)
 * ============================================================================ */

/**
 * @brief Coordinator settings structure
 *
 * Contains all configurable coordinator settings and network parameters.
 */
typedef struct {
    uint16_t pan_id;                /**< Network PAN ID */
    uint8_t channel;                /**< Current Zigbee channel (11-26) */
    uint8_t transmit_power;         /**< TX power level (dBm) */
    bool permit_join_default;       /**< Default permit join state on startup */
    uint8_t max_children;           /**< Maximum number of child devices */
    uint8_t extended_pan_id[8];     /**< Extended PAN ID (64-bit) */
    uint8_t network_key[16];        /**< Network encryption key (masked in responses) */
    uint8_t network_update_id;      /**< Network update identifier */
    bool network_formed;            /**< True if network is formed */
} zb_coordinator_settings_t;

/**
 * @brief Coordinator version information
 */
typedef struct {
    char type[32];                  /**< Coordinator type (e.g., "ESP32-C5") */
    char firmware_version[32];      /**< Firmware version string */
    char firmware_build[32];        /**< Firmware build date/revision */
    char zigbee_version[16];        /**< Zigbee stack version */
    char ieee_address[24];          /**< Coordinator IEEE address string */
    uint16_t short_address;         /**< Coordinator short address */
} zb_coordinator_version_t;

/**
 * @brief Coordinator health status
 */
typedef struct {
    bool healthy;                   /**< Overall health status */
    bool network_up;                /**< Network is formed and operational */
    bool zigbee_stack_running;      /**< Zigbee stack is running */
    uint32_t uptime_seconds;        /**< Uptime since coordinator started */
    uint32_t joined_devices;        /**< Number of devices currently joined */
    uint32_t message_count_tx;      /**< Total messages transmitted */
    uint32_t message_count_rx;      /**< Total messages received */
    int8_t last_rssi;               /**< Last received signal strength */
    uint8_t last_lqi;               /**< Last link quality indicator */
} zb_coordinator_health_t;

/**
 * @brief Get coordinator settings
 *
 * Retrieves current coordinator configuration and network settings.
 *
 * @param[out] settings Output structure for settings
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if settings is NULL
 * @return ESP_ERR_INVALID_STATE if coordinator not initialized
 */
esp_err_t zb_coordinator_get_settings(zb_coordinator_settings_t *settings);

/**
 * @brief Get coordinator version information
 *
 * Retrieves coordinator type, firmware version, and Zigbee stack info.
 *
 * @param[out] version Output structure for version info
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if version is NULL
 */
esp_err_t zb_coordinator_get_version(zb_coordinator_version_t *version);

/**
 * @brief Perform coordinator health check
 *
 * Checks coordinator health status including network state and statistics.
 *
 * @param[out] health Output structure for health status
 * @return ESP_OK on success (healthy)
 * @return ESP_ERR_INVALID_ARG if health is NULL
 * @return ESP_FAIL if health check fails (unhealthy)
 */
esp_err_t zb_coordinator_health_check(zb_coordinator_health_t *health);

/**
 * @brief Get coordinator version string
 *
 * Returns a formatted version string for MQTT responses.
 *
 * @param[out] version_buf Output buffer for version string
 * @param[in] buf_len Length of output buffer
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_coordinator_get_version_string(char *version_buf, size_t buf_len);

/**
 * @brief Update TX message count
 *
 * Called when a message is transmitted.
 */
void zb_coordinator_update_tx_count(void);

/**
 * @brief Update RX message count
 *
 * Called when a message is received.
 */
void zb_coordinator_update_rx_count(void);

/**
 * @brief Update signal quality metrics
 *
 * Called when a message is received with signal quality info.
 *
 * @param[in] rssi Received signal strength indicator (dBm)
 * @param[in] lqi Link quality indicator (0-255)
 */
void zb_coordinator_update_signal_quality(int8_t rssi, uint8_t lqi);

/**
 * @brief Update route error count (API-009)
 *
 * Called when a route discovery fails or route error occurs.
 * Increments the internal route error counter for diagnostics.
 */
void zb_coordinator_update_route_error_count(void);

/**
 * @brief Update Trust Center rejoin count (API-009)
 *
 * Called when a device successfully completes a Trust Center rejoin.
 * Increments the internal TC rejoin counter for diagnostics.
 */
void zb_coordinator_update_tc_rejoin_count(void);

/**
 * @brief Get route error count (API-009)
 *
 * Returns the total number of route errors since coordinator start.
 *
 * @return Number of route errors
 */
uint32_t zb_coordinator_get_route_error_count(void);

/**
 * @brief Get Trust Center rejoin count (API-009)
 *
 * Returns the total number of TC rejoins since coordinator start.
 *
 * @return Number of TC rejoins
 */
uint32_t zb_coordinator_get_tc_rejoin_count(void);

/* ============================================================================
 * SDK Version Information (API-016)
 * ============================================================================ */

/**
 * @brief SDK version information structure
 *
 * Contains ESP-Zigbee-SDK version details for runtime queries.
 */
typedef struct {
    uint8_t major;                  /**< Major version number */
    uint8_t minor;                  /**< Minor version number */
    uint8_t patch;                  /**< Patch version number */
    char version_string[16];        /**< Version string (e.g., "1.6.8") */
    uint8_t min_major;              /**< Minimum required major version */
    uint8_t min_minor;              /**< Minimum required minor version */
    uint8_t min_patch;              /**< Minimum required patch version */
    bool meets_minimum;             /**< True if SDK meets minimum requirements */
} zb_sdk_version_t;

/**
 * @brief Get ESP-Zigbee-SDK version information
 *
 * Retrieves the current SDK version and minimum required version.
 * This function can be used for runtime feature detection and
 * compatibility checking.
 *
 * @param[out] version Output structure for SDK version info
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if version is NULL
 */
esp_err_t zb_coordinator_get_sdk_version(zb_sdk_version_t *version);

/**
 * @brief Get ESP-Zigbee-SDK version as string
 *
 * Returns the SDK version as a formatted string (e.g., "1.6.8").
 * Useful for logging and MQTT responses.
 *
 * @param[out] version_buf Output buffer for version string
 * @param[in] buf_len Length of output buffer
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_coordinator_get_sdk_version_string(char *version_buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* ZB_COORDINATOR_H */
