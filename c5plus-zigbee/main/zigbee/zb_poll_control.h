/**
 * @file zb_poll_control.h
 * @brief Zigbee Poll Control Cluster Client Implementation (Cluster 0x0020)
 *
 * Manages check-in intervals for sleepy end devices (battery-powered).
 * The coordinator acts as a Poll Control Client to manage when sleepy
 * devices wake up and poll for pending messages.
 *
 * Key Concepts:
 * - Check-In Interval: How often sleepy device wakes to check for messages
 * - Long Poll Interval: Normal polling interval when device is sleeping
 * - Short Poll Interval: Fast polling during active communication
 * - Fast Poll Timeout: Duration of fast polling mode
 *
 * Time units: Quarter-seconds (250ms)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_POLL_CONTROL_H
#define ZB_POLL_CONTROL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants and Definitions
 * ============================================================================ */

/**
 * @brief Poll Control Cluster ID (ZCL)
 */
#define ZB_ZCL_CLUSTER_ID_POLL_CONTROL      0x0020

/**
 * @brief Poll Control Cluster Attribute IDs
 */
#define ZB_POLL_ATTR_CHECK_IN_INTERVAL      0x0000  /**< uint32 (quarter-seconds) */
#define ZB_POLL_ATTR_LONG_POLL_INTERVAL     0x0001  /**< uint32 (quarter-seconds) */
#define ZB_POLL_ATTR_SHORT_POLL_INTERVAL    0x0002  /**< uint16 (quarter-seconds) */
#define ZB_POLL_ATTR_FAST_POLL_TIMEOUT      0x0003  /**< uint16 (quarter-seconds) */
#define ZB_POLL_ATTR_CHECK_IN_INTERVAL_MIN  0x0004  /**< uint32 (quarter-seconds) */
#define ZB_POLL_ATTR_LONG_POLL_INTERVAL_MIN 0x0005  /**< uint32 (quarter-seconds) */
#define ZB_POLL_ATTR_FAST_POLL_TIMEOUT_MAX  0x0006  /**< uint16 (quarter-seconds) */

/**
 * @brief Poll Control Server Commands (received by coordinator)
 */
#define ZB_POLL_CMD_CHECK_IN                0x00    /**< Device checking in */

/**
 * @brief Poll Control Client Commands (sent by coordinator)
 */
#define ZB_POLL_CMD_CHECK_IN_RESPONSE       0x00    /**< Response to check-in */
#define ZB_POLL_CMD_FAST_POLL_STOP          0x01    /**< Stop fast polling */
#define ZB_POLL_CMD_SET_LONG_POLL_INTERVAL  0x02    /**< Set long poll interval */
#define ZB_POLL_CMD_SET_SHORT_POLL_INTERVAL 0x03    /**< Set short poll interval */

/**
 * @brief Default Check-In Interval (1 hour = 14400 quarter-seconds)
 */
#define ZB_POLL_DEFAULT_CHECK_IN_INTERVAL   14400

/**
 * @brief Minimum Check-In Interval (5 minutes = 1200 quarter-seconds)
 */
#define ZB_POLL_MIN_CHECK_IN_INTERVAL       1200

/**
 * @brief Maximum Check-In Interval (24 hours = 345600 quarter-seconds)
 */
#define ZB_POLL_MAX_CHECK_IN_INTERVAL       345600

/**
 * @brief Default Long Poll Interval (7.5 seconds = 30 quarter-seconds)
 */
#define ZB_POLL_DEFAULT_LONG_POLL_INTERVAL  30

/**
 * @brief Default Short Poll Interval (0.5 seconds = 2 quarter-seconds)
 */
#define ZB_POLL_DEFAULT_SHORT_POLL_INTERVAL 2

/**
 * @brief Default Fast Poll Timeout (10 seconds = 40 quarter-seconds)
 */
#define ZB_POLL_DEFAULT_FAST_POLL_TIMEOUT   40

/**
 * @brief Maximum tracked sleepy devices
 */
#define ZB_POLL_MAX_DEVICES                 32

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief Poll Control Device Configuration
 *
 * Per-device poll control settings
 */
typedef struct {
    uint32_t check_in_interval;     /**< Check-in interval (quarter-seconds) */
    uint32_t long_poll_interval;    /**< Long poll interval (quarter-seconds) */
    uint16_t short_poll_interval;   /**< Short poll interval (quarter-seconds) */
    uint16_t fast_poll_timeout;     /**< Fast poll timeout (quarter-seconds) */
    bool start_fast_polling;        /**< Start fast poll on check-in */
} zb_poll_control_config_t;

/**
 * @brief Default Poll Control Configuration
 */
#define ZB_POLL_CONTROL_CONFIG_DEFAULT { \
    .check_in_interval = ZB_POLL_DEFAULT_CHECK_IN_INTERVAL, \
    .long_poll_interval = ZB_POLL_DEFAULT_LONG_POLL_INTERVAL, \
    .short_poll_interval = ZB_POLL_DEFAULT_SHORT_POLL_INTERVAL, \
    .fast_poll_timeout = ZB_POLL_DEFAULT_FAST_POLL_TIMEOUT, \
    .start_fast_polling = false \
}

/**
 * @brief Poll Control Device State
 */
typedef struct {
    uint16_t short_addr;            /**< Device short address */
    uint8_t endpoint;               /**< Device endpoint with Poll Control */
    uint8_t ieee_addr[8];           /**< Device IEEE address */
    zb_poll_control_config_t config;/**< Device configuration */
    uint32_t last_check_in;         /**< Timestamp of last check-in (ticks) */
    bool bound;                     /**< Device is bound to coordinator */
    bool pending_config;            /**< Configuration change pending */
} zb_poll_control_device_t;

/**
 * @brief Check-In Callback
 *
 * Called when a sleepy device checks in.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] user_data User data passed to registration
 */
typedef void (*zb_poll_check_in_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                      void *user_data);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize Poll Control Cluster Client
 *
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_init(void);

/**
 * @brief Deinitialize Poll Control Cluster Client
 *
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_deinit(void);

/**
 * @brief Check if Poll Control is initialized
 *
 * @return true if initialized
 */
bool zb_poll_control_is_initialized(void);

/**
 * @brief Register check-in callback
 *
 * @param[in] callback Callback function
 * @param[in] user_data User data for callback
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_register_callback(zb_poll_check_in_cb_t callback,
                                            void *user_data);

/**
 * @brief Handle Check-In command from device
 *
 * Called when a sleepy device sends a Check-In command.
 * Sends Check-In Response to keep device awake if needed.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_handle_check_in(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set device check-in interval
 *
 * Configures how often a sleepy device wakes up.
 *
 * @param[in] short_addr Device short address
 * @param[in] interval_qs Interval in quarter-seconds
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_poll_control_set_interval(uint16_t short_addr, uint32_t interval_qs);

/**
 * @brief Set device check-in interval in seconds
 *
 * @param[in] short_addr Device short address
 * @param[in] interval_sec Interval in seconds
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_set_interval_seconds(uint16_t short_addr,
                                                uint32_t interval_sec);

/**
 * @brief Get device check-in interval
 *
 * @param[in] short_addr Device short address
 * @param[out] interval_qs Output for interval in quarter-seconds
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_poll_control_get_interval(uint16_t short_addr, uint32_t *interval_qs);

/**
 * @brief Set device configuration
 *
 * @param[in] short_addr Device short address
 * @param[in] config Configuration to apply
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_set_config(uint16_t short_addr,
                                     const zb_poll_control_config_t *config);

/**
 * @brief Get device configuration
 *
 * @param[in] short_addr Device short address
 * @param[out] config Output for configuration
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_poll_control_get_config(uint16_t short_addr,
                                     zb_poll_control_config_t *config);

/**
 * @brief Send Check-In Response to device
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] start_fast_poll Start fast polling mode
 * @param[in] fast_poll_timeout Timeout in quarter-seconds (ignored if start_fast_poll=false)
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_send_check_in_response(uint16_t short_addr,
                                                  uint8_t endpoint,
                                                  bool start_fast_poll,
                                                  uint16_t fast_poll_timeout);

/**
 * @brief Send Fast Poll Stop command to device
 *
 * Tells device to return to normal (long) polling.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_send_fast_poll_stop(uint16_t short_addr,
                                               uint8_t endpoint);

/**
 * @brief Send Set Long Poll Interval command
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] interval_qs New interval in quarter-seconds
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_send_set_long_poll(uint16_t short_addr,
                                              uint8_t endpoint,
                                              uint32_t interval_qs);

/**
 * @brief Send Set Short Poll Interval command
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] interval_qs New interval in quarter-seconds
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_send_set_short_poll(uint16_t short_addr,
                                               uint8_t endpoint,
                                               uint16_t interval_qs);

/**
 * @brief Add device to poll control tracking
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] ieee_addr Device IEEE address (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if device list full
 */
esp_err_t zb_poll_control_add_device(uint16_t short_addr, uint8_t endpoint,
                                     const uint8_t *ieee_addr);

/**
 * @brief Remove device from poll control tracking
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 */
esp_err_t zb_poll_control_remove_device(uint16_t short_addr);

/**
 * @brief Get device tracking info
 *
 * @param[in] short_addr Device short address
 * @param[out] device Output for device info
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not tracked
 */
esp_err_t zb_poll_control_get_device(uint16_t short_addr,
                                     zb_poll_control_device_t *device);

/**
 * @brief Get all tracked devices
 *
 * @param[out] devices Output array
 * @param[in] max_count Maximum devices to return
 * @return Number of devices copied
 */
size_t zb_poll_control_get_all_devices(zb_poll_control_device_t *devices,
                                       size_t max_count);

/**
 * @brief Get number of tracked devices
 *
 * @return Number of tracked devices
 */
size_t zb_poll_control_get_device_count(void);

/**
 * @brief Convert quarter-seconds to seconds
 *
 * @param[in] quarter_seconds Time in quarter-seconds
 * @return Time in seconds
 */
static inline uint32_t zb_poll_qs_to_seconds(uint32_t quarter_seconds)
{
    return quarter_seconds / 4;
}

/**
 * @brief Convert seconds to quarter-seconds
 *
 * @param[in] seconds Time in seconds
 * @return Time in quarter-seconds
 */
static inline uint32_t zb_poll_seconds_to_qs(uint32_t seconds)
{
    return seconds * 4;
}

/**
 * @brief Self-test function
 *
 * @return ESP_OK if all tests pass
 */
esp_err_t zb_poll_control_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_POLL_CONTROL_H */
