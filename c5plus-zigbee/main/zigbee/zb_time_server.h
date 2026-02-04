/**
 * @file zb_time_server.h
 * @brief Zigbee Time Cluster Server Implementation (Cluster 0x000A)
 *
 * Provides time synchronization for Zigbee end devices.
 * The coordinator acts as a time server, providing UTC time based on
 * the ESP32's system time (NTP synchronized).
 *
 * Zigbee Time Epoch: 2000-01-01 00:00:00 UTC
 * Unix Time Epoch: 1970-01-01 00:00:00 UTC
 * Offset: 946684800 seconds
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_TIME_SERVER_H
#define ZB_TIME_SERVER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants and Definitions
 * ============================================================================ */

/**
 * @brief Time Cluster ID (ZCL)
 */
#define ZB_ZCL_CLUSTER_ID_TIME              0x000A

/**
 * @brief Time Cluster Attribute IDs
 */
#define ZB_TIME_ATTR_TIME                   0x0000  /**< UTCTime (uint32, seconds since 2000-01-01) */
#define ZB_TIME_ATTR_TIME_STATUS            0x0001  /**< bitmap8 */
#define ZB_TIME_ATTR_TIME_ZONE              0x0002  /**< int32 (seconds offset from UTC) */
#define ZB_TIME_ATTR_DST_START              0x0003  /**< uint32 (UTC time DST starts) */
#define ZB_TIME_ATTR_DST_END                0x0004  /**< uint32 (UTC time DST ends) */
#define ZB_TIME_ATTR_DST_SHIFT              0x0005  /**< int32 (DST offset in seconds) */
#define ZB_TIME_ATTR_STANDARD_TIME          0x0006  /**< uint32 (standard time = UTC + timezone) */
#define ZB_TIME_ATTR_LOCAL_TIME             0x0007  /**< uint32 (local time = standard + DST) */
#define ZB_TIME_ATTR_LAST_SET_TIME          0x0008  /**< UTCTime (when time was last set) */
#define ZB_TIME_ATTR_VALID_UNTIL_TIME       0x0009  /**< UTCTime (time valid until) */

/**
 * @brief Time Status Bits
 */
#define ZB_TIME_STATUS_MASTER               (1 << 0)  /**< Device is a time master */
#define ZB_TIME_STATUS_SYNCHRONIZED         (1 << 1)  /**< Time is synchronized (e.g., NTP) */
#define ZB_TIME_STATUS_MASTER_ZONE_DST      (1 << 2)  /**< Device is master for zone/DST */
#define ZB_TIME_STATUS_SUPERSEDING          (1 << 3)  /**< Time may supersede other masters */

/**
 * @brief Zigbee Time Epoch offset from Unix Epoch
 *
 * Zigbee uses 2000-01-01 00:00:00 UTC as epoch.
 * Unix uses 1970-01-01 00:00:00 UTC as epoch.
 * Difference: 30 years = 946684800 seconds
 */
#define ZB_TIME_EPOCH_OFFSET_SECONDS        946684800UL

/**
 * @brief Invalid time value (0xFFFFFFFF)
 */
#define ZB_TIME_INVALID                     0xFFFFFFFFUL

/**
 * @brief Default timezone offset (0 = UTC)
 */
#define ZB_TIME_DEFAULT_TIMEZONE            0

/**
 * @brief Default DST shift (0 = no DST)
 */
#define ZB_TIME_DEFAULT_DST_SHIFT           0

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief Time Cluster Server Configuration
 */
typedef struct {
    int32_t timezone_offset;        /**< Timezone offset in seconds (e.g., 3600 for UTC+1) */
    int32_t dst_shift;              /**< DST shift in seconds (e.g., 3600 for +1 hour) */
    uint32_t dst_start;             /**< DST start time (Zigbee UTC) */
    uint32_t dst_end;               /**< DST end time (Zigbee UTC) */
    bool is_master;                 /**< Act as time master */
    bool is_synchronized;           /**< Time is synchronized (NTP) */
} zb_time_server_config_t;

/**
 * @brief Time Cluster Current State
 */
typedef struct {
    uint32_t utc_time;              /**< Current UTC time (Zigbee epoch) */
    uint8_t time_status;            /**< Time status bitmap */
    int32_t timezone;               /**< Timezone offset in seconds */
    uint32_t dst_start;             /**< DST start time */
    uint32_t dst_end;               /**< DST end time */
    int32_t dst_shift;              /**< DST shift in seconds */
    uint32_t standard_time;         /**< Standard time (UTC + timezone) */
    uint32_t local_time;            /**< Local time (standard + DST if active) */
    uint32_t last_set_time;         /**< When time was last set */
    uint32_t valid_until_time;      /**< Time valid until */
} zb_time_server_state_t;

/**
 * @brief Callback for time read requests from devices
 *
 * @param[in] short_addr Requesting device short address
 * @param[in] endpoint Requesting device endpoint
 * @param[in] attr_id Requested attribute ID
 * @param[out] value Output buffer for attribute value
 * @param[in] max_len Maximum output buffer length
 * @return Actual data length written
 */
typedef size_t (*zb_time_read_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t attr_id, void *value, size_t max_len);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize Time Cluster Server
 *
 * Initializes the time cluster server with default configuration.
 * Should be called after coordinator initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NO_MEM on memory allocation failure
 */
esp_err_t zb_time_server_init(void);

/**
 * @brief Deinitialize Time Cluster Server
 *
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_deinit(void);

/**
 * @brief Check if Time Server is initialized
 *
 * @return true if initialized
 */
bool zb_time_server_is_initialized(void);

/**
 * @brief Set Time Server Configuration
 *
 * @param[in] config Configuration to apply
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_time_server_set_config(const zb_time_server_config_t *config);

/**
 * @brief Get Time Server Configuration
 *
 * @param[out] config Output for current configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_time_server_get_config(zb_time_server_config_t *config);

/**
 * @brief Set current UTC time
 *
 * Sets the current time using Zigbee UTC format (seconds since 2000-01-01).
 *
 * @param[in] utc_time Time in Zigbee UTC format
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_set_time(uint32_t utc_time);

/**
 * @brief Set current time from Unix timestamp
 *
 * Converts Unix timestamp to Zigbee UTC and sets it.
 *
 * @param[in] unix_time Unix timestamp (seconds since 1970-01-01)
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_set_time_unix(time_t unix_time);

/**
 * @brief Get current UTC time
 *
 * Returns current time in Zigbee UTC format (seconds since 2000-01-01).
 *
 * @return Current UTC time, or ZB_TIME_INVALID if not synchronized
 */
uint32_t zb_time_server_get_time(void);

/**
 * @brief Get current time as Unix timestamp
 *
 * @return Unix timestamp, or 0 if not synchronized
 */
time_t zb_time_server_get_time_unix(void);

/**
 * @brief Set timezone offset
 *
 * @param[in] offset_seconds Timezone offset in seconds (e.g., 3600 for UTC+1)
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_set_timezone(int32_t offset_seconds);

/**
 * @brief Get timezone offset
 *
 * @return Timezone offset in seconds
 */
int32_t zb_time_server_get_timezone(void);

/**
 * @brief Set DST parameters
 *
 * @param[in] dst_start DST start time (Zigbee UTC)
 * @param[in] dst_end DST end time (Zigbee UTC)
 * @param[in] dst_shift DST shift in seconds (typically 3600)
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_set_dst(uint32_t dst_start, uint32_t dst_end, int32_t dst_shift);

/**
 * @brief Check if DST is currently active
 *
 * @return true if DST is active
 */
bool zb_time_server_is_dst_active(void);

/**
 * @brief Get current Time Server state
 *
 * Returns complete current state of the time server.
 *
 * @param[out] state Output for current state
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if state is NULL
 */
esp_err_t zb_time_server_get_state(zb_time_server_state_t *state);

/**
 * @brief Mark time as synchronized
 *
 * Call this after successful NTP synchronization.
 *
 * @param[in] synchronized true if synchronized
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_set_synchronized(bool synchronized);

/**
 * @brief Check if time is synchronized
 *
 * @return true if time is synchronized (e.g., via NTP)
 */
bool zb_time_server_is_synchronized(void);

/**
 * @brief Handle Time Cluster Read Attribute request
 *
 * Called when a device requests to read a Time Cluster attribute.
 *
 * @param[in] short_addr Requesting device
 * @param[in] endpoint Requesting endpoint
 * @param[in] attr_id Attribute ID to read
 * @param[out] value Output buffer
 * @param[in] max_len Maximum output length
 * @return Actual data length, or 0 on error
 */
size_t zb_time_server_handle_read(uint16_t short_addr, uint8_t endpoint,
                                  uint16_t attr_id, void *value, size_t max_len);

/**
 * @brief Register Time Cluster with Zigbee stack
 *
 * Registers the Time Cluster as a server on the coordinator endpoint.
 *
 * @param[in] endpoint Endpoint to register on
 * @return ESP_OK on success
 */
esp_err_t zb_time_server_register_cluster(uint8_t endpoint);

/**
 * @brief Convert Unix timestamp to Zigbee UTC
 *
 * @param[in] unix_time Unix timestamp
 * @return Zigbee UTC time
 */
uint32_t zb_time_unix_to_zigbee(time_t unix_time);

/**
 * @brief Convert Zigbee UTC to Unix timestamp
 *
 * @param[in] zigbee_time Zigbee UTC time
 * @return Unix timestamp
 */
time_t zb_time_zigbee_to_unix(uint32_t zigbee_time);

/**
 * @brief Self-test function
 *
 * @return ESP_OK if all tests pass
 */
esp_err_t zb_time_server_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_TIME_SERVER_H */
