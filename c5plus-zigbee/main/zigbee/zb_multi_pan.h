/**
 * @file zb_multi_pan.h
 * @brief Multiple Zigbee Network (Multi-PAN) Support API
 *
 * This module provides experimental support for managing multiple Zigbee networks
 * on a single 802.15.4 radio using time-division multiplexing. This allows
 * device isolation (e.g., test network vs. production) without additional hardware.
 *
 * @note EXPERIMENTAL FEATURE - Not recommended for production use!
 *
 * Limitations:
 * - Higher latency during secondary network active periods
 * - Not all Zigbee operations supported during network switch
 * - Single radio shared via time-slicing (80/20 default ratio)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_MULTI_PAN_H
#define ZB_MULTI_PAN_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of supported networks
 */
#define ZB_MULTI_PAN_MAX_NETWORKS       2

/**
 * @brief Maximum devices per network context
 */
#define ZB_MULTI_PAN_MAX_DEVICES        50

/**
 * @brief Network key length in bytes
 */
#define ZB_MULTI_PAN_KEY_LEN            16

/**
 * @brief Extended PAN ID length in bytes
 */
#define ZB_MULTI_PAN_EXT_PAN_ID_LEN     8

/**
 * @brief Minimum switch interval in milliseconds
 */
#define ZB_MULTI_PAN_MIN_SWITCH_MS      100

/**
 * @brief Default switch interval in milliseconds
 */
#define ZB_MULTI_PAN_DEFAULT_SWITCH_MS  1000

/**
 * @brief Minimum valid Zigbee channel
 */
#define ZB_MULTI_PAN_CHANNEL_MIN        11

/**
 * @brief Maximum valid Zigbee channel
 */
#define ZB_MULTI_PAN_CHANNEL_MAX        26

/**
 * @brief Minimum time ratio for primary network (%)
 */
#define ZB_MULTI_PAN_MIN_RATIO          50

/**
 * @brief Maximum time ratio for primary network (%)
 */
#define ZB_MULTI_PAN_MAX_RATIO          95

/**
 * @brief Default time ratio for primary network (%)
 */
#define ZB_MULTI_PAN_DEFAULT_RATIO      80

/**
 * @brief Network identifier enumeration
 */
typedef enum {
    ZB_MULTI_PAN_NETWORK_PRIMARY = 0,   /**< Primary network (default) */
    ZB_MULTI_PAN_NETWORK_SECONDARY,     /**< Secondary network */
    ZB_MULTI_PAN_NETWORK_NONE = -1      /**< No network / invalid */
} zb_multi_pan_network_id_t;

/**
 * @brief Multi-PAN state enumeration
 */
typedef enum {
    ZB_MULTI_PAN_STATE_DISABLED = 0,    /**< Multi-PAN disabled */
    ZB_MULTI_PAN_STATE_INITIALIZING,    /**< Multi-PAN initializing */
    ZB_MULTI_PAN_STATE_RUNNING,         /**< Multi-PAN active and running */
    ZB_MULTI_PAN_STATE_SWITCHING,       /**< Currently switching networks */
    ZB_MULTI_PAN_STATE_ERROR            /**< Error state */
} zb_multi_pan_state_t;

/**
 * @brief Network switch reason enumeration
 */
typedef enum {
    ZB_MULTI_PAN_SWITCH_TIMER = 0,      /**< Automatic timer-based switch */
    ZB_MULTI_PAN_SWITCH_MANUAL,         /**< Manual switch request */
    ZB_MULTI_PAN_SWITCH_PRIORITY,       /**< Priority event (e.g., join request) */
    ZB_MULTI_PAN_SWITCH_EMERGENCY       /**< Emergency switch */
} zb_multi_pan_switch_reason_t;

/**
 * @brief Device entry in network context
 */
typedef struct {
    uint16_t short_addr;                                /**< Network short address */
    uint8_t ieee_addr[8];                               /**< IEEE 64-bit address */
    uint8_t device_type;                                /**< Device type */
    bool online;                                        /**< Device online status */
    uint32_t last_seen;                                 /**< Last seen timestamp (ms) */
} zb_multi_pan_device_t;

/**
 * @brief Network context structure
 *
 * Contains all state information for a single Zigbee network.
 */
typedef struct {
    /* Network identification */
    uint16_t pan_id;                                    /**< PAN ID */
    uint8_t ext_pan_id[ZB_MULTI_PAN_EXT_PAN_ID_LEN];   /**< Extended PAN ID */
    uint8_t channel;                                    /**< Radio channel (11-26) */

    /* Security */
    uint8_t network_key[ZB_MULTI_PAN_KEY_LEN];         /**< Network encryption key */
    bool key_set;                                       /**< Network key configured */

    /* Network state */
    bool formed;                                        /**< Network formed flag */
    bool permit_join;                                   /**< Permit join enabled */
    uint8_t permit_join_remaining;                      /**< Permit join seconds remaining */

    /* Device management */
    zb_multi_pan_device_t devices[ZB_MULTI_PAN_MAX_DEVICES]; /**< Device table */
    uint8_t device_count;                               /**< Number of devices */

    /* Statistics */
    uint32_t active_time_ms;                            /**< Total active time (ms) */
    uint32_t switch_count;                              /**< Number of switches to this network */
    uint32_t last_active;                               /**< Last active timestamp */

    /* Configuration */
    char name[32];                                      /**< Human-readable network name */
    bool enabled;                                       /**< Network enabled */
} zb_multi_pan_network_ctx_t;

/**
 * @brief Multi-PAN configuration structure
 */
typedef struct {
    uint8_t primary_channel;                            /**< Primary network channel */
    uint8_t secondary_channel;                          /**< Secondary network channel */
    uint8_t time_ratio;                                 /**< Primary network time ratio (%) */
    uint32_t switch_interval_ms;                        /**< Base switch interval (ms) */
    bool graceful_switch;                               /**< Wait for active transactions */
    uint32_t switch_timeout_ms;                         /**< Timeout for graceful switch */
} zb_multi_pan_config_t;

/**
 * @brief Multi-PAN status structure
 */
typedef struct {
    zb_multi_pan_state_t state;                         /**< Current state */
    zb_multi_pan_network_id_t active_network;           /**< Currently active network */
    uint32_t uptime_ms;                                 /**< Total uptime */
    uint32_t total_switches;                            /**< Total network switches */
    uint32_t failed_switches;                           /**< Failed switch attempts */
    uint32_t pending_transactions;                      /**< Pending Zigbee transactions */
} zb_multi_pan_status_t;

/**
 * @brief Network switch callback type
 *
 * @param from_network Network being switched from
 * @param to_network Network being switched to
 * @param reason Switch reason
 * @param user_data User-provided callback data
 */
typedef void (*zb_multi_pan_switch_cb_t)(
    zb_multi_pan_network_id_t from_network,
    zb_multi_pan_network_id_t to_network,
    zb_multi_pan_switch_reason_t reason,
    void *user_data
);

/**
 * @brief Initialize Multi-PAN module
 *
 * Initializes the Multi-PAN module with default configuration.
 * Must be called before any other Multi-PAN functions.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_ERR_NOT_SUPPORTED if Multi-PAN not enabled in Kconfig
 */
esp_err_t zb_multi_pan_init(void);

/**
 * @brief Deinitialize Multi-PAN module
 *
 * Stops all Multi-PAN operations and frees resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_multi_pan_deinit(void);

/**
 * @brief Start Multi-PAN operation
 *
 * Begins time-division multiplexing between networks.
 * At least one network must be added before starting.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized or no networks
 */
esp_err_t zb_multi_pan_start(void);

/**
 * @brief Stop Multi-PAN operation
 *
 * Stops time-division multiplexing. Primary network remains active.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not running
 */
esp_err_t zb_multi_pan_stop(void);

/**
 * @brief Add a network to Multi-PAN management
 *
 * Adds a new network context. Maximum of ZB_MULTI_PAN_MAX_NETWORKS supported.
 *
 * @param network_id Network identifier (PRIMARY or SECONDARY)
 * @param pan_id PAN ID for the network
 * @param channel Radio channel (11-26)
 * @param name Human-readable network name (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters invalid
 * @return ESP_ERR_NO_MEM if maximum networks reached
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_multi_pan_add_network(
    zb_multi_pan_network_id_t network_id,
    uint16_t pan_id,
    uint8_t channel,
    const char *name
);

/**
 * @brief Remove a network from Multi-PAN management
 *
 * Removes a network context and its associated device table.
 *
 * @param network_id Network to remove
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if network not found
 * @return ESP_ERR_INVALID_STATE if network is currently active
 */
esp_err_t zb_multi_pan_remove_network(zb_multi_pan_network_id_t network_id);

/**
 * @brief Manually switch to a specific network
 *
 * Immediately switches to the specified network. If graceful_switch is enabled
 * in configuration, waits for pending transactions to complete.
 *
 * @param network_id Target network
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if network not configured
 * @return ESP_ERR_INVALID_STATE if not running
 * @return ESP_ERR_TIMEOUT if graceful switch timeout
 */
esp_err_t zb_multi_pan_switch(zb_multi_pan_network_id_t network_id);

/**
 * @brief Get currently active network
 *
 * @return Currently active network ID
 * @return ZB_MULTI_PAN_NETWORK_NONE if not running
 */
zb_multi_pan_network_id_t zb_multi_pan_get_active(void);

/**
 * @brief Get network context
 *
 * Retrieves the context structure for a network.
 *
 * @param network_id Network to query
 * @param ctx Pointer to receive context (copy)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if ctx is NULL
 * @return ESP_ERR_NOT_FOUND if network not configured
 */
esp_err_t zb_multi_pan_get_network_ctx(
    zb_multi_pan_network_id_t network_id,
    zb_multi_pan_network_ctx_t *ctx
);

/**
 * @brief Set network configuration
 *
 * Updates the Multi-PAN configuration. Changes take effect immediately
 * if already running.
 *
 * @param config New configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config invalid
 */
esp_err_t zb_multi_pan_set_config(const zb_multi_pan_config_t *config);

/**
 * @brief Get current configuration
 *
 * @param config Pointer to receive configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_multi_pan_get_config(zb_multi_pan_config_t *config);

/**
 * @brief Get Multi-PAN status
 *
 * @param status Pointer to receive status
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if status is NULL
 */
esp_err_t zb_multi_pan_get_status(zb_multi_pan_status_t *status);

/**
 * @brief Register network switch callback
 *
 * Registers a callback to be notified of network switches.
 *
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return ESP_OK on success
 */
esp_err_t zb_multi_pan_register_switch_callback(
    zb_multi_pan_switch_cb_t callback,
    void *user_data
);

/**
 * @brief Set network key for a network
 *
 * @param network_id Target network
 * @param key Network key (16 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if key is NULL
 * @return ESP_ERR_NOT_FOUND if network not configured
 */
esp_err_t zb_multi_pan_set_network_key(
    zb_multi_pan_network_id_t network_id,
    const uint8_t key[ZB_MULTI_PAN_KEY_LEN]
);

/**
 * @brief Set extended PAN ID for a network
 *
 * @param network_id Target network
 * @param ext_pan_id Extended PAN ID (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if ext_pan_id is NULL
 * @return ESP_ERR_NOT_FOUND if network not configured
 */
esp_err_t zb_multi_pan_set_ext_pan_id(
    zb_multi_pan_network_id_t network_id,
    const uint8_t ext_pan_id[ZB_MULTI_PAN_EXT_PAN_ID_LEN]
);

/**
 * @brief Enable permit join on a network
 *
 * @param network_id Target network
 * @param duration Permit join duration in seconds (0=disable, 255=forever)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if network not configured
 */
esp_err_t zb_multi_pan_permit_join(
    zb_multi_pan_network_id_t network_id,
    uint8_t duration
);

/**
 * @brief Add device to network context
 *
 * @param network_id Target network
 * @param short_addr Device short address
 * @param ieee_addr Device IEEE address (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if device table full
 * @return ESP_ERR_NOT_FOUND if network not configured
 */
esp_err_t zb_multi_pan_add_device(
    zb_multi_pan_network_id_t network_id,
    uint16_t short_addr,
    const uint8_t ieee_addr[8]
);

/**
 * @brief Remove device from network context
 *
 * @param network_id Target network
 * @param short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device or network not found
 */
esp_err_t zb_multi_pan_remove_device(
    zb_multi_pan_network_id_t network_id,
    uint16_t short_addr
);

/**
 * @brief Save Multi-PAN configuration to NVS
 *
 * Persists all network contexts and configuration to non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_multi_pan_save_config(void);

/**
 * @brief Load Multi-PAN configuration from NVS
 *
 * Loads previously saved configuration from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no saved configuration
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_multi_pan_load_config(void);

/**
 * @brief Reset Multi-PAN configuration
 *
 * Erases all saved configuration and resets to defaults.
 * WARNING: This is a destructive operation!
 *
 * @return ESP_OK on success
 */
esp_err_t zb_multi_pan_reset_config(void);

/**
 * @brief Check if Multi-PAN is enabled in Kconfig
 *
 * @return true if enabled
 * @return false if disabled
 */
bool zb_multi_pan_is_enabled(void);

/**
 * @brief Check if Multi-PAN is currently running
 *
 * @return true if running
 * @return false otherwise
 */
bool zb_multi_pan_is_running(void);

/**
 * @brief Get state as string
 *
 * @param state State to convert
 * @return String representation
 */
const char *zb_multi_pan_state_to_str(zb_multi_pan_state_t state);

/**
 * @brief Get network ID as string
 *
 * @param network_id Network ID to convert
 * @return String representation
 */
const char *zb_multi_pan_network_to_str(zb_multi_pan_network_id_t network_id);

/**
 * @brief Self-test function
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_multi_pan_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_MULTI_PAN_H */
