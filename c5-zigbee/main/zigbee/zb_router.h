/**
 * @file zb_router.h
 * @brief Zigbee Router API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee router functionality allowing the gateway
 * to operate as a mesh extender in an existing Zigbee network. The router
 * mode is an alternative to coordinator mode - only one can be active.
 *
 * Use cases:
 * - Extend Zigbee mesh coverage in large installations
 * - Bridge between Zigbee network and MQTT while another coordinator exists
 * - Report Zigbee device data to Home Assistant via MQTT
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_ROUTER_H
#define ZB_ROUTER_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of neighbors in the neighbor table
 */
#define ZB_ROUTER_MAX_NEIGHBORS     16

/**
 * @brief Maximum length for friendly names
 */
#define ZB_ROUTER_NAME_MAX_LEN      64

/**
 * @brief Zigbee router state enumeration
 */
typedef enum {
    ZB_ROUTER_STATE_UNINITIALIZED = 0,  /**< Router not initialized */
    ZB_ROUTER_STATE_INITIALIZED,        /**< Router initialized but not started */
    ZB_ROUTER_STATE_SEARCHING,          /**< Searching for networks to join */
    ZB_ROUTER_STATE_JOINING,            /**< Joining a network */
    ZB_ROUTER_STATE_JOINED,             /**< Successfully joined a network */
    ZB_ROUTER_STATE_LOST,               /**< Lost connection to parent/network */
    ZB_ROUTER_STATE_ERROR,              /**< Router in error state */
    ZB_ROUTER_STATE_STOPPED             /**< Router stopped */
} zb_router_state_t;

/**
 * @brief Network scan result structure
 */
typedef struct {
    uint16_t pan_id;                    /**< PAN ID of the network */
    uint8_t extended_pan_id[8];         /**< Extended PAN ID */
    uint8_t channel;                    /**< Radio channel */
    uint8_t lqi;                        /**< Link Quality Indicator */
    int8_t rssi;                        /**< Received Signal Strength */
    bool permit_join;                   /**< Network allows joining */
    uint8_t depth;                      /**< Network depth */
} zb_router_network_t;

/**
 * @brief Parent (coordinator or router) information structure
 */
typedef struct {
    uint16_t short_addr;                /**< Parent short address */
    uint8_t ieee_addr[8];               /**< Parent IEEE address */
    uint8_t lqi;                        /**< Link Quality to parent */
    int8_t rssi;                        /**< Signal strength to parent */
    uint8_t depth;                      /**< Our network depth */
    bool is_coordinator;                /**< true if parent is coordinator */
    uint32_t last_seen;                 /**< Timestamp of last communication */
} zb_router_parent_info_t;

/**
 * @brief Neighbor entry structure
 */
typedef struct {
    uint16_t short_addr;                /**< Neighbor short address */
    uint8_t ieee_addr[8];               /**< Neighbor IEEE address */
    uint8_t lqi;                        /**< Link Quality Indicator */
    int8_t rssi;                        /**< Signal strength */
    uint8_t device_type;                /**< 0=Coord, 1=Router, 2=EndDevice */
    bool rx_on_idle;                    /**< Receiver on when idle */
    uint8_t relationship;               /**< 0=Parent, 1=Child, 2=Sibling */
    uint32_t last_seen;                 /**< Timestamp of last activity */
} zb_router_neighbor_t;

/**
 * @brief Router configuration structure
 */
typedef struct {
    bool auto_join;                     /**< Automatically join available network */
    uint16_t preferred_pan_id;          /**< Preferred PAN ID (0xFFFF = any) */
    uint8_t preferred_channel;          /**< Preferred channel (0 = any from mask) */
    uint8_t extended_pan_id[8];         /**< Preferred extended PAN ID (all 0 = any) */
    uint32_t scan_duration_ms;          /**< Network scan duration in ms */
    uint32_t rejoin_interval_ms;        /**< Interval between rejoin attempts */
    uint8_t max_rejoin_attempts;        /**< Max rejoin attempts (0 = infinite) */
    bool route_discovery_enabled;       /**< Enable route discovery */
    uint8_t max_children;               /**< Max end devices that can join via this router */
    char friendly_name[ZB_ROUTER_NAME_MAX_LEN]; /**< Router friendly name for MQTT */
} zb_router_config_t;

/**
 * @brief Router statistics structure
 */
typedef struct {
    uint32_t join_count;                /**< Successful network joins */
    uint32_t rejoin_count;              /**< Rejoin attempts */
    uint32_t packets_routed;            /**< Packets forwarded as router */
    uint32_t packets_received;          /**< Packets received */
    uint32_t packets_sent;              /**< Packets sent */
    uint32_t route_discoveries;         /**< Route discovery operations */
    uint32_t parent_changes;            /**< Number of parent changes */
    uint32_t network_lost_count;        /**< Times network connection was lost */
    uint32_t uptime_seconds;            /**< Time since joining network */
} zb_router_stats_t;

/**
 * @brief Router state change callback type
 *
 * @param[in] old_state Previous router state
 * @param[in] new_state New router state
 */
typedef void (*zb_router_state_callback_t)(zb_router_state_t old_state,
                                           zb_router_state_t new_state);

/**
 * @brief Network found callback type (during scanning)
 *
 * @param[in] network Pointer to network information
 */
typedef void (*zb_router_network_found_callback_t)(const zb_router_network_t *network);

/**
 * @brief Initialize the Zigbee router
 *
 * Initializes the Zigbee stack in router mode, configures endpoints,
 * and prepares for network joining. Must be called before zb_router_start().
 *
 * @note Cannot be used if coordinator mode is already initialized.
 *       The device can only operate as coordinator OR router, not both.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized or coordinator active
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_router_init(void);

/**
 * @brief Start the Zigbee router
 *
 * Begins network discovery and join process. Creates the router task
 * that manages network operations and reconnection logic.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_router_start(void);

/**
 * @brief Stop the Zigbee router
 *
 * Stops the router task, leaves the network, and shuts down the Zigbee stack.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not running
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_router_stop(void);

/**
 * @brief Get current router state
 *
 * Thread-safe retrieval of the current router state.
 *
 * @return Current router state
 */
zb_router_state_t zb_router_get_state(void);

/**
 * @brief Get router state as string
 *
 * Returns human-readable string representation of router state.
 *
 * @param[in] state Router state to convert
 * @return Pointer to static string describing the state
 */
const char *zb_router_state_to_str(zb_router_state_t state);

/**
 * @brief Check if router is connected to network
 *
 * @return true if router is in JOINED state
 * @return false otherwise
 */
bool zb_router_is_joined(void);

/**
 * @brief Get parent information
 *
 * Retrieves information about the router's parent node (coordinator
 * or another router that this device joined through).
 *
 * @param[out] parent_info Pointer to parent info structure to fill
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parent_info is NULL
 * @return ESP_ERR_INVALID_STATE if not joined to network
 */
esp_err_t zb_router_get_parent_info(zb_router_parent_info_t *parent_info);

/**
 * @brief Get neighbor table
 *
 * Retrieves the router's neighbor table containing information about
 * nearby Zigbee devices.
 *
 * @param[out] neighbors Array to store neighbor entries
 * @param[in,out] count Input: array size, Output: number of neighbors found
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if arguments are invalid
 * @return ESP_ERR_INVALID_STATE if not joined
 */
esp_err_t zb_router_get_neighbors(zb_router_neighbor_t *neighbors, uint8_t *count);

/**
 * @brief Get router statistics
 *
 * Retrieves operational statistics for the router.
 *
 * @param[out] stats Pointer to statistics structure to fill
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if stats is NULL
 */
esp_err_t zb_router_get_stats(zb_router_stats_t *stats);

/**
 * @brief Set router configuration
 *
 * Updates router configuration. Some settings only take effect
 * on next join/rejoin.
 *
 * @param[in] config Pointer to new configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_router_set_config(const zb_router_config_t *config);

/**
 * @brief Get current router configuration
 *
 * Retrieves current router configuration.
 *
 * @param[out] config Pointer to configuration structure to fill
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_router_get_config(zb_router_config_t *config);

/**
 * @brief Scan for available Zigbee networks
 *
 * Initiates a network scan. Results are provided via the network_found
 * callback if registered, or can be retrieved after scan completes.
 *
 * @param[in] channel_mask Channels to scan (0 = all channels)
 * @param[in] duration_ms Scan duration per channel in milliseconds
 * @return ESP_OK if scan started
 * @return ESP_ERR_INVALID_STATE if router is busy
 * @return ESP_FAIL on error
 */
esp_err_t zb_router_scan_networks(uint32_t channel_mask, uint32_t duration_ms);

/**
 * @brief Join a specific network
 *
 * Attempts to join a specific Zigbee network.
 *
 * @param[in] pan_id PAN ID of the network to join
 * @param[in] channel Channel of the network
 * @return ESP_OK if join initiated
 * @return ESP_ERR_INVALID_STATE if not in correct state
 * @return ESP_FAIL on error
 */
esp_err_t zb_router_join_network(uint16_t pan_id, uint8_t channel);

/**
 * @brief Leave current network
 *
 * Gracefully leaves the current network and notifies neighbors.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not joined
 */
esp_err_t zb_router_leave_network(void);

/**
 * @brief Force rejoin to network
 *
 * Forces a rejoin operation, useful when connectivity issues are detected.
 *
 * @param[in] secured Use secured rejoin (with network key)
 * @return ESP_OK if rejoin initiated
 * @return ESP_ERR_INVALID_STATE if not previously joined
 */
esp_err_t zb_router_rejoin(bool secured);

/**
 * @brief Register state change callback
 *
 * Registers a callback to be notified when router state changes.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_router_register_state_callback(zb_router_state_callback_t callback);

/**
 * @brief Register network found callback
 *
 * Registers a callback to be notified when networks are found during scan.
 *
 * @param[in] callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t zb_router_register_network_callback(zb_router_network_found_callback_t callback);

/**
 * @brief Publish router status to MQTT
 *
 * Publishes current router state and statistics to MQTT topics.
 * Called automatically on state changes, but can be triggered manually.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if MQTT not connected
 */
esp_err_t zb_router_publish_status(void);

/**
 * @brief Get the router mutex for thread-safe API access
 *
 * @return Mutex handle or NULL if not initialized
 */
SemaphoreHandle_t zb_router_get_mutex(void);

/**
 * @brief Self-test function for router initialization
 *
 * Verifies that the router has been properly initialized.
 * Useful for debugging and validation.
 *
 * @return ESP_OK if all checks pass
 * @return ESP_FAIL if any check fails
 */
esp_err_t zb_router_self_test(void);

/**
 * @brief Get default router configuration
 *
 * Fills a configuration structure with default values from Kconfig.
 *
 * @param[out] config Configuration structure to fill
 * @return ESP_OK on success
 */
esp_err_t zb_router_get_default_config(zb_router_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* ZB_ROUTER_H */
