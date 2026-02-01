/**
 * @file zb_network.h
 * @brief Zigbee Network Management API
 *
 * This module manages Zigbee network configuration, persistence,
 * and network-level operations.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_NETWORK_H
#define ZB_NETWORK_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "zb_constants.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Zigbee network key length
 */
#define ZB_NETWORK_KEY_LEN 16

/* ============================================================================
 * Network Default Values
 * ============================================================================ */

/** @brief Default PAN ID when CONFIG_ZIGBEE_PAN_ID is not defined */
#define ZB_DEFAULT_PAN_ID                   0x1A62

/** @brief Default radio channel (11-26 for 2.4GHz Zigbee)
 *  @note Uses ZB_DEFAULT_PRIMARY_CHANNEL from zb_constants.h */
#define ZB_DEFAULT_CHANNEL                  ZB_DEFAULT_PRIMARY_CHANNEL

/* Note: ZB_DEFAULT_MAX_CHILDREN (32) is defined by ESP-Zigbee-SDK */

/** @brief Default TX power in dBm */
#define ZB_DEFAULT_TX_POWER                 20

/** @brief Default permit join duration in seconds (reduced for radio coexistence) */
#define ZB_DEFAULT_PERMIT_JOIN_DURATION_SEC 60

/**
 * @brief Zigbee network information structure
 */
typedef struct {
    uint16_t pan_id;                        /**< PAN ID */
    uint8_t channel;                        /**< Radio channel (11-26) */
    uint8_t extended_pan_id[8];             /**< Extended PAN ID */
    uint8_t network_key[ZB_NETWORK_KEY_LEN]; /**< Network encryption key */
    bool permit_join;                       /**< Permit join status */
    uint8_t device_count;                   /**< Number of joined devices */
    uint16_t short_addr;                    /**< Coordinator short address (0x0000) */
    uint8_t depth;                          /**< Network depth */
    bool network_formed;                    /**< Network formation status */
} zb_network_info_t;

/**
 * @brief Initialize network manager
 *
 * Initializes the network manager and loads stored configuration from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 */
esp_err_t zb_network_init(void);

/**
 * @brief Deinitialize network manager
 *
 * Frees all resources used by the network manager.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_network_deinit(void);

/**
 * @brief Get network information
 *
 * Retrieves current network configuration and status.
 *
 * @param info Pointer to network info structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if info is NULL
 * @return ESP_ERR_INVALID_STATE if network not formed
 */
esp_err_t zb_network_get_info(zb_network_info_t *info);

/**
 * @brief Save network configuration to NVS
 *
 * Persists the current network configuration to non-volatile storage.
 * This allows the coordinator to restore the network after reboot.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_network_save_config(void);

/**
 * @brief Load network configuration from NVS
 *
 * Loads previously saved network configuration from non-volatile storage.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no stored configuration exists
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_network_load_config(void);

/**
 * @brief Reset network configuration
 *
 * Erases stored network configuration and forces creation of a new network.
 * All devices will need to re-pair after this operation.
 *
 * WARNING: This is a destructive operation!
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on error
 */
esp_err_t zb_network_reset(void);

/**
 * @brief Check if stored network configuration exists
 *
 * Checks if valid network configuration is stored in NVS.
 *
 * @return true if configuration exists
 * @return false otherwise
 */
bool zb_network_has_stored_config(void);

/**
 * @brief Update permit join status
 *
 * Updates the internal permit join status.
 * Called by the coordinator when permit join state changes.
 *
 * @param permit_join New permit join status
 * @return ESP_OK on success
 */
esp_err_t zb_network_set_permit_join(bool permit_join);

/**
 * @brief Update device count
 *
 * Updates the internal device count.
 * Called by device handler when devices join/leave.
 *
 * @param count New device count
 * @return ESP_OK on success
 */
esp_err_t zb_network_set_device_count(uint8_t count);

/**
 * @brief Mark network as formed
 *
 * Sets the network_formed flag after successful network creation.
 *
 * @param formed Network formation status
 * @return ESP_OK on success
 */
esp_err_t zb_network_set_formed(bool formed);

/**
 * @brief Get PAN ID from Kconfig
 *
 * @return Configured PAN ID
 */
uint16_t zb_network_get_pan_id_config(void);

/**
 * @brief Get channel from Kconfig
 *
 * @return Configured channel (11-26)
 */
uint8_t zb_network_get_channel_config(void);

/**
 * @brief Get max children from Kconfig
 *
 * @return Configured maximum number of children
 */
uint8_t zb_network_get_max_children_config(void);

/**
 * @brief Get permit join duration from Kconfig
 *
 * @return Configured permit join duration in seconds
 */
uint8_t zb_network_get_permit_join_duration_config(void);

/**
 * @brief Self-test function for network manager
 *
 * Tests network info retrieval and configuration.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_network_test(void);

/* ============================================================================
 * Network Channel Change (ZG-016)
 * ============================================================================ */

/**
 * @brief Channel change state
 */
typedef enum {
    ZB_CHANNEL_CHANGE_IDLE = 0,      /**< No channel change in progress */
    ZB_CHANNEL_CHANGE_PENDING,        /**< Channel change requested, waiting */
    ZB_CHANNEL_CHANGE_NOTIFYING,      /**< Notifying devices of change */
    ZB_CHANNEL_CHANGE_SWITCHING,      /**< Switching to new channel */
    ZB_CHANNEL_CHANGE_COMPLETE,       /**< Channel change completed */
    ZB_CHANNEL_CHANGE_FAILED          /**< Channel change failed */
} zb_channel_change_state_t;

/**
 * @brief Channel change result callback
 *
 * @param[in] result ESP_OK on success, error code on failure
 * @param[in] old_channel Previous channel
 * @param[in] new_channel New channel
 * @param[in] user_data User data passed to change request
 */
typedef void (*zb_channel_change_cb_t)(esp_err_t result, uint8_t old_channel,
                                       uint8_t new_channel, void *user_data);

/**
 * @brief Validate Zigbee channel
 *
 * Checks if channel is valid (11-26 for 2.4GHz Zigbee).
 *
 * @param[in] channel Channel to validate
 * @return true if channel is valid
 * @return false if channel is invalid
 */
bool zb_network_validate_channel(uint8_t channel);

/**
 * @brief Check if channel change is pending
 *
 * @return true if a channel change is in progress
 */
bool zb_network_is_channel_change_pending(void);

/**
 * @brief Get current channel change state
 *
 * @return Current channel change state
 */
zb_channel_change_state_t zb_network_get_channel_change_state(void);

/**
 * @brief Change network channel
 *
 * Initiates a network-wide channel change. This is a complex operation that:
 * 1. Validates the new channel
 * 2. Sends Mgmt_NWK_Update_req to all devices
 * 3. Waits for acknowledgments
 * 4. Switches coordinator to new channel
 * 5. Saves new channel to NVS
 *
 * The callback is called when the operation completes (success or failure).
 *
 * WARNING: This operation may cause temporary network disruption.
 * All devices must support the new channel.
 *
 * @param[in] new_channel New channel (11-26)
 * @param[in] callback Completion callback (can be NULL)
 * @param[in] user_data User data for callback
 * @return ESP_OK if channel change initiated
 * @return ESP_ERR_INVALID_ARG if channel is invalid
 * @return ESP_ERR_INVALID_STATE if change already in progress
 * @return ESP_ERR_NOT_SUPPORTED if network not formed
 */
esp_err_t zb_network_change_channel(uint8_t new_channel,
                                    zb_channel_change_cb_t callback,
                                    void *user_data);

/**
 * @brief Change network channel with scan duration
 *
 * Same as zb_network_change_channel() but allows specifying the scan duration
 * for the Mgmt_NWK_Update_req command.
 *
 * @param[in] new_channel New channel (11-26)
 * @param[in] scan_duration Scan duration (0-14, or 0xFE for channel change)
 * @param[in] callback Completion callback (can be NULL)
 * @param[in] user_data User data for callback
 * @return ESP_OK if channel change initiated
 */
esp_err_t zb_network_change_channel_ex(uint8_t new_channel,
                                       uint8_t scan_duration,
                                       zb_channel_change_cb_t callback,
                                       void *user_data);

/**
 * @brief Cancel pending channel change
 *
 * Attempts to cancel a channel change that hasn't completed yet.
 * Only possible during the PENDING or NOTIFYING states.
 *
 * @return ESP_OK if cancelled
 * @return ESP_ERR_INVALID_STATE if not in cancellable state
 */
esp_err_t zb_network_cancel_channel_change(void);

/**
 * @brief Get available channels (from energy scan)
 *
 * Performs energy scan and returns channels with acceptable noise levels.
 *
 * @param[out] channels_mask Bitmask of recommended channels
 * @param[in] threshold_dbm Maximum noise level threshold in dBm
 * @return ESP_OK on success
 */
esp_err_t zb_network_get_available_channels(uint32_t *channels_mask,
                                            int8_t threshold_dbm);

/**
 * @brief Set channel directly (internal use)
 *
 * Directly sets the channel without network update procedure.
 * Only for internal use during network formation or restore.
 *
 * @param[in] channel New channel
 * @return ESP_OK on success
 */
esp_err_t zb_network_set_channel_internal(uint8_t channel);

/* ============================================================================
 * Extended PAN ID Management (API-004)
 * ============================================================================ */

/** @brief Extended PAN ID length in bytes */
#define ZB_EXTENDED_PAN_ID_LEN  8

/**
 * @brief Get the current Extended PAN ID
 *
 * Retrieves the current Extended PAN ID from the Zigbee stack.
 * Uses the ESP-Zigbee-SDK v1.6.8 API esp_zb_nwk_get_extended_pan_id().
 *
 * @param[out] ext_pan_id Buffer to store Extended PAN ID (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if ext_pan_id is NULL
 * @return ESP_ERR_INVALID_STATE if network manager not initialized
 */
esp_err_t zb_network_get_extended_pan_id(uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN]);

/**
 * @brief Set the Extended PAN ID
 *
 * Sets a new Extended PAN ID in the Zigbee stack.
 * Uses the ESP-Zigbee-SDK v1.6.8 API esp_zb_nwk_set_extended_pan_id().
 *
 * WARNING: This should only be called before network formation or during
 * network reset. Changing the Extended PAN ID on an active network
 * will cause all devices to lose connectivity.
 *
 * @param[in] ext_pan_id New Extended PAN ID (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if ext_pan_id is NULL
 * @return ESP_ERR_INVALID_STATE if network manager not initialized
 */
esp_err_t zb_network_set_extended_pan_id(const uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN]);

/**
 * @brief Set Extended PAN ID from IEEE MAC address
 *
 * Sets the Extended PAN ID to the device's IEEE 802.15.4 MAC address.
 * This is the recommended approach for coordinator setup.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if MAC address cannot be read
 */
esp_err_t zb_network_set_extended_pan_id_from_mac(void);

/**
 * @brief Check if Extended PAN ID is configured (non-zero)
 *
 * @return true if Extended PAN ID is configured (not all zeros)
 * @return false if Extended PAN ID is not configured
 */
bool zb_network_has_extended_pan_id(void);

/**
 * @brief Format Extended PAN ID as string
 *
 * Formats the Extended PAN ID as a hex string for display/logging.
 * Format: "0x0123456789ABCDEF"
 *
 * @param[in] ext_pan_id Extended PAN ID (8 bytes)
 * @param[out] str_buf Output string buffer (min 19 bytes: "0x" + 16 hex + null)
 * @param[in] buf_len Buffer length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if arguments are invalid
 * @return ESP_ERR_NO_MEM if buffer too small
 */
esp_err_t zb_network_format_extended_pan_id(const uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN],
                                             char *str_buf, size_t buf_len);

/**
 * @brief Parse Extended PAN ID from string
 *
 * Parses an Extended PAN ID from a hex string.
 * Accepts formats: "0x0123456789ABCDEF" or "0123456789ABCDEF"
 *
 * @param[in] str_in Input string
 * @param[out] ext_pan_id Output Extended PAN ID (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if arguments are invalid or format incorrect
 */
esp_err_t zb_network_parse_extended_pan_id(const char *str_in,
                                            uint8_t ext_pan_id[ZB_EXTENDED_PAN_ID_LEN]);

/* ============================================================================
 * Network Key Rotation (API-006)
 * ============================================================================ */

/** @brief NVS key for key rotation counter */
#define NVS_KEY_KEY_ROTATION_SEQ    "key_rot_seq"

/**
 * @brief Rotate network key
 *
 * Initiates a network-wide key rotation by broadcasting a new network key
 * to all devices using esp_zb_secur_broadcast_network_key_switch().
 *
 * The key rotation sequence counter is stored in NVS for persistence.
 *
 * WARNING: Key rotation is a sensitive security operation. All devices
 * must successfully receive the new key or they will lose network access.
 *
 * @param[out] sequence Optional output for the new sequence number
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if network manager not initialized or network not formed
 * @return ESP_FAIL if key broadcast fails
 */
esp_err_t zb_network_rotate_key(uint32_t *sequence);

/**
 * @brief Get current key rotation sequence number
 *
 * Retrieves the current key rotation sequence counter from NVS.
 *
 * @param[out] sequence Output sequence number
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if sequence is NULL
 * @return ESP_ERR_NOT_FOUND if no sequence stored (never rotated)
 */
esp_err_t zb_network_get_key_rotation_sequence(uint32_t *sequence);

#ifdef __cplusplus
}
#endif

#endif /* ZB_NETWORK_H */
