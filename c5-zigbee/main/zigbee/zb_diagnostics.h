/**
 * @file zb_diagnostics.h
 * @brief Zigbee Diagnostics Cluster Implementation (Cluster 0x0B05)
 *
 * Provides network health monitoring and diagnostic information for
 * both the coordinator and joined devices.
 *
 * The Diagnostics Cluster provides:
 * - MAC layer statistics (TX/RX counts, retries, failures)
 * - APS layer statistics (message counts, failures)
 * - Network layer statistics (route discovery, neighbor table)
 * - Device health indicators (LQI, RSSI)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_DIAGNOSTICS_H
#define ZB_DIAGNOSTICS_H

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
 * @brief Diagnostics Cluster ID (ZCL)
 */
#define ZB_ZCL_CLUSTER_ID_DIAGNOSTICS       0x0B05

/**
 * @brief Diagnostics Cluster Attribute IDs
 */
/* Hardware Information */
#define ZB_DIAG_ATTR_NUMBER_OF_RESETS           0x0000  /**< uint16 */
#define ZB_DIAG_ATTR_PERSISTENT_MEMORY_WRITES   0x0001  /**< uint16 */

/* MAC Layer Counters */
#define ZB_DIAG_ATTR_MAC_RX_BCAST               0x0100  /**< uint32 */
#define ZB_DIAG_ATTR_MAC_TX_BCAST               0x0101  /**< uint32 */
#define ZB_DIAG_ATTR_MAC_RX_UCAST               0x0102  /**< uint32 */
#define ZB_DIAG_ATTR_MAC_TX_UCAST               0x0103  /**< uint32 */
#define ZB_DIAG_ATTR_MAC_TX_UCAST_RETRY         0x0104  /**< uint16 */
#define ZB_DIAG_ATTR_MAC_TX_UCAST_FAIL          0x0105  /**< uint16 */

/* APS Layer Counters */
#define ZB_DIAG_ATTR_APS_RX_BCAST               0x0106  /**< uint16 */
#define ZB_DIAG_ATTR_APS_TX_BCAST               0x0107  /**< uint16 */
#define ZB_DIAG_ATTR_APS_RX_UCAST               0x0108  /**< uint16 */
#define ZB_DIAG_ATTR_APS_TX_UCAST_SUCCESS       0x0109  /**< uint16 */
#define ZB_DIAG_ATTR_APS_TX_UCAST_RETRY         0x010A  /**< uint16 */
#define ZB_DIAG_ATTR_APS_TX_UCAST_FAIL          0x010B  /**< uint16 */

/* Network Layer Counters */
#define ZB_DIAG_ATTR_ROUTE_DISC_INITIATED       0x010C  /**< uint16 */
#define ZB_DIAG_ATTR_NEIGHBOR_ADDED             0x010D  /**< uint16 */
#define ZB_DIAG_ATTR_NEIGHBOR_REMOVED           0x010E  /**< uint16 */
#define ZB_DIAG_ATTR_NEIGHBOR_STALE             0x010F  /**< uint16 */
#define ZB_DIAG_ATTR_JOIN_INDICATION            0x0110  /**< uint16 */
#define ZB_DIAG_ATTR_CHILD_MOVED                0x0111  /**< uint16 */

/* Security Counters */
#define ZB_DIAG_ATTR_NWK_FC_FAILURE             0x0112  /**< uint16 */
#define ZB_DIAG_ATTR_APS_FC_FAILURE             0x0113  /**< uint16 */
#define ZB_DIAG_ATTR_APS_UNAUTHORIZED_KEY       0x0114  /**< uint16 */
#define ZB_DIAG_ATTR_NWK_DECRYPT_FAILURES       0x0115  /**< uint16 */
#define ZB_DIAG_ATTR_APS_DECRYPT_FAILURES       0x0116  /**< uint16 */

/* Packet Buffer */
#define ZB_DIAG_ATTR_PACKET_BUFFER_ALLOC_FAIL   0x0117  /**< uint16 */

/* Message Statistics */
#define ZB_DIAG_ATTR_RELAYED_UCAST              0x0118  /**< uint16 */
#define ZB_DIAG_ATTR_PHY_TO_MAC_QUEUE_LIMIT     0x0119  /**< uint16 */
#define ZB_DIAG_ATTR_PACKET_VALIDATE_DROP_COUNT 0x011A  /**< uint16 */
#define ZB_DIAG_ATTR_AVG_MAC_RETRY_PER_APS_MSG  0x011B  /**< uint16 */

/* Link Quality */
#define ZB_DIAG_ATTR_LAST_MESSAGE_LQI           0x011C  /**< uint8 */
#define ZB_DIAG_ATTR_LAST_MESSAGE_RSSI          0x011D  /**< int8 */

/* ============================================================================
 * Device Temperature Configuration Cluster (0x0002) Constants
 * ============================================================================ */

/**
 * @brief Device Temperature Configuration Cluster ID (ZCL)
 */
#define ZB_ZCL_CLUSTER_ID_DEVICE_TEMP_CONFIG    0x0002

/**
 * @brief Device Temperature Cluster Attribute IDs
 */
#define ZB_DEVICE_TEMP_ATTR_CURRENT             0x0000  /**< int16 - Current temperature in 0.01C */
#define ZB_DEVICE_TEMP_ATTR_MIN_EXPERIENCED     0x0001  /**< int16 - Minimum temperature experienced */
#define ZB_DEVICE_TEMP_ATTR_MAX_EXPERIENCED     0x0002  /**< int16 - Maximum temperature experienced */
#define ZB_DEVICE_TEMP_ATTR_OVER_ALARM          0x0010  /**< uint16 - Over-temp alarm bitmask */
#define ZB_DEVICE_TEMP_ATTR_ALARM_MASK          0x0012  /**< uint24 - Alarm mask */
#define ZB_DEVICE_TEMP_ATTR_LOW_THRESHOLD       0x0013  /**< int16 - Low temp threshold */
#define ZB_DEVICE_TEMP_ATTR_HIGH_THRESHOLD      0x0014  /**< int16 - High temp threshold */
#define ZB_DEVICE_TEMP_ATTR_LOW_DWELL_TRIP      0x0015  /**< uint24 - Low trip point dwell */
#define ZB_DEVICE_TEMP_ATTR_HIGH_DWELL_TRIP     0x0016  /**< uint24 - High trip point dwell */

/**
 * @brief Device temperature scale factor (1/100 degree Celsius)
 */
#define ZB_DEVICE_TEMP_SCALE                    100

/**
 * @brief Invalid temperature value (not available)
 */
#define ZB_DEVICE_TEMP_INVALID                  0x8000

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief MAC Layer Diagnostics
 */
typedef struct {
    uint32_t rx_bcast;              /**< Broadcast messages received */
    uint32_t tx_bcast;              /**< Broadcast messages transmitted */
    uint32_t rx_ucast;              /**< Unicast messages received */
    uint32_t tx_ucast;              /**< Unicast messages transmitted */
    uint16_t tx_ucast_retry;        /**< Unicast transmit retries */
    uint16_t tx_ucast_fail;         /**< Unicast transmit failures */
} zb_diag_mac_t;

/**
 * @brief APS Layer Diagnostics
 */
typedef struct {
    uint16_t rx_bcast;              /**< APS broadcast received */
    uint16_t tx_bcast;              /**< APS broadcast transmitted */
    uint16_t rx_ucast;              /**< APS unicast received */
    uint16_t tx_ucast_success;      /**< APS unicast transmitted successfully */
    uint16_t tx_ucast_retry;        /**< APS unicast retries */
    uint16_t tx_ucast_fail;         /**< APS unicast failures */
} zb_diag_aps_t;

/**
 * @brief Network Layer Diagnostics
 */
typedef struct {
    uint16_t route_disc_initiated;  /**< Route discoveries initiated */
    uint16_t neighbor_added;        /**< Neighbors added to table */
    uint16_t neighbor_removed;      /**< Neighbors removed from table */
    uint16_t neighbor_stale;        /**< Stale neighbor entries */
    uint16_t join_indication;       /**< Device join indications */
    uint16_t child_moved;           /**< Child devices that moved */
} zb_diag_network_t;

/**
 * @brief Security Diagnostics
 */
typedef struct {
    uint16_t nwk_fc_failure;        /**< Network frame counter failures */
    uint16_t aps_fc_failure;        /**< APS frame counter failures */
    uint16_t aps_unauthorized_key;  /**< Unauthorized key usage attempts */
    uint16_t nwk_decrypt_failures;  /**< Network decryption failures */
    uint16_t aps_decrypt_failures;  /**< APS decryption failures */
} zb_diag_security_t;

/**
 * @brief Complete Diagnostics Structure
 */
typedef struct {
    /* Hardware */
    uint16_t number_of_resets;      /**< Number of device resets */
    uint16_t persistent_mem_writes; /**< NVS write count */

    /* MAC Layer */
    zb_diag_mac_t mac;

    /* APS Layer */
    zb_diag_aps_t aps;

    /* Network Layer */
    zb_diag_network_t network;

    /* Security */
    zb_diag_security_t security;

    /* Packet Buffer */
    uint16_t packet_buffer_alloc_fail; /**< Buffer allocation failures */

    /* Additional Stats */
    uint16_t relayed_ucast;         /**< Relayed unicast messages */
    uint16_t phy_to_mac_queue_limit;/**< PHY to MAC queue limit hits */
    uint16_t packet_validate_drop;  /**< Packets dropped in validation */
    uint16_t avg_mac_retry_per_msg; /**< Average MAC retries per APS message */

    /* Link Quality */
    uint8_t last_message_lqi;       /**< LQI of last received message */
    int8_t last_message_rssi;       /**< RSSI of last received message */
} zb_diagnostics_t;

/**
 * @brief Per-Device Diagnostics (for remote devices)
 */
typedef struct {
    uint16_t short_addr;            /**< Device short address */
    uint8_t ieee_addr[8];           /**< Device IEEE address */
    zb_diagnostics_t diagnostics;   /**< Diagnostic data */
    uint32_t last_updated;          /**< Timestamp of last update */
    bool valid;                     /**< Data is valid */
} zb_device_diagnostics_t;

/**
 * @brief Device Temperature Configuration Data
 *
 * Represents the internal device temperature from Cluster 0x0002.
 * Temperatures are stored in 0.01 degree Celsius units.
 */
typedef struct {
    int16_t current_temperature;    /**< Current device temperature (0.01C) */
    int16_t min_temp_experienced;   /**< Minimum temperature experienced (0.01C) */
    int16_t max_temp_experienced;   /**< Maximum temperature experienced (0.01C) */
    uint16_t over_temp_alarm;       /**< Over-temperature alarm bitmask */
    int16_t low_threshold;          /**< Low temperature threshold (0.01C) */
    int16_t high_threshold;         /**< High temperature threshold (0.01C) */
    uint32_t last_updated;          /**< Timestamp of last update */
    bool valid;                     /**< Data is valid/available */
} zb_device_temperature_t;

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize Diagnostics Cluster
 *
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_init(void);

/**
 * @brief Deinitialize Diagnostics Cluster
 *
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_deinit(void);

/**
 * @brief Check if Diagnostics is initialized
 *
 * @return true if initialized
 */
bool zb_diagnostics_is_initialized(void);

/**
 * @brief Get coordinator diagnostics
 *
 * Returns diagnostic data for the local coordinator.
 *
 * @param[out] diag Output for diagnostic data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if diag is NULL
 */
esp_err_t zb_diagnostics_get_coordinator(zb_diagnostics_t *diag);

/**
 * @brief Read diagnostics from remote device
 *
 * Sends ZCL Read Attributes request to device and waits for response.
 *
 * @param[in] short_addr Device short address
 * @param[out] diag Output for diagnostic data
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not responding
 * @return ESP_ERR_TIMEOUT if request timed out
 */
esp_err_t zb_diagnostics_read(uint16_t short_addr, zb_diagnostics_t *diag);

/**
 * @brief Read specific diagnostic attribute from device
 *
 * @param[in] short_addr Device short address
 * @param[in] attr_id Attribute ID to read
 * @param[out] value Output buffer
 * @param[in] max_len Maximum output length
 * @return Actual data length, or 0 on error
 */
size_t zb_diagnostics_read_attr(uint16_t short_addr, uint16_t attr_id,
                                void *value, size_t max_len);

/**
 * @brief Update MAC counter
 *
 * @param[in] counter Counter type (ZB_DIAG_ATTR_MAC_*)
 * @param[in] increment Increment amount
 */
void zb_diagnostics_update_mac(uint16_t counter, uint32_t increment);

/**
 * @brief Update APS counter
 *
 * @param[in] counter Counter type (ZB_DIAG_ATTR_APS_*)
 * @param[in] increment Increment amount
 */
void zb_diagnostics_update_aps(uint16_t counter, uint16_t increment);

/**
 * @brief Update network counter
 *
 * @param[in] counter Counter type (ZB_DIAG_ATTR_NEIGHBOR_*, etc.)
 * @param[in] increment Increment amount
 */
void zb_diagnostics_update_network(uint16_t counter, uint16_t increment);

/**
 * @brief Update link quality metrics
 *
 * @param[in] lqi Link Quality Indicator (0-255)
 * @param[in] rssi Received Signal Strength (dBm)
 */
void zb_diagnostics_update_link_quality(uint8_t lqi, int8_t rssi);

/**
 * @brief Increment reset counter
 *
 * Should be called on startup after reading from NVS.
 */
void zb_diagnostics_increment_reset_count(void);

/**
 * @brief Reset all counters
 *
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_reset_counters(void);

/**
 * @brief Get network map with diagnostics
 *
 * Returns diagnostic data for all known devices.
 *
 * @param[out] devices Output array
 * @param[in] max_count Maximum devices to return
 * @return Number of devices copied
 */
size_t zb_diagnostics_get_network_map(zb_device_diagnostics_t *devices,
                                      size_t max_count);

/**
 * @brief Publish diagnostics to MQTT
 *
 * Publishes coordinator diagnostics as JSON to MQTT topic.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_publish_mqtt(void);

/**
 * @brief Handle MQTT request for network diagnostics
 *
 * @param[in] topic Request topic
 * @param[in] payload Request payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_process_mqtt_request(const char *topic,
                                              const char *payload,
                                              size_t len);

/**
 * @brief Format diagnostics as JSON string
 *
 * @param[in] diag Diagnostics data
 * @param[out] json_buf Output buffer
 * @param[in] buf_len Buffer length
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_to_json(const zb_diagnostics_t *diag,
                                 char *json_buf, size_t buf_len);

/**
 * @brief Self-test function
 *
 * @return ESP_OK if all tests pass
 */
esp_err_t zb_diagnostics_test(void);

/* ============================================================================
 * Device Temperature Cluster (0x0002) Functions
 * ============================================================================ */

/**
 * @brief Read device temperature from remote device
 *
 * Reads the Device Temperature Configuration cluster (0x0002) attributes
 * from a remote Zigbee device. This provides internal device temperature
 * monitoring for diagnostics.
 *
 * @param[in] short_addr Device short address
 * @param[out] temp Output for temperature data
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if temp is NULL
 * @return ESP_ERR_INVALID_STATE if module not initialized
 * @return ESP_ERR_NOT_FOUND if device doesn't support cluster
 * @return ESP_ERR_TIMEOUT if device doesn't respond
 */
esp_err_t zb_diagnostics_read_device_temperature(uint16_t short_addr,
                                                  zb_device_temperature_t *temp);

/**
 * @brief Read specific device temperature attribute
 *
 * Reads a single attribute from the Device Temperature cluster.
 *
 * @param[in] short_addr Device short address
 * @param[in] attr_id Attribute ID (ZB_DEVICE_TEMP_ATTR_*)
 * @param[out] value Output buffer for attribute value
 * @param[in] max_len Maximum output buffer length
 * @return Actual data length, or 0 on error
 */
size_t zb_diagnostics_read_device_temp_attr(uint16_t short_addr,
                                            uint16_t attr_id,
                                            void *value, size_t max_len);

/**
 * @brief Convert raw device temperature to Celsius
 *
 * @param[in] raw_temp Raw temperature value from cluster (0.01C units)
 * @return Temperature in degrees Celsius
 */
float zb_device_temp_to_celsius(int16_t raw_temp);

/**
 * @brief Check if temperature value is valid
 *
 * @param[in] raw_temp Raw temperature value from cluster
 * @return true if valid, false if invalid/not available
 */
bool zb_device_temp_is_valid(int16_t raw_temp);

/**
 * @brief Format device temperature data as JSON
 *
 * Creates a JSON object with device temperature information suitable
 * for MQTT publishing.
 *
 * @param[in] temp Device temperature data
 * @param[out] json_buf Output buffer for JSON string
 * @param[in] buf_len Buffer length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_NO_MEM if buffer too small
 */
esp_err_t zb_device_temperature_to_json(const zb_device_temperature_t *temp,
                                        char *json_buf, size_t buf_len);

/**
 * @brief Handle device temperature attribute report
 *
 * Called when a device reports a device temperature attribute.
 * Updates internal cache and triggers state publishing if needed.
 *
 * @param[in] short_addr Device short address
 * @param[in] attr_id Attribute ID that was reported
 * @param[in] value Attribute value
 * @param[in] value_len Value length
 * @return ESP_OK on success
 */
esp_err_t zb_diagnostics_handle_device_temp_report(uint16_t short_addr,
                                                   uint16_t attr_id,
                                                   const void *value,
                                                   size_t value_len);

/**
 * @brief Check if device supports Device Temperature cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device supports cluster 0x0002
 */
bool zb_diagnostics_has_device_temp_cluster(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_DIAGNOSTICS_H */
