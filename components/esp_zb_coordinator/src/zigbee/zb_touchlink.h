/**
 * @file zb_touchlink.h
 * @brief Zigbee Touchlink Commissioning API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee Touchlink (ZLL) commissioning functionality:
 * - Touchlink scanning on all Zigbee channels (11-26)
 * - Device identification (blink/identify)
 * - Remote factory reset via Touchlink
 * - Network join for discovered ZLL devices
 * - Inter-PAN communication handling
 *
 * Touchlink is primarily used for:
 * - Philips Hue and similar ZLL-certified devices
 * - Commissioning without physical reset button access
 * - Proximity-based pairing (RSSI threshold)
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/touchlink/scan - Start Touchlink scan
 * - zigbee2mqtt/bridge/request/touchlink/identify/{ieee} - Identify device
 * - zigbee2mqtt/bridge/request/touchlink/factoryreset/{ieee} - Factory reset
 * - zigbee2mqtt/bridge/response/touchlink/scan - Scan results
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_TOUCHLINK_H
#define ZB_TOUCHLINK_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Touchlink Cluster and Command Definitions (ZCL Commissioning Cluster)
 * ============================================================================ */

/**
 * @brief Touchlink Commissioning Cluster ID (ZLL)
 */
#define ZB_TOUCHLINK_CLUSTER_ID             0x1000

/**
 * @brief Inter-PAN Transmission Cluster ID
 */
#define ZB_TOUCHLINK_INTERPAN_CLUSTER_ID    0x1000

/* --------------------------------------------------------------------------
 * Touchlink Command IDs (Client to Server)
 * -------------------------------------------------------------------------- */

/**
 * @brief Scan Request command ID
 */
#define ZB_TOUCHLINK_CMD_SCAN_REQUEST               0x00

/**
 * @brief Device Information Request command ID
 */
#define ZB_TOUCHLINK_CMD_DEVICE_INFO_REQUEST        0x02

/**
 * @brief Identify Request command ID
 */
#define ZB_TOUCHLINK_CMD_IDENTIFY_REQUEST           0x06

/**
 * @brief Reset to Factory New Request command ID
 */
#define ZB_TOUCHLINK_CMD_FACTORY_RESET_REQUEST      0x07

/**
 * @brief Network Start Request command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_START_REQUEST      0x10

/**
 * @brief Network Join Router Request command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_JOIN_ROUTER_REQUEST    0x12

/**
 * @brief Network Join End Device Request command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_JOIN_ED_REQUEST    0x14

/**
 * @brief Network Update Request command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_UPDATE_REQUEST     0x16

/* --------------------------------------------------------------------------
 * Touchlink Command IDs (Server to Client / Responses)
 * -------------------------------------------------------------------------- */

/**
 * @brief Scan Response command ID
 */
#define ZB_TOUCHLINK_CMD_SCAN_RESPONSE              0x01

/**
 * @brief Device Information Response command ID
 */
#define ZB_TOUCHLINK_CMD_DEVICE_INFO_RESPONSE       0x03

/**
 * @brief Network Start Response command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_START_RESPONSE     0x11

/**
 * @brief Network Join Router Response command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_JOIN_ROUTER_RESPONSE   0x13

/**
 * @brief Network Join End Device Response command ID
 */
#define ZB_TOUCHLINK_CMD_NETWORK_JOIN_ED_RESPONSE   0x15

/* ============================================================================
 * Touchlink Constants and Limits
 * ============================================================================ */

/**
 * @brief Maximum number of devices in scan results
 */
#define ZB_TOUCHLINK_MAX_SCAN_RESULTS       32

/**
 * @brief Default scan duration per channel (milliseconds)
 */
#define ZB_TOUCHLINK_SCAN_DURATION_MS       250

/**
 * @brief Total scan timeout (milliseconds)
 */
#define ZB_TOUCHLINK_SCAN_TIMEOUT_MS        15000

/**
 * @brief Minimum RSSI for Touchlink (proximity requirement)
 * @note -40 dBm typical for close proximity
 */
#define ZB_TOUCHLINK_MIN_RSSI               (-80)

/**
 * @brief Default RSSI correction value
 */
#define ZB_TOUCHLINK_RSSI_CORRECTION        0

/**
 * @brief Identify duration in seconds
 */
#define ZB_TOUCHLINK_IDENTIFY_DURATION      5

/**
 * @brief Touchlink transaction timeout (milliseconds)
 */
#define ZB_TOUCHLINK_TRANSACTION_TIMEOUT_MS 8000

/**
 * @brief First Zigbee channel for scan
 */
#define ZB_TOUCHLINK_CHANNEL_FIRST          11

/**
 * @brief Last Zigbee channel for scan
 */
#define ZB_TOUCHLINK_CHANNEL_LAST           26

/**
 * @brief NVS namespace for Touchlink storage
 */
#define ZB_TOUCHLINK_NVS_NAMESPACE          "zb_touchlink"

/* ============================================================================
 * Touchlink Status and Error Codes
 * ============================================================================ */

/**
 * @brief Touchlink scan status enumeration
 */
typedef enum {
    ZB_TOUCHLINK_SCAN_STATUS_IDLE = 0,      /**< No scan in progress */
    ZB_TOUCHLINK_SCAN_STATUS_SCANNING,      /**< Scan in progress */
    ZB_TOUCHLINK_SCAN_STATUS_COMPLETE,      /**< Scan completed successfully */
    ZB_TOUCHLINK_SCAN_STATUS_TIMEOUT,       /**< Scan timed out */
    ZB_TOUCHLINK_SCAN_STATUS_ERROR          /**< Scan error */
} zb_touchlink_scan_status_t;

/**
 * @brief Touchlink device type enumeration
 */
typedef enum {
    ZB_TOUCHLINK_DEVICE_TYPE_COORDINATOR = 0,   /**< Coordinator */
    ZB_TOUCHLINK_DEVICE_TYPE_ROUTER = 1,        /**< Router */
    ZB_TOUCHLINK_DEVICE_TYPE_END_DEVICE = 2     /**< End Device */
} zb_touchlink_device_type_t;

/* ============================================================================
 * Touchlink Data Structures
 * ============================================================================ */

/**
 * @brief Touchlink transaction ID structure
 *
 * Used for correlating request/response pairs in Touchlink operations.
 */
typedef struct {
    uint32_t transaction_id;    /**< 32-bit transaction identifier */
    uint32_t timestamp;         /**< Transaction start time (tick count) */
    bool active;                /**< Transaction is active */
} zb_touchlink_transaction_t;

/**
 * @brief RSSI correction structure for Touchlink proximity
 *
 * Different devices may report RSSI differently. This allows calibration.
 */
typedef struct {
    int8_t correction_value;    /**< RSSI correction in dB */
    int8_t threshold;           /**< Minimum RSSI threshold for commissioning */
    bool enabled;               /**< RSSI filtering enabled */
} zb_touchlink_rssi_config_t;

/**
 * @brief Touchlink scan result for a single device
 *
 * Contains all information received from a Scan Response.
 */
typedef struct {
    /* Device addressing */
    uint8_t ieee_addr[8];       /**< IEEE address (64-bit) */
    uint16_t short_addr;        /**< Network short address */
    uint16_t pan_id;            /**< PAN ID */
    uint8_t extended_pan_id[8]; /**< Extended PAN ID */

    /* Radio parameters */
    uint8_t channel;            /**< Current channel (11-26) */
    int8_t rssi;                /**< Received signal strength (dBm) */
    int8_t rssi_corrected;      /**< RSSI after correction */

    /* Device capabilities */
    zb_touchlink_device_type_t device_type;     /**< Device type */
    uint8_t endpoint;           /**< Touchlink endpoint */
    uint16_t profile_id;        /**< Application profile ID */
    uint16_t device_id;         /**< Device ID */
    uint8_t version;            /**< Application version */

    /* Touchlink specific */
    uint32_t response_id;       /**< Response identifier */
    uint8_t touchlink_info;     /**< Touchlink information field */
    uint8_t key_bitmask;        /**< Supported key bitmask */

    /* Status flags */
    bool factory_new;           /**< Device is factory new */
    bool address_assignment;    /**< Supports address assignment */
    bool link_initiator;        /**< Can be link initiator */
    bool link_priority;         /**< Has link priority */

    /* Metadata */
    uint32_t timestamp;         /**< When this result was received */
    bool valid;                 /**< Entry is valid */
} zb_touchlink_scan_result_t;

/**
 * @brief Touchlink scan configuration
 */
typedef struct {
    uint8_t channel_mask[4];    /**< Channel mask (channels 11-26) */
    uint16_t scan_duration_ms;  /**< Duration per channel */
    int8_t rssi_threshold;      /**< Minimum RSSI to include */
    bool scan_factory_new_only; /**< Only scan for factory new devices */
} zb_touchlink_scan_config_t;

/**
 * @brief Touchlink identify request parameters
 */
typedef struct {
    uint8_t ieee_addr[8];       /**< Target device IEEE address */
    uint16_t duration;          /**< Identify duration in seconds */
} zb_touchlink_identify_params_t;

/**
 * @brief Touchlink network join parameters
 */
typedef struct {
    uint8_t ieee_addr[8];       /**< Target device IEEE address */
    uint16_t pan_id;            /**< Network PAN ID to join */
    uint8_t extended_pan_id[8]; /**< Extended PAN ID */
    uint8_t channel;            /**< Network channel */
    uint8_t network_key[16];    /**< Network encryption key */
    uint16_t short_addr;        /**< Assigned short address */
    uint8_t network_update_id;  /**< Network update identifier */
} zb_touchlink_join_params_t;

/* ============================================================================
 * Callback Type Definitions
 * ============================================================================ */

/**
 * @brief Touchlink scan result callback type
 *
 * Called for each device found during Touchlink scan.
 *
 * @param[in] result Pointer to scan result (valid only during callback)
 * @param[in] user_data User-provided context pointer
 */
typedef void (*zb_touchlink_scan_result_cb_t)(const zb_touchlink_scan_result_t *result,
                                               void *user_data);

/**
 * @brief Touchlink scan complete callback type
 *
 * Called when scan operation completes.
 *
 * @param[in] status Final scan status
 * @param[in] device_count Number of devices found
 * @param[in] user_data User-provided context pointer
 */
typedef void (*zb_touchlink_scan_complete_cb_t)(zb_touchlink_scan_status_t status,
                                                 size_t device_count,
                                                 void *user_data);

/**
 * @brief Touchlink operation result callback type
 *
 * Called when Touchlink operations (identify, factory reset, join) complete.
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[in] result Operation result (ESP_OK on success)
 * @param[in] user_data User-provided context pointer
 */
typedef void (*zb_touchlink_operation_cb_t)(const uint8_t *ieee_addr,
                                            esp_err_t result,
                                            void *user_data);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize Touchlink commissioning module
 *
 * Initializes the Touchlink module, allocates resources, and registers
 * Inter-PAN message handlers with the Zigbee stack.
 *
 * Must be called after zb_coordinator_init() and before any other
 * Touchlink functions.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized or coordinator not ready
 */
esp_err_t zb_touchlink_init(void);

/**
 * @brief Deinitialize Touchlink commissioning module
 *
 * Frees all resources and unregisters handlers.
 * Any ongoing operations will be cancelled.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_touchlink_deinit(void);

/**
 * @brief Start Touchlink scan
 *
 * Initiates a Touchlink scan across all Zigbee channels (11-26).
 * Uses Inter-PAN communication to discover Touchlink-capable devices.
 *
 * The scan is asynchronous. Results are delivered via callbacks:
 * - result_cb: Called for each discovered device
 * - complete_cb: Called when scan completes
 *
 * @param[in] config Scan configuration (NULL for defaults)
 * @param[in] result_cb Callback for each result (can be NULL)
 * @param[in] complete_cb Callback on completion (can be NULL)
 * @param[in] user_data User context passed to callbacks
 * @return ESP_OK if scan started successfully
 * @return ESP_ERR_INVALID_STATE if scan already in progress
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_touchlink_scan(const zb_touchlink_scan_config_t *config,
                            zb_touchlink_scan_result_cb_t result_cb,
                            zb_touchlink_scan_complete_cb_t complete_cb,
                            void *user_data);

/**
 * @brief Stop ongoing Touchlink scan
 *
 * Cancels any scan in progress. The complete callback will be called
 * with ZB_TOUCHLINK_SCAN_STATUS_COMPLETE and current results.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if no scan in progress
 */
esp_err_t zb_touchlink_scan_stop(void);

/**
 * @brief Get Touchlink scan status
 *
 * @return Current scan status
 */
zb_touchlink_scan_status_t zb_touchlink_get_scan_status(void);

/**
 * @brief Get scan results
 *
 * Copies current scan results to provided array.
 * Can be called during or after scan.
 *
 * @param[out] results Array to store results
 * @param[in] max_count Maximum number of results to copy
 * @return Number of results copied
 */
size_t zb_touchlink_get_scan_results(zb_touchlink_scan_result_t *results, size_t max_count);

/**
 * @brief Get scan result by IEEE address
 *
 * Finds a specific device in scan results.
 *
 * @param[in] ieee_addr IEEE address (8 bytes)
 * @param[out] result Output result structure
 * @return ESP_OK if found
 * @return ESP_ERR_NOT_FOUND if device not in results
 * @return ESP_ERR_INVALID_ARG if parameters are NULL
 */
esp_err_t zb_touchlink_get_result_by_ieee(const uint8_t *ieee_addr,
                                          zb_touchlink_scan_result_t *result);

/**
 * @brief Send Touchlink Identify Request
 *
 * Causes the target device to identify itself (typically by blinking).
 * Uses Inter-PAN communication.
 *
 * @param[in] ieee_addr Target device IEEE address (8 bytes)
 * @param[in] duration_sec Identify duration in seconds (0 to stop)
 * @param[in] callback Result callback (can be NULL)
 * @param[in] user_data User context for callback
 * @return ESP_OK if request sent successfully
 * @return ESP_ERR_NOT_FOUND if device not in scan results
 * @return ESP_ERR_INVALID_ARG if ieee_addr is NULL
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_touchlink_identify(const uint8_t *ieee_addr,
                                uint16_t duration_sec,
                                zb_touchlink_operation_cb_t callback,
                                void *user_data);

/**
 * @brief Send Touchlink Factory Reset Request
 *
 * Resets the target device to factory defaults via Touchlink.
 * This removes the device from any network and clears all configuration.
 *
 * WARNING: This is a destructive operation and cannot be undone!
 *
 * @param[in] ieee_addr Target device IEEE address (8 bytes)
 * @param[in] callback Result callback (can be NULL)
 * @param[in] user_data User context for callback
 * @return ESP_OK if request sent successfully
 * @return ESP_ERR_NOT_FOUND if device not in scan results
 * @return ESP_ERR_INVALID_ARG if ieee_addr is NULL
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_touchlink_factory_reset(const uint8_t *ieee_addr,
                                     zb_touchlink_operation_cb_t callback,
                                     void *user_data);

/**
 * @brief Join device to network via Touchlink
 *
 * Sends Network Join Request to add a Touchlink device to the
 * coordinator's network. The device type (Router/End Device) is
 * automatically determined from scan results.
 *
 * @param[in] ieee_addr Target device IEEE address (8 bytes)
 * @param[in] callback Result callback (can be NULL)
 * @param[in] user_data User context for callback
 * @return ESP_OK if request sent successfully
 * @return ESP_ERR_NOT_FOUND if device not in scan results
 * @return ESP_ERR_INVALID_STATE if network not formed
 * @return ESP_ERR_INVALID_ARG if ieee_addr is NULL
 * @return ESP_FAIL on Zigbee stack error
 */
esp_err_t zb_touchlink_join_device(const uint8_t *ieee_addr,
                                   zb_touchlink_operation_cb_t callback,
                                   void *user_data);

/* ============================================================================
 * RSSI Configuration Functions
 * ============================================================================ */

/**
 * @brief Set RSSI correction configuration
 *
 * Configures RSSI correction for proximity-based filtering.
 *
 * @param[in] config RSSI configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_touchlink_set_rssi_config(const zb_touchlink_rssi_config_t *config);

/**
 * @brief Get RSSI correction configuration
 *
 * @param[out] config Output configuration
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if config is NULL
 */
esp_err_t zb_touchlink_get_rssi_config(zb_touchlink_rssi_config_t *config);

/* ============================================================================
 * MQTT Integration Functions
 * ============================================================================ */

/**
 * @brief Process MQTT Touchlink request
 *
 * Main entry point for handling Touchlink-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/touchlink/scan
 * - zigbee2mqtt/bridge/request/touchlink/identify/{ieee}
 * - zigbee2mqtt/bridge/request/touchlink/factoryreset/{ieee}
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request format
 * @return ESP_ERR_NOT_SUPPORTED if topic not recognized
 */
esp_err_t zb_touchlink_process_mqtt_request(const char *topic,
                                            const char *payload,
                                            size_t len);

/**
 * @brief Publish Touchlink scan results via MQTT
 *
 * Publishes current scan results to MQTT.
 * Topic: zigbee2mqtt/bridge/response/touchlink/scan
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on MQTT error
 */
esp_err_t zb_touchlink_publish_scan_results(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert IEEE address to string
 *
 * Formats IEEE address as "0x00158D0001234567" string.
 *
 * @param[in] ieee_addr IEEE address (8 bytes)
 * @param[out] str_buf Output string buffer (must be at least 19 bytes)
 * @param[in] buf_len Buffer length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters invalid
 */
esp_err_t zb_touchlink_ieee_to_string(const uint8_t *ieee_addr,
                                      char *str_buf,
                                      size_t buf_len);

/**
 * @brief Parse IEEE address from string
 *
 * Parses "0x00158D0001234567" format to 8-byte array.
 *
 * @param[in] str_buf Input string
 * @param[out] ieee_addr Output IEEE address (8 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if format invalid
 */
esp_err_t zb_touchlink_string_to_ieee(const char *str_buf, uint8_t *ieee_addr);

/**
 * @brief Check if module is initialized
 *
 * @return true if initialized
 * @return false otherwise
 */
bool zb_touchlink_is_initialized(void);

/**
 * @brief Self-test function for Touchlink module
 *
 * Tests initialization, data structures, and basic operations.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_touchlink_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_TOUCHLINK_H */
