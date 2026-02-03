/**
 * @file zb_ota.h
 * @brief Zigbee OTA (Over-The-Air) Update Server API
 *
 * This module provides Zigbee OTA upgrade server functionality for the
 * ESP32-C5 Zigbee2MQTT Gateway. The gateway acts as an OTA server,
 * allowing end devices to request and download firmware updates.
 *
 * Features:
 * - OTA Upgrade Cluster (0x0019) server implementation
 * - Image storage in SPIFFS partition
 * - Rate-limited block transfer
 * - MQTT integration for image upload and status
 * - Support for multiple simultaneous device updates
 *
 * MQTT Topics:
 * - zigbee2mqtt/bridge/request/ota/update/{ieee} - Trigger OTA for device
 * - zigbee2mqtt/bridge/request/ota/check/{ieee} - Check for device updates
 * - zigbee2mqtt/bridge/ota/images - List available OTA images
 * - zigbee2mqtt/bridge/ota/status/{ieee} - OTA progress status
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_OTA_H
#define ZB_OTA_H

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
 * OTA Cluster Definitions (Zigbee Cluster Library 0x0019)
 * ============================================================================ */

/**
 * @brief OTA Upgrade Cluster ID
 */
#define ZB_OTA_CLUSTER_ID                       0x0019

/**
 * @brief OTA Cluster Attribute IDs (Server Side)
 */
#define ZB_OTA_ATTR_UPGRADE_SERVER_ID           0x0000  /**< Upgrade Server IEEE Address */
#define ZB_OTA_ATTR_FILE_OFFSET                 0x0001  /**< File Offset */
#define ZB_OTA_ATTR_CURRENT_FILE_VERSION        0x0002  /**< Current File Version */
#define ZB_OTA_ATTR_CURRENT_ZB_STACK_VERSION    0x0003  /**< Current Zigbee Stack Version */
#define ZB_OTA_ATTR_DOWNLOADED_FILE_VERSION     0x0004  /**< Downloaded File Version */
#define ZB_OTA_ATTR_DOWNLOADED_ZB_STACK_VERSION 0x0005  /**< Downloaded Zigbee Stack Version */
#define ZB_OTA_ATTR_IMAGE_UPGRADE_STATUS        0x0006  /**< Image Upgrade Status */
#define ZB_OTA_ATTR_MANUFACTURER_ID             0x0007  /**< Manufacturer ID */
#define ZB_OTA_ATTR_IMAGE_TYPE_ID               0x0008  /**< Image Type ID */
#define ZB_OTA_ATTR_MIN_BLOCK_REQ_DELAY         0x0009  /**< Minimum Block Request Delay */
#define ZB_OTA_ATTR_IMAGE_STAMP                 0x000A  /**< Image Stamp */

/**
 * @brief OTA Cluster Command IDs (Client to Server)
 */
#define ZB_OTA_CMD_QUERY_NEXT_IMAGE_REQ         0x01    /**< Query Next Image Request */
#define ZB_OTA_CMD_IMAGE_BLOCK_REQ              0x03    /**< Image Block Request */
#define ZB_OTA_CMD_IMAGE_PAGE_REQ               0x04    /**< Image Page Request */
#define ZB_OTA_CMD_UPGRADE_END_REQ              0x06    /**< Upgrade End Request */
#define ZB_OTA_CMD_QUERY_DEVICE_SPECIFIC_FILE   0x08    /**< Query Specific File Request */

/**
 * @brief OTA Cluster Command IDs (Server to Client)
 */
#define ZB_OTA_CMD_IMAGE_NOTIFY                 0x00    /**< Image Notify */
#define ZB_OTA_CMD_QUERY_NEXT_IMAGE_RSP         0x02    /**< Query Next Image Response */
#define ZB_OTA_CMD_IMAGE_BLOCK_RSP              0x05    /**< Image Block Response */
#define ZB_OTA_CMD_UPGRADE_END_RSP              0x07    /**< Upgrade End Response */
#define ZB_OTA_CMD_QUERY_DEVICE_SPECIFIC_FILE_RSP 0x09  /**< Query Specific File Response */

/**
 * @brief OTA Status Codes
 */
#define ZB_OTA_STATUS_SUCCESS                   0x00    /**< Success */
#define ZB_OTA_STATUS_ABORT                     0x95    /**< Abort */
#define ZB_OTA_STATUS_NOT_AUTHORIZED            0x7E    /**< Not authorized */
#define ZB_OTA_STATUS_INVALID_IMAGE             0x96    /**< Invalid image */
#define ZB_OTA_STATUS_WAIT_FOR_DATA             0x97    /**< Wait for data */
#define ZB_OTA_STATUS_NO_IMAGE_AVAILABLE        0x98    /**< No image available */
#define ZB_OTA_STATUS_MALFORMED_COMMAND         0x80    /**< Malformed command */
#define ZB_OTA_STATUS_UNSUP_CLUSTER_COMMAND     0x81    /**< Unsupported cluster command */
#define ZB_OTA_STATUS_REQUIRE_MORE_IMAGE        0x99    /**< Require more image */

/**
 * @brief OTA Image Upgrade Status Values
 */
#define ZB_OTA_UPGRADE_STATUS_NORMAL            0x00    /**< Normal operation */
#define ZB_OTA_UPGRADE_STATUS_DOWNLOAD_IN_PROG  0x01    /**< Download in progress */
#define ZB_OTA_UPGRADE_STATUS_DOWNLOAD_COMPLETE 0x02    /**< Download complete */
#define ZB_OTA_UPGRADE_STATUS_WAITING_TO_UPGRADE 0x03   /**< Waiting to upgrade */
#define ZB_OTA_UPGRADE_STATUS_COUNT_DOWN        0x04    /**< Count down */
#define ZB_OTA_UPGRADE_STATUS_WAIT_FOR_MORE     0x05    /**< Wait for more */

/**
 * @brief OTA Image Notify Payload Types
 */
#define ZB_OTA_NOTIFY_TYPE_JITTER               0x00    /**< Query jitter only */
#define ZB_OTA_NOTIFY_TYPE_MFG_CODE             0x01    /**< + Manufacturer code */
#define ZB_OTA_NOTIFY_TYPE_IMAGE_TYPE           0x02    /**< + Image type */
#define ZB_OTA_NOTIFY_TYPE_NEW_FILE_VERSION     0x03    /**< + New file version */

/* ============================================================================
 * OTA File Header Definitions
 * ============================================================================ */

/**
 * @brief OTA File Header Magic Number
 */
#define ZB_OTA_HEADER_MAGIC                     0x0BEEF11E

/**
 * @brief OTA File Header Version
 */
#define ZB_OTA_HEADER_VERSION                   0x0100

/**
 * @brief OTA Header Field Control Bits
 */
#define ZB_OTA_HEADER_FC_SECURITY_CREDENTIAL    0x0001  /**< Security credential version present */
#define ZB_OTA_HEADER_FC_DEVICE_SPECIFIC        0x0002  /**< Device specific file */
#define ZB_OTA_HEADER_FC_HW_VERSION             0x0004  /**< Hardware version present */

/**
 * @brief Minimum OTA Header Length
 */
#define ZB_OTA_HEADER_MIN_LEN                   56

/**
 * @brief Maximum OTA Header String Length
 */
#define ZB_OTA_HEADER_STRING_MAX_LEN            32

/* ============================================================================
 * OTA Configuration
 * ============================================================================ */

/**
 * @brief Maximum number of OTA images stored
 */
#define ZB_OTA_MAX_IMAGES                       8

/**
 * @brief Maximum number of simultaneous OTA transfers
 */
#define ZB_OTA_MAX_TRANSFERS                    4

/**
 * @brief Default block size for OTA transfer (bytes)
 */
#define ZB_OTA_DEFAULT_BLOCK_SIZE               64

/**
 * @brief Minimum block size (bytes)
 */
#define ZB_OTA_MIN_BLOCK_SIZE                   64

/**
 * @brief Maximum block size (bytes)
 */
#define ZB_OTA_MAX_BLOCK_SIZE                   255

/**
 * @brief Maximum blocks per second (rate limiting)
 */
#define ZB_OTA_MAX_BLOCKS_PER_SEC               10

/**
 * @brief Block request timeout (milliseconds)
 */
#define ZB_OTA_BLOCK_TIMEOUT_MS                 60000

/**
 * @brief Maximum OTA image size (bytes)
 */
#define ZB_OTA_MAX_IMAGE_SIZE                   (512 * 1024)

/**
 * @brief SPIFFS mount point for OTA images
 */
#define ZB_OTA_SPIFFS_MOUNT_POINT               "/ota_storage"

/**
 * @brief NVS namespace for OTA metadata
 */
#define ZB_OTA_NVS_NAMESPACE                    "zb_ota"

/* ============================================================================
 * APS Fragmentation Configuration (API-011)
 *
 * These settings optimize the APS layer fragmentation for large OTA transfers.
 * Larger window sizes allow more fragments in flight, improving throughput.
 * The interframe delay prevents network congestion.
 * ============================================================================ */

/**
 * @brief APS Fragment Window Size for OTA transfers
 *
 * Maximum number of fragments that can be in flight simultaneously.
 * Higher values improve throughput but require more buffer memory.
 * Range: 1-8 (Zigbee spec maximum is 8)
 * Default: 8 (maximum for optimal OTA performance)
 */
#define ZB_OTA_APS_FRAGMENT_WINDOW_SIZE         8

/**
 * @brief APS Fragment Interframe Delay in milliseconds
 *
 * Minimum delay between sending consecutive fragments.
 * Lower values improve speed but may cause congestion.
 * Range: 0-255 ms
 * Default: 50 ms (balanced for reliability and speed)
 */
#define ZB_OTA_APS_INTERFRAME_DELAY_MS          50

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief OTA File Header Structure
 *
 * This structure represents the Zigbee OTA file header as defined in
 * the Zigbee Cluster Library specification.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;                              /**< Magic number (0x0BEEF11E) */
    uint16_t header_version;                     /**< Header version */
    uint16_t header_length;                      /**< Total header length */
    uint16_t field_control;                      /**< Header field control */
    uint16_t manufacturer_code;                  /**< Manufacturer code */
    uint16_t image_type;                         /**< Image type */
    uint32_t file_version;                       /**< File version */
    uint16_t zigbee_stack_version;               /**< Zigbee stack version */
    char header_string[ZB_OTA_HEADER_STRING_MAX_LEN]; /**< OTA header string */
    uint32_t total_image_size;                   /**< Total image size including header */
    /* Optional fields based on field_control */
    uint8_t security_credential_version;         /**< Security credential version (optional) */
    uint64_t upgrade_file_destination;           /**< Upgrade file destination (optional) */
    uint16_t min_hw_version;                     /**< Minimum hardware version (optional) */
    uint16_t max_hw_version;                     /**< Maximum hardware version (optional) */
} zb_ota_file_header_t;

/**
 * @brief OTA Image Information Structure
 */
typedef struct {
    uint16_t manufacturer_code;                  /**< Manufacturer code */
    uint16_t image_type;                         /**< Image type */
    uint32_t file_version;                       /**< File version */
    uint32_t image_size;                         /**< Total image size */
    uint16_t header_length;                      /**< Header length */
    char file_path[64];                          /**< Path to image file in SPIFFS */
    char header_string[ZB_OTA_HEADER_STRING_MAX_LEN]; /**< Human-readable header string */
    uint16_t min_hw_version;                     /**< Minimum hardware version (0=any) */
    uint16_t max_hw_version;                     /**< Maximum hardware version (0xFFFF=any) */
    bool valid;                                  /**< Image is valid and available */
    uint32_t add_timestamp;                      /**< Timestamp when image was added */
} zb_ota_image_info_t;

/**
 * @brief OTA Transfer State
 */
typedef enum {
    ZB_OTA_TRANSFER_IDLE = 0,                    /**< No transfer in progress */
    ZB_OTA_TRANSFER_QUERY,                       /**< Device queried for image */
    ZB_OTA_TRANSFER_DOWNLOADING,                 /**< Transfer in progress */
    ZB_OTA_TRANSFER_COMPLETE,                    /**< Transfer complete, waiting for upgrade end */
    ZB_OTA_TRANSFER_UPGRADING,                   /**< Device is upgrading */
    ZB_OTA_TRANSFER_SUCCESS,                     /**< Upgrade successful */
    ZB_OTA_TRANSFER_FAILED,                      /**< Transfer failed */
    ZB_OTA_TRANSFER_ABORTED                      /**< Transfer aborted */
} zb_ota_transfer_state_t;

/**
 * @brief OTA Transfer Session Structure
 *
 * Tracks an ongoing OTA transfer to a device.
 */
typedef struct {
    esp_zb_ieee_addr_t ieee_addr;                /**< Device IEEE address */
    uint16_t short_addr;                         /**< Device short address */
    uint16_t manufacturer_code;                  /**< Manufacturer code */
    uint16_t image_type;                         /**< Image type */
    uint32_t file_version;                       /**< File version being transferred */
    zb_ota_transfer_state_t state;               /**< Current transfer state */
    uint32_t file_offset;                        /**< Current file offset */
    uint32_t image_size;                         /**< Total image size */
    uint8_t block_size;                          /**< Block size for this transfer */
    uint32_t last_block_time;                    /**< Timestamp of last block request */
    uint32_t blocks_sent;                        /**< Number of blocks sent */
    uint32_t start_time;                         /**< Transfer start timestamp */
    uint8_t retry_count;                         /**< Retry count for current block */
    uint8_t endpoint;                            /**< Device endpoint */
    zb_ota_image_info_t *image;                  /**< Pointer to image info */
} zb_ota_transfer_t;

/**
 * @brief OTA Progress Callback Function Type
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[in] state Current transfer state
 * @param[in] progress Progress percentage (0-100)
 */
typedef void (*zb_ota_progress_cb_t)(const esp_zb_ieee_addr_t ieee_addr,
                                      zb_ota_transfer_state_t state,
                                      uint8_t progress);

/**
 * @brief OTA Statistics Structure
 */
typedef struct {
    uint32_t total_upgrades;                     /**< Total successful upgrades */
    uint32_t failed_upgrades;                    /**< Total failed upgrades */
    uint32_t aborted_upgrades;                   /**< Total aborted upgrades */
    uint32_t total_bytes_sent;                   /**< Total bytes transferred */
    uint32_t total_blocks_sent;                  /**< Total blocks sent */
    uint32_t active_transfers;                   /**< Currently active transfers */
} zb_ota_stats_t;

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize Zigbee OTA server
 *
 * Initializes the OTA upgrade server including:
 * - SPIFFS partition mounting
 * - NVS metadata loading
 * - OTA cluster registration
 * - APS fragmentation optimization (API-011)
 *
 * The APS fragmentation is optimized for OTA transfers by setting:
 * - Fragment window size to 8 (allows up to 8 fragments in flight)
 * - Interframe delay to 50ms (time between fragment sends)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_ota_init(void);

/**
 * @brief Deinitialize Zigbee OTA server
 *
 * Stops all transfers and releases resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_ota_deinit(void);

/**
 * @brief Add OTA image from file
 *
 * Adds an OTA image from the specified file path. The file must be
 * a valid Zigbee OTA image with proper header.
 *
 * @param[in] file_path Path to OTA image file
 * @param[out] image_info Output: Image information (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if file_path is NULL
 * @return ESP_ERR_NOT_FOUND if file not found
 * @return ESP_ERR_INVALID_SIZE if file too large
 * @return ESP_ERR_NO_MEM if max images reached
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_ota_add_image(const char *file_path, zb_ota_image_info_t *image_info);

/**
 * @brief Add OTA image from binary data
 *
 * Adds an OTA image from raw binary data. Useful for receiving
 * images via MQTT or HTTP.
 *
 * @param[in] data OTA image data
 * @param[in] size Data size in bytes
 * @param[out] image_info Output: Image information (can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if data is NULL or size is 0
 * @return ESP_ERR_INVALID_SIZE if image too large
 * @return ESP_ERR_NO_MEM if max images reached
 * @return ESP_FAIL on other errors
 */
esp_err_t zb_ota_add_image_data(const uint8_t *data, size_t size,
                                 zb_ota_image_info_t *image_info);

/**
 * @brief Remove OTA image
 *
 * Removes an OTA image identified by manufacturer code, image type,
 * and file version.
 *
 * @param[in] manufacturer_code Manufacturer code
 * @param[in] image_type Image type
 * @param[in] file_version File version
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if image not found
 */
esp_err_t zb_ota_remove_image(uint16_t manufacturer_code, uint16_t image_type,
                               uint32_t file_version);

/**
 * @brief Remove all OTA images
 *
 * Removes all stored OTA images.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_ota_remove_all_images(void);

/**
 * @brief Get list of available OTA images
 *
 * @param[out] images Array to store image info
 * @param[in] max_count Maximum number of images to return
 * @return Number of images copied to array
 */
size_t zb_ota_get_images(zb_ota_image_info_t *images, size_t max_count);

/**
 * @brief Get OTA image count
 *
 * @return Number of stored OTA images
 */
size_t zb_ota_get_image_count(void);

/**
 * @brief Find OTA image for device
 *
 * Searches for an available OTA image matching the device criteria.
 *
 * @param[in] manufacturer_code Device manufacturer code
 * @param[in] image_type Device image type
 * @param[in] current_version Device's current version
 * @param[in] hw_version Device hardware version (0=ignore)
 * @return Pointer to image info or NULL if not found
 */
const zb_ota_image_info_t* zb_ota_find_image(uint16_t manufacturer_code,
                                              uint16_t image_type,
                                              uint32_t current_version,
                                              uint16_t hw_version);

/**
 * @brief Notify device of available update
 *
 * Sends an Image Notify command to the specified device to trigger
 * an OTA update check.
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[in] endpoint Device OTA endpoint
 * @param[in] payload_type Notify payload type
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_FAIL on send error
 */
esp_err_t zb_ota_notify_device(const esp_zb_ieee_addr_t ieee_addr,
                                uint8_t endpoint,
                                uint8_t payload_type);

/**
 * @brief Notify device by short address
 *
 * Sends an Image Notify command using short address.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device OTA endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_ota_notify_device_short(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Broadcast Image Notify
 *
 * Sends a broadcast Image Notify to trigger update checks on all devices.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_ota_notify_broadcast(void);

/**
 * @brief Check for update for specific device
 *
 * Checks if an update is available for the specified device.
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[in] manufacturer_code Device manufacturer code
 * @param[in] image_type Device image type
 * @param[in] current_version Current file version
 * @param[out] new_version Output: Available version (can be NULL)
 * @return true if update available, false otherwise
 */
bool zb_ota_check_update_available(const esp_zb_ieee_addr_t ieee_addr,
                                    uint16_t manufacturer_code,
                                    uint16_t image_type,
                                    uint32_t current_version,
                                    uint32_t *new_version);

/**
 * @brief Get transfer status for device
 *
 * @param[in] ieee_addr Device IEEE address
 * @param[out] transfer Transfer info (can be NULL)
 * @return ESP_OK if transfer exists
 * @return ESP_ERR_NOT_FOUND if no transfer
 */
esp_err_t zb_ota_get_transfer_status(const esp_zb_ieee_addr_t ieee_addr,
                                      zb_ota_transfer_t *transfer);

/**
 * @brief Abort OTA transfer
 *
 * Aborts an ongoing OTA transfer to the specified device.
 *
 * @param[in] ieee_addr Device IEEE address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no transfer
 */
esp_err_t zb_ota_abort_transfer(const esp_zb_ieee_addr_t ieee_addr);

/**
 * @brief Abort all transfers
 *
 * Aborts all ongoing OTA transfers.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_ota_abort_all_transfers(void);

/**
 * @brief Get OTA statistics
 *
 * @param[out] stats Statistics structure
 * @return ESP_OK on success
 */
esp_err_t zb_ota_get_stats(zb_ota_stats_t *stats);

/**
 * @brief Reset OTA statistics
 *
 * @return ESP_OK on success
 */
esp_err_t zb_ota_reset_stats(void);

/**
 * @brief Register progress callback
 *
 * @param[in] callback Progress callback function
 * @return ESP_OK on success
 */
esp_err_t zb_ota_register_callback(zb_ota_progress_cb_t callback);

/**
 * @brief Set block size
 *
 * Sets the default block size for OTA transfers.
 *
 * @param[in] block_size Block size (64-255)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if size out of range
 */
esp_err_t zb_ota_set_block_size(uint8_t block_size);

/**
 * @brief Get block size
 *
 * @return Current block size
 */
uint8_t zb_ota_get_block_size(void);

/**
 * @brief Set rate limit
 *
 * Sets the maximum blocks per second rate limit.
 *
 * @param[in] max_blocks_per_sec Maximum blocks per second
 * @return ESP_OK on success
 */
esp_err_t zb_ota_set_rate_limit(uint8_t max_blocks_per_sec);

/* ============================================================================
 * ZCL Command Handlers (Internal Use)
 *
 * Note: ESP-Zigbee-SDK v1.6.x handles OTA commands via the action handler
 * callback system. These functions are called with the custom cluster command
 * message type provided by the SDK.
 * ============================================================================ */

/**
 * @brief Handle Query Next Image Request
 *
 * Called when a device sends Query Next Image Request.
 *
 * @param[in] message Custom cluster command message from action handler
 * @return ESP_OK on success
 */
esp_err_t zb_ota_handle_query_next_image_req(const esp_zb_zcl_custom_cluster_command_message_t *message);

/**
 * @brief Handle Image Block Request
 *
 * Called when a device sends Image Block Request.
 *
 * @param[in] message Custom cluster command message from action handler
 * @return ESP_OK on success
 */
esp_err_t zb_ota_handle_image_block_req(const esp_zb_zcl_custom_cluster_command_message_t *message);

/**
 * @brief Handle Upgrade End Request
 *
 * Called when a device sends Upgrade End Request.
 *
 * @param[in] message Custom cluster command message from action handler
 * @return ESP_OK on success
 */
esp_err_t zb_ota_handle_upgrade_end_req(const esp_zb_zcl_custom_cluster_command_message_t *message);

/* ============================================================================
 * MQTT Integration
 * ============================================================================ */

/**
 * @brief Process MQTT OTA request
 *
 * Handles OTA-related MQTT requests:
 * - zigbee2mqtt/bridge/request/ota/update/{ieee}
 * - zigbee2mqtt/bridge/request/ota/check/{ieee}
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 */
esp_err_t zb_ota_process_mqtt_request(const char *topic, const char *payload, size_t len);

/**
 * @brief Handle MQTT image upload
 *
 * Receives OTA image data via MQTT. Called with binary data
 * chunks for large file uploads.
 *
 * @param[in] data Binary image data
 * @param[in] len Data length
 * @param[in] final true if this is the final chunk
 * @return ESP_OK on success
 */
esp_err_t zb_ota_mqtt_receive_image(const uint8_t *data, size_t len, bool final);

/**
 * @brief Publish OTA images list
 *
 * Publishes list of available OTA images to MQTT.
 * Topic: zigbee2mqtt/bridge/ota/images
 *
 * @return ESP_OK on success
 */
esp_err_t zb_ota_publish_images(void);

/**
 * @brief Publish OTA status for device
 *
 * Publishes OTA transfer status for a device.
 * Topic: zigbee2mqtt/bridge/ota/status/{ieee}
 *
 * @param[in] ieee_addr Device IEEE address
 * @return ESP_OK on success
 */
esp_err_t zb_ota_publish_status(const esp_zb_ieee_addr_t ieee_addr);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Parse OTA file header
 *
 * Parses and validates an OTA file header from raw data.
 *
 * @param[in] data Raw header data
 * @param[in] len Data length
 * @param[out] header Parsed header structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid header
 */
esp_err_t zb_ota_parse_header(const uint8_t *data, size_t len,
                               zb_ota_file_header_t *header);

/**
 * @brief Validate OTA file header
 *
 * @param[in] header Header to validate
 * @return true if valid, false otherwise
 */
bool zb_ota_validate_header(const zb_ota_file_header_t *header);

/**
 * @brief Get state string
 *
 * @param[in] state Transfer state
 * @return State string
 */
const char* zb_ota_get_state_string(zb_ota_transfer_state_t state);

/**
 * @brief Self-test function
 *
 * Tests OTA functionality.
 *
 * @return ESP_OK if all tests pass
 */
esp_err_t zb_ota_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_OTA_H */
