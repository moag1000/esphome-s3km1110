/**
 * @file zb_interview.h
 * @brief Zigbee Device Interview API (ZG-001)
 *
 * This module provides device interview functionality to discover all endpoints,
 * clusters, and attributes of a newly joined Zigbee device. The interview process
 * is triggered automatically after device join and provides complete device
 * capability information via callback.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_INTERVIEW_H
#define ZB_INTERVIEW_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of endpoints per device
 */
#define ZB_INTERVIEW_MAX_ENDPOINTS          32

/**
 * @brief Maximum number of clusters per endpoint
 */
#define ZB_INTERVIEW_MAX_CLUSTERS           64

/**
 * @brief Maximum number of attributes per cluster
 */
#define ZB_INTERVIEW_MAX_ATTRIBUTES         32

/**
 * @brief Maximum number of concurrent interviews
 */
#define ZB_INTERVIEW_MAX_CONCURRENT         5

/* Note: ZB_INTERVIEW_TIMEOUT_MS is defined in zb_constants.h */

/**
 * @brief Interview retry count
 */
#define ZB_INTERVIEW_RETRY_COUNT            3

/**
 * @brief Interview status enumeration
 */
typedef enum {
    ZB_INTERVIEW_STATUS_IDLE = 0,           /**< No interview in progress */
    ZB_INTERVIEW_STATUS_ACTIVE_ENDPOINTS,   /**< Discovering active endpoints */
    ZB_INTERVIEW_STATUS_SIMPLE_DESC,        /**< Reading simple descriptor */
    ZB_INTERVIEW_STATUS_ATTRIBUTES,         /**< Reading attribute list */
    ZB_INTERVIEW_STATUS_BASIC_INFO,         /**< Reading basic cluster info */
    ZB_INTERVIEW_STATUS_COMPLETE,           /**< Interview completed successfully */
    ZB_INTERVIEW_STATUS_FAILED,             /**< Interview failed */
    ZB_INTERVIEW_STATUS_TIMEOUT             /**< Interview timed out */
} zb_interview_status_t;

/**
 * @brief Cluster information structure
 */
typedef struct {
    uint16_t cluster_id;                    /**< Cluster ID */
    uint16_t *attribute_ids;                /**< Array of attribute IDs */
    uint8_t attribute_count;                /**< Number of attributes */
} zb_cluster_info_t;

/**
 * @brief Endpoint information structure
 */
typedef struct {
    uint8_t endpoint;                       /**< Endpoint number */
    uint16_t profile_id;                    /**< Profile ID (e.g., HA=0x0104) */
    uint16_t device_id;                     /**< Device ID */
    uint8_t device_version;                 /**< Device version */
    uint16_t *server_clusters;              /**< Array of server cluster IDs */
    uint8_t server_cluster_count;           /**< Number of server clusters */
    uint16_t *client_clusters;              /**< Array of client cluster IDs */
    uint8_t client_cluster_count;           /**< Number of client clusters */
} zb_endpoint_info_t;

/**
 * @brief Interview result structure
 */
typedef struct {
    uint64_t ieee_addr;                     /**< Device IEEE address (64-bit) */
    uint16_t short_addr;                    /**< Device short address */
    zb_endpoint_info_t *endpoints;          /**< Array of endpoint information */
    uint8_t endpoint_count;                 /**< Number of endpoints */
    char manufacturer[33];                  /**< Manufacturer name (from Basic cluster) */
    char model[33];                         /**< Model identifier (from Basic cluster) */
    uint8_t sw_version;                     /**< Software version (from Basic cluster) */
    zb_interview_status_t status;           /**< Final interview status */
    bool interview_complete;                /**< Interview completion flag */
    uint32_t duration_ms;                   /**< Interview duration in milliseconds */
} zb_interview_result_t;

/**
 * @brief Interview completion callback type
 *
 * This callback is invoked when a device interview completes (success or failure).
 *
 * @param result Pointer to interview result (valid only during callback)
 */
typedef void (*zb_interview_complete_cb_t)(const zb_interview_result_t *result);

/**
 * @brief Interview progress callback type
 *
 * Optional callback for interview progress updates.
 *
 * @param ieee_addr Device IEEE address
 * @param status Current interview status
 * @param progress Progress percentage (0-100)
 */
typedef void (*zb_interview_progress_cb_t)(uint64_t ieee_addr,
                                            zb_interview_status_t status,
                                            uint8_t progress);

/**
 * @brief Interview configuration structure
 */
typedef struct {
    zb_interview_complete_cb_t complete_cb; /**< Completion callback (required) */
    zb_interview_progress_cb_t progress_cb; /**< Progress callback (optional) */
    uint32_t timeout_ms;                    /**< Timeout in ms (0=default) */
    uint8_t retry_count;                    /**< Retry count (0=default) */
    bool read_attributes;                   /**< Also read attribute lists */
    bool auto_interview_on_join;            /**< Auto-start on device join */
} zb_interview_config_t;

/**
 * @brief Initialize the interview module
 *
 * Initializes the interview module with the specified configuration.
 * Must be called before any other interview functions.
 *
 * @param config Interview configuration (NULL for defaults)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_interview_init(const zb_interview_config_t *config);

/**
 * @brief Deinitialize the interview module
 *
 * Stops all active interviews and frees all resources.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_interview_deinit(void);

/**
 * @brief Start device interview
 *
 * Begins the interview process for a device. The interview will discover
 * all endpoints, clusters, and optionally attributes.
 *
 * @param ieee_addr Device IEEE address (64-bit)
 * @param short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if addresses are invalid
 * @return ESP_ERR_NO_MEM if max concurrent interviews reached
 */
esp_err_t zb_interview_start(uint64_t ieee_addr, uint16_t short_addr);

/**
 * @brief Cancel device interview
 *
 * Cancels an in-progress interview for the specified device.
 *
 * @param ieee_addr Device IEEE address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no interview in progress for this device
 */
esp_err_t zb_interview_cancel(uint64_t ieee_addr);

/**
 * @brief Get interview result
 *
 * Retrieves the cached interview result for a device.
 * The result is valid until the device is removed or re-interviewed.
 *
 * @param ieee_addr Device IEEE address
 * @return Pointer to interview result, or NULL if not found
 */
const zb_interview_result_t* zb_interview_get_result(uint64_t ieee_addr);

/**
 * @brief Check if interview is in progress
 *
 * @param ieee_addr Device IEEE address
 * @return true if interview is active
 * @return false otherwise
 */
bool zb_interview_is_active(uint64_t ieee_addr);

/**
 * @brief Get current interview status
 *
 * @param ieee_addr Device IEEE address
 * @return Current interview status, or ZB_INTERVIEW_STATUS_IDLE if not found
 */
zb_interview_status_t zb_interview_get_status(uint64_t ieee_addr);

/**
 * @brief Set completion callback
 *
 * Updates the completion callback after initialization.
 *
 * @param callback New completion callback
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_interview_set_complete_callback(zb_interview_complete_cb_t callback);

/**
 * @brief Set progress callback
 *
 * Updates the progress callback after initialization.
 *
 * @param callback New progress callback (NULL to disable)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_interview_set_progress_callback(zb_interview_progress_cb_t callback);

/**
 * @brief Free interview result
 *
 * Frees the memory associated with an interview result.
 * Call this to release cached results when no longer needed.
 *
 * @param ieee_addr Device IEEE address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if result not found
 */
esp_err_t zb_interview_free_result(uint64_t ieee_addr);

/**
 * @brief Get number of active interviews
 *
 * @return Number of interviews currently in progress
 */
uint8_t zb_interview_get_active_count(void);

/**
 * @brief Handle ZDO response for interview
 *
 * Internal function called by ZDO callback handlers to process
 * interview-related responses.
 *
 * @param short_addr Device short address
 * @param status ZDO status
 * @param data Response data
 * @param data_len Data length
 * @return ESP_OK if handled
 * @return ESP_ERR_NOT_FOUND if no matching interview
 */
esp_err_t zb_interview_handle_zdo_response(uint16_t short_addr,
                                            esp_zb_zdp_status_t status,
                                            void *data, size_t data_len);

/**
 * @brief Handle active endpoints response
 *
 * Called when active endpoints response is received.
 *
 * @param short_addr Device short address
 * @param status ZDO status
 * @param endpoints Array of endpoint numbers
 * @param ep_count Number of endpoints
 * @return ESP_OK on success
 */
esp_err_t zb_interview_handle_active_ep_resp(uint16_t short_addr,
                                              esp_zb_zdp_status_t status,
                                              uint8_t *endpoints,
                                              uint8_t ep_count);

/**
 * @brief Handle simple descriptor response
 *
 * Called when simple descriptor response is received.
 *
 * @param short_addr Device short address
 * @param status ZDO status
 * @param simple_desc Simple descriptor data
 * @return ESP_OK on success
 */
esp_err_t zb_interview_handle_simple_desc_resp(uint16_t short_addr,
                                                esp_zb_zdp_status_t status,
                                                esp_zb_af_simple_desc_1_1_t *simple_desc);

/**
 * @brief Self-test function for interview module
 *
 * Tests interview initialization and basic functionality.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_interview_test(void);

/**
 * @brief Handle read attributes response for Basic cluster during interview
 *
 * Called from zb_callbacks.c when a read attributes response is received.
 * This processes manufacturer name and model identifier from the Basic cluster.
 *
 * @param short_addr Device short address
 * @param cluster_id Cluster ID (must be ESP_ZB_ZCL_CLUSTER_ID_BASIC)
 * @param attr_id Attribute ID that was read
 * @param status Status of the read operation
 * @param attr_type Attribute data type
 * @param value Attribute value
 * @param value_len Length of the attribute value
 * @return ESP_OK if handled
 * @return ESP_ERR_NOT_FOUND if no matching interview context
 */
esp_err_t zb_interview_handle_read_attr_resp(uint16_t short_addr,
                                               uint16_t cluster_id,
                                               uint16_t attr_id,
                                               uint8_t status,
                                               uint8_t attr_type,
                                               const void *value,
                                               size_t value_len);

#ifdef __cplusplus
}
#endif

#endif /* ZB_INTERVIEW_H */
