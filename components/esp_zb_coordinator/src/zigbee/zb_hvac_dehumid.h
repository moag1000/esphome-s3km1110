/**
 * @file zb_hvac_dehumid.h
 * @brief Zigbee Dehumidification Control Cluster (0x0203) Support
 *
 * This module provides support for the ZCL Dehumidification Control cluster,
 * which is used by HVAC devices to control humidity levels.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_HVAC_DEHUMID_H
#define ZB_HVAC_DEHUMID_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Dehumidification Control Cluster (0x0203) Definitions
 * ============================================================================ */

/**
 * @brief Dehumidification Control Cluster ID
 */
#define ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL  0x0203

/**
 * @brief Maximum dehumidification control devices tracked
 */
#define ZB_STATE_MAX_DEHUMID                        16

/**
 * @brief Dehumidification Control Cluster Attribute IDs
 */
#define ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_ID            0x0000  /**< RelativeHumidity (uint8, 0-100%, read-only) */
#define ZB_ZCL_ATTR_DEHUMID_DEHUMIDIFICATION_COOLING_ID     0x0001  /**< DehumidificationCooling (uint8, 0-100%) */
#define ZB_ZCL_ATTR_DEHUMID_RH_DEHUMID_SETPOINT_ID          0x0010  /**< RhDehumidificationSetpoint (uint8, 0-100%, writable) */
#define ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_MODE_ID       0x0011  /**< RelativeHumidityMode (enum8) */
#define ZB_ZCL_ATTR_DEHUMID_DEHUMID_LOCKOUT_ID              0x0012  /**< DehumidificationLockout (enum8) */
#define ZB_ZCL_ATTR_DEHUMID_DEHUMID_HYSTERESIS_ID           0x0013  /**< DehumidificationHysteresis (uint8) */
#define ZB_ZCL_ATTR_DEHUMID_DEHUMID_MAX_COOL_ID             0x0014  /**< DehumidificationMaxCool (uint8, 0-100%) */
#define ZB_ZCL_ATTR_DEHUMID_RELATIVE_HUMIDITY_DISPLAY_ID    0x0015  /**< RelativeHumidityDisplay (enum8) */

/**
 * @brief RelativeHumidityMode values (enum8)
 */
typedef enum {
    ZB_DEHUMID_RH_MODE_MEASURED_LOCALLY = 0x00,     /**< Measured locally */
    ZB_DEHUMID_RH_MODE_UPDATED_OVER_NETWORK = 0x01  /**< Updated over network */
} zb_dehumid_rh_mode_t;

/**
 * @brief DehumidificationLockout values (enum8)
 */
typedef enum {
    ZB_DEHUMID_LOCKOUT_NOT_ALLOWED = 0x00,  /**< Dehumidification not allowed */
    ZB_DEHUMID_LOCKOUT_ALLOWED = 0x01       /**< Dehumidification allowed */
} zb_dehumid_lockout_t;

/**
 * @brief RelativeHumidityDisplay values (enum8)
 */
typedef enum {
    ZB_DEHUMID_DISPLAY_NOT_DISPLAYED = 0x00,  /**< Not displayed */
    ZB_DEHUMID_DISPLAY_DISPLAYED = 0x01       /**< Displayed */
} zb_dehumid_display_t;

/**
 * @brief Dehumidification control state structure
 */
typedef struct {
    uint8_t relative_humidity;          /**< Current humidity (0-100%) */
    uint8_t dehumid_cooling;            /**< Cooling effect percentage (0-100%) */
    uint8_t rh_setpoint;                /**< Target humidity setpoint (0-100%) */
    uint8_t relative_humidity_mode;     /**< Relative humidity mode (enum8) */
    uint8_t dehumid_lockout;            /**< Dehumidification lockout state (enum8) */
    uint8_t dehumid_hysteresis;         /**< Dehumidification hysteresis (uint8) */
    uint8_t dehumid_max_cool;           /**< Maximum cooling percentage (0-100%) */
    uint8_t relative_humidity_display;  /**< Display mode (enum8) */
} zb_dehumid_state_t;

/**
 * @brief Dehumidification control state change callback type
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] state Current dehumidification control state
 */
typedef void (*zb_dehumid_state_cb_t)(uint16_t short_addr, uint8_t endpoint,
                                       const zb_dehumid_state_t *state);

/* ============================================================================
 * Initialization and Deinitialization
 * ============================================================================ */

/**
 * @brief Initialize dehumidification control module
 *
 * Initializes the dehumidification control state storage.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_init(void);

/**
 * @brief Deinitialize dehumidification control module
 *
 * Clears all state storage and resets the module.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_deinit(void);

/* ============================================================================
 * State Management
 * ============================================================================ */

/**
 * @brief Register dehumidification state change callback
 *
 * @param[in] callback Callback function to register
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_register_callback(zb_dehumid_state_cb_t callback);

/**
 * @brief Check if device has Dehumidification Control cluster
 *
 * @param[in] short_addr Device short address
 * @return true if device has Dehumidification Control cluster
 */
bool zb_device_has_dehumid_control(uint16_t short_addr);

/**
 * @brief Handle dehumidification attribute report
 *
 * Processes incoming attribute reports from dehumidification devices.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] attr_id Attribute ID
 * @param[in] value Attribute value
 * @param[in] value_len Value length in bytes
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_handle_report(uint16_t short_addr, uint8_t endpoint,
                                    uint16_t attr_id, void *value, size_t value_len);

/**
 * @brief Get current dehumidification state for a device
 *
 * @param[in] short_addr Device short address
 * @param[out] state Output state structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_dehumid_get_state(uint16_t short_addr, zb_dehumid_state_t *state);

/* ============================================================================
 * Control Commands
 * ============================================================================ */

/**
 * @brief Read dehumidification state from device
 *
 * Reads all dehumidification attributes from the device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_read_state(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set dehumidification setpoint (target humidity)
 *
 * Sets the target humidity percentage for dehumidification.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] setpoint Target humidity (0-100%)
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_set_setpoint(uint16_t short_addr, uint8_t endpoint,
                                   uint8_t setpoint);

/**
 * @brief Set dehumidification hysteresis
 *
 * Sets the hysteresis value for dehumidification control.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] hysteresis Hysteresis value
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_set_hysteresis(uint16_t short_addr, uint8_t endpoint,
                                     uint8_t hysteresis);

/**
 * @brief Set dehumidification maximum cooling
 *
 * Sets the maximum cooling percentage for dehumidification.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Device endpoint
 * @param[in] max_cool Maximum cooling percentage (0-100%)
 * @return ESP_OK on success
 */
esp_err_t zb_dehumid_set_max_cool(uint16_t short_addr, uint8_t endpoint,
                                   uint8_t max_cool);

/* ============================================================================
 * Testing
 * ============================================================================ */

/**
 * @brief Self-test function for dehumidification control module
 *
 * Tests state storage and retrieval operations.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_dehumid_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_HVAC_DEHUMID_H */
