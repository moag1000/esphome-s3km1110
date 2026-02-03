/**
 * @file tuya_device_driver.h
 * @brief Tuya Device Driver Vtable Interface
 *
 * Defines the interface (vtable) for Tuya device-specific drivers.
 * Each Tuya device type (Fingerbot, Curtain, Valve, etc.) implements
 * this interface to handle its DPs, commands, state, and discovery.
 *
 * Tuya devices may use BOTH the Tuya Private Cluster (0xEF00) AND
 * standard ZCL clusters (e.g. On/Off 0x0006). The driver decides
 * which protocol to use for each command.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef TUYA_DEVICE_DRIVER_H
#define TUYA_DEVICE_DRIVER_H

#include "esp_err.h"
#include "zb_tuya.h"
#include "zb_device_handler.h"
#include "cJSON.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tuya device driver interface (vtable)
 *
 * Each Tuya device type provides a static instance of this struct
 * with function pointers for device-specific behavior.
 */
typedef struct tuya_device_driver {
    /** @brief Driver name (e.g. "fingerbot", "curtain") */
    const char *name;

    /**
     * @brief Check if this driver handles the given device
     *
     * @param manufacturer Manufacturer string (e.g. "_TZ3210")
     * @param model Model string (e.g. "TS004F")
     * @return true if this driver handles the device
     */
    bool (*match)(const char *manufacturer, const char *model);

    /**
     * @brief Process incoming Tuya DP (cmd 0x01/0x02/0x05)
     *
     * Called when a DP report is received from the device.
     *
     * @param short_addr Device short address
     * @param dp Parsed DP structure
     * @return ESP_OK on success
     */
    esp_err_t (*process_dp)(uint16_t short_addr, const tuya_dp_t *dp);

    /**
     * @brief Process incoming ZCL attribute report (optional, may be NULL)
     *
     * Called when a ZCL attribute report is received and a driver is bound.
     * Used by non-Tuya drivers (e.g. Xiaomi/Aqara) that communicate via
     * standard or manufacturer-specific ZCL attributes instead of Tuya DPs.
     *
     * @param short_addr Device short address
     * @param endpoint Source endpoint
     * @param cluster_id ZCL cluster ID
     * @param attr_id ZCL attribute ID
     * @param data Pointer to attribute data payload
     * @param data_size Size of attribute data in bytes
     * @param data_type ZCL attribute data type
     * @return ESP_OK if handled, ESP_ERR_NOT_FOUND if not handled
     */
    esp_err_t (*process_zcl_attr)(uint16_t short_addr, uint8_t endpoint,
                                   uint16_t cluster_id, uint16_t attr_id,
                                   const void *data, uint16_t data_size,
                                   uint8_t data_type);

    /**
     * @brief Handle MQTT command JSON
     *
     * Can send both Tuya DPs (via zb_tuya_send_dp) and standard ZCL
     * commands (via command_send_on_off etc.).
     *
     * @param short_addr Device short address
     * @param endpoint Device endpoint
     * @param json Parsed JSON command object
     * @return ESP_OK if at least one field was handled
     * @return ESP_ERR_NOT_FOUND if no known field in JSON
     */
    esp_err_t (*handle_command)(uint16_t short_addr, uint8_t endpoint,
                                const cJSON *json);

    /**
     * @brief Build device-specific state JSON for MQTT publish
     *
     * @param short_addr Device short address
     * @return cJSON object (caller must delete), or NULL on error
     */
    cJSON *(*build_state_json)(uint16_t short_addr);

    /**
     * @brief Publish HA discovery entities
     *
     * @param device Zigbee device
     * @return ESP_OK on success
     */
    esp_err_t (*publish_discovery)(const zb_device_t *device);

    /**
     * @brief Initialize device state (optional, may be NULL)
     *
     * Called when driver is bound to a device.
     *
     * @param short_addr Device short address
     * @return ESP_OK on success
     */
    esp_err_t (*init_device)(uint16_t short_addr);

    /**
     * @brief Remove device state (optional, may be NULL)
     *
     * Called when device is removed from the network.
     *
     * @param short_addr Device short address
     */
    void (*remove_device)(uint16_t short_addr);

} tuya_device_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* TUYA_DEVICE_DRIVER_H */
