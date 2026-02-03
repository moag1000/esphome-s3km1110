/**
 * @file zb_cluster_hvac.h
 * @brief Thermostat (0x0201) and Fan Control (0x0202) Cluster APIs
 *
 * Provides control for HVAC devices including thermostats and fans.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CLUSTER_HVAC_H
#define ZB_CLUSTER_HVAC_H

#include "esp_err.h"
#include "zb_device_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * HVAC Clusters API - Declared in zb_device_handler.h
 *
 * Thermostat (0x0201):
 * - zb_thermostat_state_t
 * - zb_thermostat_state_cb_t
 * - zb_thermostat_register_callback()
 * - zb_device_has_thermostat()
 * - zb_thermostat_read_state()
 * - zb_thermostat_set_heating_setpoint()
 * - zb_thermostat_set_cooling_setpoint()
 * - zb_thermostat_set_system_mode()
 * - zb_thermostat_handle_report()
 * - zb_thermostat_get_state()
 *
 * Fan Control (0x0202):
 * - zb_fan_control_state_t
 * - zb_fan_control_state_cb_t
 * - zb_fan_control_register_callback()
 * - zb_device_has_fan_control()
 * - zb_fan_control_read_state()
 * - zb_fan_control_set_mode()
 * - zb_fan_control_handle_report()
 * - zb_fan_control_get_state()
 *
 * This header provides internal initialization functions.
 * ============================================================================ */

/**
 * @brief Initialize HVAC cluster state storage
 *
 * Called by zb_device_handler_init() to initialize cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_hvac_init(void);

/**
 * @brief Deinitialize HVAC cluster state storage
 *
 * Called by zb_device_handler_deinit() to clean up cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_hvac_deinit(void);

/**
 * @brief Clear all HVAC cluster state entries
 *
 * Removes all stored Thermostat and Fan Control states. Used when clearing
 * the device registry.
 */
void zb_cluster_hvac_clear_all(void);

/**
 * @brief Remove HVAC cluster state for a device
 *
 * @param[in] short_addr Device short address
 */
void zb_cluster_hvac_remove_device(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CLUSTER_HVAC_H */
