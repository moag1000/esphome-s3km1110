/**
 * @file zb_cluster_electrical.h
 * @brief Electrical Measurement (0x0B04) and Metering (0x0702) Cluster APIs
 *
 * Provides support for electrical measurement and smart metering devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CLUSTER_ELECTRICAL_H
#define ZB_CLUSTER_ELECTRICAL_H

#include "esp_err.h"
#include "zb_device_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Electrical Clusters API - Declared in zb_device_handler.h
 *
 * Electrical Measurement (0x0B04):
 * - zb_electrical_state_t
 * - zb_electrical_state_cb_t
 * - zb_electrical_register_callback()
 * - zb_device_has_electrical_measurement()
 * - zb_electrical_read_values()
 * - zb_electrical_read_scaling()
 * - zb_electrical_handle_report()
 * - zb_electrical_get_state()
 * - zb_electrical_get_voltage_v()
 * - zb_electrical_get_current_a()
 * - zb_electrical_get_power_w()
 * - zb_electrical_get_reactive_power_var()
 * - zb_electrical_get_apparent_power_va()
 * - zb_electrical_get_power_factor()
 * - zb_electrical_get_frequency_hz()
 *
 * Metering (0x0702):
 * - zb_metering_state_t
 * - zb_metering_state_cb_t
 * - zb_metering_register_callback()
 * - zb_device_has_metering()
 * - zb_metering_read_values()
 * - zb_metering_handle_report()
 * - zb_metering_get_state()
 * - zb_metering_get_total_energy()
 * - zb_metering_get_power_w()
 * - zb_metering_get_current_day_energy()
 * - zb_metering_get_previous_day_energy()
 * - zb_metering_get_unit_string()
 * - zb_metering_parse_uint48()
 * - zb_metering_parse_int24()
 * - zb_metering_parse_uint24()
 * - zb_metering_get_decimal_places()
 *
 * This header provides internal initialization functions.
 * ============================================================================ */

/**
 * @brief Initialize Electrical cluster state storage
 *
 * Called by zb_device_handler_init() to initialize cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_electrical_init(void);

/**
 * @brief Deinitialize Electrical cluster state storage
 *
 * Called by zb_device_handler_deinit() to clean up cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_electrical_deinit(void);

/**
 * @brief Clear all Electrical cluster state entries
 *
 * Removes all stored Electrical Measurement and Metering states.
 * Used when clearing the device registry.
 */
void zb_cluster_electrical_clear_all(void);

/**
 * @brief Remove Electrical cluster state for a device
 *
 * @param[in] short_addr Device short address
 */
void zb_cluster_electrical_remove_device(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CLUSTER_ELECTRICAL_H */
