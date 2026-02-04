/**
 * @file zb_cluster_measurement.h
 * @brief Measurement Cluster APIs
 *
 * Provides support for:
 * - Illuminance Measurement (0x0400)
 * - Pressure Measurement (0x0403)
 * - PM2.5 Measurement (0x042A)
 *
 * Note: Temperature (0x0402) and Humidity (0x0405) are handled by
 * zb_device_handler.c as they are commonly used and have
 * simpler implementations.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CLUSTER_MEASUREMENT_H
#define ZB_CLUSTER_MEASUREMENT_H

#include "esp_err.h"
#include "zb_device_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Measurement Clusters API - Declared in zb_device_handler.h
 *
 * Illuminance (0x0400):
 * - zb_illuminance_state_t
 * - zb_illuminance_state_cb_t
 * - zb_illuminance_register_callback()
 * - zb_device_has_illuminance()
 * - zb_illuminance_read_value()
 * - zb_illuminance_handle_report()
 * - zb_illuminance_get_state()
 * - zb_illuminance_to_lux()
 *
 * Pressure (0x0403):
 * - zb_pressure_state_t
 * - zb_pressure_state_cb_t
 * - zb_pressure_register_callback()
 * - zb_device_has_pressure()
 * - zb_pressure_read_value()
 * - zb_pressure_handle_report()
 * - zb_pressure_get_state()
 * - zb_pressure_to_hpa()
 *
 * PM2.5 (0x042A):
 * - zb_pm25_state_t
 * - zb_pm25_state_cb_t
 * - zb_pm25_register_callback()
 * - zb_device_has_pm25()
 * - zb_pm25_read_value()
 * - zb_pm25_handle_report()
 * - zb_pm25_get_state()
 *
 * This header provides internal initialization functions.
 * ============================================================================ */

/**
 * @brief Initialize Measurement cluster state storage
 *
 * Called by zb_device_handler_init() to initialize cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_measurement_init(void);

/**
 * @brief Deinitialize Measurement cluster state storage
 *
 * Called by zb_device_handler_deinit() to clean up cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_measurement_deinit(void);

/**
 * @brief Clear all Measurement cluster state entries
 *
 * Removes all stored Illuminance, Pressure, and PM2.5 states.
 * Used when clearing the device registry.
 */
void zb_cluster_measurement_clear_all(void);

/**
 * @brief Remove Measurement cluster state for a device
 *
 * @param[in] short_addr Device short address
 */
void zb_cluster_measurement_remove_device(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CLUSTER_MEASUREMENT_H */
