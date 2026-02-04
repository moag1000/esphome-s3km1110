/**
 * @file zb_cluster_security.h
 * @brief IAS Zone (0x0500) and Door Lock (0x0101) Cluster APIs
 *
 * Provides support for security sensors and door lock devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CLUSTER_SECURITY_H
#define ZB_CLUSTER_SECURITY_H

#include "esp_err.h"
#include "zb_device_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Security Clusters API - Declared in zb_device_handler.h
 *
 * IAS Zone (0x0500):
 * - zb_ias_zone_state_struct_t
 * - zb_ias_zone_state_cb_t
 * - zb_ias_zone_register_callback()
 * - zb_device_has_ias_zone()
 * - zb_ias_zone_read_state()
 * - zb_ias_zone_write_cie_address()
 * - zb_ias_zone_enroll_response()
 * - zb_ias_zone_handle_report()
 * - zb_ias_zone_handle_status_change()
 * - zb_ias_zone_handle_enroll_request()
 * - zb_ias_zone_get_state()
 * - zb_ias_zone_get_device_class()
 * - zb_ias_zone_parse_status()
 * - zb_ias_zone_set_auto_enroll()
 * - zb_ias_zone_get_auto_enroll()
 *
 * Door Lock (0x0101):
 * - zb_door_lock_state_struct_t
 * - zb_door_lock_state_cb_t
 * - zb_door_lock_register_callback()
 * - zb_device_has_door_lock()
 * - zb_door_lock_read_state()
 * - zb_door_lock_cmd_lock()
 * - zb_door_lock_cmd_unlock()
 * - zb_door_lock_handle_report()
 * - zb_door_lock_get_state()
 *
 * This header provides internal initialization functions.
 * ============================================================================ */

/**
 * @brief Initialize Security cluster state storage
 *
 * Called by zb_device_handler_init() to initialize cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_security_init(void);

/**
 * @brief Deinitialize Security cluster state storage
 *
 * Called by zb_device_handler_deinit() to clean up cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_security_deinit(void);

/**
 * @brief Clear all Security cluster state entries
 *
 * Removes all stored IAS Zone and Door Lock states. Used when clearing
 * the device registry.
 */
void zb_cluster_security_clear_all(void);

/**
 * @brief Remove Security cluster state for a device
 *
 * @param[in] short_addr Device short address
 */
void zb_cluster_security_remove_device(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CLUSTER_SECURITY_H */
