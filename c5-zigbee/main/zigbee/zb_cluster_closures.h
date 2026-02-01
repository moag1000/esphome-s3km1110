/**
 * @file zb_cluster_closures.h
 * @brief Window Covering Cluster (0x0102) API
 *
 * Provides control for blinds, shades, curtains, and other window
 * covering devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CLUSTER_CLOSURES_H
#define ZB_CLUSTER_CLOSURES_H

#include "esp_err.h"
#include "zb_device_handler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Window Covering Cluster (0x0102) API - Declared in zb_device_handler.h
 *
 * The following types and functions are already declared in zb_device_handler.h:
 * - zb_window_covering_state_t
 * - zb_window_covering_state_cb_t
 * - zb_window_covering_register_callback()
 * - zb_device_has_window_covering()
 * - zb_window_covering_read_position()
 * - zb_window_covering_open()
 * - zb_window_covering_close()
 * - zb_window_covering_stop()
 * - zb_window_covering_set_position()
 * - zb_window_covering_handle_report()
 * - zb_window_covering_get_state()
 *
 * This header provides internal initialization functions.
 * ============================================================================ */

/**
 * @brief Initialize Window Covering cluster state storage
 *
 * Called by zb_device_handler_init() to initialize cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_closures_init(void);

/**
 * @brief Deinitialize Window Covering cluster state storage
 *
 * Called by zb_device_handler_deinit() to clean up cluster state.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_cluster_closures_deinit(void);

/**
 * @brief Clear all Window Covering state entries
 *
 * Removes all stored window covering states. Used when clearing
 * the device registry.
 */
void zb_cluster_closures_clear_all(void);

/**
 * @brief Remove Window Covering state for a device
 *
 * @param[in] short_addr Device short address
 */
void zb_cluster_closures_remove_device(uint16_t short_addr);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CLUSTER_CLOSURES_H */
