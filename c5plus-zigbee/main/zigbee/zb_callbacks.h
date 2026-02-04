/**
 * @file zb_callbacks.h
 * @brief Zigbee Event Callbacks API
 *
 * This module handles Zigbee stack callbacks including device join/leave,
 * attribute changes, and ZCL commands.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_CALLBACKS_H
#define ZB_CALLBACKS_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize callback handlers
 *
 * Registers all Zigbee stack callbacks.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_callbacks_init(void);

/**
 * @brief Device join callback
 *
 * Called when a new device joins the network.
 *
 * @param zdo_status ZDO status code
 * @param short_addr Device short address
 * @param ieee_addr Device IEEE address
 */
void zb_callback_device_join(esp_zb_zdp_status_t zdo_status, uint16_t short_addr,
                              esp_zb_ieee_addr_t ieee_addr);

/**
 * @brief Device leave callback
 *
 * Called when a device leaves the network.
 *
 * @param short_addr Device short address
 */
void zb_callback_device_leave(uint16_t short_addr);

/**
 * @brief Attribute change callback
 *
 * Called when a device attribute changes locally.
 *
 * @param endpoint Endpoint number
 * @param cluster_id Cluster ID
 * @param attr_id Attribute ID
 * @param value Attribute value
 * @param value_len Value length in bytes
 */
void zb_callback_attribute_change(uint8_t endpoint, uint16_t cluster_id,
                                   uint16_t attr_id, void *value, size_t value_len);

/**
 * @brief Attribute report callback
 *
 * Called when an attribute report is received from a device.
 * This is the main callback for handling device state updates.
 *
 * @param message ZCL report attribute message
 */
void zb_callback_report_attr(esp_zb_zcl_report_attr_message_t *message);

/**
 * @brief ZDO signal handler callback
 *
 * Main Zigbee stack signal handler. Routes signals to appropriate callbacks.
 *
 * @param signal_struct Zigbee signal structure
 */
void zb_callback_signal_handler(esp_zb_app_signal_t *signal_struct);

/*
 * Note: ZCL command handling has changed in ESP-Zigbee-SDK v1.6.x.
 * The esp_zb_zcl_cmd_recv_message_t type no longer exists.
 * ZCL commands are now handled via the action handler callback registered
 * with esp_zb_core_action_handler_register(). See zb_coordinator.c for
 * the action handler implementation.
 */

/**
 * @brief Device announce callback
 *
 * Called when a device sends an announce message (typically after power on).
 *
 * @param device_annce Device announce parameters
 */
void zb_callback_device_announce(esp_zb_zdo_signal_device_annce_params_t *device_annce);

/**
 * @brief Network formed callback
 *
 * Called when the coordinator successfully forms a network.
 */
void zb_callback_network_formed(void);

/**
 * @brief BDB commissioning callback
 *
 * Called when BDB commissioning completes.
 *
 * @param status Commissioning status
 */
void zb_callback_bdb_commissioning_complete(esp_zb_bdb_commissioning_status_t status);

/**
 * @brief Permit join callback
 *
 * Called when permit join status changes.
 *
 * @param permit_duration Permit join duration (0=closed)
 */
void zb_callback_permit_join_changed(uint8_t permit_duration);

#ifdef __cplusplus
}
#endif

#endif /* ZB_CALLBACKS_H */
