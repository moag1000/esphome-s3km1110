/**
 * @file aqara_vibration.h
 * @brief Aqara DJT11LM Vibration Sensor Driver
 *
 * Driver for the Aqara DJT11LM vibration sensor (model: lumi.vibration.aq1).
 * Uses Cluster 0x0101 (Door Lock, repurposed) for vibration/tilt/drop events
 * and Cluster 0x0000 with Xiaomi proprietary attribute 0xFF01 for battery.
 *
 * Implements the tuya_device_driver_t vtable interface via process_zcl_attr.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef AQARA_VIBRATION_H
#define AQARA_VIBRATION_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register the Aqara DJT11LM vibration sensor driver
 *
 * Installs the driver into the driver registry so that DJT11LM devices
 * are automatically recognized and handled when they join the network.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if registration fails
 * @return ESP_ERR_INVALID_STATE if registry is not initialized
 */
esp_err_t aqara_vibration_register(void);

#ifdef __cplusplus
}
#endif

#endif /* AQARA_VIBRATION_H */
