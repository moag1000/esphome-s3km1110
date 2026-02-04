/**
 * @file tuya_fingerbot.h
 * @brief Tuya Fingerbot Device Driver
 *
 * Public header for the Fingerbot Tuya device driver. Provides the
 * registration function to install the Fingerbot driver into the
 * Tuya driver registry.
 *
 * DP defines, state struct, and mode enum remain in zb_tuya.h for
 * backward compatibility.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef TUYA_FINGERBOT_H
#define TUYA_FINGERBOT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Forward declaration of the driver vtable type */
typedef struct tuya_device_driver tuya_device_driver_t;

/**
 * @brief Register the Fingerbot device driver
 *
 * Installs the Fingerbot driver into the Tuya driver registry so that
 * Fingerbot devices are automatically recognized and handled when they
 * join the network.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if registration fails due to memory
 * @return ESP_ERR_INVALID_STATE if registry is not initialized
 */
esp_err_t tuya_fingerbot_register(void);

#ifdef __cplusplus
}
#endif

#endif /* TUYA_FINGERBOT_H */
