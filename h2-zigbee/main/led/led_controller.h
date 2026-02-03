/**
 * @file led_controller.h
 * @brief LED Controller for ESP32-H2 (Stub)
 *
 * The H2-Zero module typically has no addressable LED.
 * This is a stub implementation that provides no-op functions.
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LED Status Definitions */
typedef enum {
    LED_STATUS_OFF = 0,
    LED_STATUS_BOOT,
    LED_STATUS_NORMAL,
    LED_STATUS_PAIRING,
    LED_STATUS_ERROR,
} led_status_t;

/**
 * @brief Initialize LED controller
 * @return ESP_ERR_NOT_SUPPORTED on H2-Zero (no LED)
 */
esp_err_t led_controller_init(void);

/**
 * @brief Set LED status
 * @param status LED status to set
 */
void led_set_status(led_status_t status);

/**
 * @brief De-initialize LED controller
 */
void led_controller_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROLLER_H */
