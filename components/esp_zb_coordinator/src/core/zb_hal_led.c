/**
 * @file zb_hal_led.c
 * @brief HAL LED Callback Implementation
 *
 * Provides the callback registration mechanism for LED functions.
 * The actual LED controller is chip-specific and registered by main.
 */

#include "zb_hal.h"
#include "esp_log.h"

static const char *TAG = "ZB_HAL_LED";

/* Registered callback functions (NULL = no LED controller) */
static zb_hal_led_blink_fn s_led_blink_fn = NULL;
static zb_hal_led_status_fn s_led_status_fn = NULL;

void zb_hal_register_led_callbacks(zb_hal_led_blink_fn blink_fn,
                                   zb_hal_led_status_fn status_fn)
{
    s_led_blink_fn = blink_fn;
    s_led_status_fn = status_fn;
    ESP_LOGI(TAG, "LED callbacks registered: blink=%s, status=%s",
             blink_fn ? "yes" : "no",
             status_fn ? "yes" : "no");
}

void zb_hal_led_blink_notify(zb_hal_led_notify_t notify)
{
    if (s_led_blink_fn != NULL) {
        s_led_blink_fn(notify);
    }
}

void zb_hal_led_set_status(zb_hal_led_status_t status)
{
    if (s_led_status_fn != NULL) {
        s_led_status_fn(status);
    }
}
