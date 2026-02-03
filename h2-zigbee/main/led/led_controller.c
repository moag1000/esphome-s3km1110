/**
 * @file led_controller.c
 * @brief LED Controller for ESP32-H2 (Stub Implementation)
 *
 * The H2-Zero module typically has no addressable LED.
 * This stub implementation provides no-op functions and returns
 * ESP_ERR_NOT_SUPPORTED to indicate no LED is available.
 */

#include "led_controller.h"
#include "zb_hal.h"
#include "esp_log.h"

static const char *TAG = "LED_H2";

esp_err_t led_controller_init(void)
{
    int led_gpio = zb_hal_get_led_gpio();

    if (led_gpio < 0) {
        ESP_LOGI(TAG, "No LED available on this board (GPIO=%d)", led_gpio);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* If a GPIO is configured, we could initialize it here */
    ESP_LOGI(TAG, "LED controller initialized (GPIO=%d)", led_gpio);
    return ESP_OK;
}

void led_set_status(led_status_t status)
{
    int led_gpio = zb_hal_get_led_gpio();

    if (led_gpio < 0) {
        /* No LED available - silently ignore */
        return;
    }

    /* If LED is available, set status here */
    const char *status_names[] = {"OFF", "BOOT", "NORMAL", "PAIRING", "ERROR"};
    ESP_LOGD(TAG, "LED status: %s", status_names[status]);
}

void led_controller_deinit(void)
{
    /* Nothing to clean up */
}
