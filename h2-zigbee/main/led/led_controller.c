/**
 * @file led_controller.c
 * @brief LED Controller for ESP32-H2-Zero with WS2812 RGB LED
 *
 * The Waveshare ESP32-H2-Zero has a WS2812 RGB LED on GPIO8.
 * This implementation provides status indication and direct color control.
 */

#include "led_controller.h"
#include "zb_hal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LED_H2";

/* LED strip handle */
static led_strip_handle_t s_led_strip = NULL;

/* Current state */
static led_status_t s_current_status = LED_STATUS_BOOT;
static bool s_is_on = true;
static uint8_t s_red = 0, s_green = 0, s_blue = 32;  /* Default: blue */

/* Blink timer (for notifications) */
static esp_timer_handle_t s_blink_timer = NULL;
static uint8_t s_saved_r, s_saved_g, s_saved_b;
static bool s_saved_on;

/* Pairing blink timer (continuous blink while pairing) */
static esp_timer_handle_t s_pairing_timer = NULL;
static bool s_pairing_blink_state = true;

/* Status colors (R, G, B) - order matches led_status_t enum */
static const uint8_t STATUS_COLORS[][3] = {
    [LED_STATUS_BOOT]            = {0,   0,   64},    /* Blue - booting */
    [LED_STATUS_WIFI_CONNECTING] = {64,  48,  0},     /* Yellow - WiFi connecting */
    [LED_STATUS_WIFI_CONNECTED]  = {0,   64,  0},     /* Green - WiFi connected */
    [LED_STATUS_NORMAL]          = {0,   64,  0},     /* Green - normal operation */
    [LED_STATUS_PAIRING]         = {64,  32,  0},     /* Orange - pairing mode */
    [LED_STATUS_WARNING]         = {64,  32,  0},     /* Orange - warning */
    [LED_STATUS_ERROR]           = {64,  0,   0},     /* Red - error */
    [LED_STATUS_OTA]             = {32,  0,   64},    /* Purple - OTA */
    [LED_STATUS_OFF]             = {0,   0,   0},     /* Off */
};

static const char *STATUS_NAMES[] = {
    "BOOT", "WIFI_CONNECTING", "WIFI_CONNECTED", "NORMAL",
    "PAIRING", "WARNING", "ERROR", "OTA", "OFF"
};

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

static void update_led_hw(void)
{
    if (s_led_strip == NULL) {
        return;
    }

    if (s_is_on) {
        led_strip_set_pixel(s_led_strip, 0, s_red, s_green, s_blue);
    } else {
        led_strip_set_pixel(s_led_strip, 0, 0, 0, 0);
    }
    led_strip_refresh(s_led_strip);
}

static void blink_timer_callback(void *arg)
{
    /* Restore saved state after blink */
    s_red = s_saved_r;
    s_green = s_saved_g;
    s_blue = s_saved_b;
    s_is_on = s_saved_on;
    update_led_hw();
    ESP_LOGD(TAG, "Blink complete, restored state");
}

static void pairing_timer_callback(void *arg)
{
    /* Toggle LED on/off for pairing blink effect */
    s_pairing_blink_state = !s_pairing_blink_state;

    if (s_pairing_blink_state) {
        /* On: show pairing color (orange) */
        led_strip_set_pixel(s_led_strip, 0,
                           STATUS_COLORS[LED_STATUS_PAIRING][0],
                           STATUS_COLORS[LED_STATUS_PAIRING][1],
                           STATUS_COLORS[LED_STATUS_PAIRING][2]);
    } else {
        /* Off */
        led_strip_set_pixel(s_led_strip, 0, 0, 0, 0);
    }
    led_strip_refresh(s_led_strip);
}

static void start_pairing_blink(void)
{
    if (s_pairing_timer == NULL) return;

    s_pairing_blink_state = true;
    esp_timer_stop(s_pairing_timer);
    /* Blink every 500ms (1 Hz) */
    esp_timer_start_periodic(s_pairing_timer, 500 * 1000);
    ESP_LOGD(TAG, "Pairing blink started");
}

static void stop_pairing_blink(void)
{
    if (s_pairing_timer == NULL) return;

    esp_timer_stop(s_pairing_timer);
    ESP_LOGD(TAG, "Pairing blink stopped");
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t led_controller_init(void)
{
    int led_gpio = zb_hal_get_led_gpio();

    if (led_gpio < 0) {
        ESP_LOGI(TAG, "No LED configured (GPIO=%d)", led_gpio);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Initializing WS2812 LED on GPIO%d", led_gpio);

    /* Configure LED strip */
    led_strip_config_t strip_config = {
        .strip_gpio_num = led_gpio,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags.invert_out = false,
    };

    /* Use RMT backend */
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,  /* 10MHz */
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create blink timer (for notifications) */
    const esp_timer_create_args_t timer_args = {
        .callback = blink_timer_callback,
        .name = "led_blink"
    };
    esp_timer_create(&timer_args, &s_blink_timer);

    /* Create pairing blink timer */
    const esp_timer_create_args_t pairing_timer_args = {
        .callback = pairing_timer_callback,
        .name = "led_pairing"
    };
    esp_timer_create(&pairing_timer_args, &s_pairing_timer);

    /* Set initial boot color (blue) */
    led_set_status(LED_STATUS_BOOT);

    ESP_LOGI(TAG, "LED controller initialized successfully");
    return ESP_OK;
}

void led_controller_deinit(void)
{
    if (s_pairing_timer != NULL) {
        esp_timer_stop(s_pairing_timer);
        esp_timer_delete(s_pairing_timer);
        s_pairing_timer = NULL;
    }

    if (s_blink_timer != NULL) {
        esp_timer_stop(s_blink_timer);
        esp_timer_delete(s_blink_timer);
        s_blink_timer = NULL;
    }

    if (s_led_strip != NULL) {
        led_strip_clear(s_led_strip);
        led_strip_del(s_led_strip);
        s_led_strip = NULL;
    }
    ESP_LOGI(TAG, "LED controller deinitialized");
}

void led_set_status(led_status_t status)
{
    if (s_led_strip == NULL) {
        return;
    }

    if (status >= LED_STATUS_MAX) {
        status = LED_STATUS_NORMAL;
    }

    /* Stop pairing blink if leaving pairing mode */
    if (s_current_status == LED_STATUS_PAIRING && status != LED_STATUS_PAIRING) {
        stop_pairing_blink();
    }

    s_current_status = status;
    s_red = STATUS_COLORS[status][0];
    s_green = STATUS_COLORS[status][1];
    s_blue = STATUS_COLORS[status][2];
    s_is_on = (status != LED_STATUS_OFF);

    /* Start pairing blink if entering pairing mode */
    if (status == LED_STATUS_PAIRING) {
        start_pairing_blink();
        return;  /* Timer handles LED updates */
    }

    update_led_hw();

    ESP_LOGI(TAG, "Status: %s (R=%d G=%d B=%d)", STATUS_NAMES[status], s_red, s_green, s_blue);
}

led_status_t led_get_status(void)
{
    return s_current_status;
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_strip == NULL) {
        return;
    }

    s_red = r;
    s_green = g;
    s_blue = b;
    s_current_status = LED_STATUS_OFF;  /* Custom color = no status */

    update_led_hw();

    ESP_LOGD(TAG, "Color set: R=%d G=%d B=%d", r, g, b);
}

void led_set_on(bool on)
{
    if (s_led_strip == NULL) {
        return;
    }

    s_is_on = on;
    update_led_hw();

    ESP_LOGD(TAG, "LED %s", on ? "ON" : "OFF");
}

bool led_is_on(void)
{
    return s_is_on;
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    if (s_led_strip == NULL || s_blink_timer == NULL) {
        return;
    }

    /* Save current state */
    s_saved_r = s_red;
    s_saved_g = s_green;
    s_saved_b = s_blue;
    s_saved_on = s_is_on;

    /* Set blink color */
    s_red = r;
    s_green = g;
    s_blue = b;
    s_is_on = true;
    update_led_hw();

    /* Start timer to restore state */
    esp_timer_stop(s_blink_timer);
    esp_timer_start_once(s_blink_timer, duration_ms * 1000);

    ESP_LOGD(TAG, "Blink: R=%d G=%d B=%d for %lums", r, g, b, (unsigned long)duration_ms);
}

void led_blink_notify(led_notify_t notify)
{
    switch (notify) {
        case LED_NOTIFY_DEVICE_NEW:
            /* New device - 1x cyan blink */
            led_blink(0, 32, 64, 300);
            break;
        case LED_NOTIFY_DEVICE_KNOWN:
            /* Known device - 1x green blink */
            led_blink(0, 64, 0, 200);
            break;
        case LED_NOTIFY_DEVICE_FAILED:
            /* Failed - 1x red blink */
            led_blink(64, 0, 0, 500);
            break;
        default:
            break;
    }
}
