/**
 * @file led_controller.c
 * @brief LED Controller for onboard WS2812 RGB LED
 *
 * Supports:
 * - On/Off control
 * - Brightness adjustment
 * - RGB color setting
 * - LED effects (breathing, blink, rainbow, candle, strobe)
 * - Status indicators (boot, wifi, error, OTA)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "led_controller.h"
#include "led_effects.h"
#include "sdkconfig.h"
#include <string.h>

#if CONFIG_GW_LED_ENABLED

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

/* LED Strip v3.0 API */
#include "led_strip.h"
#if CONFIG_GW_LED_BACKEND_RMT
#include "led_strip_rmt.h"
#elif CONFIG_GW_LED_BACKEND_SPI
#include "led_strip_spi.h"
#endif

static const char *TAG = "LED_CTRL";

/* ============================================================================
 * Static Data
 * ============================================================================ */

static led_strip_handle_t s_led_strip = NULL;
static esp_timer_handle_t s_effect_timer = NULL;

/* Current state */
static bool s_on = false;
static uint8_t s_brightness = 32;
static uint8_t s_red = 0;
static uint8_t s_green = 0;
static uint8_t s_blue = 255;

/* Effect state */
static led_effect_t s_current_effect = LED_EFFECT_NONE;
static uint16_t s_effect_speed = 0;
static uint32_t s_effect_step = 0;

/* Notification blink state */
static esp_timer_handle_t s_notify_timer = NULL;
static led_state_t s_saved_state = {0};
static uint8_t s_notify_blink_count = 0;
static uint8_t s_notify_blink_remaining = 0;
static uint8_t s_notify_r = 0, s_notify_g = 0, s_notify_b = 0;
static bool s_notify_phase = false;  /* true = LED on, false = LED off */

/* Status flash state (for temporary status like WIFI_CONNECTED) */
static esp_timer_handle_t s_status_flash_timer = NULL;
static led_status_t s_previous_status = LED_STATUS_BOOT;
static led_status_t s_current_status = LED_STATUS_BOOT;

/* Effect array - MUST have LED_EFFECT_MAX elements! */
const led_effect_def_t led_effects[] = {
    { "None",       LED_EFFECT_NONE,       0                               },
    { "Breathing",  LED_EFFECT_BREATHING,  LED_EFFECT_FADE_DURATION_MS     },
    { "Blink",      LED_EFFECT_BLINK,      LED_EFFECT_BLINK_DURATION_MS    },
    { "Fast Blink", LED_EFFECT_BLINK_FAST, LED_EFFECT_FAST_BLINK_DURATION_MS },
    { "Rainbow",    LED_EFFECT_RAINBOW,    LED_EFFECT_BREATHE_DURATION_MS  },
    { "Candle",     LED_EFFECT_CANDLE,     LED_EFFECT_CANDLE_PERIOD_MS     },
    { "Strobe",     LED_EFFECT_STROBE,     LED_EFFECT_STROBE_OFF_MS        },
};

/**
 * @brief Sine lookup table for breathing effect (64 entries, quarter wave)
 *
 * Values represent sin(x) * 255 for x = 0 to pi/2 (first quadrant).
 * Full wave is reconstructed by mirroring: 0-63 rising, 64-127 falling.
 * Using integer math saves CPU vs calling sinf() in timer callback.
 */
static const uint8_t sine_lut[64] = {
      0,   6,  13,  19,  25,  31,  37,  44,
     50,  56,  62,  68,  74,  80,  86,  92,
     98, 103, 109, 115, 120, 126, 131, 136,
    142, 147, 152, 157, 162, 167, 171, 176,
    181, 185, 189, 193, 197, 201, 205, 209,
    212, 216, 219, 222, 225, 228, 231, 234,
    236, 238, 241, 243, 245, 247, 248, 250,
    251, 252, 253, 254, 255, 255, 255, 255
};

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Convert HSV to RGB
 */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0) {
        *r = *g = *b = v;
        return;
    }

    uint8_t region = h / HSV_HUE_REGION_DIVISOR;
    uint8_t remainder = (h - (region * HSV_HUE_REGION_DIVISOR)) * LED_BRIGHTNESS_MAX / HSV_HUE_REGION_DIVISOR;

    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - ((s * remainder) / 255))) / 255;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;

    switch (region) {
        case 0:  *r = v; *g = t; *b = p; break;
        case 1:  *r = q; *g = v; *b = p; break;
        case 2:  *r = p; *g = v; *b = t; break;
        case 3:  *r = p; *g = q; *b = v; break;
        case 4:  *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

/**
 * @brief Set LED pixel directly (bypassing state)
 */
static void set_pixel_direct(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_strip == NULL) return;
    led_strip_set_pixel(s_led_strip, 0, r, g, b);
    led_strip_refresh(s_led_strip);
}

/**
 * @brief Update LED from current state (no effect)
 */
static void update_led(void)
{
    if (s_led_strip == NULL) {
        ESP_LOGW(TAG, "LED strip not initialized!");
        return;
    }

    if (s_on && s_current_effect == LED_EFFECT_NONE) {
        uint8_t r = (s_red * s_brightness) / 255;
        uint8_t g = (s_green * s_brightness) / 255;
        uint8_t b = (s_blue * s_brightness) / 255;
        ESP_LOGD(TAG, "Setting LED: R=%d G=%d B=%d", r, g, b);
        led_strip_set_pixel(s_led_strip, 0, r, g, b);
    } else if (!s_on) {
        led_strip_set_pixel(s_led_strip, 0, 0, 0, 0);
    }
    /* If effect is active, don't touch - timer handles it */

    if (s_current_effect == LED_EFFECT_NONE) {
        led_strip_refresh(s_led_strip);
    }
}

/* ============================================================================
 * Effect Implementations
 * ============================================================================ */

/**
 * @brief Get sine value from LUT (0-255 over full cycle)
 * @param phase 0-127 representing 0-360 degrees
 * @return sine value 0-255
 */
static uint8_t get_sine_value(uint8_t phase)
{
    if (phase < LED_PHASE_QUADRANT_1) {
        /* First quadrant: 0 to peak, use LUT[0..63] */
        return sine_lut[phase * 2];
    } else if (phase < LED_PHASE_QUADRANT_2) {
        /* Second quadrant: peak to 0, mirror LUT */
        return sine_lut[63 - ((phase - LED_PHASE_QUADRANT_1) * 2)];
    } else {
        /* Third/Fourth quadrant would be negative for sine, but for
         * breathing we want 0-255-0, so we return 0 for second half */
        return 0;
    }
}

static void effect_breathing(void)
{
    uint32_t total_steps = s_effect_speed / LED_EFFECT_TIMER_INTERVAL_MS;
    if (total_steps == 0) total_steps = LED_BREATHING_DEFAULT_STEPS;

    uint8_t phase = (uint8_t)((s_effect_step * 128) / total_steps);
    uint8_t factor = get_sine_value(phase);

    uint8_t r = (uint8_t)(((uint32_t)s_red * s_brightness * factor) / (255 * 255));
    uint8_t g = (uint8_t)(((uint32_t)s_green * s_brightness * factor) / (255 * 255));
    uint8_t b = (uint8_t)(((uint32_t)s_blue * s_brightness * factor) / (255 * 255));

    set_pixel_direct(r, g, b);

    s_effect_step++;
    if (s_effect_step >= total_steps) {
        s_effect_step = 0;
    }
}

static void effect_blink(void)
{
    uint32_t half_period = s_effect_speed / (LED_EFFECT_TIMER_INTERVAL_MS * 2);
    if (half_period == 0) half_period = LED_BLINK_DEFAULT_HALF_PERIOD;

    bool on_phase = (s_effect_step < half_period);

    if (on_phase) {
        uint8_t r = (s_red * s_brightness) / 255;
        uint8_t g = (s_green * s_brightness) / 255;
        uint8_t b = (s_blue * s_brightness) / 255;
        set_pixel_direct(r, g, b);
    } else {
        set_pixel_direct(0, 0, 0);
    }

    s_effect_step++;
    if (s_effect_step >= half_period * 2) {
        s_effect_step = 0;
    }
}

static void effect_rainbow(void)
{
    uint32_t total_steps = s_effect_speed / LED_EFFECT_TIMER_INTERVAL_MS;
    if (total_steps == 0) total_steps = LED_RAINBOW_DEFAULT_STEPS;

    uint16_t hue = (s_effect_step * HSV_HUE_FULL_CIRCLE) / total_steps;
    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, s_brightness, &r, &g, &b);

    set_pixel_direct(r, g, b);

    s_effect_step++;
    if (s_effect_step >= total_steps) {
        s_effect_step = 0;
    }
}

static void effect_candle(void)
{
    uint8_t flicker = (esp_random() % LED_CANDLE_BRIGHTNESS_VARIATION) + LED_CANDLE_BASE_BRIGHTNESS;
    uint8_t red_flicker = (esp_random() % LED_CANDLE_RED_VARIATION);

    uint8_t r = ((LED_BRIGHTNESS_MAX - red_flicker) * flicker * s_brightness) / (LED_BRIGHTNESS_MAX * LED_BRIGHTNESS_MAX);
    uint8_t g = ((LED_CANDLE_GREEN_BASE + (esp_random() % LED_CANDLE_GREEN_VARIATION)) * flicker * s_brightness) / (LED_BRIGHTNESS_MAX * LED_BRIGHTNESS_MAX);
    uint8_t b = (LED_CANDLE_BLUE_BASE * flicker * s_brightness) / (LED_BRIGHTNESS_MAX * LED_BRIGHTNESS_MAX);

    set_pixel_direct(r, g, b);
}

static void effect_strobe(void)
{
    s_effect_step = !s_effect_step;

    if (s_effect_step) {
        uint8_t r = (s_red * s_brightness) / 255;
        uint8_t g = (s_green * s_brightness) / 255;
        uint8_t b = (s_blue * s_brightness) / 255;
        set_pixel_direct(r, g, b);
    } else {
        set_pixel_direct(0, 0, 0);
    }
}

/* ============================================================================
 * Effect Timer
 * ============================================================================ */

static void effect_timer_callback(void *arg)
{
    (void)arg;

    if (!s_on || s_current_effect == LED_EFFECT_NONE) {
        return;
    }

    switch (s_current_effect) {
        case LED_EFFECT_BREATHING:
            effect_breathing();
            break;
        case LED_EFFECT_BLINK:
        case LED_EFFECT_BLINK_FAST:
            effect_blink();
            break;
        case LED_EFFECT_RAINBOW:
            effect_rainbow();
            break;
        case LED_EFFECT_CANDLE:
            effect_candle();
            break;
        case LED_EFFECT_STROBE:
            effect_strobe();
            break;
        default:
            break;
    }
}

static void start_effect_timer(void)
{
    if (s_effect_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = effect_timer_callback,
            .name = "led_effect"
        };
        esp_timer_create(&timer_args, &s_effect_timer);
    }

    uint32_t period_us = LED_EFFECT_DEFAULT_PERIOD_US;

    switch (s_current_effect) {
        case LED_EFFECT_STROBE:
            period_us = s_effect_speed * 1000;
            if (period_us < LED_STROBE_MIN_PERIOD_US) period_us = LED_STROBE_MIN_PERIOD_US;
            break;
        case LED_EFFECT_CANDLE:
            period_us = LED_EFFECT_CANDLE_BASE_PERIOD_US + (esp_random() % LED_EFFECT_CANDLE_RANDOM_MAX_US);
            break;
        default:
            period_us = LED_EFFECT_DEFAULT_PERIOD_US;
            break;
    }

    esp_timer_stop(s_effect_timer);
    esp_timer_start_periodic(s_effect_timer, period_us);
    ESP_LOGI(TAG, "Effect timer started: %lu us", (unsigned long)period_us);
}

static void stop_effect_timer(void)
{
    if (s_effect_timer != NULL) {
        esp_timer_stop(s_effect_timer);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t led_controller_init(void)
{
#if CONFIG_GW_LED_BACKEND_RMT
    ESP_LOGI(TAG, "Initializing LED on GPIO%d (RMT backend)", CONFIG_GW_LED_GPIO);
#elif CONFIG_GW_LED_BACKEND_SPI
    ESP_LOGI(TAG, "Initializing LED on GPIO%d (SPI backend, host %d)",
             CONFIG_GW_LED_GPIO, CONFIG_GW_LED_SPI_HOST);
#else
    ESP_LOGI(TAG, "Initializing LED on GPIO%d", CONFIG_GW_LED_GPIO);
#endif

    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_GW_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags.invert_out = false,
    };

    esp_err_t ret;

#if CONFIG_GW_LED_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = CONFIG_GW_LED_SPI_HOST,
        .flags.with_dma = true,
    };
    ret = led_strip_new_spi_device(&strip_config, &spi_config, &s_led_strip);
#else
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_PWM_FREQUENCY_HZ,
        .flags.with_dma = false,
    };
    ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
#endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set initial state */
    s_on = true;
    s_red = 0;
    s_green = 0;
    s_blue = 255;
    s_brightness = CONFIG_GW_LED_BRIGHTNESS;
    s_current_effect = LED_EFFECT_NONE;

    update_led();

    ESP_LOGI(TAG, "LED initialized - blue");
    return ESP_OK;
}

void led_controller_deinit(void)
{
    stop_effect_timer();
    if (s_effect_timer != NULL) {
        esp_timer_delete(s_effect_timer);
        s_effect_timer = NULL;
    }

    if (s_led_strip != NULL) {
        led_strip_clear(s_led_strip);
        led_strip_del(s_led_strip);
        s_led_strip = NULL;
    }
}

void led_set_on(bool on)
{
    s_on = on;
    if (on && s_current_effect != LED_EFFECT_NONE) {
        start_effect_timer();
    } else {
        stop_effect_timer();
    }
    update_led();
}

bool led_get_on(void)
{
    return s_on;
}

void led_set_brightness(uint8_t brightness)
{
    s_brightness = brightness;
    if (s_current_effect == LED_EFFECT_NONE) {
        update_led();
    }
}

uint8_t led_get_brightness(void)
{
    return s_brightness;
}

void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    s_red = r;
    s_green = g;
    s_blue = b;
    if (s_current_effect == LED_EFFECT_NONE) {
        update_led();
    }
}

void led_get_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r) *r = s_red;
    if (g) *g = s_green;
    if (b) *b = s_blue;
}

void led_set_effect(led_effect_t effect, uint16_t speed)
{
    ESP_LOGI(TAG, "Setting effect: %d, speed: %d", effect, speed);

    s_current_effect = effect;
    s_effect_speed = speed;
    s_effect_step = 0;

    if (effect == LED_EFFECT_NONE) {
        stop_effect_timer();
        update_led();
    } else if (s_on) {
        start_effect_timer();
    }
}

esp_err_t led_set_effect_by_name(const char *name)
{
    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Setting effect by name: %s", name);

    for (int i = 0; i < LED_EFFECT_MAX; i++) {
        if (strcasecmp(name, led_effects[i].name) == 0) {
            led_set_effect(led_effects[i].effect, led_effects[i].default_speed);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Unknown effect: %s", name);
    return ESP_ERR_NOT_FOUND;
}

led_effect_t led_get_effect(void)
{
    return s_current_effect;
}

const char *led_get_effect_name(void)
{
    if (s_current_effect < LED_EFFECT_MAX) {
        return led_effects[s_current_effect].name;
    }
    return "None";
}

void led_get_state(led_state_t *state)
{
    if (state == NULL) return;
    state->on = s_on;
    state->brightness = s_brightness;
    state->red = s_red;
    state->green = s_green;
    state->blue = s_blue;
    state->effect = s_current_effect;
    state->effect_speed = s_effect_speed;
}

void led_set_state(const led_state_t *state)
{
    if (state == NULL) return;
    s_on = state->on;
    s_brightness = state->brightness;
    s_red = state->red;
    s_green = state->green;
    s_blue = state->blue;
    led_set_effect(state->effect, state->effect_speed);
}

/* ============================================================================
 * Status Indicator
 * ============================================================================ */

static led_status_t s_flash_target_status = LED_STATUS_NORMAL;

static void status_flash_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Status flash done, transitioning to status %d", s_flash_target_status);
    led_set_status(s_flash_target_status);
}

void led_set_status(led_status_t status)
{
    ESP_LOGI(TAG, "led_set_status called with status=%d", status);

    /* Stop any status flash timer */
    if (s_status_flash_timer != NULL) {
        esp_timer_stop(s_status_flash_timer);
    }

    /* Stop any running effect */
    led_set_effect(LED_EFFECT_NONE, 0);

    /* Track status changes (but not for temporary flashes) */
    bool is_temporary = (status == LED_STATUS_WIFI_CONNECTED);
    if (!is_temporary) {
        s_previous_status = s_current_status;
        s_current_status = status;
    }

    switch (status) {
        case LED_STATUS_BOOT:
            s_red = 0; s_green = 0; s_blue = 255;  /* Blue */
            led_set_effect(LED_EFFECT_BREATHING, LED_STATUS_BOOT_BREATHING_MS);
            break;
        case LED_STATUS_WIFI_CONNECTING:
            s_red = 255; s_green = 165; s_blue = 0;  /* Orange */
            led_set_effect(LED_EFFECT_BLINK, LED_STATUS_WIFI_BREATHING_MS / 4);
            break;
        case LED_STATUS_WIFI_CONNECTED:
            /* Temporary green flash, then transition to NORMAL */
            s_red = 0; s_green = 255; s_blue = 0;  /* Green */
            s_on = true;
            update_led();

            if (s_status_flash_timer == NULL) {
                esp_timer_create_args_t timer_args = {
                    .callback = status_flash_timer_cb,
                    .name = "led_status_flash"
                };
                esp_timer_create(&timer_args, &s_status_flash_timer);
            }
            s_flash_target_status = LED_STATUS_NORMAL;
            esp_timer_start_once(s_status_flash_timer, LED_STATUS_FLASH_DURATION_US);
            return;
        case LED_STATUS_NORMAL:
            s_red = 0; s_green = 255; s_blue = 0;  /* Green */
            break;
        case LED_STATUS_PAIRING:
            s_red = 0; s_green = 255; s_blue = 255;  /* Cyan */
            led_set_effect(LED_EFFECT_BREATHING, LED_STATUS_PAIRING_BREATHING_MS);
            break;
        case LED_STATUS_WARNING:
            s_red = 255; s_green = 165; s_blue = 0;  /* Orange */
            led_set_effect(LED_EFFECT_BLINK, LED_STATUS_WARNING_BLINK_MS);
            break;
        case LED_STATUS_ERROR:
            s_red = 255; s_green = 0; s_blue = 0;  /* Red */
            led_set_effect(LED_EFFECT_BLINK_FAST, LED_STATUS_ERROR_BLINK_FAST_MS);
            break;
        case LED_STATUS_OTA:
            s_red = 128; s_green = 0; s_blue = 255;  /* Purple */
            led_set_effect(LED_EFFECT_BREATHING, LED_STATUS_ZIGBEE_BREATHING_MS);
            break;
        default:
            s_red = 0; s_green = 0; s_blue = 255;  /* Blue default */
            break;
    }
    s_on = true;
    if (s_current_effect == LED_EFFECT_NONE) {
        update_led();
    }
}

led_status_t led_get_status(void)
{
    return s_current_status;
}

bool led_is_manual_mode(void)
{
    return false;
}

void led_release_manual(void)
{
}

/* ============================================================================
 * Notification Blink
 * ============================================================================ */

static void notify_timer_callback(void *arg)
{
    (void)arg;

    if (s_notify_phase) {
        set_pixel_direct(0, 0, 0);
        s_notify_phase = false;

        if (s_notify_blink_remaining > 0) {
            s_notify_blink_remaining--;
        }

        if (s_notify_blink_remaining == 0) {
            esp_timer_stop(s_notify_timer);
            led_set_state(&s_saved_state);
            ESP_LOGD(TAG, "Notification blink done, state restored");
        }
    } else {
        uint8_t r = (s_notify_r * s_brightness) / 255;
        uint8_t g = (s_notify_g * s_brightness) / 255;
        uint8_t b = (s_notify_b * s_brightness) / 255;
        set_pixel_direct(r, g, b);
        s_notify_phase = true;
    }
}

void led_blink_notify(led_notify_t notify)
{
    switch (notify) {
        case LED_NOTIFY_DEVICE_NEW:
            s_notify_r = 0;
            s_notify_g = 0;
            s_notify_b = 255;
            s_notify_blink_count = 1;
            ESP_LOGI(TAG, "Notification: New device (1x blue)");
            break;
        case LED_NOTIFY_DEVICE_KNOWN:
            s_notify_r = 0;
            s_notify_g = 0;
            s_notify_b = 255;
            s_notify_blink_count = 2;
            ESP_LOGI(TAG, "Notification: Known device (2x blue)");
            break;
        case LED_NOTIFY_DEVICE_FAILED:
            s_notify_r = 255;
            s_notify_g = 0;
            s_notify_b = 0;
            s_notify_blink_count = 1;
            ESP_LOGI(TAG, "Notification: Device failed (1x red)");
            break;
        default:
            return;
    }

    led_get_state(&s_saved_state);
    stop_effect_timer();

    if (s_notify_timer == NULL) {
        esp_timer_create_args_t timer_args = {
            .callback = notify_timer_callback,
            .name = "led_notify"
        };
        if (esp_timer_create(&timer_args, &s_notify_timer) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create notify timer");
            return;
        }
    }

    s_notify_blink_remaining = s_notify_blink_count;
    s_notify_phase = false;

    uint8_t r = (s_notify_r * s_brightness) / 255;
    uint8_t g = (s_notify_g * s_brightness) / 255;
    uint8_t b = (s_notify_b * s_brightness) / 255;
    set_pixel_direct(r, g, b);
    s_notify_phase = true;

    esp_timer_stop(s_notify_timer);
    esp_timer_start_periodic(s_notify_timer, LED_NOTIFY_BLINK_PERIOD_US);
}

/* ============================================================================
 * Self-Test
 * ============================================================================ */

esp_err_t led_controller_test(void)
{
    ESP_LOGI(TAG, "Running LED controller self-test...");

    if (s_led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip not initialized");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LED controller self-test PASSED");
    return ESP_OK;
}

#else /* !CONFIG_GW_LED_ENABLED */

#include "led_effects.h"

/* Stub implementations when LED is disabled */
const led_effect_def_t led_effects[] = {
    { "None",       LED_EFFECT_NONE,       0                               },
    { "Breathing",  LED_EFFECT_BREATHING,  LED_EFFECT_FADE_DURATION_MS     },
    { "Blink",      LED_EFFECT_BLINK,      LED_EFFECT_BLINK_DURATION_MS    },
    { "Fast Blink", LED_EFFECT_BLINK_FAST, LED_EFFECT_FAST_BLINK_DURATION_MS },
    { "Rainbow",    LED_EFFECT_RAINBOW,    LED_EFFECT_BREATHE_DURATION_MS  },
    { "Candle",     LED_EFFECT_CANDLE,     LED_EFFECT_CANDLE_PERIOD_MS     },
    { "Strobe",     LED_EFFECT_STROBE,     LED_EFFECT_STROBE_OFF_MS        },
};

esp_err_t led_controller_init(void) { return ESP_OK; }
void led_controller_deinit(void) {}
void led_set_on(bool on) { (void)on; }
bool led_get_on(void) { return false; }
void led_set_brightness(uint8_t brightness) { (void)brightness; }
uint8_t led_get_brightness(void) { return 0; }
void led_set_color(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
void led_get_color(uint8_t *r, uint8_t *g, uint8_t *b) { if(r)*r=0; if(g)*g=0; if(b)*b=0; }
void led_set_effect(led_effect_t effect, uint16_t speed) { (void)effect; (void)speed; }
esp_err_t led_set_effect_by_name(const char *name) { (void)name; return ESP_ERR_NOT_SUPPORTED; }
led_effect_t led_get_effect(void) { return LED_EFFECT_NONE; }
const char *led_get_effect_name(void) { return "None"; }
void led_get_state(led_state_t *state) { if(state) memset(state, 0, sizeof(*state)); }
void led_set_state(const led_state_t *state) { (void)state; }
void led_set_status(led_status_t status) { (void)status; }
led_status_t led_get_status(void) { return LED_STATUS_NORMAL; }
bool led_is_manual_mode(void) { return false; }
void led_release_manual(void) {}
void led_blink_notify(led_notify_t notify) { (void)notify; }
esp_err_t led_controller_test(void) { return ESP_OK; }

#endif /* CONFIG_GW_LED_ENABLED */
