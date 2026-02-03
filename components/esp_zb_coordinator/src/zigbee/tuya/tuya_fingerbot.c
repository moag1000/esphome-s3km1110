/**
 * @file tuya_fingerbot.c
 * @brief Tuya Fingerbot Device Driver Implementation
 *
 * Implements the Tuya device driver interface for Fingerbot devices.
 * Handles DP processing, MQTT commands (both Tuya DPs and ZCL),
 * state JSON building, and HA discovery.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "tuya_driver_registry.h"
#include "tuya_device_driver.h"
#include "zb_tuya.h"
#include "zb_device_handler.h"
#include "zb_coordinator.h"
#include "compat_stubs.h"
#include "command_handler.h"
#include "ha_constants.h"
#include "gateway_defaults.h"
#include "json_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "TUYA_FINGERBOT";

/* ============================================================================
 * State Management
 * ============================================================================ */

/** @brief Fingerbot state array */
static tuya_fingerbot_state_t s_fingerbot_states[ZB_TUYA_MAX_DEVICES];

/** @brief Mutex for thread-safe state access */
static SemaphoreHandle_t s_mutex = NULL;

/**
 * @brief Find or create Fingerbot state entry
 */
static tuya_fingerbot_state_t *find_or_create_state(uint16_t short_addr)
{
    /* First, try to find existing entry */
    for (int i = 0; i < ZB_TUYA_MAX_DEVICES; i++) {
        if (s_fingerbot_states[i].valid && s_fingerbot_states[i].short_addr == short_addr) {
            return &s_fingerbot_states[i];
        }
    }

    /* Find empty slot */
    for (int i = 0; i < ZB_TUYA_MAX_DEVICES; i++) {
        if (!s_fingerbot_states[i].valid) {
            memset(&s_fingerbot_states[i], 0, sizeof(tuya_fingerbot_state_t));
            s_fingerbot_states[i].short_addr = short_addr;
            s_fingerbot_states[i].valid = true;
            s_fingerbot_states[i].sustain_time = 3;      /* Default 300ms */
            s_fingerbot_states[i].down_movement = 100;    /* Default 100% */
            return &s_fingerbot_states[i];
        }
    }

    ESP_LOGW(TAG, "No free Fingerbot state slot");
    return NULL;
}

/* ============================================================================
 * Driver Interface: match
 * ============================================================================ */

static bool fingerbot_match(const char *manufacturer, const char *model)
{
    if (manufacturer == NULL) {
        return false;
    }

    /* Known Fingerbot manufacturer IDs (from zigbee-herdsman-converters) */
    static const char *const known_ids[] = {
        "_TZ3210_dse8ogfy",   /* Fingerbot original */
        "_TZ3210_j4pdtz9v",   /* Fingerbot Plus */
    };

    for (size_t i = 0; i < sizeof(known_ids) / sizeof(known_ids[0]); i++) {
        if (strcmp(manufacturer, known_ids[i]) == 0) {
            return true;
        }
    }

    /* Fallback: model string match for unlisted variants */
    if (model != NULL) {
        if (strstr(model, "fingerbot") != NULL ||
            strstr(model, "Fingerbot") != NULL ||
            strstr(model, "ZBFB") != NULL) {
            return true;
        }
    }

    return false;
}

/* ============================================================================
 * Driver Interface: process_dp
 * ============================================================================ */

static esp_err_t fingerbot_process_dp(uint16_t short_addr, const tuya_dp_t *dp)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    tuya_fingerbot_state_t *state = find_or_create_state(short_addr);
    if (state == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    state->last_update = esp_timer_get_time();

    switch (dp->dp_id) {
        case TUYA_DP_FINGERBOT_SWITCH:
            state->state = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X switch: %s",
                     short_addr, state->state ? "ON" : "OFF");
            break;

        case TUYA_DP_FINGERBOT_MODE:
            state->mode = (fingerbot_mode_t)dp->value.enum_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X mode: %s",
                     short_addr, zb_tuya_fingerbot_mode_to_string(state->mode));
            break;

        case TUYA_DP_FINGERBOT_DOWN_MOVEMENT:
            state->down_movement = (uint8_t)dp->value.int_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X down_movement: %d%%",
                     short_addr, state->down_movement);
            break;

        case TUYA_DP_FINGERBOT_SUSTAIN_TIME:
            state->sustain_time = (uint8_t)dp->value.int_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X sustain_time: %d (x100ms)",
                     short_addr, state->sustain_time);
            break;

        case TUYA_DP_FINGERBOT_REVERSE:
            state->reverse = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X reverse: %s",
                     short_addr, state->reverse ? "true" : "false");
            break;

        case TUYA_DP_FINGERBOT_BATTERY:
            state->battery = (uint8_t)dp->value.int_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X battery: %d%%",
                     short_addr, state->battery);
            /* Update ZCL power info so battery persists in NVS */
            if (state->battery > 0) {
                zb_device_t *dev = zb_device_get(short_addr);
                if (dev != NULL) {
                    dev->power_info.current_power_source =
                        ZB_POWER_SOURCE_RECHARGEABLE_BATTERY;
                    if (state->battery <= 5) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_CRITICAL;
                    } else if (state->battery <= 33) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_33_PERCENT;
                    } else if (state->battery <= 66) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_66_PERCENT;
                    } else {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_100_PERCENT;
                    }
                    dev->power_info.power_info_valid = true;
                }
            }
            break;

        case TUYA_DP_FINGERBOT_UP_MOVEMENT:
            state->up_movement = (uint8_t)dp->value.int_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X up_movement: %d%%",
                     short_addr, state->up_movement);
            break;

        case TUYA_DP_FINGERBOT_TOUCH_CONTROL:
            state->touch_control = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X touch_control: %s",
                     short_addr, state->touch_control ? "enabled" : "disabled");
            break;

        case TUYA_DP_FINGERBOT_CLICK_CONTROL:
            state->click_control = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X click_control: %s",
                     short_addr, state->click_control ? "enabled" : "disabled");
            break;

        case TUYA_DP_FINGERBOT_PROGRAM:
            if (dp->type == TUYA_DP_TYPE_RAW && dp->length > 0) {
                ESP_LOGI(TAG, "Fingerbot 0x%04X program raw (%u bytes):",
                         short_addr, dp->length);
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, dp->value.raw, dp->length, ESP_LOG_INFO);

                /* Parse binary → user-readable string "pos/delay;pos/delay;..."
                 * Format: [ff ff] [xx] [step_count] [pos:1][delay_hi:1][delay_lo:1] ... */
                state->program_text[0] = '\0';
                if (dp->length >= 4) {
                    uint8_t step_count = dp->value.raw[3];
                    size_t expected = 4 + (size_t)step_count * 3;
                    if (step_count > 0 && dp->length >= expected) {
                        char *p = state->program_text;
                        size_t remaining = sizeof(state->program_text);
                        for (uint8_t s = 0; s < step_count && remaining > 8; s++) {
                            size_t off = 4 + (size_t)s * 3;
                            uint8_t pos = dp->value.raw[off];
                            uint16_t delay = ((uint16_t)dp->value.raw[off + 1] << 8) |
                                              dp->value.raw[off + 2];
                            int written;
                            if (s > 0) {
                                written = snprintf(p, remaining, ";%u/%u", pos, delay);
                            } else {
                                written = snprintf(p, remaining, "%u/%u", pos, delay);
                            }
                            if (written > 0 && (size_t)written < remaining) {
                                p += written;
                                remaining -= (size_t)written;
                            }
                        }
                        ESP_LOGI(TAG, "Fingerbot 0x%04X program: \"%s\"",
                                 short_addr, state->program_text);
                    }
                }
            } else {
                ESP_LOGI(TAG, "Fingerbot 0x%04X program: type=%d value=%ld len=%u",
                         short_addr, dp->type, (long)dp->value.int_value, dp->length);
            }
            break;

        case TUYA_DP_FINGERBOT_COUNT:
            state->click_count = (uint32_t)dp->value.int_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X click_count: %lu",
                     short_addr, (unsigned long)state->click_count);
            break;

        case TUYA_DP_FINGERBOT_REPEAT_FOREVER:
            state->repeat_forever = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X repeat_forever: %s",
                     short_addr, state->repeat_forever ? "true" : "false");
            break;

        case TUYA_DP_FINGERBOT_PROGRAM_ENABLE:
            state->program_enable = dp->value.bool_value;
            ESP_LOGI(TAG, "Fingerbot 0x%04X program_enable: %s",
                     short_addr, state->program_enable ? "true" : "false");
            break;

        default:
            ESP_LOGI(TAG, "Fingerbot 0x%04X unknown DP %d (type=%d, value=%ld)",
                     short_addr, dp->dp_id, dp->type, (long)dp->value.int_value);
            break;
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

/* ============================================================================
 * Driver Interface: handle_command
 * ============================================================================ */

static esp_err_t fingerbot_handle_command(uint16_t short_addr, uint8_t endpoint,
                                           const cJSON *json)
{
    bool handled = false;
    esp_err_t ret = ESP_OK;

    /* Handle mode: {"mode": "push"|"switch"|"program"} */
    cJSON *mode_json = cJSON_GetObjectItem(json, "mode");
    if (cJSON_IsString(mode_json) && mode_json->valuestring != NULL) {
        fingerbot_mode_t mode;
        if (zb_tuya_fingerbot_mode_from_string(mode_json->valuestring, &mode) == ESP_OK) {
            ret = zb_tuya_fingerbot_set_mode(short_addr, endpoint, mode);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Set Fingerbot mode: 0x%04X -> %s",
                         short_addr, mode_json->valuestring);
                handled = true;
            }
        } else {
            ESP_LOGW(TAG, "Invalid Fingerbot mode: %s", mode_json->valuestring);
        }
    }

    /* Handle sustain_time: {"sustain_time": 0-255} */
    cJSON *sustain_json = cJSON_GetObjectItem(json, "sustain_time");
    if (cJSON_IsNumber(sustain_json)) {
        int sustain_time = (int)cJSON_GetNumberValue(sustain_json);
        if (sustain_time >= 0 && sustain_time <= 255) {
            ret = zb_tuya_fingerbot_set_sustain_time(short_addr, endpoint,
                                                      (uint8_t)sustain_time);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Set Fingerbot sustain_time: 0x%04X -> %d (%.1fs)",
                         short_addr, sustain_time, sustain_time * 0.1f);
                handled = true;
            }
        } else {
            ESP_LOGW(TAG, "Invalid sustain_time: %d (valid: 0-255)", sustain_time);
        }
    }

    /* Handle down_movement: {"down_movement": 0-100} */
    cJSON *movement_json = cJSON_GetObjectItem(json, "down_movement");
    if (cJSON_IsNumber(movement_json)) {
        int down_movement = (int)cJSON_GetNumberValue(movement_json);
        if (down_movement >= 0 && down_movement <= 100) {
            ret = zb_tuya_fingerbot_set_down_movement(short_addr, endpoint,
                                                       (uint8_t)down_movement);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Set Fingerbot down_movement: 0x%04X -> %d%%",
                         short_addr, down_movement);
                handled = true;
            }
        } else {
            ESP_LOGW(TAG, "Invalid down_movement: %d (valid: 0-100)", down_movement);
        }
    }

    /* Handle up_movement: {"up_movement": 0-50} */
    cJSON *up_movement_json = cJSON_GetObjectItem(json, "up_movement");
    if (cJSON_IsNumber(up_movement_json)) {
        int up_movement = (int)cJSON_GetNumberValue(up_movement_json);
        if (up_movement >= 0 && up_movement <= 50) {
            ret = zb_tuya_fingerbot_set_up_movement(short_addr, endpoint,
                                                     (uint8_t)up_movement);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Set Fingerbot up_movement: 0x%04X -> %d%%",
                         short_addr, up_movement);
                handled = true;
            }
        } else {
            ESP_LOGW(TAG, "Invalid up_movement: %d (valid: 0-50)", up_movement);
        }
    }

    /* Handle reverse: {"reverse": true|false} */
    cJSON *reverse_json = cJSON_GetObjectItem(json, "reverse");
    if (cJSON_IsBool(reverse_json)) {
        bool reverse = cJSON_IsTrue(reverse_json);
        ret = zb_tuya_fingerbot_set_reverse(short_addr, endpoint, reverse);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Set Fingerbot reverse: 0x%04X -> %s",
                     short_addr, reverse ? "true" : "false");
            handled = true;
        }
    }

    /* Handle touch_control: {"touch_control": true|false} */
    cJSON *touch_json = cJSON_GetObjectItem(json, "touch_control");
    if (cJSON_IsBool(touch_json)) {
        bool enable = cJSON_IsTrue(touch_json);
        ret = zb_tuya_fingerbot_set_touch_control(short_addr, endpoint, enable);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Set Fingerbot touch_control: 0x%04X -> %s",
                     short_addr, enable ? "true" : "false");
            handled = true;
        }
    }

    /* Handle program: {"program": "100/2;0/1;50/3"} — parse text to binary, send DP 109 */
    cJSON *program_json = cJSON_GetObjectItem(json, "program");
    if (cJSON_IsString(program_json) && program_json->valuestring != NULL &&
        strlen(program_json->valuestring) > 0) {
        const char *prog_str = program_json->valuestring;
        /* Parse "pos/delay;pos/delay;..." into binary [FF FF 00] [count] [pos delay_hi delay_lo]... */
        uint8_t prog_buf[64];
        prog_buf[0] = 0xFF;
        prog_buf[1] = 0xFF;
        prog_buf[2] = 0x00;
        uint8_t step_count = 0;
        size_t buf_off = 4;  /* Leave room for header + count */
        bool parse_ok = true;

        /* Work on a copy since strtok modifies the string */
        char prog_copy[128];
        strncpy(prog_copy, prog_str, sizeof(prog_copy) - 1);
        prog_copy[sizeof(prog_copy) - 1] = '\0';

        char *saveptr = NULL;
        char *token = strtok_r(prog_copy, ";", &saveptr);
        while (token != NULL && step_count < 20 && buf_off + 3 <= sizeof(prog_buf)) {
            unsigned int pos = 0;
            unsigned int delay = 0;
            if (sscanf(token, "%u/%u", &pos, &delay) >= 1) {
                if (pos > 100) { pos = 100; }
                if (delay > 65535) { delay = 65535; }
                prog_buf[buf_off++] = (uint8_t)pos;
                prog_buf[buf_off++] = (delay >> 8) & 0xFF;
                prog_buf[buf_off++] = delay & 0xFF;
                step_count++;
            } else {
                ESP_LOGW(TAG, "Invalid program step: '%s'", token);
                parse_ok = false;
                break;
            }
            token = strtok_r(NULL, ";", &saveptr);
        }

        if (parse_ok && step_count > 0) {
            prog_buf[3] = step_count;
            ret = zb_tuya_fingerbot_send_program_raw(short_addr, endpoint,
                                                      prog_buf, buf_off);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Set Fingerbot program: 0x%04X -> \"%s\" (%d steps)",
                         short_addr, prog_str, step_count);

                /* Store in state for UI feedback */
                if (s_mutex != NULL) {
                    xSemaphoreTake(s_mutex, portMAX_DELAY);
                    tuya_fingerbot_state_t *state = find_or_create_state(short_addr);
                    if (state != NULL) {
                        strncpy(state->program_text, prog_str,
                                sizeof(state->program_text) - 1);
                        state->program_text[sizeof(state->program_text) - 1] = '\0';
                    }
                    xSemaphoreGive(s_mutex);
                }
                handled = true;
            }
        } else if (step_count == 0) {
            ESP_LOGW(TAG, "Empty program string: '%s'", prog_str);
        }
    }

    /* Handle program_enable: {"program_enable": true|false} */
    cJSON *program_enable_json = cJSON_GetObjectItem(json, "program_enable");
    if (cJSON_IsBool(program_enable_json)) {
        bool enable = cJSON_IsTrue(program_enable_json);
        ret = zb_tuya_fingerbot_set_program_enable(short_addr, endpoint, enable);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Set Fingerbot program_enable: 0x%04X -> %s",
                     short_addr, enable ? "true" : "false");
            handled = true;
        }
    }

    /* Handle repeat_forever: {"repeat_forever": true|false} */
    cJSON *repeat_json = cJSON_GetObjectItem(json, "repeat_forever");
    if (cJSON_IsBool(repeat_json)) {
        bool repeat = cJSON_IsTrue(repeat_json);
        ret = zb_tuya_fingerbot_set_repeat_forever(short_addr, endpoint, repeat);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Set Fingerbot repeat_forever: 0x%04X -> %s",
                     short_addr, repeat ? "true" : "false");
            handled = true;
        }
    }

    /* Handle action: {"action": "on"|"off"|"push"|...} */
    cJSON *action_json = cJSON_GetObjectItem(json, "action");
    if (cJSON_IsString(action_json) && action_json->valuestring != NULL) {
        const char *action = action_json->valuestring;
        if (strcmp(action, "on") == 0) {
            /* ZCL On/Off triggers actual arm movement on _TZ3210 variant.
             * Tuya DP 1 alone does NOT cause movement on this device. */
            ret = command_send_on_off(short_addr, endpoint, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> ON (ZCL)", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "off") == 0) {
            ret = command_send_on_off(short_addr, endpoint, false);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> OFF (ZCL)", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "push") == 0) {
            ret = command_send_on_off(short_addr, endpoint, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> PUSH (ZCL)", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "move_down") == 0) {
            /* Tuya DP 1 = true in switch mode: arm moves down and STAYS down.
             * Unlike ZCL On/Off which always does full push+return cycle. */
            ret = zb_tuya_fingerbot_set_switch(short_addr, endpoint, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> MOVE_DOWN (DP1=true)",
                         short_addr);
                handled = true;
            }
        } else if (strcmp(action, "move_up") == 0) {
            /* Tuya DP 1 = false: arm moves back up */
            ret = zb_tuya_fingerbot_set_switch(short_addr, endpoint, false);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> MOVE_UP (DP1=false)",
                         short_addr);
                handled = true;
            }
        } else if (strcmp(action, "program_enable") == 0) {
            ret = zb_tuya_fingerbot_set_program_enable(short_addr, endpoint, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> PROGRAM_ENABLE", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "program_disable") == 0) {
            ret = zb_tuya_fingerbot_set_program_enable(short_addr, endpoint, false);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> PROGRAM_DISABLE", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "program") == 0) {
            static const uint8_t prog[] = {
                0xFF, 0xFF, 0xFF,   /* 3-byte header (default) */
                0x02,               /* 2 steps */
                0x64, 0x00, 0x02,   /* step 1: pos=100%, delay=2s */
                0x00, 0x00, 0x01,   /* step 2: pos=0%, delay=1s */
            };
            ret = zb_tuya_fingerbot_send_program_raw(short_addr, endpoint,
                                                      prog, sizeof(prog));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> PROGRAM (10 bytes)", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "run_program") == 0) {
            /* Read stored program text from state and re-parse to binary.
             * If no program is stored, use a default 2-step program. */
            bool has_stored = false;
            if (s_mutex != NULL) {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                tuya_fingerbot_state_t *st = find_or_create_state(short_addr);
                if (st != NULL && st->program_text[0] != '\0') {
                    /* Parse stored program text → binary */
                    uint8_t prog_buf[64];
                    prog_buf[0] = 0xFF;
                    prog_buf[1] = 0xFF;
                    prog_buf[2] = 0x00;
                    uint8_t step_count = 0;
                    size_t buf_off = 4;

                    char prog_copy[128];
                    strncpy(prog_copy, st->program_text, sizeof(prog_copy) - 1);
                    prog_copy[sizeof(prog_copy) - 1] = '\0';
                    xSemaphoreGive(s_mutex);

                    char *saveptr = NULL;
                    char *token = strtok_r(prog_copy, ";", &saveptr);
                    while (token != NULL && step_count < 20 && buf_off + 3 <= sizeof(prog_buf)) {
                        unsigned int pos = 0, delay = 0;
                        if (sscanf(token, "%u/%u", &pos, &delay) >= 1) {
                            if (pos > 100) { pos = 100; }
                            if (delay > 65535) { delay = 65535; }
                            prog_buf[buf_off++] = (uint8_t)pos;
                            prog_buf[buf_off++] = (delay >> 8) & 0xFF;
                            prog_buf[buf_off++] = delay & 0xFF;
                            step_count++;
                        }
                        token = strtok_r(NULL, ";", &saveptr);
                    }
                    if (step_count > 0) {
                        prog_buf[3] = step_count;
                        zb_tuya_fingerbot_send_program_raw(short_addr, endpoint,
                                                            prog_buf, buf_off);
                        has_stored = true;
                    }
                } else {
                    xSemaphoreGive(s_mutex);
                }
            }
            if (!has_stored) {
                /* Fallback: default 2-step program */
                static const uint8_t prog[] = {
                    0xFF, 0xFF, 0x00, 0x02,
                    0x64, 0x00, 0x02,   /* 100% for 2s */
                    0x00, 0x00, 0x01,   /* 0% for 1s */
                };
                zb_tuya_fingerbot_send_program_raw(short_addr, endpoint,
                                                    prog, sizeof(prog));
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            zb_tuya_fingerbot_set_program_enable(short_addr, endpoint, true);
            vTaskDelay(pdMS_TO_TICKS(500));
            ret = command_send_on_off(short_addr, endpoint, true);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> RUN_PROGRAM (DP109+DP121+ZCL)",
                         short_addr);
                handled = true;
            }
        } else if (strcmp(action, "factory_reset_device") == 0) {
            zb_device_t *dev = zb_device_get(short_addr);
            if (dev != NULL) {
                ret = zb_coordinator_permit_join(60);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Permit join failed: %s", esp_err_to_name(ret));
                }

                esp_zb_zdo_mgmt_leave_req_param_t leave_req = {
                    .dst_nwk_addr = short_addr,
                    .remove_children = false,
                    .rejoin = true,
                };
                memcpy(leave_req.device_address, dev->ieee_addr, 8);

                zb_device_remove(short_addr);

                esp_zb_lock_acquire(portMAX_DELAY);
                esp_zb_zdo_device_leave_req(&leave_req, NULL, NULL);
                esp_zb_lock_release();

                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> FACTORY_RESET_DEVICE (leave+rejoin)",
                         short_addr);
                ret = ESP_OK;
                handled = true;
            }
        } else if (strcmp(action, "reset_mode") == 0) {
            zb_tuya_fingerbot_set_program_enable(short_addr, endpoint, false);
            vTaskDelay(pdMS_TO_TICKS(200));
            ret = zb_tuya_fingerbot_set_mode(short_addr, endpoint, FINGERBOT_MODE_PUSH);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Fingerbot action: 0x%04X -> RESET_MODE", short_addr);
                handled = true;
            }
        } else if (strcmp(action, "test_dp") == 0) {
            /* Debug: send arbitrary DP for experimentation.
             * Usage: {"action":"test_dp","test_dp_id":2,"test_dp_type":"bool","test_dp_value":1}
             * Optional: "test_dp_cmd":4  (Tuya CMD: 0=set, 4=command) */
            cJSON *dp_id_j = cJSON_GetObjectItem(json, "test_dp_id");
            cJSON *dp_type_j = cJSON_GetObjectItem(json, "test_dp_type");
            cJSON *dp_val_j = cJSON_GetObjectItem(json, "test_dp_value");
            cJSON *dp_cmd_j = cJSON_GetObjectItem(json, "test_dp_cmd");
            if (cJSON_IsNumber(dp_id_j) && cJSON_IsNumber(dp_val_j)) {
                uint8_t dp_id = (uint8_t)cJSON_GetNumberValue(dp_id_j);
                int32_t dp_val = (int32_t)cJSON_GetNumberValue(dp_val_j);
                uint8_t tuya_cmd = ZB_TUYA_CMD_SEND_DATA;  /* Default: cmd 0x04 */
                if (cJSON_IsNumber(dp_cmd_j)) {
                    tuya_cmd = (uint8_t)cJSON_GetNumberValue(dp_cmd_j);
                }
                tuya_dp_t dp = {0};
                dp.dp_id = dp_id;
                dp.value.int_value = dp_val;
                /* Determine type from string or default to bool */
                if (cJSON_IsString(dp_type_j)) {
                    const char *t = dp_type_j->valuestring;
                    if (strcmp(t, "value") == 0 || strcmp(t, "int") == 0) {
                        dp.type = TUYA_DP_TYPE_VALUE;
                        dp.length = 4;
                    } else if (strcmp(t, "enum") == 0) {
                        dp.type = TUYA_DP_TYPE_ENUM;
                        dp.length = 1;
                    } else {
                        dp.type = TUYA_DP_TYPE_BOOL;
                        dp.length = 1;
                    }
                } else {
                    dp.type = TUYA_DP_TYPE_BOOL;
                    dp.length = 1;
                }
                /* Check for flags (5th parameter): 0=normal, 1=no header, 2=direction CLI */
                cJSON *dp_flags_j = cJSON_GetObjectItem(json, "test_dp_flags");
                uint8_t flags = 0;
                if (cJSON_IsNumber(dp_flags_j)) {
                    flags = (uint8_t)cJSON_GetNumberValue(dp_flags_j);
                }

                if (flags & 0x01) {
                    /* Flag 1: Send raw DP without status+seq header.
                     * Payload: [dp_id:1][type:1][len:2][data] */
                    uint8_t raw[8];
                    raw[0] = dp_id;
                    raw[1] = (uint8_t)dp.type;
                    uint16_t dlen = (dp.type == TUYA_DP_TYPE_VALUE) ? 4 : 1;
                    raw[2] = (dlen >> 8) & 0xFF;
                    raw[3] = dlen & 0xFF;
                    if (dp.type == TUYA_DP_TYPE_VALUE) {
                        raw[4] = (dp_val >> 24) & 0xFF;
                        raw[5] = (dp_val >> 16) & 0xFF;
                        raw[6] = (dp_val >> 8) & 0xFF;
                        raw[7] = dp_val & 0xFF;
                    } else {
                        raw[4] = dp_val & 0xFF;
                    }
                    size_t raw_len = 4 + dlen;

                    uint8_t direction = (flags & 0x02) ?
                        ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI :
                        ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;

                    esp_zb_zcl_custom_cluster_cmd_t zcl_cmd = {
                        .zcl_basic_cmd = {
                            .dst_addr_u.addr_short = short_addr,
                            .dst_endpoint = endpoint,
                            .src_endpoint = 1,
                        },
                        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                        .cluster_id = ZB_TUYA_CLUSTER_ID,
                        .custom_cmd_id = tuya_cmd,
                        .direction = direction,
                        .data = {
                            .type = ESP_ZB_ZCL_ATTR_TYPE_SET,
                            .size = raw_len,
                            .value = raw,
                        },
                    };
                    esp_zb_lock_acquire(portMAX_DELAY);
                    ret = esp_zb_zcl_custom_cluster_cmd_req(&zcl_cmd);
                    esp_zb_lock_release();
                    if (ret != ESP_OK) { ret = ESP_OK; } /* sleepy OK */
                    ESP_LOGW(TAG, "TEST_DP RAW: 0x%04X -> dp=%d t=%d v=%ld cmd=0x%02X dir=%d len=%zu",
                             short_addr, dp_id, dp.type, (long)dp_val, tuya_cmd, direction, raw_len);
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, raw, raw_len, ESP_LOG_WARN);
                    handled = true;
                } else {
                    /* Use normal send (direction flag ignored without raw mode) */
                    if (flags & 0x02) {
                        ESP_LOGW(TAG, "Direction TO_CLI flag requires raw mode (flags|=1)");
                    }
                    ret = zb_tuya_send_dp_with_cmd(short_addr, endpoint, &dp, tuya_cmd);
                    ESP_LOGW(TAG, "TEST_DP: 0x%04X -> dp=%d t=%d v=%ld cmd=0x%02X flags=0x%02X",
                             short_addr, dp_id, dp.type, (long)dp_val, tuya_cmd, flags);
                    handled = (ret == ESP_OK);
                }
            } else {
                ESP_LOGW(TAG, "test_dp requires: test_dp_id, test_dp_value (numbers)");
            }
        } else if (strcmp(action, "raw_cmd") == 0) {
            /* Debug: send arbitrary raw bytes to Tuya cluster 0xEF00.
             * Usage: {"action":"raw_cmd","raw_cmd":"0010"}
             * Format: hex string, first 2 chars = CMD ID, rest = payload.
             * Optional: "raw_src_ep":242, "raw_dst_ep":1, "raw_cluster":0xEF00
             * Example: "10" = CMD 0x10 (MCU Version Query), empty payload
             *          "00" = CMD 0x00, empty payload */
            cJSON *raw_j = cJSON_GetObjectItem(json, "raw_cmd");
            if (cJSON_IsString(raw_j) && raw_j->valuestring != NULL) {
                const char *hex = raw_j->valuestring;
                size_t hex_len = strlen(hex);
                if (hex_len >= 2 && (hex_len % 2) == 0) {
                    /* Parse hex string to bytes */
                    size_t byte_len = hex_len / 2;
                    uint8_t bytes[64];
                    if (byte_len > sizeof(bytes)) {
                        byte_len = sizeof(bytes);
                    }
                    bool parse_ok = true;
                    for (size_t i = 0; i < byte_len; i++) {
                        char h[3] = {hex[i * 2], hex[i * 2 + 1], '\0'};
                        unsigned int val;
                        if (sscanf(h, "%x", &val) != 1) {
                            parse_ok = false;
                            break;
                        }
                        bytes[i] = (uint8_t)val;
                    }
                    if (parse_ok) {
                        uint8_t cmd_id = bytes[0];
                        uint8_t *payload = (byte_len > 1) ? &bytes[1] : NULL;
                        size_t payload_len = (byte_len > 1) ? (byte_len - 1) : 0;

                        /* Optional endpoint and cluster overrides */
                        uint8_t src_ep = 1;
                        uint8_t dst_ep = endpoint;
                        uint16_t cluster = ZB_TUYA_CLUSTER_ID;
                        cJSON *src_ep_j = cJSON_GetObjectItem(json, "raw_src_ep");
                        cJSON *dst_ep_j = cJSON_GetObjectItem(json, "raw_dst_ep");
                        cJSON *cluster_j = cJSON_GetObjectItem(json, "raw_cluster");
                        if (cJSON_IsNumber(src_ep_j)) {
                            src_ep = (uint8_t)cJSON_GetNumberValue(src_ep_j);
                        }
                        if (cJSON_IsNumber(dst_ep_j)) {
                            dst_ep = (uint8_t)cJSON_GetNumberValue(dst_ep_j);
                        }
                        if (cJSON_IsNumber(cluster_j)) {
                            cluster = (uint16_t)cJSON_GetNumberValue(cluster_j);
                        }

                        esp_zb_zcl_custom_cluster_cmd_t zcl_cmd = {
                            .zcl_basic_cmd = {
                                .dst_addr_u.addr_short = short_addr,
                                .dst_endpoint = dst_ep,
                                .src_endpoint = src_ep,
                            },
                            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                            .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                            .cluster_id = cluster,
                            .custom_cmd_id = cmd_id,
                            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                            .data = {
                                .type = ESP_ZB_ZCL_ATTR_TYPE_SET,
                                .size = payload_len,
                                .value = payload,
                            },
                        };
                        esp_zb_lock_acquire(portMAX_DELAY);
                        ret = esp_zb_zcl_custom_cluster_cmd_req(&zcl_cmd);
                        esp_zb_lock_release();
                        if (ret != ESP_OK) { ret = ESP_OK; }
                        ESP_LOGW(TAG, "RAW_CMD: 0x%04X -> cmd=0x%02X len=%zu "
                                 "src_ep=%d dst_ep=%d cluster=0x%04X",
                                 short_addr, cmd_id, payload_len,
                                 src_ep, dst_ep, cluster);
                        if (payload_len > 0) {
                            ESP_LOG_BUFFER_HEX_LEVEL(TAG, payload, payload_len, ESP_LOG_WARN);
                        }
                        handled = true;
                    } else {
                        ESP_LOGW(TAG, "raw_cmd: invalid hex string");
                    }
                } else {
                    ESP_LOGW(TAG, "raw_cmd: need even hex string, min 2 chars (cmd byte)");
                }
            } else {
                ESP_LOGW(TAG, "raw_cmd requires string parameter");
            }
        } else {
            ESP_LOGW(TAG, "Unknown Fingerbot action: %s", action);
        }
    }

    /* Handle state: {"state": "ON"|"OFF"} — ZCL On/Off only.
     * Tuya DP 1 does not trigger movement on _TZ3210 variant. */
    cJSON *state_json = cJSON_GetObjectItem(json, "state");
    if (cJSON_IsString(state_json) && state_json->valuestring != NULL) {
        bool on = (strcmp(state_json->valuestring, "ON") == 0);
        ret = command_send_on_off(short_addr, endpoint, on);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Set Fingerbot state: 0x%04X -> %s (ZCL)",
                     short_addr, on ? "ON" : "OFF");
            handled = true;
        }
    }

    return handled ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * Driver Interface: build_state_json
 * ============================================================================ */

static cJSON *fingerbot_build_state_json(uint16_t short_addr)
{
    if (s_mutex == NULL) {
        return NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    const tuya_fingerbot_state_t *state = NULL;
    for (int i = 0; i < ZB_TUYA_MAX_DEVICES; i++) {
        if (s_fingerbot_states[i].valid && s_fingerbot_states[i].short_addr == short_addr) {
            state = &s_fingerbot_states[i];
            break;
        }
    }

    if (state == NULL || !state->valid) {
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    /* Basic state */
    cJSON_AddStringToObject(json, "state", state->state ? "ON" : "OFF");
    cJSON_AddStringToObject(json, "mode", zb_tuya_fingerbot_mode_to_string(state->mode));

    /* Movement settings */
    cJSON_AddNumberToObject(json, "down_movement", state->down_movement);
    cJSON_AddNumberToObject(json, "up_movement", state->up_movement);
    cJSON_AddNumberToObject(json, "sustain_time", state->sustain_time);
    cJSON_AddBoolToObject(json, "reverse", state->reverse);
    cJSON_AddBoolToObject(json, "touch_control", state->touch_control);
    cJSON_AddBoolToObject(json, "program_enable", state->program_enable);
    cJSON_AddBoolToObject(json, "repeat_forever", state->repeat_forever);

    /* Battery: prefer Tuya DP 105, fall back to ZCL Power Descriptor */
    uint8_t battery = state->battery;
    if (battery == 0) {
        int zcl_battery = zb_device_get_battery_percent(short_addr);
        if (zcl_battery > 0) {
            battery = (uint8_t)zcl_battery;
        }
    }
    cJSON_AddNumberToObject(json, "battery", battery);

    /* Program text */
    if (state->program_text[0] != '\0') {
        cJSON_AddStringToObject(json, "program", state->program_text);
    }

    ESP_LOGI(TAG, "Built Fingerbot state for 0x%04X: state=%s mode=%s battery=%d%%",
             short_addr,
             state->state ? "ON" : "OFF",
             zb_tuya_fingerbot_mode_to_string(state->mode),
             state->battery);

    xSemaphoreGive(s_mutex);
    return json;
}

/* ============================================================================
 * Discovery Helpers
 * ============================================================================ */

/**
 * @brief Serialize and publish discovery config
 */
static esp_err_t publish_discovery_config(const char *topic, cJSON *config,
                                           const char *entity_type,
                                           const char *device_name)
{
    char *json_str = json_to_string_and_delete(config);
    if (!json_str) {
        ESP_LOGE(TAG, "Failed to serialize %s discovery", entity_type);
        return ESP_FAIL;
    }

    esp_err_t ret = mqtt_client_publish(topic, json_str, strlen(json_str), 1, true);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Published %s discovery: %s", entity_type, device_name);
    } else {
        ESP_LOGE(TAG, "Failed to publish %s discovery: %s", entity_type, esp_err_to_name(ret));
    }

    free(json_str);
    return ret;
}

/**
 * @brief Add device info block to discovery config
 */
static void add_device_info(cJSON *config, const zb_device_t *device)
{
    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    cJSON *dev_info = cJSON_CreateObject();
    cJSON *identifiers = cJSON_CreateArray();
    char identifier[GW_BUFFER_SIZE_SMALL];
    snprintf(identifier, sizeof(identifier), "zigbee2mqtt_%s", ieee_str);
    cJSON_AddItemToArray(identifiers, cJSON_CreateString(identifier));
    cJSON_AddItemToObject(dev_info, "identifiers", identifiers);
    cJSON_AddItemToObject(config, "device", dev_info);
}

/**
 * @brief Publish HA discovery for an action button
 */
static esp_err_t publish_action_button(const zb_device_t *device,
                                        const char *action, const char *icon)
{
    cJSON *config = cJSON_CreateObject();
    if (config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", ieee_str, action);

    char topic[MQTT_TOPIC_MAX_LEN];
    esp_err_t ret = mqtt_topic_ha_discovery("button", unique_id, topic, sizeof(topic));
    if (ret != ESP_OK) {
        cJSON_Delete(config);
        return ret;
    }

    char name[GW_BUFFER_SIZE_SMALL];
    char action_cap[16];
    strncpy(action_cap, action, sizeof(action_cap) - 1);
    action_cap[sizeof(action_cap) - 1] = '\0';
    if (action_cap[0] >= 'a' && action_cap[0] <= 'z') {
        action_cap[0] -= 32;
    }
    snprintf(name, sizeof(name), "%s %s", device->friendly_name, action_cap);
    cJSON_AddStringToObject(config, "name", name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);

    char cmd_topic[MQTT_LONG_STR_MAX_LEN];
    snprintf(cmd_topic, sizeof(cmd_topic), "zigbee2mqtt/%s/set", device->friendly_name);
    cJSON_AddStringToObject(config, "command_topic", cmd_topic);

    char payload[GW_BUFFER_SIZE_SMALL];
    snprintf(payload, sizeof(payload), "{\"action\": \"%s\"}", action);
    cJSON_AddStringToObject(config, "payload_press", payload);

    add_device_info(config, device);
    cJSON_AddStringToObject(config, "icon", icon ? icon : "mdi:gesture-tap");

    char log_msg[GW_BUFFER_SIZE_SMALL];
    snprintf(log_msg, sizeof(log_msg), "button %s", action);
    return publish_discovery_config(topic, config, log_msg, device->friendly_name);
}

/**
 * @brief Publish HA discovery for Fingerbot mode select entity
 */
static esp_err_t publish_mode_select(const zb_device_t *device)
{
    cJSON *config = cJSON_CreateObject();
    if (config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
    snprintf(unique_id, sizeof(unique_id), "%s_mode", ieee_str);

    char topic[MQTT_TOPIC_MAX_LEN];
    esp_err_t ret = mqtt_topic_ha_discovery("select", unique_id, topic, sizeof(topic));
    if (ret != ESP_OK) {
        cJSON_Delete(config);
        return ret;
    }

    char name[GW_BUFFER_SIZE_SMALL];
    snprintf(name, sizeof(name), "%s Mode", device->friendly_name);
    cJSON_AddStringToObject(config, "name", name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);

    char state_topic[GW_BUFFER_SIZE_MEDIUM];
    snprintf(state_topic, sizeof(state_topic), "zigbee2mqtt/%s", device->friendly_name);
    cJSON_AddStringToObject(config, "state_topic", state_topic);

    char cmd_topic[MQTT_LONG_STR_MAX_LEN];
    snprintf(cmd_topic, sizeof(cmd_topic), "zigbee2mqtt/%s/set", device->friendly_name);
    cJSON_AddStringToObject(config, "command_topic", cmd_topic);

    cJSON_AddStringToObject(config, "value_template", "{{ value_json.mode }}");
    cJSON_AddStringToObject(config, "command_template", "{\"mode\": \"{{ value }}\"}");

    cJSON *options = cJSON_CreateArray();
    cJSON_AddItemToArray(options, cJSON_CreateString("push"));
    cJSON_AddItemToArray(options, cJSON_CreateString("switch"));
    cJSON_AddItemToArray(options, cJSON_CreateString("program"));
    cJSON_AddItemToObject(config, "options", options);

    add_device_info(config, device);
    cJSON_AddStringToObject(config, "icon", "mdi:gesture-tap-button");

    return publish_discovery_config(topic, config, "select mode", device->friendly_name);
}

/**
 * @brief Publish HA discovery for a number entity
 */
static esp_err_t publish_number_entity(const zb_device_t *device,
                                        const char *field, const char *display_name,
                                        double min, double max, double step,
                                        const char *unit, const char *icon)
{
    cJSON *config = cJSON_CreateObject();
    if (config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", ieee_str, field);

    char topic[MQTT_TOPIC_MAX_LEN];
    esp_err_t ret = mqtt_topic_ha_discovery("number", unique_id, topic, sizeof(topic));
    if (ret != ESP_OK) {
        cJSON_Delete(config);
        return ret;
    }

    char name[GW_BUFFER_SIZE_SMALL];
    snprintf(name, sizeof(name), "%s %s", device->friendly_name, display_name);
    cJSON_AddStringToObject(config, "name", name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);

    char state_topic[GW_BUFFER_SIZE_MEDIUM];
    snprintf(state_topic, sizeof(state_topic), "zigbee2mqtt/%s", device->friendly_name);
    cJSON_AddStringToObject(config, "state_topic", state_topic);

    char cmd_topic[MQTT_LONG_STR_MAX_LEN];
    snprintf(cmd_topic, sizeof(cmd_topic), "zigbee2mqtt/%s/set", device->friendly_name);
    cJSON_AddStringToObject(config, "command_topic", cmd_topic);

    char value_template[GW_BUFFER_SIZE_SMALL];
    snprintf(value_template, sizeof(value_template), "{{ value_json.%s }}", field);
    cJSON_AddStringToObject(config, "value_template", value_template);

    char cmd_template[GW_BUFFER_SIZE_SMALL];
    snprintf(cmd_template, sizeof(cmd_template), "{\"%s\": {{ value }}}", field);
    cJSON_AddStringToObject(config, "command_template", cmd_template);

    cJSON_AddNumberToObject(config, "min", min);
    cJSON_AddNumberToObject(config, "max", max);
    cJSON_AddNumberToObject(config, "step", step);
    if (unit != NULL) {
        cJSON_AddStringToObject(config, "unit_of_measurement", unit);
    }
    cJSON_AddStringToObject(config, "icon", icon);

    add_device_info(config, device);

    char log_msg[GW_BUFFER_SIZE_SMALL];
    snprintf(log_msg, sizeof(log_msg), "number %s", field);
    return publish_discovery_config(topic, config, log_msg, device->friendly_name);
}

/**
 * @brief Publish HA discovery for a switch entity
 */
static esp_err_t publish_switch_entity(const zb_device_t *device,
                                        const char *field, const char *display_name,
                                        const char *icon)
{
    cJSON *config = cJSON_CreateObject();
    if (config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", ieee_str, field);

    char topic[MQTT_TOPIC_MAX_LEN];
    esp_err_t ret = mqtt_topic_ha_discovery("switch", unique_id, topic, sizeof(topic));
    if (ret != ESP_OK) {
        cJSON_Delete(config);
        return ret;
    }

    char name[GW_BUFFER_SIZE_SMALL];
    snprintf(name, sizeof(name), "%s %s", device->friendly_name, display_name);
    cJSON_AddStringToObject(config, "name", name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);

    char state_topic[GW_BUFFER_SIZE_MEDIUM];
    snprintf(state_topic, sizeof(state_topic), "zigbee2mqtt/%s", device->friendly_name);
    cJSON_AddStringToObject(config, "state_topic", state_topic);

    char cmd_topic[MQTT_LONG_STR_MAX_LEN];
    snprintf(cmd_topic, sizeof(cmd_topic), "zigbee2mqtt/%s/set", device->friendly_name);
    cJSON_AddStringToObject(config, "command_topic", cmd_topic);

    /* Jinja2 renders JSON booleans as Python True/False (capital),
     * so use explicit ON/OFF mapping — standard HA switch pattern */
    char value_template[GW_BUFFER_SIZE_MEDIUM];
    snprintf(value_template, sizeof(value_template),
             "{{ 'ON' if value_json.%s else 'OFF' }}", field);
    cJSON_AddStringToObject(config, "value_template", value_template);

    char payload_on[GW_BUFFER_SIZE_SMALL];
    snprintf(payload_on, sizeof(payload_on), "{\"%s\": true}", field);
    cJSON_AddStringToObject(config, "payload_on", payload_on);

    char payload_off[GW_BUFFER_SIZE_SMALL];
    snprintf(payload_off, sizeof(payload_off), "{\"%s\": false}", field);
    cJSON_AddStringToObject(config, "payload_off", payload_off);

    cJSON_AddStringToObject(config, "state_on", "ON");
    cJSON_AddStringToObject(config, "state_off", "OFF");
    cJSON_AddStringToObject(config, "icon", icon);

    add_device_info(config, device);

    char log_msg[GW_BUFFER_SIZE_SMALL];
    snprintf(log_msg, sizeof(log_msg), "switch %s", field);
    return publish_discovery_config(topic, config, log_msg, device->friendly_name);
}

/* ============================================================================
 * Driver Interface: publish_discovery
 * ============================================================================ */

static esp_err_t fingerbot_publish_discovery(const zb_device_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Publishing Fingerbot extras for: %s", device->friendly_name);

    esp_err_t ret;
    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));

    /* === 1. Mode (Select: push/switch/program) === */
    ret = publish_mode_select(device);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish mode: %s", esp_err_to_name(ret));
    }

    /* === 2. Push (Button) === */
    ret = publish_action_button(device, "push", "mdi:gesture-tap");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish push button: %s", esp_err_to_name(ret));
    }

    /* === 3. On / Off (Buttons) === */
    ret = publish_action_button(device, "on", "mdi:gesture-tap-hold");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish on button: %s", esp_err_to_name(ret));
    }

    ret = publish_action_button(device, "off", "mdi:gesture-swipe-up");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish off button: %s", esp_err_to_name(ret));
    }

    /* === 4. Down Movement (Number 51-100%) === */
    ret = publish_number_entity(device, "down_movement", "Down Movement",
                                 51, 100, 1, "%", "mdi:arrow-down-bold");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish down_movement: %s", esp_err_to_name(ret));
    }

    /* === 5. Up Movement (Number 0-50%) === */
    ret = publish_number_entity(device, "up_movement", "Up Movement",
                                 0, 50, 1, "%", "mdi:arrow-up-bold");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish up_movement: %s", esp_err_to_name(ret));
    }

    /* === 6. Sustain Time (Number 0-255 seconds) === */
    ret = publish_number_entity(device, "sustain_time", "Sustain Time",
                                 0, 255, 1, "s", "mdi:timer-outline");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish sustain_time: %s", esp_err_to_name(ret));
    }

    /* === 7. Reverse (Switch) === */
    ret = publish_switch_entity(device, "reverse", "Reverse", "mdi:swap-horizontal");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish reverse: %s", esp_err_to_name(ret));
    }

    /* === 8. Touch Control (Switch) === */
    ret = publish_switch_entity(device, "touch_control", "Touch Control",
                                 "mdi:gesture-tap");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish touch_control: %s", esp_err_to_name(ret));
    }

    /* === 9. Battery (Sensor) === */
    {
        char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
        snprintf(unique_id, sizeof(unique_id), "%s_battery", ieee_str);

        char topic[MQTT_TOPIC_MAX_LEN];
        ret = mqtt_topic_ha_discovery("sensor", unique_id, topic, sizeof(topic));
        if (ret == ESP_OK) {
            cJSON *config = cJSON_CreateObject();
            if (config != NULL) {
                char name[GW_BUFFER_SIZE_SMALL];
                snprintf(name, sizeof(name), "%s Battery", device->friendly_name);
                cJSON_AddStringToObject(config, "name", name);
                cJSON_AddStringToObject(config, "unique_id", unique_id);
                cJSON_AddStringToObject(config, "device_class", "battery");
                cJSON_AddStringToObject(config, "state_class", "measurement");
                cJSON_AddStringToObject(config, "unit_of_measurement", "%");
                cJSON_AddStringToObject(config, "icon", "mdi:battery");

                char state_topic[GW_BUFFER_SIZE_MEDIUM];
                snprintf(state_topic, sizeof(state_topic),
                         "zigbee2mqtt/%s", device->friendly_name);
                cJSON_AddStringToObject(config, "state_topic", state_topic);
                cJSON_AddStringToObject(config, "value_template",
                                         "{{ value_json.battery | default(0) }}");

                add_device_info(config, device);
                ret = publish_discovery_config(topic, config, "sensor battery",
                                                device->friendly_name);
            }
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to publish battery sensor: %s", esp_err_to_name(ret));
        }
    }

    /* === 10. Program (Text input) === */
    {
        char uid[GW_BUFFER_SIZE_SMALL];
        snprintf(uid, sizeof(uid), "%s_program", ieee_str);

        char topic[MQTT_TOPIC_MAX_LEN];
        ret = mqtt_topic_ha_discovery("text", uid, topic, sizeof(topic));
        if (ret == ESP_OK) {
            cJSON *config = cJSON_CreateObject();
            if (config != NULL) {
                char name[GW_BUFFER_SIZE_SMALL];
                snprintf(name, sizeof(name), "%s Program", device->friendly_name);
                cJSON_AddStringToObject(config, "name", name);
                cJSON_AddStringToObject(config, "unique_id", uid);
                cJSON_AddStringToObject(config, "icon", "mdi:script-text-outline");

                char cmd_topic[MQTT_LONG_STR_MAX_LEN];
                snprintf(cmd_topic, sizeof(cmd_topic),
                         "zigbee2mqtt/%s/set", device->friendly_name);
                cJSON_AddStringToObject(config, "command_topic", cmd_topic);
                cJSON_AddStringToObject(config, "command_template",
                                         "{\"program\": \"{{ value }}\"}");

                char state_topic[GW_BUFFER_SIZE_MEDIUM];
                snprintf(state_topic, sizeof(state_topic),
                         "zigbee2mqtt/%s", device->friendly_name);
                cJSON_AddStringToObject(config, "state_topic", state_topic);
                cJSON_AddStringToObject(config, "value_template",
                                         "{{ value_json.program | default('') }}");

                cJSON_AddNumberToObject(config, "min", 3);
                cJSON_AddNumberToObject(config, "max", 127);
                cJSON_AddStringToObject(config, "mode", "text");
                cJSON_AddStringToObject(config, "pattern",
                    "^[0-9]{1,3}/[0-9]{1,5}(;[0-9]{1,3}/[0-9]{1,5})*$");

                add_device_info(config, device);
                ret = publish_discovery_config(topic, config, "text program",
                                                device->friendly_name);
            }
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to publish program text: %s", esp_err_to_name(ret));
        }
    }

    /* === 11. Program Enable (Switch) === */
    ret = publish_switch_entity(device, "program_enable", "Program Enable",
                                 "mdi:play-circle-outline");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish program_enable: %s", esp_err_to_name(ret));
    }

    /* === 12. Repeat Forever (Switch) === */
    ret = publish_switch_entity(device, "repeat_forever", "Repeat Forever", "mdi:repeat");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish repeat_forever: %s", esp_err_to_name(ret));
    }

    /* === 13. Run Program (Button) === */
    ret = publish_action_button(device, "run_program", "mdi:play");
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish run_program button: %s", esp_err_to_name(ret));
    }

    /* === 14. Test DP (Text, disabled by default) === */
    {
        char uid[GW_BUFFER_SIZE_SMALL];
        snprintf(uid, sizeof(uid), "zigbee2mqtt_%s_test_dp", ieee_str);

        char topic[GW_BUFFER_SIZE_MEDIUM];
        snprintf(topic, sizeof(topic),
                 "homeassistant/text/%s/test_dp/config", ieee_str);

        cJSON *config = cJSON_CreateObject();
        if (config != NULL) {
            cJSON_AddStringToObject(config, "name", "Test DP");
            cJSON_AddStringToObject(config, "unique_id", uid);
            cJSON_AddStringToObject(config, "icon", "mdi:test-tube");
            cJSON_AddBoolToObject(config, "enabled_by_default", false);

            char cmd_topic[GW_BUFFER_SIZE_MEDIUM];
            snprintf(cmd_topic, sizeof(cmd_topic),
                     "zigbee2mqtt/%s/set", device->friendly_name);
            cJSON_AddStringToObject(config, "command_topic", cmd_topic);

            cJSON_AddStringToObject(config, "command_template",
                "{% set parts = value.split(':') %}"
                "{\"action\":\"test_dp\","
                "\"test_dp_id\":{{ parts[0] }},"
                "\"test_dp_type\":\"{{ parts[1] }}\","
                "\"test_dp_value\":{{ parts[2] }}"
                "{% if parts | length > 3 %},\"test_dp_cmd\":{{ parts[3] }}{% endif %}"
                "{% if parts | length > 4 %},\"test_dp_flags\":{{ parts[4] }}{% endif %}"
                "}");

            char state_topic[GW_BUFFER_SIZE_MEDIUM];
            snprintf(state_topic, sizeof(state_topic),
                     "zigbee2mqtt/%s", device->friendly_name);
            cJSON_AddStringToObject(config, "state_topic", state_topic);
            cJSON_AddStringToObject(config, "value_template",
                                     "{{ value_json.test_dp | default('') }}");

            cJSON_AddNumberToObject(config, "min", 1);
            cJSON_AddNumberToObject(config, "max", 20);
            cJSON_AddStringToObject(config, "mode", "text");
            cJSON_AddStringToObject(config, "pattern",
                                     "^[0-9]+:(bool|enum|value|int):-?[0-9]+(:[0-9]+)?(:[0-9]+)?$");

            add_device_info(config, device);

            ret = publish_discovery_config(topic, config, "text test_dp",
                                            device->friendly_name);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to publish test_dp text: %s", esp_err_to_name(ret));
            }
        }
    }

    /* === 15. Raw CMD (Text, disabled by default) === */
    {
        char uid_raw[GW_BUFFER_SIZE_SMALL];
        snprintf(uid_raw, sizeof(uid_raw), "zigbee2mqtt_%s_raw_cmd", ieee_str);

        char topic_raw[GW_BUFFER_SIZE_MEDIUM];
        snprintf(topic_raw, sizeof(topic_raw),
                 "homeassistant/text/%s/raw_cmd/config", ieee_str);

        cJSON *config_raw = cJSON_CreateObject();
        if (config_raw != NULL) {
            cJSON_AddStringToObject(config_raw, "name", "Raw CMD");
            cJSON_AddStringToObject(config_raw, "unique_id", uid_raw);
            cJSON_AddStringToObject(config_raw, "icon", "mdi:console");
            cJSON_AddBoolToObject(config_raw, "enabled_by_default", false);

            char cmd_topic_raw[GW_BUFFER_SIZE_MEDIUM];
            snprintf(cmd_topic_raw, sizeof(cmd_topic_raw),
                     "zigbee2mqtt/%s/set", device->friendly_name);
            cJSON_AddStringToObject(config_raw, "command_topic", cmd_topic_raw);

            cJSON_AddStringToObject(config_raw, "command_template",
                "{% set parts = value.split(':') %}"
                "{\"action\":\"raw_cmd\",\"raw_cmd\":\"{{ parts[0] }}\""
                "{% if parts | length > 1 %},\"raw_src_ep\":{{ parts[1] }}{% endif %}"
                "{% if parts | length > 2 %},\"raw_dst_ep\":{{ parts[2] }}{% endif %}"
                "{% if parts | length > 3 %},\"raw_cluster\":{{ parts[3] }}{% endif %}"
                "}");

            char state_topic_raw[GW_BUFFER_SIZE_MEDIUM];
            snprintf(state_topic_raw, sizeof(state_topic_raw),
                     "zigbee2mqtt/%s", device->friendly_name);
            cJSON_AddStringToObject(config_raw, "state_topic", state_topic_raw);
            cJSON_AddStringToObject(config_raw, "value_template",
                                     "{{ value_json.raw_cmd | default('') }}");

            cJSON_AddNumberToObject(config_raw, "min", 2);
            cJSON_AddNumberToObject(config_raw, "max", 128);
            cJSON_AddStringToObject(config_raw, "mode", "text");
            cJSON_AddStringToObject(config_raw, "pattern",
                                     "^[0-9a-fA-F:]+$");

            add_device_info(config_raw, device);

            ret = publish_discovery_config(topic_raw, config_raw, "text raw_cmd",
                                            device->friendly_name);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to publish raw_cmd text: %s", esp_err_to_name(ret));
            }
        }
    }

    return ESP_OK;
}

/* ============================================================================
 * Driver Interface: init_device / remove_device
 * ============================================================================ */

static esp_err_t fingerbot_init_device(uint16_t short_addr)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    tuya_fingerbot_state_t *state = find_or_create_state(short_addr);
    xSemaphoreGive(s_mutex);

    if (state == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initialized Fingerbot state for 0x%04X", short_addr);
    return ESP_OK;
}

static void fingerbot_remove_device(uint16_t short_addr)
{
    if (s_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ZB_TUYA_MAX_DEVICES; i++) {
        if (s_fingerbot_states[i].valid && s_fingerbot_states[i].short_addr == short_addr) {
            memset(&s_fingerbot_states[i], 0, sizeof(tuya_fingerbot_state_t));
            ESP_LOGI(TAG, "Removed Fingerbot state for 0x%04X", short_addr);
            break;
        }
    }
    xSemaphoreGive(s_mutex);
}

/* ============================================================================
 * Public API: Get Fingerbot State (backward compat)
 * ============================================================================ */

const tuya_fingerbot_state_t *tuya_fingerbot_get_state(uint16_t short_addr)
{
    if (s_mutex == NULL) {
        return NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    const tuya_fingerbot_state_t *result = NULL;
    for (int i = 0; i < ZB_TUYA_MAX_DEVICES; i++) {
        if (s_fingerbot_states[i].valid && s_fingerbot_states[i].short_addr == short_addr) {
            result = &s_fingerbot_states[i];
            break;
        }
    }

    xSemaphoreGive(s_mutex);
    return result;
}

/* ============================================================================
 * Driver Vtable
 * ============================================================================ */

static const tuya_device_driver_t s_fingerbot_driver = {
    .name             = "fingerbot",
    .match            = fingerbot_match,
    .process_dp       = fingerbot_process_dp,
    .handle_command   = fingerbot_handle_command,
    .build_state_json = fingerbot_build_state_json,
    .publish_discovery = fingerbot_publish_discovery,
    .init_device      = fingerbot_init_device,
    .remove_device    = fingerbot_remove_device,
};

/* ============================================================================
 * Registration
 * ============================================================================ */

esp_err_t tuya_fingerbot_register(void)
{
    /* Create mutex for state access */
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create Fingerbot mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Clear state array */
    memset(s_fingerbot_states, 0, sizeof(s_fingerbot_states));

    /* Register with the driver registry */
    esp_err_t ret = tuya_driver_register(&s_fingerbot_driver);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Fingerbot driver registered");
    }
    return ret;
}
