/**
 * @file command_handler.c
 * @brief MQTT Command Handler Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "command_handler.h"
#include "zigbee/zb_device_handler.h"
#include "zigbee/zb_coordinator.h"
#include "zigbee/zb_alarms.h"
#include "zigbee/zb_constants.h"
#include "zigbee/zb_tuya.h"
#include "zigbee/zb_interview.h"
#include "zigbee/zb_cmd_retry.h"
#include "zigbee/tuya/tuya_driver_registry.h"
#include "zigbee/tuya/tuya_device_driver.h"
#include "core/compat_stubs.h"
#include "utils/json_utils.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "cJSON.h"
#include <string.h>

static const char *TAG = "CMD_HANDLER";

/* Statistics */
static uint32_t s_commands_processed = 0;
static uint32_t s_command_errors = 0;

esp_err_t command_handler_init(void)
{
    s_commands_processed = 0;
    s_command_errors = 0;
    ESP_LOGI(TAG, "Command handler initialized");
    return ESP_OK;
}

esp_err_t command_handler_process(const char *topic, const char *payload, size_t len)
{
    if (topic == NULL || payload == NULL || len == 0) {
        s_command_errors++;
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing command: %s", topic);
    ESP_LOGD(TAG, "Payload: %.*s", (int)len, payload);

    /* Extract friendly name from topic */
    char friendly_name[ZB_DEVICE_FRIENDLY_NAME_LEN];
    esp_err_t ret = mqtt_topic_extract_friendly_name(topic, friendly_name, sizeof(friendly_name));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to extract friendly name from topic");
        s_command_errors++;
        return ret;
    }

    ESP_LOGD(TAG, "Friendly name: %s", friendly_name);

    /* Find device by friendly name (avoids stack allocation) */
    zb_device_t *target_device = zb_device_find_by_name(friendly_name);
    if (target_device == NULL) {
        ESP_LOGW(TAG, "Device not found: %s", friendly_name);
        s_command_errors++;
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if device address is still pending (loaded from NVS but hasn't communicated yet) */
    if (target_device->short_addr == ZB_SHORT_ADDR_PENDING) {
        ESP_LOGW(TAG, "Device %s address pending - waiting for device to communicate", friendly_name);
        s_command_errors++;
        return ESP_ERR_INVALID_STATE;
    }

    /* Null-terminate payload for JSON parsing */
    char *payload_str = (char *)malloc(len + 1);
    if (payload_str == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for payload");
        s_command_errors++;
        return ESP_ERR_NO_MEM;
    }
    memcpy(payload_str, payload, len);
    payload_str[len] = '\0';

    /* Check if this is a Door Lock device and handle LOCK/UNLOCK commands */
    if (target_device->device_type == ZB_DEVICE_TYPE_DOOR_LOCK) {
        bool lock_cmd = false;
        ret = json_parse_lock_command(payload_str, &lock_cmd);
        free(payload_str);

        if (ret == ESP_OK) {
            if (lock_cmd) {
                ret = command_send_lock(target_device->short_addr, target_device->endpoint);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Sent LOCK command: %s", friendly_name);
                    s_commands_processed++;
                    return ESP_OK;
                }
            } else {
                ret = command_send_unlock(target_device->short_addr, target_device->endpoint);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Sent UNLOCK command: %s", friendly_name);
                    s_commands_processed++;
                    return ESP_OK;
                }
            }
            ESP_LOGE(TAG, "Failed to send lock command: %s", esp_err_to_name(ret));
            s_command_errors++;
            return ret;
        }
        /* If lock command parsing failed, the payload might not be a lock command */
        ESP_LOGW(TAG, "Failed to parse lock command, invalid payload");
        s_command_errors++;
        return ESP_FAIL;
    }

    /* Check for alarm reset commands: {"alarm_reset": code} or {"alarm_reset_all": true} */
    if (strstr(payload, "\"alarm_reset\"") != NULL || strstr(payload, "\"alarm_reset_all\"") != NULL) {
        cJSON *json = cJSON_Parse(payload_str);
        if (json != NULL) {
            /* Check for alarm_reset_all first */
            cJSON *reset_all = cJSON_GetObjectItem(json, "alarm_reset_all");
            if (cJSON_IsTrue(reset_all)) {
                cJSON_Delete(json);
                free(payload_str);
                ret = command_send_alarm_reset_all(target_device->short_addr, target_device->endpoint);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Reset all alarms for %s", friendly_name);
                    s_commands_processed++;
                } else {
                    s_command_errors++;
                }
                return ret;
            }

            /* Check for alarm_reset with code and cluster */
            cJSON *reset_obj = cJSON_GetObjectItem(json, "alarm_reset");
            if (cJSON_IsObject(reset_obj)) {
                cJSON *code_json = cJSON_GetObjectItem(reset_obj, "code");
                cJSON *cluster_json = cJSON_GetObjectItem(reset_obj, "cluster");
                if (cJSON_IsNumber(code_json) && cJSON_IsNumber(cluster_json)) {
                    uint8_t alarm_code = (uint8_t)cJSON_GetNumberValue(code_json);
                    uint16_t cluster_id = (uint16_t)cJSON_GetNumberValue(cluster_json);
                    cJSON_Delete(json);
                    free(payload_str);
                    ret = command_send_alarm_reset(target_device->short_addr, target_device->endpoint,
                                                   alarm_code, cluster_id);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "Reset alarm %d (cluster 0x%04X) for %s",
                                 alarm_code, cluster_id, friendly_name);
                        s_commands_processed++;
                    } else {
                        s_command_errors++;
                    }
                    return ret;
                }
            } else if (cJSON_IsNumber(reset_obj)) {
                /* Simple format: {"alarm_reset": code} - reset all alarms with this code */
                uint8_t alarm_code = (uint8_t)cJSON_GetNumberValue(reset_obj);
                cJSON_Delete(json);
                free(payload_str);
                /* For simple format, reset from all clusters (use 0xFFFF) */
                ret = zb_alarms_remove_from_table(target_device->short_addr, alarm_code, 0xFFFF);
                ESP_LOGI(TAG, "Removed alarm code %d from table for %s", alarm_code, friendly_name);
                s_commands_processed++;
                return ret;
            }
            cJSON_Delete(json);
        }
    }

    /* Handle Tuya device-specific commands via driver registry */
    if (zb_device_is_tuya(target_device->short_addr)) {
        bool tuya_cmd_handled = false;
        const tuya_device_driver_t *drv = tuya_driver_get(target_device->short_addr);

        /* On-demand binding: after gateway reboot the driver isn't bound yet
         * because no DPs or Basic Cluster reports have arrived. Try to bind now. */
        if (drv == NULL) {
            drv = tuya_driver_find(target_device->manufacturer, target_device->model);
            if (drv != NULL) {
                tuya_driver_bind(target_device->short_addr, drv);
                if (drv->init_device) {
                    drv->init_device(target_device->short_addr);
                }
                ESP_LOGI(TAG, "On-demand Tuya driver bind: 0x%04X -> '%s'",
                         target_device->short_addr, drv->name);
            }
        }

        if (drv != NULL && drv->handle_command != NULL) {
            cJSON *json = cJSON_Parse(payload_str);
            if (json != NULL) {
                ret = drv->handle_command(target_device->short_addr,
                                          target_device->endpoint, json);
                if (ret == ESP_OK) {
                    tuya_cmd_handled = true;
                    s_commands_processed++;
                    ESP_LOGI(TAG, "Tuya driver '%s' handled command for %s",
                             drv->name, friendly_name);
                }
                cJSON_Delete(json);
            }
        }

        /* If the Tuya driver handled the command, we're done.
         * The driver is responsible for ALL commands for its device type,
         * including state (ZCL On/Off + Tuya DP). No ZCL fallback needed. */
        if (tuya_cmd_handled) {
            free(payload_str);
            return ESP_OK;
        }
    }

    /* Parse command JSON for non-lock devices */
    bool state = false;
    uint8_t brightness = 0;
    uint16_t color_x = 0, color_y = 0;
    uint16_t transition = 0;

    ret = json_parse_command(payload_str, &state, &brightness, &color_x, &color_y, &transition);
    free(payload_str);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse command JSON");
        s_command_errors++;
        return ret;
    }

    ESP_LOGD(TAG, "Parsed: state=%d, brightness=%d, color_x=%d, color_y=%d, transition=%d",
             state, brightness, color_x, color_y, transition);

    /* Send ZCL commands based on parsed data */
    bool command_sent = false;

    /* Check for state command (on/off) */
    if (strstr(payload, "\"state\"") != NULL) {
        /* Send standard ZCL On/Off command (0x0006 cluster) */
        ret = command_send_on_off(target_device->short_addr, target_device->endpoint, state);
        if (ret == ESP_OK) {
            command_sent = true;
            ESP_LOGI(TAG, "Sent ON/OFF command: %s -> %s",
                     friendly_name, state ? "ON" : "OFF");
        } else {
            ESP_LOGE(TAG, "Failed to send ON/OFF command: %s", esp_err_to_name(ret));
        }

        /* Note: Tuya devices are fully handled by their driver above and
         * never reach this point. */
    }

    /* Check for brightness command */
    if (strstr(payload, "\"brightness\"") != NULL) {
        /* Convert transition from seconds to 1/10 seconds */
        uint16_t trans_time = (transition > 0) ? (transition * ZCL_TRANSITION_TIME_SCALE) : 0;
        ret = command_send_level(target_device->short_addr, target_device->endpoint,
                                brightness, trans_time);
        if (ret == ESP_OK) {
            command_sent = true;
            ESP_LOGI(TAG, "Sent LEVEL command: %s -> %d", friendly_name, brightness);
        } else {
            ESP_LOGE(TAG, "Failed to send LEVEL command: %s", esp_err_to_name(ret));
        }
    }

    /* Check for color command */
    if (strstr(payload, "\"color\"") != NULL && (color_x > 0 || color_y > 0)) {
        uint16_t trans_time = (transition > 0) ? (transition * ZCL_TRANSITION_TIME_SCALE) : 0;
        ret = command_send_color(target_device->short_addr, target_device->endpoint,
                                color_x, color_y, trans_time);
        if (ret == ESP_OK) {
            command_sent = true;
            ESP_LOGI(TAG, "Sent COLOR command: %s -> x=%d, y=%d", friendly_name, color_x, color_y);
        } else {
            ESP_LOGE(TAG, "Failed to send COLOR command: %s", esp_err_to_name(ret));
        }
    }

    if (command_sent) {
        s_commands_processed++;
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "No valid commands found in payload");
        s_command_errors++;
        return ESP_FAIL;
    }
}

esp_err_t command_send_on_off(uint16_t short_addr, uint8_t endpoint, bool on)
{
    ESP_LOGD(TAG, "Sending ON/OFF to 0x%04X EP%d: %s", short_addr, endpoint, on ? "ON" : "OFF");

    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1, /* Coordinator endpoint */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        /* For sleepy/battery devices, the stack may return an error but still queue
         * the command for delivery when the device polls. This is expected behavior
         * for battery-powered devices with long poll intervals. */
        ESP_LOGW(TAG, "On/off command for 0x%04X queued (sleepy device, ret=%s)",
                 short_addr, esp_err_to_name(ret));
        /* Treat as success - command will be delivered on next poll */
        ret = ESP_OK;
    }

    /* Update TX statistics */
    zb_coordinator_update_tx_count();

    /* Store for retry on device unavailable / timeout */
    cmd_retry_store_on_off(short_addr, endpoint, on);

    return ret;
}

esp_err_t command_send_level(uint16_t short_addr, uint8_t endpoint,
                             uint8_t level, uint16_t transition_time)
{
    ESP_LOGD(TAG, "Sending LEVEL to 0x%04X EP%d: %d (trans=%d)",
             short_addr, endpoint, level, transition_time);

    esp_zb_zcl_move_to_level_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1, /* Coordinator endpoint */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .level = level,
        .transition_time = transition_time,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_level_move_to_level_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send level command: %s", esp_err_to_name(ret));
    }

    /* Update TX statistics */
    if (ret == ESP_OK) {
        zb_coordinator_update_tx_count();

        /* Store for retry on device unavailable / timeout */
        cmd_retry_store_level(short_addr, endpoint, level, transition_time);
    }

    return ret;
}

esp_err_t command_send_color(uint16_t short_addr, uint8_t endpoint,
                             uint16_t x, uint16_t y, uint16_t transition_time)
{
    ESP_LOGD(TAG, "Sending COLOR to 0x%04X EP%d: x=%d, y=%d (trans=%d)",
             short_addr, endpoint, x, y, transition_time);

    esp_zb_zcl_color_move_to_color_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1, /* Coordinator endpoint */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .color_x = x,
        .color_y = y,
        .transition_time = transition_time,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_color_move_to_color_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send color command: %s", esp_err_to_name(ret));
    }

    /* Update TX statistics */
    if (ret == ESP_OK) {
        zb_coordinator_update_tx_count();

        /* Store for retry on device unavailable / timeout */
        cmd_retry_store_color(short_addr, endpoint, x, y, transition_time);
    }

    return ret;
}

esp_err_t command_send_toggle(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGD(TAG, "Sending TOGGLE to 0x%04X EP%d", short_addr, endpoint);

    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send toggle command: %s", esp_err_to_name(ret));
    }

    /* Update TX statistics */
    if (ret == ESP_OK) {
        zb_coordinator_update_tx_count();
    }

    return ret;
}

esp_err_t command_send_lock(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending LOCK command to 0x%04X EP%d", short_addr, endpoint);

    /* Use the Door Lock API from zb_device_handler */
    esp_err_t ret = zb_door_lock_cmd_lock(short_addr, endpoint);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send lock command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t command_send_unlock(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending UNLOCK command to 0x%04X EP%d", short_addr, endpoint);

    /* Use the Door Lock API from zb_device_handler */
    esp_err_t ret = zb_door_lock_cmd_unlock(short_addr, endpoint);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send unlock command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t command_send_alarm_reset(uint16_t short_addr, uint8_t endpoint,
                                    uint8_t alarm_code, uint16_t cluster_id)
{
    ESP_LOGI(TAG, "Sending RESET ALARM to 0x%04X EP%d: code=%d cluster=0x%04X",
             short_addr, endpoint, alarm_code, cluster_id);

    /* Use the Alarms API from zb_alarms */
    esp_err_t ret = zb_alarms_cmd_reset_alarm(short_addr, endpoint, alarm_code, cluster_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send reset alarm command: %s", esp_err_to_name(ret));
    }

    /* Update TX statistics */
    if (ret == ESP_OK) {
        zb_coordinator_update_tx_count();
    }

    return ret;
}

esp_err_t command_send_alarm_reset_all(uint16_t short_addr, uint8_t endpoint)
{
    ESP_LOGI(TAG, "Sending RESET ALL ALARMS to 0x%04X EP%d", short_addr, endpoint);

    /* Use the Alarms API from zb_alarms */
    esp_err_t ret = zb_alarms_cmd_reset_all(short_addr, endpoint);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send reset all alarms command: %s", esp_err_to_name(ret));
    }

    /* Update TX statistics */
    if (ret == ESP_OK) {
        zb_coordinator_update_tx_count();
    }

    return ret;
}

esp_err_t command_handler_get_stats(uint32_t *processed, uint32_t *errors)
{
    if (processed != NULL) {
        *processed = s_commands_processed;
    }
    if (errors != NULL) {
        *errors = s_command_errors;
    }
    return ESP_OK;
}

esp_err_t command_handler_test(void)
{
    ESP_LOGI(TAG, "Running command handler test...");

    /* Test command parsing */
    const char *test_topic = "zigbee2mqtt/test_light/set";
    const char *test_payload = "{\"state\":\"ON\",\"brightness\":254}";

    /* Extract friendly name */
    char friendly_name[64];
    esp_err_t ret = mqtt_topic_extract_friendly_name(test_topic, friendly_name, sizeof(friendly_name));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to extract friendly name");
        return ESP_FAIL;
    }

    if (strcmp(friendly_name, "test_light") != 0) {
        ESP_LOGE(TAG, "Friendly name mismatch: expected 'test_light', got '%s'", friendly_name);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Extracted friendly name: %s", friendly_name);

    /* Parse command */
    bool state = false;
    uint8_t brightness = 0;
    ret = json_parse_command(test_payload, &state, &brightness, NULL, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse command");
        return ESP_FAIL;
    }

    if (!state || brightness != ZCL_BRIGHTNESS_MAX) {
        ESP_LOGE(TAG, "Command parsing failed: state=%d, brightness=%d", state, brightness);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Command handler test PASSED");
    return ESP_OK;
}
