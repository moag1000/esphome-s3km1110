/**
 * @file uart_bridge.c
 * @brief UART Bridge Implementation — Zigbee to UART Translation
 *
 * Replaces mqtt_bridge. Sends JSON Lines over UART1 to the ESP32-S3.
 * Receives JSON commands from S3 and dispatches to Zigbee stack.
 */

#include "uart_bridge.h"
#include "uart_protocol.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* From shared esp_zb_coordinator component */
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "zb_network.h"
#include "zb_tuya.h"
#include "tuya/tuya_driver_registry.h"
#include "tuya/tuya_device_driver.h"
#include "core/device_state.h"
#include "zb_hal.h"
#include "esp_zigbee_core.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "UART_BRIDGE";

/* Bridge state */
static bool s_bridge_enabled = false;
static bridge_stats_t s_stats = {0};

/* ============================================================================
 * Low-level UART I/O
 * ============================================================================ */

esp_err_t uart_bridge_send_line(const char *json_str)
{
    if (json_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(json_str);
    int written = uart_write_bytes(UART_BRIDGE_PORT_NUM, json_str, len);
    if (written < 0) {
        s_stats.error_count++;
        return ESP_FAIL;
    }

    /* Send newline terminator */
    uart_write_bytes(UART_BRIDGE_PORT_NUM, "\n", 1);

    s_stats.publish_count++;
    ESP_LOGD(TAG, "TX: %s", json_str);
    return ESP_OK;
}

/* Helper: send a built message and free it */
static esp_err_t send_and_free(char *msg)
{
    if (msg == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t ret = uart_bridge_send_line(msg);
    free(msg);
    return ret;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t uart_bridge_init(void)
{
    /* Get pins from HAL (chip-specific) */
    int tx_pin = zb_hal_get_uart_tx_pin();
    int rx_pin = zb_hal_get_uart_rx_pin();

    /* Configure UART1 */
    uart_config_t uart_config = {
        .baud_rate = UART_BRIDGE_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(UART_BRIDGE_PORT_NUM,
                                         UART_BRIDGE_BUF_SIZE * 2,
                                         UART_BRIDGE_BUF_SIZE * 2,
                                         0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_BRIDGE_PORT_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_BRIDGE_PORT_NUM,
                       tx_pin,
                       rx_pin,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    ESP_LOGI(TAG, "UART bridge initialized (UART%d, TX=IO%d, RX=IO%d, %d baud)",
             UART_BRIDGE_PORT_NUM, tx_pin, rx_pin,
             UART_BRIDGE_BAUD_RATE);

    return ESP_OK;
}

esp_err_t uart_bridge_start(void)
{
    s_bridge_enabled = true;
    s_stats.enabled = true;

    /* Send ready message with network info */
    zb_network_info_t net_info = {0};
    zb_network_get_info(&net_info);

    char *msg = uart_proto_build_ready(net_info.pan_id, net_info.channel);
    esp_err_t ret = send_and_free(msg);

    ESP_LOGI(TAG, "UART bridge started (PAN=0x%04X, CH=%d)",
             net_info.pan_id, net_info.channel);
    return ret;
}

esp_err_t uart_bridge_stop(void)
{
    s_bridge_enabled = false;
    s_stats.enabled = false;
    ESP_LOGI(TAG, "UART bridge stopped");
    return ESP_OK;
}

bool uart_bridge_is_enabled(void)
{
    return s_bridge_enabled;
}

bridge_stats_t uart_bridge_get_stats(void)
{
    return s_stats;
}

/* ============================================================================
 * Device Event Handlers (called by zb_callbacks.c)
 * ============================================================================ */

esp_err_t uart_bridge_on_device_join(uint16_t short_addr)
{
    if (!s_bridge_enabled) return ESP_ERR_INVALID_STATE;

    s_stats.device_join_count++;

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X not found for join event", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char *msg = uart_proto_build_device_joined(ieee_str, short_addr);
    return send_and_free(msg);
}

esp_err_t uart_bridge_on_device_leave(uint16_t short_addr)
{
    if (!s_bridge_enabled) return ESP_ERR_INVALID_STATE;

    s_stats.device_leave_count++;

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%04X not found for leave event", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));

    char *msg = uart_proto_build_device_left(ieee_str);
    return send_and_free(msg);
}

esp_err_t uart_bridge_on_attribute_change(uint16_t short_addr,
                                           uint16_t cluster_id,
                                           uint16_t attr_id,
                                           const void *value,
                                           size_t value_len)
{
    if (!s_bridge_enabled) return ESP_ERR_INVALID_STATE;

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) return ESP_ERR_NOT_FOUND;

    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));

    /* Convert value to double for numeric attributes */
    double val = 0;
    if (value != NULL && value_len > 0) {
        switch (value_len) {
            case 1: val = (double)(*(uint8_t *)value); break;
            case 2: val = (double)(*(uint16_t *)value); break;
            case 4: val = (double)(*(uint32_t *)value); break;
            default: val = 0; break;
        }
    }

    /* Use endpoint 1 as default for attribute reports */
    char *msg = uart_proto_build_attribute_report(ieee_str, 1,
                                                   cluster_id, attr_id, val);
    return send_and_free(msg);
}

esp_err_t uart_bridge_publish_device_state(uint16_t short_addr)
{
    if (!s_bridge_enabled) return ESP_ERR_INVALID_STATE;

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) return ESP_ERR_NOT_FOUND;

    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));

    /* Build device state JSON */
    cJSON *state = device_state_to_cjson(device);
    if (state == NULL) return ESP_ERR_NO_MEM;

    char *msg = uart_proto_build_device_state(ieee_str, state);
    cJSON_Delete(state);

    return send_and_free(msg);
}

esp_err_t uart_bridge_publish_device_list(void)
{
    if (!s_bridge_enabled) return ESP_ERR_INVALID_STATE;

    size_t count = zb_device_get_count();
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;

    cJSON_AddStringToObject(root, "type", "device_list");
    cJSON *devices = cJSON_AddArrayToObject(root, "devices");

    for (size_t i = 0; i < count; i++) {
        zb_device_t *device = zb_device_get_by_index(i);
        if (device == NULL) continue;

        cJSON *dev = cJSON_CreateObject();
        if (dev == NULL) continue;

        char ieee_str[20];
        uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));
        cJSON_AddStringToObject(dev, "ieee", ieee_str);

        char addr_str[8];
        snprintf(addr_str, sizeof(addr_str), "0x%04X", device->short_addr);
        cJSON_AddStringToObject(dev, "short_addr", addr_str);

        if (device->manufacturer[0]) {
            cJSON_AddStringToObject(dev, "manufacturer", device->manufacturer);
        }
        if (device->model[0]) {
            cJSON_AddStringToObject(dev, "model", device->model);
        }
        if (device->friendly_name[0]) {
            cJSON_AddStringToObject(dev, "friendly_name", device->friendly_name);
        }
        cJSON_AddBoolToObject(dev, "online", device->online);

        cJSON_AddItemToArray(devices, dev);
    }

    char *msg = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_err_t ret = send_and_free(msg);
    return ret;
}

esp_err_t uart_bridge_publish_event(const char *type, const char *json_data)
{
    if (!s_bridge_enabled || type == NULL) return ESP_ERR_INVALID_STATE;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;

    cJSON_AddStringToObject(root, "type", type);

    if (json_data) {
        cJSON *data = cJSON_Parse(json_data);
        if (data) {
            cJSON_AddItemToObject(root, "data", data);
        }
    }

    char *msg = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return send_and_free(msg);
}

/* ============================================================================
 * Command Processing (S3 → C5)
 * ============================================================================ */

esp_err_t uart_bridge_process_command(const char *json_line)
{
    uart_command_t cmd;
    esp_err_t ret = uart_proto_parse_command(json_line, &cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to parse command: %s", json_line);
        return ret;
    }

    s_stats.command_count++;
    ESP_LOGI(TAG, "CMD: %s (id=%" PRIu32 ")", cmd.cmd, cmd.id);

    /* Dispatch command */
    if (strcmp(cmd.cmd, "permit_join") == 0) {
        uint8_t duration = (cmd.duration > 0) ? (uint8_t)cmd.duration : 180;
        ret = zb_coordinator_permit_join(duration);

        char *resp = uart_proto_build_response(cmd.id,
                                                ret == ESP_OK ? "ok" : "error",
                                                ret == ESP_OK ? NULL : esp_err_to_name(ret));
        send_and_free(resp);

    } else if (strcmp(cmd.cmd, "network_info") == 0) {
        zb_network_info_t net_info = {0};
        zb_network_get_info(&net_info);

        char *resp = uart_proto_build_network_info(net_info.pan_id,
                                                    net_info.channel,
                                                    (uint8_t)zb_device_get_count());
        send_and_free(resp);

    } else if (strcmp(cmd.cmd, "device_list") == 0) {
        uart_bridge_publish_device_list();

        char *resp = uart_proto_build_response(cmd.id, "ok", NULL);
        send_and_free(resp);

    } else if (strcmp(cmd.cmd, "set") == 0) {
        /* Set attribute on a device */
        if (cmd.ieee[0] == '\0') {
            char *resp = uart_proto_build_response(cmd.id, "error", "missing ieee");
            send_and_free(resp);
        } else {
            uint8_t ieee_addr[8];
            uart_proto_str_to_ieee(cmd.ieee, ieee_addr);
            zb_device_t *device = zb_device_get_by_ieee(ieee_addr);

            if (device == NULL) {
                char *resp = uart_proto_build_response(cmd.id, "error", "device not found");
                send_and_free(resp);
            } else {
                /* For on/off cluster (0x0006), send on/off command */
                if (cmd.cluster == 0x0006 && cmd.attr == 0) {
                    bool on_off = (cmd.value_int != 0);
                    esp_zb_zcl_on_off_cmd_t on_off_cmd = {
                        .zcl_basic_cmd = {
                            .dst_addr_u.addr_short = device->short_addr,
                            .dst_endpoint = cmd.ep > 0 ? cmd.ep : 1,
                            .src_endpoint = 1,
                        },
                        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                        .on_off_cmd_id = on_off ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID
                                                : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
                    };
                    esp_zb_lock_acquire(portMAX_DELAY);
                    ret = esp_zb_zcl_on_off_cmd_req(&on_off_cmd);
                    esp_zb_lock_release();
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "On/off cmd queued (ret=%s, may be sleepy device)",
                                 esp_err_to_name(ret));
                        ret = ESP_OK;  /* Treat as success for battery devices */
                    }
                } else {
                    /* Generic write attribute */
                    ret = ESP_ERR_NOT_SUPPORTED;
                }

                char *resp = uart_proto_build_response(cmd.id,
                                                        ret == ESP_OK ? "ok" : "error",
                                                        ret == ESP_OK ? NULL : esp_err_to_name(ret));
                send_and_free(resp);
            }
        }

    } else if (strcmp(cmd.cmd, "get") == 0) {
        /* Read attribute from a device */
        if (cmd.ieee[0] == '\0') {
            char *resp = uart_proto_build_response(cmd.id, "error", "missing ieee");
            send_and_free(resp);
        } else {
            uint8_t ieee_addr[8];
            uart_proto_str_to_ieee(cmd.ieee, ieee_addr);
            zb_device_t *device = zb_device_get_by_ieee(ieee_addr);

            if (device == NULL) {
                char *resp = uart_proto_build_response(cmd.id, "error", "device not found");
                send_and_free(resp);
            } else {
                /* Send read attribute request */
                uint16_t attr_list[] = { cmd.attr };
                esp_zb_zcl_read_attr_cmd_t read_cmd = {
                    .zcl_basic_cmd = {
                        .dst_addr_u.addr_short = device->short_addr,
                        .dst_endpoint = cmd.ep > 0 ? cmd.ep : 1,
                        .src_endpoint = 1,
                    },
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                    .clusterID = cmd.cluster,
                    .attr_number = 1,
                    .attr_field = attr_list,
                };
                esp_zb_lock_acquire(portMAX_DELAY);
                ret = esp_zb_zcl_read_attr_cmd_req(&read_cmd);
                esp_zb_lock_release();

                char *resp = uart_proto_build_response(cmd.id,
                                                        ret == ESP_OK ? "ok" : "error",
                                                        ret == ESP_OK ? NULL : esp_err_to_name(ret));
                send_and_free(resp);
            }
        }

    } else if (strcmp(cmd.cmd, "tuya_set") == 0) {
        /* Set a Tuya DP on a device */
        /* Parse directly from JSON for dp, dp_type, value, str_value */
        cJSON *root = cJSON_Parse(json_line);
        if (root == NULL) {
            char *resp = uart_proto_build_response(cmd.id, "error", "json parse error");
            send_and_free(resp);
        } else {
            cJSON *ieee_j = cJSON_GetObjectItem(root, "ieee");
            cJSON *dp_j = cJSON_GetObjectItem(root, "dp");
            cJSON *dp_type_j = cJSON_GetObjectItem(root, "dp_type");
            cJSON *value_j = cJSON_GetObjectItem(root, "value");
            cJSON *str_value_j = cJSON_GetObjectItem(root, "str_value");

            if (!ieee_j || !dp_j || !dp_type_j) {
                char *resp = uart_proto_build_response(cmd.id, "error", "missing dp/dp_type/ieee");
                send_and_free(resp);
            } else {
                uint8_t ieee_addr[8];
                uart_proto_str_to_ieee(ieee_j->valuestring, ieee_addr);
                zb_device_t *device = zb_device_get_by_ieee(ieee_addr);

                if (device == NULL) {
                    char *resp = uart_proto_build_response(cmd.id, "error", "device not found");
                    send_and_free(resp);
                } else {
                    tuya_dp_t dp = {0};
                    dp.dp_id = (uint8_t)dp_j->valueint;
                    const char *type_str = dp_type_j->valuestring;

                    if (strcmp(type_str, "bool") == 0) {
                        dp.type = TUYA_DP_TYPE_BOOL;
                        dp.length = 1;
                        dp.value.bool_value = value_j ? (value_j->valueint != 0) : false;
                    } else if (strcmp(type_str, "value") == 0) {
                        dp.type = TUYA_DP_TYPE_VALUE;
                        dp.length = 4;
                        dp.value.int_value = value_j ? value_j->valueint : 0;
                    } else if (strcmp(type_str, "enum") == 0) {
                        dp.type = TUYA_DP_TYPE_ENUM;
                        dp.length = 1;
                        dp.value.enum_value = value_j ? (uint8_t)value_j->valueint : 0;
                    } else if (strcmp(type_str, "string") == 0) {
                        dp.type = TUYA_DP_TYPE_STRING;
                        const char *sv = str_value_j ? str_value_j->valuestring : "";
                        dp.length = strlen(sv);
                        if (dp.length > ZB_TUYA_DP_MAX_RAW_SIZE) dp.length = ZB_TUYA_DP_MAX_RAW_SIZE;
                        memcpy(dp.value.raw, sv, dp.length);
                    } else {
                        dp.type = TUYA_DP_TYPE_RAW;
                        dp.length = 1;
                        dp.value.raw[0] = value_j ? (uint8_t)value_j->valueint : 0;
                    }

                    ESP_LOGI(TAG, "Tuya SET: 0x%04X dp=%d type=%s val=%d",
                             device->short_addr, dp.dp_id, type_str,
                             value_j ? value_j->valueint : 0);

                    esp_zb_lock_acquire(portMAX_DELAY);
                    ret = zb_tuya_send_dp(device->short_addr, device->endpoint, &dp);
                    esp_zb_lock_release();

                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "Tuya DP send queued (ret=%s, may be sleepy device)",
                                 esp_err_to_name(ret));
                        ret = ESP_OK;  /* Treat as success for battery devices */
                    }

                    char *resp = uart_proto_build_response(cmd.id,
                                                            ret == ESP_OK ? "ok" : "error",
                                                            ret == ESP_OK ? NULL : esp_err_to_name(ret));
                    send_and_free(resp);
                }
            }
            cJSON_Delete(root);
        }

    } else if (strcmp(cmd.cmd, "device_cmd") == 0) {
        /* Forward a JSON payload to the bound device driver's handle_command */
        if (cmd.ieee[0] == '\0') {
            char *resp = uart_proto_build_response(cmd.id, "error", "missing ieee");
            send_and_free(resp);
        } else {
            cJSON *root = cJSON_Parse(json_line);
            if (root == NULL) {
                char *resp = uart_proto_build_response(cmd.id, "error", "json parse error");
                send_and_free(resp);
            } else {
                cJSON *payload_j = cJSON_GetObjectItem(root, "payload");
                if (!payload_j || !cJSON_IsString(payload_j)) {
                    cJSON_Delete(root);
                    char *resp = uart_proto_build_response(cmd.id, "error", "missing payload");
                    send_and_free(resp);
                } else {
                    uint8_t ieee_addr[8];
                    uart_proto_str_to_ieee(cmd.ieee, ieee_addr);
                    zb_device_t *device = zb_device_get_by_ieee(ieee_addr);

                    if (device == NULL) {
                        cJSON_Delete(root);
                        char *resp = uart_proto_build_response(cmd.id, "error", "device not found");
                        send_and_free(resp);
                    } else {
                        const tuya_device_driver_t *drv = tuya_driver_get(device->short_addr);
                        if (drv == NULL) {
                            /* Try on-demand binding */
                            drv = tuya_driver_find(device->manufacturer, device->model);
                            if (drv != NULL) {
                                tuya_driver_bind(device->short_addr, drv);
                                if (drv->init_device) drv->init_device(device->short_addr);
                            }
                        }

                        if (drv != NULL && drv->handle_command != NULL) {
                            cJSON *cmd_json = cJSON_Parse(payload_j->valuestring);
                            if (cmd_json != NULL) {
                                ret = drv->handle_command(device->short_addr,
                                                           device->endpoint, cmd_json);
                                cJSON_Delete(cmd_json);
                                ESP_LOGI(TAG, "Driver '%s' handled device_cmd for 0x%04X: %s",
                                         drv->name, device->short_addr,
                                         ret == ESP_OK ? "ok" : esp_err_to_name(ret));
                            } else {
                                ret = ESP_ERR_INVALID_ARG;
                            }
                        } else {
                            ret = ESP_ERR_NOT_FOUND;
                        }

                        cJSON_Delete(root);
                        char *resp = uart_proto_build_response(cmd.id,
                                                                ret == ESP_OK ? "ok" : "error",
                                                                ret == ESP_OK ? NULL : esp_err_to_name(ret));
                        send_and_free(resp);
                    }
                }
            }
        }

    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd.cmd);
        char *resp = uart_proto_build_response(cmd.id, "error", "unknown command");
        send_and_free(resp);
    }

    return ESP_OK;
}

/* ============================================================================
 * UART RX Task
 * ============================================================================ */

void uart_bridge_rx_task(void *pvParameters)
{
    static char rx_buf[UART_BRIDGE_BUF_SIZE];
    static char line_buf[UART_BRIDGE_BUF_SIZE];
    size_t line_pos = 0;

    ESP_LOGI(TAG, "UART RX task started");

    while (1) {
        int len = uart_read_bytes(UART_BRIDGE_PORT_NUM, (uint8_t *)rx_buf,
                                   sizeof(rx_buf) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        rx_buf[len] = '\0';

        /* Process byte-by-byte, looking for newline-terminated lines */
        for (int i = 0; i < len; i++) {
            char c = rx_buf[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    ESP_LOGD(TAG, "RX line: %s", line_buf);
                    uart_bridge_process_command(line_buf);
                    line_pos = 0;
                }
            } else {
                if (line_pos < sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = c;
                } else {
                    /* Line too long, discard */
                    ESP_LOGW(TAG, "RX line too long, discarding");
                    line_pos = 0;
                }
            }
        }
    }
}
