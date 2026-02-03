/**
 * @file uart_protocol.c
 * @brief UART JSON Lines Protocol Encoder/Decoder
 */

#include "uart_protocol.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "UART_PROTO";

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

void uart_proto_ieee_to_str(const uint8_t *ieee_addr, char *buf, size_t buf_len)
{
    if (buf_len < 17) {
        buf[0] = '\0';
        return;
    }
    snprintf(buf, buf_len, "%02X%02X%02X%02X%02X%02X%02X%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
}

esp_err_t uart_proto_str_to_ieee(const char *str, uint8_t *ieee_addr)
{
    if (str == NULL || strlen(str) != 16 || ieee_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < 8; i++) {
        char byte_str[3] = { str[i * 2], str[i * 2 + 1], '\0' };
        ieee_addr[7 - i] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    return ESP_OK;
}

/* Helper to create JSON string and return it */
static char *json_to_string_and_free(cJSON *json)
{
    if (json == NULL) {
        return NULL;
    }
    char *str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return str;
}

/* ============================================================================
 * Outgoing Messages (C5 → S3)
 * ============================================================================ */

char *uart_proto_build_ready(uint16_t pan_id, uint8_t channel)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "ready");
    cJSON_AddStringToObject(root, "version", "1.0");

    cJSON *network = cJSON_AddObjectToObject(root, "network");
    if (network) {
        char pan_str[8];
        snprintf(pan_str, sizeof(pan_str), "0x%04X", pan_id);
        cJSON_AddStringToObject(network, "pan_id", pan_str);
        cJSON_AddNumberToObject(network, "channel", channel);
    }

    return json_to_string_and_free(root);
}

char *uart_proto_build_device_joined(const char *ieee, uint16_t short_addr)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "device_joined");
    cJSON_AddStringToObject(root, "ieee", ieee);

    char addr_str[8];
    snprintf(addr_str, sizeof(addr_str), "0x%04X", short_addr);
    cJSON_AddStringToObject(root, "short_addr", addr_str);

    return json_to_string_and_free(root);
}

char *uart_proto_build_device_left(const char *ieee)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "device_left");
    cJSON_AddStringToObject(root, "ieee", ieee);

    return json_to_string_and_free(root);
}

char *uart_proto_build_interview_done(const char *ieee, const char *model,
                                       const char *manufacturer,
                                       uint8_t endpoint_count)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "interview_done");
    cJSON_AddStringToObject(root, "ieee", ieee);
    if (model) cJSON_AddStringToObject(root, "model", model);
    if (manufacturer) cJSON_AddStringToObject(root, "manufacturer", manufacturer);
    cJSON_AddNumberToObject(root, "endpoints", endpoint_count);

    return json_to_string_and_free(root);
}

char *uart_proto_build_attribute_report(const char *ieee, uint8_t ep,
                                         uint16_t cluster, uint16_t attr,
                                         double value)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "attribute");
    cJSON_AddStringToObject(root, "ieee", ieee);
    cJSON_AddNumberToObject(root, "ep", ep);
    cJSON_AddNumberToObject(root, "cluster", cluster);
    cJSON_AddNumberToObject(root, "attr", attr);
    cJSON_AddNumberToObject(root, "value", value);

    return json_to_string_and_free(root);
}

char *uart_proto_build_attribute_str_report(const char *ieee, uint8_t ep,
                                             uint16_t cluster, uint16_t attr,
                                             const char *value)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "attribute");
    cJSON_AddStringToObject(root, "ieee", ieee);
    cJSON_AddNumberToObject(root, "ep", ep);
    cJSON_AddNumberToObject(root, "cluster", cluster);
    cJSON_AddNumberToObject(root, "attr", attr);
    cJSON_AddStringToObject(root, "value", value ? value : "");

    return json_to_string_and_free(root);
}

char *uart_proto_build_response(uint32_t id, const char *status,
                                 const char *error_msg)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "response");
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddStringToObject(root, "status", status);
    if (error_msg) {
        cJSON_AddStringToObject(root, "error", error_msg);
    }

    return json_to_string_and_free(root);
}

char *uart_proto_build_device_state(const char *ieee, const cJSON *state_json)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "device_state");
    cJSON_AddStringToObject(root, "ieee", ieee);
    if (state_json) {
        cJSON *state_copy = cJSON_Duplicate(state_json, true);
        if (state_copy) {
            cJSON_AddItemToObject(root, "state", state_copy);
        }
    }

    return json_to_string_and_free(root);
}

char *uart_proto_build_network_info(uint16_t pan_id, uint8_t channel,
                                     uint8_t device_count)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "network_info");

    char pan_str[8];
    snprintf(pan_str, sizeof(pan_str), "0x%04X", pan_id);
    cJSON_AddStringToObject(root, "pan_id", pan_str);
    cJSON_AddNumberToObject(root, "channel", channel);
    cJSON_AddNumberToObject(root, "device_count", device_count);

    return json_to_string_and_free(root);
}

char *uart_proto_build_permit_join(bool enabled, uint8_t duration)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "type", "permit_join");
    cJSON_AddBoolToObject(root, "enabled", enabled);
    cJSON_AddNumberToObject(root, "duration", duration);

    return json_to_string_and_free(root);
}

/* ============================================================================
 * Incoming Command Parsing (S3 → C5)
 * ============================================================================ */

esp_err_t uart_proto_parse_command(const char *json_line, uart_command_t *cmd)
{
    if (json_line == NULL || cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(cmd, 0, sizeof(uart_command_t));

    cJSON *root = cJSON_Parse(json_line);
    if (root == NULL) {
        ESP_LOGW(TAG, "Failed to parse JSON: %.64s", json_line);
        return ESP_ERR_INVALID_ARG;
    }

    /* Required: cmd */
    cJSON *cmd_field = cJSON_GetObjectItem(root, "cmd");
    if (cmd_field == NULL || !cJSON_IsString(cmd_field)) {
        ESP_LOGW(TAG, "Missing 'cmd' field in command");
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }
    strlcpy(cmd->cmd, cmd_field->valuestring, sizeof(cmd->cmd));

    /* Optional: id */
    cJSON *id_field = cJSON_GetObjectItem(root, "id");
    if (id_field && cJSON_IsNumber(id_field)) {
        cmd->id = (uint32_t)id_field->valuedouble;
    }

    /* Optional: ieee */
    cJSON *ieee_field = cJSON_GetObjectItem(root, "ieee");
    if (ieee_field && cJSON_IsString(ieee_field)) {
        strlcpy(cmd->ieee, ieee_field->valuestring, sizeof(cmd->ieee));
    }

    /* Optional: ep */
    cJSON *ep_field = cJSON_GetObjectItem(root, "ep");
    if (ep_field && cJSON_IsNumber(ep_field)) {
        cmd->ep = (uint8_t)ep_field->valuedouble;
    }

    /* Optional: cluster */
    cJSON *cluster_field = cJSON_GetObjectItem(root, "cluster");
    if (cluster_field && cJSON_IsNumber(cluster_field)) {
        cmd->cluster = (uint16_t)cluster_field->valuedouble;
    }

    /* Optional: attr */
    cJSON *attr_field = cJSON_GetObjectItem(root, "attr");
    if (attr_field && cJSON_IsNumber(attr_field)) {
        cmd->attr = (uint16_t)attr_field->valuedouble;
    }

    /* Optional: value (can be number or string) */
    cJSON *value_field = cJSON_GetObjectItem(root, "value");
    if (value_field) {
        cmd->has_value = true;
        if (cJSON_IsNumber(value_field)) {
            cmd->value_double = value_field->valuedouble;
            cmd->value_int = (int32_t)value_field->valuedouble;
        } else if (cJSON_IsString(value_field)) {
            strlcpy(cmd->value_str, value_field->valuestring, sizeof(cmd->value_str));
        } else if (cJSON_IsBool(value_field)) {
            cmd->value_int = cJSON_IsTrue(value_field) ? 1 : 0;
            cmd->value_double = cmd->value_int;
        }
    }

    /* Optional: duration */
    cJSON *duration_field = cJSON_GetObjectItem(root, "duration");
    if (duration_field && cJSON_IsNumber(duration_field)) {
        cmd->duration = (uint16_t)duration_field->valuedouble;
    }

    cJSON_Delete(root);
    return ESP_OK;
}
