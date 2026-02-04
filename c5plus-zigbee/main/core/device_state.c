/**
 * @file device_state.c
 * @brief Device State to JSON Conversion
 */

#include "device_state.h"
#include "uart/uart_bridge.h"
#include "uart/uart_protocol.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DEV_STATE";

cJSON *device_state_to_cjson(const zb_device_t *device)
{
    if (device == NULL) return NULL;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    /* IEEE address */
    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));
    cJSON_AddStringToObject(root, "ieee", ieee_str);

    /* Short address */
    char addr_str[8];
    snprintf(addr_str, sizeof(addr_str), "0x%04X", device->short_addr);
    cJSON_AddStringToObject(root, "short_addr", addr_str);

    /* Device info */
    if (device->manufacturer[0]) {
        cJSON_AddStringToObject(root, "manufacturer", device->manufacturer);
    }
    if (device->model[0]) {
        cJSON_AddStringToObject(root, "model", device->model);
    }
    if (device->friendly_name[0]) {
        cJSON_AddStringToObject(root, "friendly_name", device->friendly_name);
    }

    /* Online status */
    cJSON_AddBoolToObject(root, "online", device->online);

    /* Device type */
    cJSON_AddNumberToObject(root, "device_type", device->device_type);

    /* Clusters */
    if (device->cluster_count > 0) {
        cJSON *clusters = cJSON_AddArrayToObject(root, "clusters");
        if (clusters) {
            for (uint8_t i = 0; i < device->cluster_count; i++) {
                cJSON_AddItemToArray(clusters, cJSON_CreateNumber(device->clusters[i]));
            }
        }
    }

    /* Link quality */
    if (device->link_quality > 0) {
        cJSON_AddNumberToObject(root, "link_quality", device->link_quality);
    }

    /* RSSI */
    if (device->rssi != 0) {
        cJSON_AddNumberToObject(root, "rssi", device->rssi);
    }

    /* Last seen */
    if (device->last_seen > 0) {
        cJSON_AddNumberToObject(root, "last_seen", (double)device->last_seen);
    }

    /* Endpoint */
    cJSON_AddNumberToObject(root, "endpoint", device->endpoint);

    /* Battery (if available via power info) */
    if (device->power_info.power_info_valid) {
        if (device->power_info.current_power_source & 0x04) {
            cJSON_AddNumberToObject(root, "battery",
                                     device->power_info.current_power_source_level);
        }
    }

    return root;
}

char *device_state_to_json_str(const zb_device_t *device)
{
    cJSON *json = device_state_to_cjson(device);
    if (json == NULL) return NULL;

    char *str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    return str;
}

esp_err_t device_state_publish_by_addr(uint16_t short_addr)
{
    return uart_bridge_publish_device_state(short_addr);
}

esp_err_t device_state_publish_multistate(uint16_t short_addr, uint8_t endpoint,
                                           const zb_multistate_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) return ESP_ERR_NOT_FOUND;

    char ieee_str[20];
    uart_proto_ieee_to_str(device->ieee_addr, ieee_str, sizeof(ieee_str));

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "state", state->present_value);
    cJSON_AddNumberToObject(root, "states_count", state->number_of_states);
    cJSON_AddBoolToObject(root, "out_of_service", state->out_of_service);

    char *msg = uart_proto_build_device_state(ieee_str, root);
    cJSON_Delete(root);

    if (msg == NULL) return ESP_ERR_NO_MEM;

    esp_err_t ret = uart_bridge_send_line(msg);
    free(msg);
    return ret;
}
