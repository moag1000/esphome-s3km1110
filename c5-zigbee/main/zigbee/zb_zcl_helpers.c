/**
 * @file zb_zcl_helpers.c
 * @brief Zigbee ZCL Command Helper Functions Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_zcl_helpers.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "ZB_ZCL_HELPERS";

/* ============================================================================
 * On/Off Command
 * ============================================================================ */

esp_err_t zb_zcl_send_on_off_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                           bool on)
{
    if (short_addr == 0xFFFF || endpoint == 0) {
        ESP_LOGE(TAG, "Invalid on/off command parameters: addr=0x%04X ep=%d",
                 short_addr, endpoint);
        return ESP_ERR_INVALID_ARG;
    }

    esp_zb_zcl_on_off_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .src_endpoint = 1,
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send on/off command to 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent on/off command to 0x%04X EP%d: %s",
                 short_addr, endpoint, on ? "ON" : "OFF");
    }

    return ret;
}

/* ============================================================================
 * Level Control Command
 * ============================================================================ */

esp_err_t zb_zcl_send_level_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                          uint8_t level, uint16_t transition_time)
{
    if (short_addr == 0xFFFF || endpoint == 0) {
        ESP_LOGE(TAG, "Invalid level command parameters: addr=0x%04X ep=%d",
                 short_addr, endpoint);
        return ESP_ERR_INVALID_ARG;
    }

    esp_zb_zcl_move_to_level_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .src_endpoint = 1,
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .level = level,
        .transition_time = transition_time,
    };

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_level_move_to_level_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send level command to 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent level command to 0x%04X EP%d: level=%d, time=%d",
                 short_addr, endpoint, level, transition_time);
    }

    return ret;
}

/* ============================================================================
 * Color Control Command
 * ============================================================================ */

esp_err_t zb_zcl_send_color_cmd_with_lock(uint16_t short_addr, uint8_t endpoint,
                                          uint16_t color_x, uint16_t color_y,
                                          uint16_t transition_time)
{
    if (short_addr == 0xFFFF || endpoint == 0) {
        ESP_LOGE(TAG, "Invalid color command parameters: addr=0x%04X ep=%d",
                 short_addr, endpoint);
        return ESP_ERR_INVALID_ARG;
    }

    esp_zb_zcl_color_move_to_color_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .src_endpoint = 1,
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .color_x = color_x,
        .color_y = color_y,
        .transition_time = transition_time,
    };

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_color_move_to_color_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send color command to 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent color command to 0x%04X EP%d: x=%d, y=%d, time=%d",
                 short_addr, endpoint, color_x, color_y, transition_time);
    }

    return ret;
}

/* ============================================================================
 * Read Attributes Command
 * ============================================================================ */

esp_err_t zb_zcl_read_attr_with_lock(uint16_t short_addr, uint8_t endpoint,
                                     uint16_t cluster_id, const uint16_t *attr_ids,
                                     uint16_t attr_count)
{
    if (short_addr == 0xFFFF || endpoint == 0 || attr_ids == NULL || attr_count == 0) {
        ESP_LOGE(TAG, "Invalid read attr parameters: addr=0x%04X ep=%d count=%d",
                 short_addr, endpoint, attr_count);
        return ESP_ERR_INVALID_ARG;
    }

    /* Build attribute list */
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .src_endpoint = 1,
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
        .attr_number = attr_count,
    };

    /* Note: attr_field must be set to point to attr_ids array, but ESP-Zigbee
     * SDK expects this to be mutable for its internal use, so we cast away const.
     * This is safe because the SDK only reads during command formation.
     */
    read_cmd.attr_field = (uint16_t *)attr_ids;

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_read_attr_cmd_req(&read_cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read attr command to 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent read attr command to 0x%04X EP%d cluster=0x%04X attrs=%d",
                 short_addr, endpoint, cluster_id, attr_count);
    }

    return ret;
}

/* ============================================================================
 * Write Attributes Command
 * ============================================================================ */

esp_err_t zb_zcl_write_attr_with_lock(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t cluster_id,
                                      esp_zb_zcl_attribute_t *attr_field,
                                      uint16_t attr_count)
{
    if (short_addr == 0xFFFF || endpoint == 0 || attr_field == NULL || attr_count == 0) {
        ESP_LOGE(TAG, "Invalid write attr parameters: addr=0x%04X ep=%d count=%d",
                 short_addr, endpoint, attr_count);
        return ESP_ERR_INVALID_ARG;
    }

    esp_zb_zcl_write_attr_cmd_t write_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .src_endpoint = 1,
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
        .attr_number = attr_count,
        .attr_field = attr_field,
    };

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_write_attr_cmd_req(&write_cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send write attr command to 0x%04X: %s",
                 short_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent write attr command to 0x%04X EP%d cluster=0x%04X attrs=%d",
                 short_addr, endpoint, cluster_id, attr_count);
    }

    return ret;
}

/* ============================================================================
 * Generic ZCL Command
 * ============================================================================ */

esp_err_t zb_zcl_send_cmd_with_lock(uint16_t cluster_id, uint8_t cmd_id,
                                    void *payload)
{
    if (payload == NULL) {
        ESP_LOGE(TAG, "Invalid generic command parameters: payload=NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req((esp_zb_zcl_custom_cluster_cmd_t *)payload);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send custom ZCL command cluster=0x%04X cmd=0x%02X: %s",
                 cluster_id, cmd_id, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Sent custom ZCL command cluster=0x%04X cmd=0x%02X",
                 cluster_id, cmd_id);
    }

    return ret;
}

/* ============================================================================
 * Module Initialization
 * ============================================================================ */

esp_err_t zb_zcl_helpers_init(void)
{
    ESP_LOGI(TAG, "ZCL helpers module initialized");
    return ESP_OK;
}
