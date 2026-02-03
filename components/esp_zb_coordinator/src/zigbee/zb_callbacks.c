/**
 * @file zb_callbacks.c
 * @brief Zigbee Event Callbacks Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_callbacks.h"
#include "zb_constants.h"
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "zb_device_handler_internal.h"
#include "zb_network.h"
#include "zb_availability.h"
#include "zb_interview.h"
#include "zb_install_codes.h"
#include "zb_reporting.h"
#include "zb_alarms.h"
#include "zb_demand_response.h"
#include "zb_hvac_dehumid.h"
#include "zb_poll_control.h"
#include "zb_tuya.h"
#include "zb_cmd_retry.h"
#include "tuya_driver_registry.h"
#include "tuya_device_driver.h"
#include "uart_bridge.h"
#include "uart/uart_protocol.h"
#include "device_state.h"
#include "esp_log.h"
#include "aps/esp_zigbee_aps.h"  /* API-005: APS Authentication State */
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"

/* LED status integration via HAL (chip-agnostic) */
#include "zb_hal.h"

static const char *TAG = "ZB_CB";

/* Forward declarations for internal helpers */
static esp_err_t zb_coordinator_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                                const void *message);
static const char* zdo_status_to_str(esp_zb_zdp_status_t status);
static void handle_basic_cluster_report(uint16_t short_addr, esp_zb_zcl_report_attr_message_t *msg);
static void handle_onoff_cluster_report(uint16_t short_addr, esp_zb_zcl_report_attr_message_t *msg);

/**
 * @brief Apply install code to Zigbee stack if entry exists
 *
 * Looks up install code for the device and applies it to the stack
 * for secure joining.
 *
 * @param ieee64 Device IEEE address as uint64_t
 * @param short_addr Device short address (for logging)
 * @return ESP_OK if applied, ESP_ERR_NOT_FOUND if no entry, other errors on failure
 */
static esp_err_t apply_install_code_if_present(uint64_t ieee64, uint16_t short_addr)
{
    if (!zb_install_codes_has_entry(ieee64)) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Device 0x%04X has install code, applying derived key", short_addr);
    esp_err_t ret = zb_install_codes_apply_to_stack(ieee64);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Install code applied successfully for 0x%016" PRIX64, ieee64);
    } else {
        ESP_LOGW(TAG, "Failed to apply install code: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Coordinator Action Handler for ZCL commands (ESP-Zigbee-SDK v1.6.x+)
 *
 * This is the main callback for handling ZCL commands and responses.
 * Required for proper coordinator operation with the new SDK API.
 *
 * @param callback_id Action callback type ID
 * @param message Callback-specific message data
 * @return ESP_OK on success
 */
static esp_err_t zb_coordinator_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                                const void *message)
{
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
            esp_zb_zcl_set_attr_value_message_t *msg =
                (esp_zb_zcl_set_attr_value_message_t *)message;
            if (msg != NULL) {
                ESP_LOGI(TAG, "Set attr: ep=%d cluster=0x%04x attr=0x%04x",
                         msg->info.dst_endpoint, msg->info.cluster, msg->attribute.id);

                /* Update coordinator statistics */
                zb_coordinator_update_rx_count();
            }
            return ESP_OK;
        }

        case ESP_ZB_CORE_REPORT_ATTR_CB_ID: {
            esp_zb_zcl_report_attr_message_t *msg =
                (esp_zb_zcl_report_attr_message_t *)message;
            if (msg != NULL) {
                cmd_retry_confirm(msg->src_address.u.short_addr);
                zb_callback_report_attr(msg);
                zb_coordinator_update_rx_count();
            }
            return ESP_OK;
        }

        case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID: {
            /* Handle read attributes response - forward to interview module for Basic cluster */
            esp_zb_zcl_cmd_read_attr_resp_message_t *msg =
                (esp_zb_zcl_cmd_read_attr_resp_message_t *)message;
            if (msg != NULL) {
                uint16_t short_addr = msg->info.src_address.u.short_addr;
                uint16_t cluster_id = msg->info.cluster;

                cmd_retry_confirm(short_addr);

                ESP_LOGD(TAG, "ZCL Read Attributes Response from 0x%04X cluster=0x%04X",
                         short_addr, cluster_id);

                /* Process each variable in the response list */
                esp_zb_zcl_read_attr_resp_variable_t *var = msg->variables;
                while (var != NULL) {
                    ESP_LOGD(TAG, "  attr=0x%04X status=0x%02X type=0x%02X",
                             var->attribute.id, var->status, var->attribute.data.type);

                    /* Forward to interview module for Basic cluster attributes */
                    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
                        zb_interview_handle_read_attr_resp(short_addr, cluster_id,
                                                           var->attribute.id,
                                                           var->status,
                                                           var->attribute.data.type,
                                                           var->attribute.data.value,
                                                           var->attribute.data.size);
                    }

                    /* Also handle manufacturer/model reports for non-interview scenarios */
                    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC &&
                        var->status == ESP_ZB_ZCL_STATUS_SUCCESS &&
                        var->attribute.data.value != NULL) {
                        esp_zb_zcl_report_attr_message_t report = {
                            .src_address.u.short_addr = short_addr,
                            .cluster = cluster_id,
                            .attribute.id = var->attribute.id,
                            .attribute.data = var->attribute.data
                        };
                        handle_basic_cluster_report(short_addr, &report);
                    }

                    var = var->next;
                }
            }
            zb_coordinator_update_rx_count();
            return ESP_OK;
        }

        case ESP_ZB_CORE_CMD_WRITE_ATTR_RESP_CB_ID: {
            ESP_LOGD(TAG, "ZCL Write Attributes Response received");
            zb_coordinator_update_rx_count();
            return ESP_OK;
        }

        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID: {
            esp_zb_zcl_cmd_default_resp_message_t *msg =
                (esp_zb_zcl_cmd_default_resp_message_t *)message;
            if (msg != NULL) {
                uint16_t src_addr = msg->info.src_address.u.short_addr;
                cmd_retry_confirm(src_addr);
                if (msg->status_code != 0) {
                    ESP_LOGW(TAG, "ZCL Default Response: FAILED status=0x%02x cmd=0x%02x from 0x%04X",
                             msg->status_code, msg->resp_to_cmd, src_addr);
                    /* Forward command failure to UART */
                    if (uart_bridge_is_enabled()) {
                        zb_device_t *fail_dev = zb_device_get(src_addr);
                        char ieee_str[20] = "unknown";
                        if (fail_dev) {
                            uart_proto_ieee_to_str(fail_dev->ieee_addr, ieee_str, sizeof(ieee_str));
                        }
                        char json_buf[160];
                        snprintf(json_buf, sizeof(json_buf),
                                 "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\",\"status\":\"0x%02X\",\"cmd\":\"0x%02X\"}",
                                 ieee_str, src_addr, msg->status_code, msg->resp_to_cmd);
                        uart_bridge_publish_event("command_failed", json_buf);
                    }
                } else {
                    ESP_LOGI(TAG, "ZCL Default Response: OK cmd=0x%02x from 0x%04X",
                             msg->resp_to_cmd, src_addr);
                }
            }
            return ESP_OK;
        }

        case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
            /* Handle custom cluster commands including Alarms cluster (0x0009) */
            esp_zb_zcl_custom_cluster_command_message_t *msg =
                (esp_zb_zcl_custom_cluster_command_message_t *)message;
            if (msg != NULL) {
                uint16_t short_addr = msg->info.src_address.u.short_addr;
                uint8_t endpoint = msg->info.src_endpoint;
                uint16_t cluster_id = msg->info.cluster;
                uint8_t cmd_id = msg->info.command.id;

                ESP_LOGD(TAG, "Custom cluster cmd from 0x%04X ep=%d cluster=0x%04X cmd=0x%02X",
                         short_addr, endpoint, cluster_id, cmd_id);

                /* Handle Alarms cluster (0x0009) commands */
                if (cluster_id == ZB_ZCL_CLUSTER_ID_ALARMS) {
                    if (cmd_id == ZB_ZCL_CMD_ALARMS_ALARM_ID) {
                        /* Alarm notification: parse alarm_code (uint8) + cluster_id (uint16) */
                        if (msg->data.size >= 3) {
                            uint8_t alarm_code = ((uint8_t *)msg->data.value)[0];
                            uint16_t alarm_cluster = ((uint8_t *)msg->data.value)[1] |
                                                     (((uint8_t *)msg->data.value)[2] << 8);
                            ESP_LOGI(TAG, "Alarm notification: device=0x%04X code=%d cluster=0x%04X",
                                     short_addr, alarm_code, alarm_cluster);
                            zb_alarms_handle_notification(short_addr, endpoint, alarm_code, alarm_cluster);

                            /* Forward alarm to UART */
                            if (uart_bridge_is_enabled()) {
                                zb_device_t *alarm_dev = zb_device_get(short_addr);
                                char ieee_str[20] = "unknown";
                                if (alarm_dev) {
                                    uart_proto_ieee_to_str(alarm_dev->ieee_addr, ieee_str, sizeof(ieee_str));
                                }
                                char json_buf[160];
                                snprintf(json_buf, sizeof(json_buf),
                                         "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\",\"alarm_code\":%d,\"cluster\":\"0x%04X\"}",
                                         ieee_str, short_addr, alarm_code, alarm_cluster);
                                uart_bridge_publish_event("alarm", json_buf);
                            }
                        }
                    } else if (cmd_id == ZB_ZCL_CMD_ALARMS_GET_ALARM_RESPONSE_ID) {
                        /* Get Alarm Response: parse status, alarm_code, cluster_id, timestamp */
                        if (msg->data.size >= 1) {
                            uint8_t status = ((uint8_t *)msg->data.value)[0];
                            uint8_t alarm_code = 0;
                            uint16_t alarm_cluster = 0;
                            uint32_t timestamp = 0;

                            if (status == 0x00 && msg->data.size >= 8) {
                                alarm_code = ((uint8_t *)msg->data.value)[1];
                                alarm_cluster = ((uint8_t *)msg->data.value)[2] |
                                                (((uint8_t *)msg->data.value)[3] << 8);
                                timestamp = ((uint8_t *)msg->data.value)[4] |
                                            (((uint8_t *)msg->data.value)[5] << 8) |
                                            (((uint8_t *)msg->data.value)[6] << 16) |
                                            (((uint8_t *)msg->data.value)[7] << 24);
                            }
                            zb_alarms_handle_get_response(short_addr, endpoint, status,
                                                          alarm_code, alarm_cluster, timestamp);
                        }
                    }
                }

                /* Handle Demand Response and Load Control cluster (0x0701) commands */
                if (cluster_id == ZB_ZCL_CLUSTER_ID_DEMAND_RESPONSE) {
                    if (cmd_id == ZB_ZCL_CMD_DRLC_LOAD_CONTROL_EVENT_ID) {
                        /* LoadControlEvent: parse full event structure */
                        if (msg->data.size >= ZB_EVENT_MESSAGE_MIN_SIZE) {  /* Minimum event size */
                            uint8_t *data = (uint8_t *)msg->data.value;
                            zb_load_control_event_t event = {0};

                            /* Parse event fields (little-endian) */
                            event.issuer_event_id = data[0] | (data[1] << 8) |
                                                    (data[2] << 16) | (data[3] << 24);
                            event.device_class = data[4] | (data[5] << 8);
                            event.utility_enrollment_group = data[6];
                            event.start_time = data[7] | (data[8] << 8) |
                                              (data[9] << 16) | (data[10] << 24);
                            event.duration_minutes = data[11] | (data[12] << 8);
                            event.criticality_level = data[13];
                            event.cooling_temp_offset = data[14];
                            event.heating_temp_offset = data[15];
                            event.cooling_temp_setpoint = (int16_t)(data[16] | (data[17] << 8));
                            event.heating_temp_setpoint = (int16_t)(data[18] | (data[19] << 8));
                            event.average_load_adjustment = (int8_t)data[20];
                            event.duty_cycle = data[21];
                            event.event_control = data[22];

                            ESP_LOGI(TAG, "DRLC LoadControlEvent: id=0x%08" PRIX32 " class=0x%04X",
                                     event.issuer_event_id, event.device_class);
                            zb_demand_response_handle_event(&event);
                        }
                    } else if (cmd_id == ZB_ZCL_CMD_DRLC_CANCEL_LOAD_CONTROL_EVENT_ID) {
                        /* CancelLoadControlEvent */
                        if (msg->data.size >= 8) {
                            uint8_t *data = (uint8_t *)msg->data.value;
                            uint32_t issuer_event_id = data[0] | (data[1] << 8) |
                                                        (data[2] << 16) | (data[3] << 24);
                            uint16_t device_class = data[4] | (data[5] << 8);
                            uint8_t enrollment_group = data[6];
                            uint8_t cancel_control = data[7];

                            ESP_LOGI(TAG, "DRLC CancelLoadControlEvent: id=0x%08" PRIX32,
                                     issuer_event_id);
                            zb_demand_response_handle_cancel_event(issuer_event_id, device_class,
                                                                    enrollment_group, cancel_control);
                        }
                    } else if (cmd_id == ZB_ZCL_CMD_DRLC_CANCEL_ALL_LOAD_CONTROL_ID) {
                        /* CancelAllLoadControlEvents */
                        uint8_t cancel_control = 0;
                        if (msg->data.size >= 1) {
                            cancel_control = ((uint8_t *)msg->data.value)[0];
                        }
                        ESP_LOGI(TAG, "DRLC CancelAllLoadControlEvents");
                        zb_demand_response_handle_cancel_all(cancel_control);
                    }
                }

                /* Handle Poll Control cluster (0x0020) commands */
                if (cluster_id == ZB_ZCL_CLUSTER_ID_POLL_CONTROL) {
                    if (cmd_id == ZB_POLL_CMD_CHECK_IN) {
                        /* Check-in from sleepy device */
                        ESP_LOGI(TAG, "Poll Control Check-in from 0x%04X EP%d",
                                 short_addr, endpoint);
                        zb_poll_control_handle_check_in(short_addr, endpoint);

                        /* Forward check-in to UART so S3 knows device is alive */
                        if (uart_bridge_is_enabled()) {
                            zb_device_t *poll_dev = zb_device_get(short_addr);
                            if (poll_dev) {
                                char ieee_str[20];
                                uart_proto_ieee_to_str(poll_dev->ieee_addr, ieee_str, sizeof(ieee_str));
                                char json_buf[128];
                                snprintf(json_buf, sizeof(json_buf),
                                         "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\",\"ep\":%d}",
                                         ieee_str, short_addr, endpoint);
                                uart_bridge_publish_event("poll_check_in", json_buf);
                            }
                        }

                        /* Update availability — device is definitely online */
                        zb_device_update_last_seen(short_addr);
                        zb_availability_update_last_seen(short_addr);
                    }
                }

                /* Handle Tuya private cluster (0xEF00) - Datapoint protocol */
                if (cluster_id == ZB_TUYA_CLUSTER_ID) {
                    ESP_LOGI(TAG, "Tuya cmd 0x%02X from 0x%04X EP%d len=%zu",
                             cmd_id, short_addr, endpoint, msg->data.size);
                    cmd_retry_confirm(short_addr);
                    zb_tuya_handle_command(short_addr, endpoint, cmd_id,
                                           (const uint8_t *)msg->data.value,
                                           msg->data.size);
                }

                zb_coordinator_update_rx_count();
            }
            return ESP_OK;
        }

        /* BC-002: ESP-Zigbee-SDK v1.6.x Reporting Response Callbacks */
        case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID: {
            /* Configure Reporting Response (v1.6.x struct: esp_zb_zcl_cmd_config_report_resp_message_t) */
            esp_zb_zcl_cmd_config_report_resp_message_t *msg =
                (esp_zb_zcl_cmd_config_report_resp_message_t *)message;
            if (msg != NULL) {
                uint16_t short_addr = msg->info.src_address.u.short_addr;
                uint8_t endpoint = msg->info.src_endpoint;
                uint16_t cluster_id = msg->info.cluster;

                ESP_LOGI(TAG, "Configure Reporting Response from 0x%04X ep=%d cluster=0x%04X",
                         short_addr, endpoint, cluster_id);

                /* Process each variable in the response list */
                esp_zb_zcl_config_report_resp_variable_t *var = msg->variables;
                while (var != NULL) {
                    ESP_LOGD(TAG, "  attr=0x%04X status=0x%02X direction=%d",
                             var->attribute_id, var->status, var->direction);

                    /* Forward to reporting module */
                    zb_reporting_handle_configure_response(short_addr, endpoint,
                                                            cluster_id, var->status,
                                                            var->attribute_id);
                    var = var->next;
                }
                zb_coordinator_update_rx_count();
            }
            return ESP_OK;
        }

        case ESP_ZB_CORE_CMD_READ_REPORT_CFG_RESP_CB_ID: {
            /* Read Reporting Configuration Response (v1.6.x struct: esp_zb_zcl_cmd_read_report_config_resp_message_t) */
            esp_zb_zcl_cmd_read_report_config_resp_message_t *msg =
                (esp_zb_zcl_cmd_read_report_config_resp_message_t *)message;
            if (msg != NULL) {
                uint16_t short_addr = msg->info.src_address.u.short_addr;
                uint8_t endpoint = msg->info.src_endpoint;
                uint16_t cluster_id = msg->info.cluster;

                ESP_LOGI(TAG, "Read Reporting Config Response from 0x%04X ep=%d cluster=0x%04X",
                         short_addr, endpoint, cluster_id);

                /* Process each variable in the response list */
                esp_zb_zcl_read_report_config_resp_variable_t *var = msg->variables;
                while (var != NULL) {
                    if (var->status == ESP_ZB_ZCL_STATUS_SUCCESS &&
                        var->report_direction == ZCL_REPORTING_DIRECTION_REPORTED) {
                        /* Extract reportable_change based on attribute type size */
                        uint16_t reportable_change = 0;
                        if (var->client.delta[0] != 0) {
                            /* Simple extraction for common types (1-2 bytes) */
                            reportable_change = var->client.delta[0];
                        }

                        ESP_LOGD(TAG, "  attr=0x%04X type=0x%02X min=%d max=%d change=%d",
                                 var->attribute_id, var->client.attr_type,
                                 var->client.min_interval, var->client.max_interval,
                                 reportable_change);

                        /* Forward to reporting module */
                        zb_reporting_handle_read_response(short_addr, endpoint,
                                                           cluster_id, var->status,
                                                           var->report_direction,
                                                           var->attribute_id,
                                                           var->client.attr_type,
                                                           var->client.min_interval,
                                                           var->client.max_interval,
                                                           reportable_change);
                    } else {
                        ESP_LOGD(TAG, "  attr=0x%04X status=0x%02X (failed or server direction)",
                                 var->attribute_id, var->status);

                        /* Forward failure to reporting module */
                        zb_reporting_handle_read_response(short_addr, endpoint,
                                                           cluster_id, var->status,
                                                           var->report_direction,
                                                           var->attribute_id,
                                                           0, 0, 0, 0);
                    }
                    var = var->next;
                }
                zb_coordinator_update_rx_count();
            }
            return ESP_OK;
        }

        default:
            ESP_LOGD(TAG, "Unhandled action callback: 0x%04x", callback_id);
            return ESP_OK;
    }
}

esp_err_t zb_callbacks_init(void)
{
    /* Register the coordinator action handler for ZCL command processing */
    esp_zb_core_action_handler_register(zb_coordinator_action_handler);

    ESP_LOGI(TAG, "Callback handlers initialized (action handler registered)");
    return ESP_OK;
}

void zb_callback_device_join(esp_zb_zdp_status_t zdo_status, uint16_t short_addr,
                              esp_zb_ieee_addr_t ieee_addr)
{
    if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Device join failed: addr=0x%04X, status=%s",
                 short_addr, zdo_status_to_str(zdo_status));
        return;
    }

    ESP_LOGI(TAG, "Device joined: 0x%04X [%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X]",
             short_addr,
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);

    /* API-005: APS Authentication State Check
     *
     * Note: esp_zb_aps_is_authenticated() in ESP-Zigbee-SDK v1.6.x only checks
     * the local device's (coordinator's) authentication state, not per-device
     * authentication. For per-device auth state, a custom implementation using
     * the APS key exchange mechanism would be needed.
     *
     * The esp_zb_aps_set_authenticated(bool) / esp_zb_aps_is_authenticated()
     * APIs are for the local device's APS layer state management.
     */
    bool coordinator_authenticated = esp_zb_aps_is_authenticated();
    ESP_LOGI(TAG, "Device 0x%04X joined, coordinator APS auth state: %s",
             short_addr, coordinator_authenticated ? "authenticated" : "not authenticated");

    /* Convert IEEE address to 64-bit for install code lookup and bridge events */
    uint64_t ieee64 = zb_ieee_to_u64(ieee_addr);

    /* Check if we have an install code for this device */
    apply_install_code_if_present(ieee64, short_addr);

    /* Add device to registry */
    esp_err_t ret = zb_device_add(ieee_addr, short_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to registry: %s", esp_err_to_name(ret));
        return;
    }

    /* Update network device count */
    size_t device_count = zb_device_get_count();
    zb_network_set_device_count((uint8_t)device_count);

    ESP_LOGI(TAG, "Total devices: %d", device_count);

    /* Add device to availability tracking
     * Assume mains-powered initially; will be updated during interview
     * when we read the Basic cluster PowerSource attribute */
    zb_availability_add_device(short_addr, ZB_AVAIL_POWER_UNKNOWN);

    /* Notify UART bridge of device join */
    if (uart_bridge_is_enabled()) {
        uart_bridge_on_device_join(short_addr);
    }

    /* Trigger LED notification for new device */
    zb_hal_led_blink_notify(ZB_HAL_LED_NOTIFY_DEVICE_NEW);
}

void zb_callback_device_leave(uint16_t short_addr)
{
    ESP_LOGI(TAG, "Device left: 0x%04X", short_addr);

    /* Notify UART bridge before removal */
    if (uart_bridge_is_enabled()) {
        uart_bridge_on_device_leave(short_addr);
    }

    /* Remove device from availability tracking */
    zb_availability_remove_device(short_addr);

    /* Remove device from registry */
    esp_err_t ret = zb_device_remove(short_addr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Device 0x%04X not found in registry", short_addr);
        return;
    }

    /* Update network device count */
    size_t device_count = zb_device_get_count();
    zb_network_set_device_count((uint8_t)device_count);

    ESP_LOGI(TAG, "Total devices: %d", device_count);
}

void zb_callback_attribute_change(uint8_t endpoint, uint16_t cluster_id,
                                   uint16_t attr_id, void *value, size_t value_len)
{
    ESP_LOGD(TAG, "Attribute changed: EP=%d, Cluster=0x%04X, Attr=0x%04X",
             endpoint, cluster_id, attr_id);

    /* This is for local attribute changes on the coordinator */
    /* For most coordinator implementations, this is rarely used */
}

void zb_callback_report_attr(esp_zb_zcl_report_attr_message_t *message)
{
    if (message == NULL) {
        return;
    }

    uint16_t short_addr = message->src_address.u.short_addr;
    uint16_t cluster_id = message->cluster;

    ESP_LOGI(TAG, "Attribute report from 0x%04X: Cluster=0x%04X",
             short_addr, cluster_id);

    /* Check if device exists in registry */
    const zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        /* Unknown short address - try to resolve by IEEE from Zigbee stack */
        esp_zb_ieee_addr_t ieee_addr;
        if (esp_zb_ieee_address_by_short(short_addr, ieee_addr) == ESP_OK) {
            /* Got IEEE address - check if we have this device with different short addr */
            esp_err_t ret = zb_device_update_short_addr(ieee_addr, short_addr);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Device rejoin detected: resolved short=0x%04X via IEEE", short_addr);
                device = zb_device_get(short_addr);

                /* Register in availability tracker after rejoin */
                zb_availability_power_type_t power = ZB_AVAIL_POWER_UNKNOWN;
                if (device != NULL && device->power_info.power_info_valid) {
                    if (device->power_info.current_power_source & 0x04) {
                        power = ZB_AVAIL_POWER_BATTERY;
                    } else if (device->power_info.current_power_source & 0x01) {
                        power = ZB_AVAIL_POWER_MAINS;
                    }
                }
                zb_availability_add_device(short_addr, power);
                ESP_LOGI(TAG, "Device 0x%04X: registered in availability tracker (rejoin)", short_addr);
            } else if (ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGD(TAG, "Device 0x%04X not in registry and no matching IEEE", short_addr);
            }
        }
    } else if (device->short_addr == ZB_SHORT_ADDR_PENDING) {
        /* Device was loaded from NVS but hadn't communicated yet - update short address */
        esp_zb_ieee_addr_t ieee_addr;
        if (esp_zb_ieee_address_by_short(short_addr, ieee_addr) == ESP_OK) {
            zb_device_update_short_addr(ieee_addr, short_addr);
            ESP_LOGI(TAG, "Device 0x%04X: short address resolved from pending", short_addr);

            /* Register in availability tracker now that we have a valid address */
            zb_availability_power_type_t power = ZB_AVAIL_POWER_UNKNOWN;
            zb_device_t *resolved = zb_device_get(short_addr);
            if (resolved != NULL && resolved->power_info.power_info_valid) {
                if (resolved->power_info.current_power_source & 0x04) {
                    power = ZB_AVAIL_POWER_BATTERY;
                } else if (resolved->power_info.current_power_source & 0x01) {
                    power = ZB_AVAIL_POWER_MAINS;
                }
            }
            zb_availability_add_device(short_addr, power);
            ESP_LOGI(TAG, "Device 0x%04X: registered in availability tracker", short_addr);
        }
    }

    /* Update device last seen (device handler + availability tracking) */
    zb_device_update_last_seen(short_addr);
    zb_availability_update_last_seen(short_addr);

    /* Add cluster to device if not already present */
    zb_device_add_cluster(short_addr, cluster_id);

    /* Driver dispatch: if a driver is bound and has process_zcl_attr, call it first.
     * This allows device-specific drivers (e.g. Aqara vibration) to handle
     * manufacturer-specific attributes before the generic cluster handlers. */
    {
        const tuya_device_driver_t *drv = tuya_driver_get(short_addr);
        if (drv != NULL && drv->process_zcl_attr != NULL) {
            esp_err_t drv_ret = drv->process_zcl_attr(
                short_addr,
                message->src_endpoint,
                cluster_id,
                message->attribute.id,
                message->attribute.data.value,
                message->attribute.data.size,
                message->attribute.data.type);
            if (drv_ret == ESP_OK) {
                ESP_LOGD(TAG, "Driver '%s' handled attr report: cluster=0x%04X attr=0x%04X",
                         drv->name, cluster_id, message->attribute.id);
            }
        }
    }

    /* Handle cluster-specific reports */
    switch (cluster_id) {
        case ESP_ZB_ZCL_CLUSTER_ID_BASIC:
            handle_basic_cluster_report(short_addr, message);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            handle_onoff_cluster_report(short_addr, message);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            ESP_LOGI(TAG, "Level control report from 0x%04X", short_addr);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            ESP_LOGI(TAG, "Color control report from 0x%04X", short_addr);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
            ESP_LOGI(TAG, "Temperature measurement report from 0x%04X", short_addr);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
            ESP_LOGI(TAG, "Humidity measurement report from 0x%04X", short_addr);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING:
            ESP_LOGI(TAG, "Occupancy sensing report from 0x%04X", short_addr);
            break;

        case ZB_ZCL_CLUSTER_ID_DOOR_LOCK:
            ESP_LOGI(TAG, "Door lock report from 0x%04X", short_addr);
            /* Handle door lock state report */
            if (message->attribute.id == ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID) {
                zb_door_lock_handle_report(short_addr, message->dst_endpoint,
                                           message->attribute.id,
                                           message->attribute.data.value,
                                           message->attribute.data.size);
            }
            break;

        case ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT:
            ESP_LOGI(TAG, "Electrical measurement report from 0x%04X attr=0x%04X",
                     short_addr, message->attribute.id);
            /* Handle electrical measurement report */
            zb_electrical_handle_report(short_addr, message->dst_endpoint,
                                         message->attribute.id,
                                         message->attribute.data.value,
                                         message->attribute.data.size);
            break;

        case ZB_ZCL_CLUSTER_ID_METERING:
            ESP_LOGI(TAG, "Metering report from 0x%04X attr=0x%04X",
                     short_addr, message->attribute.id);
            /* Handle metering (smart energy) report */
            zb_metering_handle_report(short_addr, message->dst_endpoint,
                                       message->attribute.id,
                                       message->attribute.data.value,
                                       message->attribute.data.size);
            break;

        case ESP_ZB_ZCL_CLUSTER_ID_DEHUMIDIFICATION_CONTROL:
            ESP_LOGI(TAG, "Dehumidification control report from 0x%04X attr=0x%04X",
                     short_addr, message->attribute.id);
            /* Handle dehumidification control report */
            zb_dehumid_handle_report(short_addr, message->dst_endpoint,
                                      message->attribute.id,
                                      message->attribute.data.value,
                                      message->attribute.data.size);
            break;

        case ZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT:
        case ZB_ZCL_CLUSTER_ID_MULTISTATE_OUTPUT:
        case ZB_ZCL_CLUSTER_ID_MULTISTATE_VALUE:
            {
                const char *type_str = (cluster_id == ZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT) ? "Input" :
                                       (cluster_id == ZB_ZCL_CLUSTER_ID_MULTISTATE_OUTPUT) ? "Output" : "Value";
                ESP_LOGI(TAG, "Multistate %s report from 0x%04X attr=0x%04X",
                         type_str, short_addr, message->attribute.id);

                /* Handle multistate report (multi-button switches, rotary switches, multi-mode outputs) */
                zb_multistate_handle_report(short_addr, message->dst_endpoint,
                                             cluster_id, message->attribute.id,
                                             message->attribute.data.value,
                                             message->attribute.data.size);

                /* Publish multistate-specific state over UART */
                if (uart_bridge_is_enabled()) {
                    zb_multistate_state_t ms_state;
                    if (zb_multistate_get_state(short_addr, message->dst_endpoint, &ms_state) == ESP_OK) {
                        device_state_publish_multistate(short_addr, message->dst_endpoint, &ms_state);
                    }
                }
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled cluster report: 0x%04X from 0x%04X",
                     cluster_id, short_addr);
            break;
    }

    /* Update device type based on clusters */
    zb_device_determine_type(short_addr);

    /* Publish attribute update over UART */
    if (uart_bridge_is_enabled()) {
        device_state_publish_by_addr(short_addr);
    }
}

void zb_callback_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            /* Always use NETWORK_FORMATION for the coordinator.
             * If a network already exists in zb_storage, the SDK restores it
             * (emitting DEVICE_REBOOT). If not, it forms a new one
             * (emitting DEVICE_FIRST_START). NETWORK_STEERING is for
             * routers/end-devices and skips the formation/restore step. */
            ESP_LOGI(TAG, "Signal: Skip startup - starting network formation");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            ESP_LOGI(TAG, "Signal: Device first start");
            zb_callback_network_formed();
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            ESP_LOGI(TAG, "Signal: Device reboot");
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Network restored from storage");
                zb_callback_network_formed();
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Signal: Network steering successful");
            } else {
                ESP_LOGW(TAG, "Signal: Network steering failed: %s",
                        esp_err_to_name(err_status));
            }
            break;

        case ESP_ZB_BDB_SIGNAL_FORMATION:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Signal: Network formation successful");
                /* Get extended PAN ID using ESP-Zigbee-SDK v1.6.x API (API-004) */
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_nwk_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
                zb_callback_network_formed();
            } else {
                ESP_LOGE(TAG, "Signal: Network formation failed: %s",
                        esp_err_to_name(err_status));
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            {
                esp_zb_zdo_signal_device_annce_params_t *dev_annce_params =
                    (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
                if (dev_annce_params) {
                    zb_callback_device_announce(dev_annce_params);
                } else {
                    ESP_LOGE(TAG, "Device announce signal received but params are NULL! "
                             "err_status=%s", esp_err_to_name(err_status));
                }
            }
            break;

        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            {
                if (err_status == ESP_OK) {
                    uint8_t duration = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
                    zb_callback_permit_join_changed(duration);
                }
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            {
                /* ESP-Zigbee-SDK v1.6.x: esp_zb_zdo_signal_leave_params_t contains leave_type.
                 * For device leave handling, we rely on the device announce timeout
                 * or explicit leave indication from the device. The coordinator
                 * doesn't receive the leaving device's short address in this signal. */
                esp_zb_zdo_signal_leave_params_t *leave_params =
                    (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
                if (leave_params) {
                    ESP_LOGW(TAG, "Device leave indication: leave_type=%d",
                             leave_params->leave_type);
                    /* Note: leave_type indicates the reason for leave.
                     * Device tracking is handled via availability module timeouts. */
                } else {
                    ESP_LOGW(TAG, "Device leave indication (no parameters)");
                }
            }
            break;

        case ESP_ZB_NLME_STATUS_INDICATION:
            /* API-009: Enhanced NLME Status Indication Handler
             *
             * Network Layer Management Entity (NLME) status indication.
             * This signal provides network layer status updates including:
             * - Network formation/join status
             * - Route discovery results
             * - Network key updates
             * - Communication failures
             *
             * Note: The parameter structure varies by SDK version. We log the
             * signal but don't access internal params for broader compatibility.
             */
            {
                ESP_LOGI(TAG, "NLME Status Indication received (status: %s)",
                         esp_err_to_name(err_status));

                if (err_status != ESP_OK) {
                    /* Log NLME errors - these often indicate routing issues */
                    ESP_LOGW(TAG, "NLME status error: %s", esp_err_to_name(err_status));
                    zb_coordinator_update_route_error_count();

                    /* Publish NLME error event via UART with error code */
                    if (uart_bridge_is_enabled()) {
                        char json_buf[80];
                        snprintf(json_buf, sizeof(json_buf),
                                 "{\"error\":\"%s\",\"code\":%d}",
                                 esp_err_to_name(err_status), (int)err_status);
                        uart_bridge_publish_event("nlme_status", json_buf);
                    }
                }
            }
            break;

        /* Note: ESP_ZB_ZDO_SIGNAL_PARENT_ANNCE_RSP and ESP_ZB_ZDO_TC_REJOIN_DONE
         * are not available in all ESP-Zigbee-SDK versions. Parent announcements
         * and TC rejoins are tracked via device announce and leave signals. */

        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            ESP_LOGI(TAG, "Signal: Production configuration ready");
            break;

        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            {
                esp_zb_zdo_device_unavailable_params_t *unavail_params =
                    (esp_zb_zdo_device_unavailable_params_t *)esp_zb_app_signal_get_params(p_sg_p);
                if (unavail_params) {
                    uint16_t unavail_addr = unavail_params->short_addr;
                    ESP_LOGW(TAG, "Device unavailable: 0x%04X (no MAC/APS ACK)", unavail_addr);
                    cmd_retry_on_unavailable(unavail_addr);

                    /* Forward device unavailable to UART */
                    if (uart_bridge_is_enabled()) {
                        zb_device_t *unavail_dev = zb_device_get(unavail_addr);
                        char ieee_str[20] = "unknown";
                        if (unavail_dev) {
                            uart_proto_ieee_to_str(unavail_dev->ieee_addr, ieee_str, sizeof(ieee_str));
                        }
                        char json_buf[128];
                        snprintf(json_buf, sizeof(json_buf),
                                 "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\"}",
                                 ieee_str, unavail_addr);
                        uart_bridge_publish_event("device_unavailable", json_buf);
                    }
                } else {
                    ESP_LOGW(TAG, "Device unavailable signal (no params)");
                }
            }
            break;

        default:
            ESP_LOGI(TAG, "Unhandled ZDO signal: 0x%x, status: %s",
                    sig_type, esp_err_to_name(err_status));
            break;
    }
}

/**
 * @brief Zigbee application signal handler (required by ESP-Zigbee-SDK)
 *
 * This is the entry point required by ESP-Zigbee-SDK. It dispatches
 * to the internal signal handler implementation.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    zb_callback_signal_handler(signal_struct);
}

/* Note: esp_zb_zcl_cmd_recv_message_t is deprecated in SDK v1.6.x
 * ZCL commands are now handled through the action handler callback.
 * See esp_zb_core_action_handler_register() for the new approach. */

void zb_callback_device_announce(esp_zb_zdo_signal_device_annce_params_t *device_annce)
{
    if (device_annce == NULL) {
        ESP_LOGE(TAG, "zb_callback_device_announce: params are NULL!");
        return;
    }

    uint16_t short_addr = device_annce->device_short_addr;

    ESP_LOGI(TAG, "Device announce: 0x%04X [%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X]",
             short_addr,
             device_annce->ieee_addr[7], device_annce->ieee_addr[6],
             device_annce->ieee_addr[5], device_annce->ieee_addr[4],
             device_annce->ieee_addr[3], device_annce->ieee_addr[2],
             device_annce->ieee_addr[1], device_annce->ieee_addr[0]);

    /* API-005: Log APS Authentication State during device announce
     *
     * This logs the coordinator's APS authentication state when a device
     * announces. A properly secured network should show authenticated state.
     */
    bool coordinator_auth = esp_zb_aps_is_authenticated();
    ESP_LOGD(TAG, "Device announce 0x%04X: coordinator APS auth=%s",
             short_addr, coordinator_auth ? "yes" : "no");

    /* Convert IEEE address to 64-bit */
    uint64_t ieee64 = zb_ieee_to_u64(device_annce->ieee_addr);

    /* Check and apply install code for secure joining */
    apply_install_code_if_present(ieee64, short_addr);

    /* Publish device announce over UART with IEEE + short_addr */
    if (uart_bridge_is_enabled()) {
        char ieee_str[20];
        uart_proto_ieee_to_str(device_annce->ieee_addr, ieee_str, sizeof(ieee_str));
        char json_buf[128];
        snprintf(json_buf, sizeof(json_buf),
                 "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\"}",
                 ieee_str, short_addr);
        uart_bridge_publish_event("device_announce", json_buf);
    }

    ESP_LOGI(TAG, "Device announce processing for 0x%04X", short_addr);

    /* Check if this is a new device or rejoin */
    zb_device_t *existing_device = zb_device_get(short_addr);
    bool is_new_device = (existing_device == NULL);

    ESP_LOGI(TAG, "Device 0x%04X: is_new=%d, existing=%p", short_addr, is_new_device, existing_device);

    /* Treat device announce as a join/rejoin event */
    zb_callback_device_join(ESP_ZB_ZDP_STATUS_SUCCESS,
                           short_addr,
                           device_annce->ieee_addr);

    ESP_LOGI(TAG, "Device join callback completed for 0x%04X", short_addr);

    /* Trigger interview for new devices */
    if (is_new_device) {
        ESP_LOGI(TAG, "New device detected, starting interview for 0x%04X", short_addr);
        zb_hal_led_blink_notify(ZB_HAL_LED_NOTIFY_DEVICE_NEW);
        esp_err_t ret = zb_interview_start(ieee64, short_addr);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to start interview: %s", esp_err_to_name(ret));
            zb_hal_led_blink_notify(ZB_HAL_LED_NOTIFY_DEVICE_FAILED);
        }
    } else {
        ESP_LOGI(TAG, "Device 0x%04X rejoined (existing device)", short_addr);
        zb_hal_led_blink_notify(ZB_HAL_LED_NOTIFY_DEVICE_KNOWN);
        /* Publish interview event for rejoin via UART with device data */
        if (uart_bridge_is_enabled()) {
            zb_device_t *rejoin_dev = zb_device_get(short_addr);
            if (rejoin_dev) {
                char ieee_str[20];
                uart_proto_ieee_to_str(rejoin_dev->ieee_addr, ieee_str, sizeof(ieee_str));
                char *msg = uart_proto_build_interview_done(
                    ieee_str,
                    rejoin_dev->model[0] ? rejoin_dev->model : "unknown",
                    rejoin_dev->manufacturer[0] ? rejoin_dev->manufacturer : "unknown",
                    rejoin_dev->endpoint);
                if (msg) {
                    uart_bridge_send_line(msg);
                    free(msg);
                }
            }
        }
    }
}

void zb_callback_network_formed(void)
{
    ESP_LOGI(TAG, "=== Network Formed ===");

    /* Update network status */
    zb_network_set_formed(true);

    /* API-005: Set APS Authentication State
     *
     * After network formation, the coordinator is authenticated with its
     * Trust Center (itself for coordinator role). Setting authenticated
     * state enables secure APS layer operations.
     */
    esp_zb_aps_set_authenticated(true);
    ESP_LOGI(TAG, "APS Authentication State: authenticated (coordinator/TC)");

    /* Get network info */
    zb_network_info_t net_info;
    if (zb_network_get_info(&net_info) == ESP_OK) {
        ESP_LOGI(TAG, "PAN ID: 0x%04X", net_info.pan_id);
        ESP_LOGI(TAG, "Channel: %d", net_info.channel);
    }

    /* Save network configuration */
    esp_err_t ret = zb_network_save_config();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Network configuration saved");
    } else {
        ESP_LOGW(TAG, "Failed to save network configuration: %s",
                esp_err_to_name(ret));
    }

    /* Publish network_started event via UART */
    uart_bridge_publish_event("network_started", NULL);

    /* Open network for joining via permit_join (lightweight alternative to
     * NETWORK_STEERING which avoids ~50-60KB internal RAM allocation for
     * Trust Center state and security material). */
    zb_coordinator_permit_join(0);
}

void zb_callback_bdb_commissioning_complete(esp_zb_bdb_commissioning_status_t status)
{
    ESP_LOGI(TAG, "BDB commissioning complete: status=0x%02X", status);

    if (status == ESP_ZB_BDB_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Commissioning successful");

        /* API-005: Verify APS Authentication State after successful commissioning
         *
         * After successful BDB commissioning, verify that the APS layer
         * authentication state is properly set. This is a sanity check
         * to ensure security is enabled.
         */
        bool is_authenticated = esp_zb_aps_is_authenticated();
        if (!is_authenticated) {
            ESP_LOGW(TAG, "APS not authenticated after commissioning, setting now");
            esp_zb_aps_set_authenticated(true);
        }
        ESP_LOGI(TAG, "APS Authentication State verified: authenticated");
    } else {
        ESP_LOGW(TAG, "Commissioning failed with status: 0x%02X", status);
    }
}

void zb_callback_permit_join_changed(uint8_t permit_duration)
{
    bool permit_join = (permit_duration > 0);

    ESP_LOGI(TAG, "Permit join %s (duration: %d seconds)",
             permit_join ? "OPENED" : "CLOSED", permit_duration);

    zb_network_set_permit_join(permit_join);

    /* Update LED for pairing mode */
    if (permit_join) {
        zb_hal_led_set_status(ZB_HAL_LED_STATUS_PAIRING);
    } else {
        zb_hal_led_set_status(ZB_HAL_LED_STATUS_NORMAL);
    }

    /* Publish permit_join event via UART */
    if (uart_bridge_is_enabled()) {
        char *msg = uart_proto_build_permit_join(permit_join, permit_duration);
        if (msg) {
            uart_bridge_send_line(msg);
            free(msg);
        }
    }
}

/* Internal helper functions */
static const char* zdo_status_to_str(esp_zb_zdp_status_t status)
{
    switch (status) {
        case ESP_ZB_ZDP_STATUS_SUCCESS: return "SUCCESS";
        case ESP_ZB_ZDP_STATUS_INV_REQUESTTYPE: return "INVALID_REQUEST";
        case ESP_ZB_ZDP_STATUS_DEVICE_NOT_FOUND: return "DEVICE_NOT_FOUND";
        case ESP_ZB_ZDP_STATUS_INVALID_EP: return "INVALID_EP";
        case ESP_ZB_ZDP_STATUS_NOT_ACTIVE: return "NOT_ACTIVE";
        case ESP_ZB_ZDP_STATUS_NOT_SUPPORTED: return "NOT_SUPPORTED";
        case ESP_ZB_ZDP_STATUS_TIMEOUT: return "TIMEOUT";
        case ESP_ZB_ZDP_STATUS_NO_MATCH: return "NO_MATCH";
        case ESP_ZB_ZDP_STATUS_NO_ENTRY: return "NO_ENTRY";
        case ESP_ZB_ZDP_STATUS_NO_DESCRIPTOR: return "NO_DESCRIPTOR";
        case ESP_ZB_ZDP_STATUS_INSUFFICIENT_SPACE: return "INSUFFICIENT_SPACE";
        case ESP_ZB_ZDP_STATUS_NOT_PERMITTED: return "NOT_PERMITTED";
        case ESP_ZB_ZDP_STATUS_TABLE_FULL: return "TABLE_FULL";
        case ESP_ZB_ZDP_STATUS_NOT_AUTHORIZED: return "NOT_AUTHORIZED";
        default: return "UNKNOWN";
    }
}

static void handle_basic_cluster_report(uint16_t short_addr, esp_zb_zcl_report_attr_message_t *msg)
{
    if (msg == NULL) {
        return;
    }

    uint16_t attr_id = msg->attribute.id;
    esp_zb_zcl_attribute_data_t *data = &msg->attribute.data;

    ESP_LOGI(TAG, "Basic cluster report from 0x%04X: attr=0x%04X", short_addr, attr_id);

    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        return;
    }

    switch (attr_id) {
        case ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID:
            if (data->type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING && data->value != NULL) {
                /* First byte is string length for ZCL strings */
                uint8_t len = ((uint8_t *)data->value)[0];
                if (len > 0 && len < sizeof(device->manufacturer)) {
                    memcpy(device->manufacturer, &((uint8_t *)data->value)[1], len);
                    device->manufacturer[len] = '\0';
                    ESP_LOGI(TAG, "Device 0x%04X manufacturer: %s", short_addr, device->manufacturer);

                    /* Try early driver binding if model is already known
                     * (manufacturer may arrive after model) */
                    if (device->model[0] != '\0' &&
                        tuya_driver_get(short_addr) == NULL) {
                        const tuya_device_driver_t *drv =
                            tuya_driver_find(device->manufacturer, device->model);
                        if (drv != NULL) {
                            tuya_driver_bind(short_addr, drv);
                            if (drv->init_device) {
                                drv->init_device(short_addr);
                            }
                            ESP_LOGI(TAG, "Device 0x%04X: early-bound to driver '%s'",
                                     short_addr, drv->name);
                            if (drv->publish_discovery) {
                                drv->publish_discovery(device);
                            }
                        }
                    }
                }
            }
            break;

        case ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID:
            if (data->type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING && data->value != NULL) {
                uint8_t len = ((uint8_t *)data->value)[0];
                if (len > 0 && len < sizeof(device->model)) {
                    memcpy(device->model, &((uint8_t *)data->value)[1], len);
                    device->model[len] = '\0';
                    ESP_LOGI(TAG, "Device 0x%04X model: %s", short_addr, device->model);

                    /* Early driver binding: model now known (manufacturer may or may not be).
                     * Bind before first DP/attribute report arrives so discovery
                     * entities are already published when the device starts reporting.
                     * Some devices (Xiaomi/Aqara) never report manufacturer via Basic. */
                    if (tuya_driver_get(short_addr) == NULL) {
                        const tuya_device_driver_t *drv =
                            tuya_driver_find(device->manufacturer, device->model);
                        if (drv != NULL) {
                            tuya_driver_bind(short_addr, drv);
                            if (drv->init_device) {
                                drv->init_device(short_addr);
                            }
                            ESP_LOGI(TAG, "Device 0x%04X: early-bound to driver '%s'",
                                     short_addr, drv->name);
                            if (drv->publish_discovery) {
                                drv->publish_discovery(device);
                            }
                        }
                    }
                }
            }
            break;

        case ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID:
            if (data->type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING && data->value != NULL) {
                uint8_t len = ((uint8_t *)data->value)[0];
                char sw_version[65];
                if (len > 0 && len < sizeof(sw_version)) {
                    memcpy(sw_version, &((uint8_t *)data->value)[1], len);
                    sw_version[len] = '\0';
                    ESP_LOGI(TAG, "Device 0x%04X SW version: %s", short_addr, sw_version);
                }
            }
            break;

        case ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID:
            if (data->value != NULL && data->size >= 1) {
                uint8_t power_source = *(uint8_t *)data->value;
                ESP_LOGI(TAG, "Device 0x%04X power source: 0x%02X", short_addr, power_source);
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled basic cluster attr 0x%04X", attr_id);
            break;
    }
}

static void handle_onoff_cluster_report(uint16_t short_addr, esp_zb_zcl_report_attr_message_t *msg)
{
    if (msg == NULL) {
        return;
    }

    uint16_t attr_id = msg->attribute.id;
    esp_zb_zcl_attribute_data_t *data = &msg->attribute.data;

    ESP_LOGI(TAG, "On/Off cluster report from 0x%04X: attr=0x%04X", short_addr, attr_id);

    if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        if (data->value != NULL && data->size >= 1) {
            bool on_off_state = *(bool *)data->value;
            ESP_LOGI(TAG, "Device 0x%04X on/off state: %s", short_addr, on_off_state ? "ON" : "OFF");

            /* Update device state via device handler */
            zb_device_update_state(short_addr, msg->dst_endpoint,
                                   ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, attr_id,
                                   data->value, data->size);
        }
    } else if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_GLOBAL_SCENE_CONTROL) {
        if (data->value != NULL && data->size >= 1) {
            bool scene_control = *(bool *)data->value;
            ESP_LOGD(TAG, "Device 0x%04X global scene control: %s", short_addr, scene_control ? "enabled" : "disabled");
        }
    } else if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME) {
        if (data->value != NULL && data->size >= 2) {
            uint16_t on_time = *(uint16_t *)data->value;
            ESP_LOGD(TAG, "Device 0x%04X on time: %d", short_addr, on_time);
        }
    } else {
        ESP_LOGD(TAG, "Unhandled on/off cluster attr 0x%04X", attr_id);
    }
}
