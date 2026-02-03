/**
 * @file aqara_vibration.c
 * @brief Aqara DJT11LM Vibration Sensor Driver Implementation
 *
 * Implements the device driver vtable for the Aqara DJT11LM vibration sensor.
 * Handles ZCL attribute reports from Cluster 0x0101 (vibration/tilt/drop)
 * and Cluster 0x0000 (Xiaomi 0xFF01 battery/temperature).
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "tuya_driver_registry.h"
#include "tuya_device_driver.h"
#include "zb_device_handler.h"
#include "zb_coordinator.h"
#include "zb_device_handler_types.h"
#include "xiaomi_tlv.h"
#include "compat_stubs.h"
#include "ha_constants.h"
#include "gateway_defaults.h"
#include "uart_bridge.h"
#include "uart/uart_protocol.h"
#include "json_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <string.h>
#include <math.h>

static const char *TAG = "AQARA_VIBRATION";

/* ============================================================================
 * Constants
 * ============================================================================ */

/** @brief Maximum number of DJT11LM devices */
#define AQARA_VIB_MAX_DEVICES     4

/** @brief Cluster 0x0101 attributes used by DJT11LM */
#define AQARA_VIB_ATTR_ACTION     0x0055  /**< Action type: 1=vibration, 2=tilt, 3=drop */
#define AQARA_VIB_ATTR_STRENGTH   0x0505  /**< Vibration strength (uint16) */
#define AQARA_VIB_ATTR_ACCEL_XYZ  0x0508  /**< XYZ acceleration (3x int16, big-endian) */

/** @brief Sensitivity write target: Cluster 0x0000, Attr 0xFF0D, Manuf 0x115F */
#define AQARA_VIB_ATTR_SENSITIVITY  0xFF0D

/** @brief Action values */
#define AQARA_ACTION_VIBRATION    1
#define AQARA_ACTION_TILT         2
#define AQARA_ACTION_DROP         3

/** @brief Sensitivity values */
#define AQARA_SENSITIVITY_HIGH    1
#define AQARA_SENSITIVITY_MEDIUM  11
#define AQARA_SENSITIVITY_LOW     21

/** @brief Binary sensor off_delay in seconds */
#define AQARA_VIB_OFF_DELAY       65

/* ============================================================================
 * State Management
 * ============================================================================ */

typedef struct {
    bool     valid;
    uint16_t short_addr;
    int64_t  last_update;

    /* Event data */
    bool     has_event_data;   /* True after first real event received */
    uint8_t  action;           /* 1=vibration, 2=tilt, 3=drop */
    uint16_t strength;         /* Vibration strength */
    int16_t  accel_x;          /* Raw accelerometer X */
    int16_t  accel_y;          /* Raw accelerometer Y */
    int16_t  accel_z;          /* Raw accelerometer Z */
    float    angle_x;          /* Calculated angle X (degrees) */
    float    angle_y;          /* Calculated angle Y (degrees) */
    float    angle_z;          /* Calculated angle Z (degrees) */

    /* Battery & device info */
    uint8_t  battery_percent;
    uint16_t battery_mv;
    int16_t  temperature;      /* °C * 100 */

    /* Configuration */
    uint8_t  sensitivity;      /* 1=high, 11=medium, 21=low */
} aqara_vibration_state_t;

static aqara_vibration_state_t s_states[AQARA_VIB_MAX_DEVICES];
static SemaphoreHandle_t s_mutex = NULL;

/* Forward declaration */
static cJSON *vibration_build_state_json(uint16_t short_addr);

static aqara_vibration_state_t *find_or_create_state(uint16_t short_addr)
{
    for (int i = 0; i < AQARA_VIB_MAX_DEVICES; i++) {
        if (s_states[i].valid && s_states[i].short_addr == short_addr) {
            return &s_states[i];
        }
    }
    for (int i = 0; i < AQARA_VIB_MAX_DEVICES; i++) {
        if (!s_states[i].valid) {
            memset(&s_states[i], 0, sizeof(aqara_vibration_state_t));
            s_states[i].short_addr = short_addr;
            s_states[i].valid = true;
            s_states[i].sensitivity = AQARA_SENSITIVITY_MEDIUM;
            return &s_states[i];
        }
    }
    ESP_LOGW(TAG, "No free state slot for 0x%04X", short_addr);
    return NULL;
}

/* ============================================================================
 * Angle Calculation
 * ============================================================================ */

static void calc_angles(aqara_vibration_state_t *st)
{
    float x = (float)st->accel_x;
    float y = (float)st->accel_y;
    float z = (float)st->accel_z;
    float mag = sqrtf(x * x + y * y + z * z);
    if (mag < 1.0f) {
        st->angle_x = 0.0f;
        st->angle_y = 0.0f;
        st->angle_z = 0.0f;
        return;
    }
    st->angle_x = acosf(x / mag) * 180.0f / (float)M_PI;
    st->angle_y = acosf(y / mag) * 180.0f / (float)M_PI;
    st->angle_z = acosf(z / mag) * 180.0f / (float)M_PI;
}

/* ============================================================================
 * Xiaomi TLV Callback
 * ============================================================================ */

static bool tlv_callback(const xiaomi_tlv_entry_t *entry, void *user_data)
{
    aqara_vibration_state_t *st = (aqara_vibration_state_t *)user_data;

    switch (entry->tag) {
        case XIAOMI_TAG_BATTERY_MV:
            if (entry->data_len >= 2) {
                st->battery_mv = (uint16_t)(entry->data[0] | (entry->data[1] << 8));
                ESP_LOGI(TAG, "  Battery: %u mV", st->battery_mv);
            }
            break;
        case XIAOMI_TAG_TEMPERATURE:
            if (entry->data_len >= 2) {
                st->temperature = (int16_t)(entry->data[0] | (entry->data[1] << 8));
                ESP_LOGI(TAG, "  Temperature: %.1f°C", st->temperature / 100.0f);
            }
            break;
        case XIAOMI_TAG_BATTERY_PCT:
            if (entry->data_len >= 1) {
                st->battery_percent = entry->data[0];
                ESP_LOGI(TAG, "  Battery: %u%%", st->battery_percent);

                /* Update ZCL power info for persistence */
                zb_device_t *dev = zb_device_get(st->short_addr);
                if (dev != NULL) {
                    dev->power_info.current_power_source =
                        ZB_POWER_SOURCE_DISPOSABLE_BATTERY;
                    if (st->battery_percent <= 5) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_CRITICAL;
                    } else if (st->battery_percent <= 33) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_33_PERCENT;
                    } else if (st->battery_percent <= 66) {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_66_PERCENT;
                    } else {
                        dev->power_info.current_power_source_level = ZB_POWER_LEVEL_100_PERCENT;
                    }
                    dev->power_info.power_info_valid = true;
                }
            }
            break;
        default:
            ESP_LOGD(TAG, "  TLV tag 0x%02X type 0x%02X len %u (ignored)",
                     entry->tag, entry->zcl_type, entry->data_len);
            break;
    }
    return true;
}

/* ============================================================================
 * Driver Interface: match
 * ============================================================================ */

static bool vibration_match(const char *manufacturer, const char *model)
{
    /* Model-based match: lumi.vibration.aq1 is unique enough.
     * Xiaomi/Aqara devices often don't report manufacturer via Basic cluster,
     * so we cannot require it. */
    if (model != NULL && strcmp(model, "lumi.vibration.aq1") == 0) {
        return true;
    }
    return false;
}

/* ============================================================================
 * Driver Interface: process_zcl_attr
 * ============================================================================ */

static esp_err_t vibration_process_zcl_attr(uint16_t short_addr, uint8_t endpoint,
                                             uint16_t cluster_id, uint16_t attr_id,
                                             const void *data, uint16_t data_size,
                                             uint8_t data_type)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    aqara_vibration_state_t *st = find_or_create_state(short_addr);
    if (st == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    st->last_update = esp_timer_get_time();

    bool handled = false;

    /* Cluster 0x0101 (Door Lock): vibration event data */
    if (cluster_id == ZB_ZCL_CLUSTER_ID_DOOR_LOCK) {
        switch (attr_id) {
            case AQARA_VIB_ATTR_ACTION: {
                if (data_size >= 2) {
                    uint16_t action = ((const uint8_t *)data)[0] |
                                      (((const uint8_t *)data)[1] << 8);
                    st->action = (uint8_t)action;
                    st->has_event_data = true;  /* Mark that we have real data */
                    const char *action_str =
                        (action == AQARA_ACTION_VIBRATION) ? "vibration" :
                        (action == AQARA_ACTION_TILT)      ? "tilt" :
                        (action == AQARA_ACTION_DROP)       ? "drop" : "unknown";
                    ESP_LOGI(TAG, "0x%04X action: %s (%u)", short_addr, action_str, action);
                    handled = true;
                }
                break;
            }
            case AQARA_VIB_ATTR_STRENGTH: {
                if (data_size >= 2) {
                    st->strength = ((const uint8_t *)data)[0] |
                                   (((const uint8_t *)data)[1] << 8);
                    st->has_event_data = true;  /* Mark that we have real data */
                    ESP_LOGI(TAG, "0x%04X strength: %u", short_addr, st->strength);
                    handled = true;
                }
                break;
            }
            case AQARA_VIB_ATTR_ACCEL_XYZ: {
                /* Aqara sends 3x int16 big-endian */
                if (data_size >= 6) {
                    const uint8_t *d = (const uint8_t *)data;
                    st->accel_x = (int16_t)((d[0] << 8) | d[1]);
                    st->accel_y = (int16_t)((d[2] << 8) | d[3]);
                    st->accel_z = (int16_t)((d[4] << 8) | d[5]);
                    st->has_event_data = true;  /* Mark that we have real data */
                    calc_angles(st);
                    ESP_LOGI(TAG, "0x%04X accel: x=%d y=%d z=%d -> angle: x=%.1f y=%.1f z=%.1f",
                             short_addr, st->accel_x, st->accel_y, st->accel_z,
                             st->angle_x, st->angle_y, st->angle_z);
                    handled = true;
                }
                break;
            }
            default:
                ESP_LOGD(TAG, "0x%04X door_lock attr 0x%04X (unhandled)", short_addr, attr_id);
                break;
        }
    }
    /* Cluster 0x0000 (Basic): Xiaomi proprietary */
    else if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_BASIC) {
        if (attr_id == XIAOMI_ATTR_SPECIAL) {
            ESP_LOGI(TAG, "0x%04X Xiaomi 0xFF01 TLV (%u bytes)", short_addr, data_size);
            xiaomi_tlv_parse((const uint8_t *)data, data_size, tlv_callback, st);
            handled = true;
        }
    }

    xSemaphoreGive(s_mutex);

    /* Publish driver state via UART to S3 */
    if (handled) {
        zb_device_t *dev = zb_device_get(short_addr);
        if (dev != NULL) {
            cJSON *state_json = vibration_build_state_json(short_addr);
            if (state_json != NULL) {
                char ieee_str[20];
                uart_proto_ieee_to_str(dev->ieee_addr, ieee_str, sizeof(ieee_str));
                char *msg = uart_proto_build_device_state(ieee_str, state_json);
                cJSON_Delete(state_json);
                if (msg != NULL) {
                    uart_bridge_send_line(msg);
                    free(msg);
                }
            }
        }
    }

    return handled ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * Driver Interface: handle_command
 * ============================================================================ */

static esp_err_t vibration_handle_command(uint16_t short_addr, uint8_t endpoint,
                                           const cJSON *json)
{
    bool handled = false;

    /* Handle sensitivity: {"sensitivity": "high"|"medium"|"low"} */
    cJSON *sens_json = cJSON_GetObjectItem(json, "sensitivity");
    if (cJSON_IsString(sens_json) && sens_json->valuestring != NULL) {
        uint8_t sens_val;
        const char *s = sens_json->valuestring;
        if (strcmp(s, "high") == 0) {
            sens_val = AQARA_SENSITIVITY_HIGH;
        } else if (strcmp(s, "medium") == 0) {
            sens_val = AQARA_SENSITIVITY_MEDIUM;
        } else if (strcmp(s, "low") == 0) {
            sens_val = AQARA_SENSITIVITY_LOW;
        } else {
            ESP_LOGW(TAG, "Invalid sensitivity: %s (valid: high/medium/low)", s);
            return ESP_ERR_INVALID_ARG;
        }

        /* ZCL Write: Cluster 0x0000, Attr 0xFF0D, Manufacturer Code 0x115F */
        esp_zb_zcl_write_attr_cmd_t write_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = short_addr,
                .dst_endpoint = endpoint,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BASIC,
            .manuf_specific = 1,
            .manuf_code = XIAOMI_MANUFACTURER_CODE,
            .attr_number = 1,
        };

        esp_zb_zcl_attribute_t attr_field = {
            .id = AQARA_VIB_ATTR_SENSITIVITY,
            .data = {
                .type = ESP_ZB_ZCL_ATTR_TYPE_U8,
                .size = sizeof(uint8_t),
                .value = &sens_val,
            },
        };
        write_cmd.attr_field = &attr_field;

        esp_zb_lock_acquire(portMAX_DELAY);
        uint8_t tsn = esp_zb_zcl_write_attr_cmd_req(&write_cmd);
        esp_zb_lock_release();
        (void)tsn;

        ESP_LOGI(TAG, "Sensitivity write queued: 0x%04X -> %s (%u) tsn=%u",
                 short_addr, s, sens_val, tsn);
        /* Update local state */
        if (s_mutex != NULL) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            aqara_vibration_state_t *st = find_or_create_state(short_addr);
            if (st != NULL) {
                st->sensitivity = sens_val;
            }
            xSemaphoreGive(s_mutex);
        }
        handled = true;
    }

    return handled ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ============================================================================
 * Driver Interface: build_state_json
 * ============================================================================ */

static cJSON *vibration_build_state_json(uint16_t short_addr)
{
    if (s_mutex == NULL) {
        return NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    const aqara_vibration_state_t *st = NULL;
    for (int i = 0; i < AQARA_VIB_MAX_DEVICES; i++) {
        if (s_states[i].valid && s_states[i].short_addr == short_addr) {
            st = &s_states[i];
            break;
        }
    }

    if (st == NULL) {
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        xSemaphoreGive(s_mutex);
        return NULL;
    }

    /* Only include event data if we've received at least one real event.
     * This prevents overwriting HA state with zeros after reboot. */
    if (st->has_event_data) {
        /* Vibration binary sensor */
        cJSON_AddBoolToObject(json, "vibration",
                               st->action == AQARA_ACTION_VIBRATION);

        /* Action */
        const char *action_str =
            (st->action == AQARA_ACTION_VIBRATION) ? "vibration" :
            (st->action == AQARA_ACTION_TILT)      ? "tilt" :
            (st->action == AQARA_ACTION_DROP)       ? "drop" : "";
        cJSON_AddStringToObject(json, "action", action_str);

        /* Angles */
        cJSON_AddNumberToObject(json, "angle_x", roundf(st->angle_x * 10.0f) / 10.0f);
        cJSON_AddNumberToObject(json, "angle_y", roundf(st->angle_y * 10.0f) / 10.0f);
        cJSON_AddNumberToObject(json, "angle_z", roundf(st->angle_z * 10.0f) / 10.0f);

        /* Strength */
        cJSON_AddNumberToObject(json, "strength", st->strength);
    }

    /* Battery (always include if available — from 0xFF01 TLV, not from event) */
    if (st->battery_percent > 0) {
        cJSON_AddNumberToObject(json, "battery", st->battery_percent);
    }
    if (st->battery_mv > 0) {
        cJSON_AddNumberToObject(json, "voltage", st->battery_mv);
    }

    /* Sensitivity (always include) */
    const char *sens_str =
        (st->sensitivity == AQARA_SENSITIVITY_HIGH)   ? "high" :
        (st->sensitivity == AQARA_SENSITIVITY_MEDIUM) ? "medium" :
        (st->sensitivity == AQARA_SENSITIVITY_LOW)    ? "low" : "medium";
    cJSON_AddStringToObject(json, "sensitivity", sens_str);

    xSemaphoreGive(s_mutex);
    return json;
}

/* ============================================================================
 * Discovery Helpers
 * ============================================================================ */

static esp_err_t publish_disc_config(const char *topic, cJSON *config,
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

static esp_err_t publish_sensor(const zb_device_t *device, const char *ieee_str,
                                 const char *field, const char *display_name,
                                 const char *device_class, const char *unit,
                                 const char *icon, const char *value_tmpl)
{
    char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", ieee_str, field);

    char topic[MQTT_TOPIC_MAX_LEN];
    esp_err_t ret = mqtt_topic_ha_discovery("sensor", unique_id, topic, sizeof(topic));
    if (ret != ESP_OK) return ret;

    cJSON *config = cJSON_CreateObject();
    if (config == NULL) return ESP_ERR_NO_MEM;

    char name[GW_BUFFER_SIZE_SMALL];
    snprintf(name, sizeof(name), "%s %s", device->friendly_name, display_name);
    cJSON_AddStringToObject(config, "name", name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);

    if (device_class) cJSON_AddStringToObject(config, "device_class", device_class);
    if (unit) cJSON_AddStringToObject(config, "unit_of_measurement", unit);
    if (icon) cJSON_AddStringToObject(config, "icon", icon);
    cJSON_AddStringToObject(config, "state_class", "measurement");

    char state_topic[GW_BUFFER_SIZE_MEDIUM];
    snprintf(state_topic, sizeof(state_topic), "zigbee2mqtt/%s", device->friendly_name);
    cJSON_AddStringToObject(config, "state_topic", state_topic);
    cJSON_AddStringToObject(config, "value_template", value_tmpl);

    add_device_info(config, device);

    char log_msg[GW_BUFFER_SIZE_SMALL];
    snprintf(log_msg, sizeof(log_msg), "sensor %s", field);
    return publish_disc_config(topic, config, log_msg, device->friendly_name);
}

/* ============================================================================
 * Driver Interface: publish_discovery
 * ============================================================================ */

static esp_err_t vibration_publish_discovery(const zb_device_t *device)
{
    if (device == NULL) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Publishing Aqara vibration discovery for: %s", device->friendly_name);

    char ieee_str[MQTT_IEEE_ADDR_STR_LEN];
    json_format_ieee_addr(device->ieee_addr, ieee_str, sizeof(ieee_str));
    esp_err_t ret;

    /* === 1. Binary Sensor: Vibration === */
    {
        char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
        snprintf(unique_id, sizeof(unique_id), "%s_vibration", ieee_str);

        char topic[MQTT_TOPIC_MAX_LEN];
        ret = mqtt_topic_ha_discovery("binary_sensor", unique_id, topic, sizeof(topic));
        if (ret == ESP_OK) {
            cJSON *config = cJSON_CreateObject();
            if (config != NULL) {
                char name[GW_BUFFER_SIZE_SMALL];
                snprintf(name, sizeof(name), "%s Vibration", device->friendly_name);
                cJSON_AddStringToObject(config, "name", name);
                cJSON_AddStringToObject(config, "unique_id", unique_id);
                cJSON_AddStringToObject(config, "device_class", "vibration");

                char state_topic[GW_BUFFER_SIZE_MEDIUM];
                snprintf(state_topic, sizeof(state_topic),
                         "zigbee2mqtt/%s", device->friendly_name);
                cJSON_AddStringToObject(config, "state_topic", state_topic);
                cJSON_AddStringToObject(config, "value_template",
                    "{{ 'ON' if value_json.vibration else 'OFF' }}");
                cJSON_AddStringToObject(config, "payload_on", "ON");
                cJSON_AddStringToObject(config, "payload_off", "OFF");
                cJSON_AddNumberToObject(config, "off_delay", AQARA_VIB_OFF_DELAY);

                add_device_info(config, device);
                publish_disc_config(topic, config, "binary_sensor vibration",
                                     device->friendly_name);
            }
        }
    }

    /* === 2. Sensor: Action === */
    {
        char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
        snprintf(unique_id, sizeof(unique_id), "%s_action", ieee_str);

        char topic[MQTT_TOPIC_MAX_LEN];
        ret = mqtt_topic_ha_discovery("sensor", unique_id, topic, sizeof(topic));
        if (ret == ESP_OK) {
            cJSON *config = cJSON_CreateObject();
            if (config != NULL) {
                char name[GW_BUFFER_SIZE_SMALL];
                snprintf(name, sizeof(name), "%s Action", device->friendly_name);
                cJSON_AddStringToObject(config, "name", name);
                cJSON_AddStringToObject(config, "unique_id", unique_id);
                cJSON_AddStringToObject(config, "icon", "mdi:gesture-double-tap");

                char state_topic[GW_BUFFER_SIZE_MEDIUM];
                snprintf(state_topic, sizeof(state_topic),
                         "zigbee2mqtt/%s", device->friendly_name);
                cJSON_AddStringToObject(config, "state_topic", state_topic);
                cJSON_AddStringToObject(config, "value_template",
                    "{{ value_json.action }}");

                add_device_info(config, device);
                publish_disc_config(topic, config, "sensor action",
                                     device->friendly_name);
            }
        }
    }

    /* === 3-5. Sensors: angle_x, angle_y, angle_z === */
    publish_sensor(device, ieee_str, "angle_x", "Angle X", NULL, "°",
                   "mdi:angle-acute", "{{ value_json.angle_x | default(0) }}");
    publish_sensor(device, ieee_str, "angle_y", "Angle Y", NULL, "°",
                   "mdi:angle-acute", "{{ value_json.angle_y | default(0) }}");
    publish_sensor(device, ieee_str, "angle_z", "Angle Z", NULL, "°",
                   "mdi:angle-acute", "{{ value_json.angle_z | default(0) }}");

    /* === 6. Sensor: Strength === */
    publish_sensor(device, ieee_str, "strength", "Strength", NULL, NULL,
                   "mdi:flash", "{{ value_json.strength | default(0) }}");

    /* === 7. Sensor: Battery === */
    publish_sensor(device, ieee_str, "battery", "Battery", "battery", "%",
                   "mdi:battery", "{{ value_json.battery | default(0) }}");

    /* === 8. Sensor: Voltage === */
    publish_sensor(device, ieee_str, "voltage", "Voltage", "voltage", "mV",
                   "mdi:flash-triangle", "{{ value_json.voltage | default(0) }}");

    /* === 9. Select: Sensitivity === */
    {
        char unique_id[MQTT_UNIQUE_ID_MAX_LEN];
        snprintf(unique_id, sizeof(unique_id), "%s_sensitivity", ieee_str);

        char topic[MQTT_TOPIC_MAX_LEN];
        ret = mqtt_topic_ha_discovery("select", unique_id, topic, sizeof(topic));
        if (ret == ESP_OK) {
            cJSON *config = cJSON_CreateObject();
            if (config != NULL) {
                char name[GW_BUFFER_SIZE_SMALL];
                snprintf(name, sizeof(name), "%s Sensitivity", device->friendly_name);
                cJSON_AddStringToObject(config, "name", name);
                cJSON_AddStringToObject(config, "unique_id", unique_id);
                cJSON_AddStringToObject(config, "icon", "mdi:tune-vertical");

                char state_topic[GW_BUFFER_SIZE_MEDIUM];
                snprintf(state_topic, sizeof(state_topic),
                         "zigbee2mqtt/%s", device->friendly_name);
                cJSON_AddStringToObject(config, "state_topic", state_topic);

                char cmd_topic[MQTT_LONG_STR_MAX_LEN];
                snprintf(cmd_topic, sizeof(cmd_topic),
                         "zigbee2mqtt/%s/set", device->friendly_name);
                cJSON_AddStringToObject(config, "command_topic", cmd_topic);

                cJSON_AddStringToObject(config, "value_template",
                    "{{ value_json.sensitivity }}");
                cJSON_AddStringToObject(config, "command_template",
                    "{\"sensitivity\": \"{{ value }}\"}");

                cJSON *options = cJSON_CreateArray();
                cJSON_AddItemToArray(options, cJSON_CreateString("high"));
                cJSON_AddItemToArray(options, cJSON_CreateString("medium"));
                cJSON_AddItemToArray(options, cJSON_CreateString("low"));
                cJSON_AddItemToObject(config, "options", options);

                add_device_info(config, device);
                publish_disc_config(topic, config, "select sensitivity",
                                     device->friendly_name);
            }
        }
    }

    return ESP_OK;
}

/* ============================================================================
 * Driver Interface: init_device / remove_device
 * ============================================================================ */

static esp_err_t vibration_init_device(uint16_t short_addr)
{
    if (s_mutex == NULL) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    aqara_vibration_state_t *st = find_or_create_state(short_addr);
    xSemaphoreGive(s_mutex);

    if (st == NULL) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Initialized vibration state for 0x%04X", short_addr);
    return ESP_OK;
}

static void vibration_remove_device(uint16_t short_addr)
{
    if (s_mutex == NULL) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < AQARA_VIB_MAX_DEVICES; i++) {
        if (s_states[i].valid && s_states[i].short_addr == short_addr) {
            memset(&s_states[i], 0, sizeof(aqara_vibration_state_t));
            ESP_LOGI(TAG, "Removed vibration state for 0x%04X", short_addr);
            break;
        }
    }
    xSemaphoreGive(s_mutex);
}

/* ============================================================================
 * Driver Vtable
 * ============================================================================ */

static const tuya_device_driver_t s_vibration_driver = {
    .name               = "aqara_vibration",
    .match              = vibration_match,
    .process_dp         = NULL,  /* Not a Tuya device — no DPs */
    .process_zcl_attr   = vibration_process_zcl_attr,
    .handle_command     = vibration_handle_command,
    .build_state_json   = vibration_build_state_json,
    .publish_discovery  = vibration_publish_discovery,
    .init_device        = vibration_init_device,
    .remove_device      = vibration_remove_device,
};

/* ============================================================================
 * Registration
 * ============================================================================ */

esp_err_t aqara_vibration_register(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create vibration mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    memset(s_states, 0, sizeof(s_states));

    esp_err_t ret = tuya_driver_register(&s_vibration_driver);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Aqara vibration driver registered");
    }
    return ret;
}
