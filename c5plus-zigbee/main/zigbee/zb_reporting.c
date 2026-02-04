/**
 * @file zb_reporting.c
 * @brief ZCL Reporting Configuration Implementation
 *
 * Implements ZCL Reporting Configuration including sending Configure Reporting
 * commands, handling responses, NVS persistence, and MQTT integration.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_reporting.h"
#include "zb_device_handler.h"
#include "zb_constants.h"
#include "gateway_defaults.h"
#include "core/compat_stubs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "ZB_REPORTING";

/* Module state */
static zb_reporting_entry_t *s_entries = NULL;
static size_t s_entry_count = 0;
static bool s_initialized = false;
static SemaphoreHandle_t s_reporting_mutex = NULL;
static zb_reporting_event_cb_t s_event_callback = NULL;

/* Pending request tracking */
typedef struct {
    uint16_t short_addr;
    uint8_t endpoint;
    uint16_t cluster_id;
    uint16_t attr_id;
    bool is_configure;      /* true = configure, false = read */
    bool pending;
    TickType_t timestamp;
    zb_reporting_config_t config;
} zb_reporting_pending_t;

#define ZB_REPORTING_MAX_PENDING 16
#define ZB_REPORTING_REQUEST_TIMEOUT_MS 15000

static zb_reporting_pending_t s_pending_requests[ZB_REPORTING_MAX_PENDING];

/* NVS keys */
#define NVS_KEY_ENTRY_COUNT "rpt_count"
#define NVS_KEY_ENTRY_DATA_FMT "rpt_%02d"

/* MQTT topic patterns */
#define TOPIC_REPORTING_CONFIGURE_REQ   "zigbee2mqtt/bridge/request/device/reporting/configure"
#define TOPIC_REPORTING_READ_REQ        "zigbee2mqtt/bridge/request/device/reporting/read"
#define TOPIC_REPORTING_CONFIGURE_RSP   "zigbee2mqtt/bridge/response/device/reporting/configure"
#define TOPIC_REPORTING_READ_RSP        "zigbee2mqtt/bridge/response/device/reporting/read"

/* Cluster name mapping */
typedef struct {
    uint16_t cluster_id;
    const char *name;
} cluster_name_map_t;

static const cluster_name_map_t s_cluster_names[] = {
    { 0x0000, "genBasic" },
    { 0x0001, "genPowerCfg" },
    { 0x0003, "genIdentify" },
    { 0x0006, "genOnOff" },
    { 0x0008, "genLevelCtrl" },
    { 0x0101, "closuresDoorLock" },
    { 0x0102, "closuresWindowCovering" },
    { 0x0201, "hvacThermostat" },
    { 0x0202, "hvacFanCtrl" },
    { 0x0300, "lightingColorCtrl" },
    { 0x0400, "msIlluminanceMeasurement" },
    { 0x0402, "msTemperatureMeasurement" },
    { 0x0403, "msPressureMeasurement" },
    { 0x0405, "msRelativeHumidity" },
    { 0x0406, "msOccupancySensing" },
    { 0x0500, "ssIasZone" },
    { 0x0702, "seMetering" },
    { 0x0B04, "haElectricalMeasurement" },
    { 0x042A, "pm25Measurement" },
    { 0, NULL }  /* Sentinel */
};

/* Attribute name mapping per cluster */
typedef struct {
    uint16_t cluster_id;
    uint16_t attr_id;
    const char *name;
    uint8_t attr_type;
} attr_name_map_t;

static const attr_name_map_t s_attr_names[] = {
    /* Temperature Measurement (0x0402) */
    { 0x0402, 0x0000, "measuredValue", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0402, 0x0001, "minMeasuredValue", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0402, 0x0002, "maxMeasuredValue", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0402, 0x0003, "tolerance", ESP_ZB_ZCL_ATTR_TYPE_U16 },

    /* Relative Humidity (0x0405) */
    { 0x0405, 0x0000, "measuredValue", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0405, 0x0001, "minMeasuredValue", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0405, 0x0002, "maxMeasuredValue", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0405, 0x0003, "tolerance", ESP_ZB_ZCL_ATTR_TYPE_U16 },

    /* Pressure Measurement (0x0403) */
    { 0x0403, 0x0000, "measuredValue", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0403, 0x0010, "scaledValue", ESP_ZB_ZCL_ATTR_TYPE_S16 },

    /* Illuminance Measurement (0x0400) */
    { 0x0400, 0x0000, "measuredValue", ESP_ZB_ZCL_ATTR_TYPE_U16 },

    /* Occupancy Sensing (0x0406) */
    { 0x0406, 0x0000, "occupancy", ESP_ZB_ZCL_ATTR_TYPE_8BITMAP },

    /* IAS Zone (0x0500) */
    { 0x0500, 0x0002, "zoneStatus", ESP_ZB_ZCL_ATTR_TYPE_16BITMAP },

    /* Power Configuration (0x0001) */
    { 0x0001, 0x0020, "batteryVoltage", ESP_ZB_ZCL_ATTR_TYPE_U8 },
    { 0x0001, 0x0021, "batteryPercentageRemaining", ESP_ZB_ZCL_ATTR_TYPE_U8 },

    /* On/Off (0x0006) */
    { 0x0006, 0x0000, "onOff", ESP_ZB_ZCL_ATTR_TYPE_BOOL },

    /* Level Control (0x0008) */
    { 0x0008, 0x0000, "currentLevel", ESP_ZB_ZCL_ATTR_TYPE_U8 },

    /* Color Control (0x0300) */
    { 0x0300, 0x0000, "currentHue", ESP_ZB_ZCL_ATTR_TYPE_U8 },
    { 0x0300, 0x0001, "currentSaturation", ESP_ZB_ZCL_ATTR_TYPE_U8 },
    { 0x0300, 0x0003, "currentX", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0300, 0x0004, "currentY", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0300, 0x0007, "colorTemperature", ESP_ZB_ZCL_ATTR_TYPE_U16 },

    /* Metering (0x0702) */
    { 0x0702, 0x0000, "currentSummationDelivered", ESP_ZB_ZCL_ATTR_TYPE_U48 },
    { 0x0702, 0x0400, "instantaneousDemand", ESP_ZB_ZCL_ATTR_TYPE_S24 },

    /* Electrical Measurement (0x0B04) */
    { 0x0B04, 0x0505, "rmsVoltage", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0B04, 0x0508, "rmsCurrent", ESP_ZB_ZCL_ATTR_TYPE_U16 },
    { 0x0B04, 0x050B, "activePower", ESP_ZB_ZCL_ATTR_TYPE_S16 },

    /* Thermostat (0x0201) */
    { 0x0201, 0x0000, "localTemperature", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0201, 0x0011, "occupiedCoolingSetpoint", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0201, 0x0012, "occupiedHeatingSetpoint", ESP_ZB_ZCL_ATTR_TYPE_S16 },
    { 0x0201, 0x001C, "systemMode", ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM },

    /* Window Covering (0x0102) */
    { 0x0102, 0x0008, "currentPositionLiftPercentage", ESP_ZB_ZCL_ATTR_TYPE_U8 },
    { 0x0102, 0x0009, "currentPositionTiltPercentage", ESP_ZB_ZCL_ATTR_TYPE_U8 },

    /* Door Lock (0x0101) */
    { 0x0101, 0x0000, "lockState", ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM },

    /* PM2.5 (0x042A) */
    { 0x042A, 0x0000, "measuredValue", ESP_ZB_ZCL_ATTR_TYPE_SINGLE },

    { 0, 0, NULL, 0 }  /* Sentinel */
};

/* Default reporting configurations */
static const zb_reporting_default_t s_defaults[] = {
    /* Temperature: min 10s, max 3600s (1hr), change 10 (0.1C) */
    { 0x0402, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_S16, 10, 3600, 10,
      "msTemperatureMeasurement", "measuredValue" },

    /* Humidity: min 10s, max 3600s, change 100 (1%) */
    { 0x0405, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U16, 10, 3600, ZB_HUMIDITY_REPORTING_THRESHOLD,
      "msRelativeHumidity", "measuredValue" },

    /* Pressure: min 60s, max 3600s, change 10 (1 hPa) */
    { 0x0403, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_S16, 60, 3600, 10,
      "msPressureMeasurement", "measuredValue" },

    /* Illuminance: min 10s, max 3600s, change 5000 */
    { 0x0400, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U16, 10, 3600, ZB_ILLUMINANCE_REPORTING_THRESHOLD,
      "msIlluminanceMeasurement", "measuredValue" },

    /* Occupancy: min 0s (immediate), max 3600s, change 0 (any) */
    { 0x0406, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, 0, 3600, 0,
      "msOccupancySensing", "occupancy" },

    /* IAS Zone: min 0s (immediate), max 3600s, change 0 (any) */
    { 0x0500, 0x0002, ESP_ZB_ZCL_ATTR_TYPE_16BITMAP, 0, 3600, 0,
      "ssIasZone", "zoneStatus" },

    /* Battery: min 3600s (1hr), max 43200s (12hr), change 0 */
    { 0x0001, 0x0021, ESP_ZB_ZCL_ATTR_TYPE_U8, 3600, 43200, 0,
      "genPowerCfg", "batteryPercentageRemaining" },

    /* On/Off: min 0s, max 3600s, change 0 (any) */
    { 0x0006, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_BOOL, 0, 3600, 0,
      "genOnOff", "onOff" },

    /* Level: min 1s, max 3600s, change 1 */
    { 0x0008, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U8, 1, 3600, 1,
      "genLevelCtrl", "currentLevel" },

    /* Color Temperature: min 1s, max 3600s, change 1 */
    { 0x0300, 0x0007, ESP_ZB_ZCL_ATTR_TYPE_U16, 1, 3600, 1,
      "lightingColorCtrl", "colorTemperature" },

    /* Power: min 10s, max 600s, change 5 (5W) */
    { 0x0B04, 0x050B, ESP_ZB_ZCL_ATTR_TYPE_S16, 10, 600, 5,
      "haElectricalMeasurement", "activePower" },

    /* Metering: min 60s, max 3600s, change 0 */
    { 0x0702, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_U48, 60, 3600, 0,
      "seMetering", "currentSummationDelivered" },

    /* Window Covering: min 1s, max 3600s, change 1 */
    { 0x0102, 0x0008, ESP_ZB_ZCL_ATTR_TYPE_U8, 1, 3600, 1,
      "closuresWindowCovering", "currentPositionLiftPercentage" },

    /* Thermostat local temp: min 10s, max 3600s, change 10 */
    { 0x0201, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_S16, 10, 3600, 10,
      "hvacThermostat", "localTemperature" },

    /* PM2.5: min 60s, max 3600s, change 0 */
    { 0x042A, 0x0000, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, 60, 3600, 0,
      "pm25Measurement", "measuredValue" },

    { 0, 0, 0, 0, 0, 0, NULL, NULL }  /* Sentinel */
};

/* Forward declarations */
static zb_reporting_entry_t* find_entry(uint64_t device_ieee, uint8_t endpoint,
                                         uint16_t cluster_id, uint16_t attr_id);
static zb_reporting_entry_t* find_free_slot(void);
static esp_err_t send_configure_reporting(uint16_t short_addr, uint8_t endpoint,
                                           zb_reporting_config_t *config);
static esp_err_t send_read_reporting(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t cluster_id, uint16_t attr_id);
static void notify_event(zb_reporting_event_type_t event, uint64_t device_ieee,
                         uint8_t endpoint, const zb_reporting_config_t *config,
                         zb_reporting_status_t status);
static esp_err_t parse_mqtt_configure_request(const char *payload,
                                               char *device_id, size_t device_len,
                                               uint8_t *endpoint,
                                               zb_reporting_config_t *config);
static esp_err_t parse_mqtt_read_request(const char *payload,
                                          char *device_id, size_t device_len,
                                          uint8_t *endpoint,
                                          uint16_t *cluster_id, uint16_t *attr_id);
static uint64_t resolve_device_ieee(const char *device_str, uint16_t *short_addr,
                                     uint8_t *default_endpoint);
static void add_pending_request(uint16_t short_addr, uint8_t endpoint,
                                uint16_t cluster_id, uint16_t attr_id,
                                bool is_configure, const zb_reporting_config_t *config);
static zb_reporting_pending_t* find_pending_request(uint16_t short_addr,
                                                     uint16_t cluster_id,
                                                     uint16_t attr_id);
static void remove_pending_request(uint16_t short_addr, uint16_t cluster_id,
                                    uint16_t attr_id);

esp_err_t zb_reporting_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Reporting module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ZCL reporting configuration...");

    /* Create mutex */
    s_reporting_mutex = xSemaphoreCreateMutex();
    if (s_reporting_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create reporting mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate entry storage - prefer PSRAM to free internal RAM for WiFi */
    s_entries = heap_caps_calloc(ZB_REPORTING_MAX_ENTRIES, sizeof(zb_reporting_entry_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_entries == NULL) {
        /* Fallback to internal RAM if PSRAM not available */
        s_entries = calloc(ZB_REPORTING_MAX_ENTRIES, sizeof(zb_reporting_entry_t));
    }
    if (s_entries == NULL) {
        ESP_LOGE(TAG, "Failed to allocate reporting storage");
        vSemaphoreDelete(s_reporting_mutex);
        s_reporting_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Reporting storage allocated: %zu bytes",
             ZB_REPORTING_MAX_ENTRIES * sizeof(zb_reporting_entry_t));

    /* Initialize all entries as inactive */
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        s_entries[i].active = false;
    }

    /* Initialize pending requests */
    memset(s_pending_requests, 0, sizeof(s_pending_requests));

    s_entry_count = 0;
    s_initialized = true;

    /* Try to load from NVS */
    esp_err_t ret = zb_reporting_load_from_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded %zu reporting configurations from NVS", s_entry_count);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved reporting configurations found");
    } else {
        ESP_LOGW(TAG, "Failed to load configurations from NVS: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Reporting configuration initialized (max: %d entries)",
             ZB_REPORTING_MAX_ENTRIES);
    return ESP_OK;
}

esp_err_t zb_reporting_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing ZCL reporting configuration...");

    /* Save configurations before shutdown */
    zb_reporting_save_to_nvs();

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    if (s_entries != NULL) {
        free(s_entries);
        s_entries = NULL;
    }

    s_entry_count = 0;
    s_initialized = false;
    s_event_callback = NULL;

    xSemaphoreGive(s_reporting_mutex);
    vSemaphoreDelete(s_reporting_mutex);
    s_reporting_mutex = NULL;

    ESP_LOGI(TAG, "Reporting configuration deinitialized");
    return ESP_OK;
}

esp_err_t zb_reporting_configure(uint16_t short_addr, uint8_t endpoint,
                                  zb_reporting_config_t *config)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Reporting module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Configuring reporting: addr=0x%04X ep=%d cluster=0x%04X attr=0x%04X",
             short_addr, endpoint, config->cluster_id, config->attr_id);
    ESP_LOGI(TAG, "  min=%ds max=%ds change=%d type=0x%02X",
             config->min_interval, config->max_interval,
             config->reportable_change, config->attr_type);

    /* Get device IEEE address */
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGE(TAG, "Device 0x%04X not found", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    uint64_t device_ieee = 0;
    memcpy(&device_ieee, device->ieee_addr, sizeof(uint64_t));

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    /* Find or create entry */
    zb_reporting_entry_t *entry = find_entry(device_ieee, endpoint,
                                              config->cluster_id, config->attr_id);
    if (entry == NULL) {
        entry = find_free_slot();
        if (entry == NULL) {
            xSemaphoreGive(s_reporting_mutex);
            ESP_LOGE(TAG, "Reporting table full (max: %d)", ZB_REPORTING_MAX_ENTRIES);
            return ESP_ERR_NO_MEM;
        }
        entry->device_ieee = device_ieee;
        entry->endpoint = endpoint;
        entry->active = true;
        s_entry_count++;
    }

    /* Update configuration */
    memcpy(&entry->config, config, sizeof(zb_reporting_config_t));

    xSemaphoreGive(s_reporting_mutex);

    /* Send Configure Reporting command */
    esp_err_t ret = send_configure_reporting(short_addr, endpoint, config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Configure Reporting: %s", esp_err_to_name(ret));
        /* Keep configuration anyway - device might be temporarily offline */
    }

    /* Track pending request */
    add_pending_request(short_addr, endpoint, config->cluster_id,
                        config->attr_id, true, config);

    /* Save to NVS */
    zb_reporting_save_to_nvs();

    /* Notify event */
    notify_event(ZB_REPORTING_EVENT_CONFIGURED, device_ieee, endpoint,
                 config, ZB_REPORTING_STATUS_SUCCESS);

    return ESP_OK;
}

esp_err_t zb_reporting_read(uint16_t short_addr, uint8_t endpoint,
                            uint16_t cluster_id, uint16_t attr_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading reporting config: addr=0x%04X ep=%d cluster=0x%04X attr=0x%04X",
             short_addr, endpoint, cluster_id, attr_id);

    /* Verify device exists */
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGE(TAG, "Device 0x%04X not found", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Send Read Reporting Configuration command */
    esp_err_t ret = send_read_reporting(short_addr, endpoint, cluster_id, attr_id);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Read Reporting Config: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Track pending request */
    add_pending_request(short_addr, endpoint, cluster_id, attr_id, false, NULL);

    return ESP_OK;
}

esp_err_t zb_reporting_set_defaults(uint16_t short_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Setting default reporting for device 0x%04X", short_addr);

    /* Get device info */
    zb_device_t *device = zb_device_get(short_addr);
    if (device == NULL) {
        ESP_LOGE(TAG, "Device 0x%04X not found", short_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check device clusters and configure defaults */
    size_t configured = 0;
    for (const zb_reporting_default_t *def = s_defaults; def->cluster_name != NULL; def++) {
        /* Check if device has this cluster */
        bool has_cluster = false;
        for (uint16_t i = 0; i < device->cluster_count; i++) {
            if (device->clusters[i] == def->cluster_id) {
                has_cluster = true;
                break;
            }
        }

        if (has_cluster) {
            zb_reporting_config_t config = {
                .cluster_id = def->cluster_id,
                .attr_id = def->attr_id,
                .attr_type = def->attr_type,
                .min_interval = def->min_interval,
                .max_interval = def->max_interval,
                .reportable_change = def->reportable_change
            };

            esp_err_t ret = zb_reporting_configure(short_addr, device->endpoint, &config);
            if (ret == ESP_OK) {
                configured++;
                ESP_LOGI(TAG, "Configured default reporting for %s/%s",
                         def->cluster_name, def->attr_name);
            } else {
                ESP_LOGW(TAG, "Failed to configure %s/%s: %s",
                         def->cluster_name, def->attr_name, esp_err_to_name(ret));
            }

            /* Small delay between commands to avoid overwhelming the device */
            vTaskDelay(pdMS_TO_TICKS(GW_TIMEOUT_SHORT_MS));
        }
    }

    ESP_LOGI(TAG, "Configured %zu default reporting entries for device 0x%04X",
             configured, short_addr);

    return configured > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t zb_reporting_process_mqtt_request(const char *topic, const char *payload)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing MQTT request: %s", topic);
    ESP_LOGD(TAG, "Payload: %s", payload);

    bool is_configure = (strcmp(topic, TOPIC_REPORTING_CONFIGURE_REQ) == 0);
    bool is_read = (strcmp(topic, TOPIC_REPORTING_READ_REQ) == 0);

    if (!is_configure && !is_read) {
        ESP_LOGW(TAG, "Unknown reporting topic: %s", topic);
        return ESP_ERR_INVALID_ARG;
    }

    char device_id[ZB_REPORTING_NAME_MAX_LEN] = {0};
    uint8_t endpoint = 1;
    zb_reporting_config_t config = {0};
    uint16_t cluster_id = 0;
    uint16_t attr_id = 0;
    esp_err_t ret;

    if (is_configure) {
        ret = parse_mqtt_configure_request(payload, device_id, sizeof(device_id),
                                           &endpoint, &config);
        if (ret != ESP_OK) {
            zb_reporting_result_t result = {
                .success = false,
                .status = ZB_REPORTING_STATUS_ERROR,
            };
            strncpy(result.error_message, "Invalid request format",
                    sizeof(result.error_message) - 1);
            zb_reporting_publish_configure_response(&result);
            return ret;
        }

        /* Resolve device */
        uint16_t short_addr = 0;
        uint8_t default_ep = 1;
        uint64_t device_ieee = resolve_device_ieee(device_id, &short_addr, &default_ep);

        if (device_ieee == 0 || short_addr == 0) {
            zb_reporting_result_t result = {
                .success = false,
                .status = ZB_REPORTING_STATUS_DEVICE_NOT_FOUND,
            };
            strncpy(result.device_name, device_id, sizeof(result.device_name) - 1);
            snprintf(result.error_message, sizeof(result.error_message),
                     "Device not found: %s", device_id);
            zb_reporting_publish_configure_response(&result);
            return ESP_ERR_NOT_FOUND;
        }

        if (endpoint == 0) {
            endpoint = default_ep;
        }

        /* Configure reporting */
        ret = zb_reporting_configure(short_addr, endpoint, &config);

        /* Publish response */
        zb_reporting_result_t result = {
            .success = (ret == ESP_OK),
            .status = (ret == ESP_OK) ? ZB_REPORTING_STATUS_SUCCESS : ZB_REPORTING_STATUS_ERROR,
            .endpoint = endpoint,
        };
        strncpy(result.device_name, device_id, sizeof(result.device_name) - 1);
        memcpy(&result.config, &config, sizeof(config));
        if (ret != ESP_OK) {
            snprintf(result.error_message, sizeof(result.error_message),
                     "Configuration failed: %s", esp_err_to_name(ret));
        }
        zb_reporting_publish_configure_response(&result);

    } else {  /* is_read */
        ret = parse_mqtt_read_request(payload, device_id, sizeof(device_id),
                                      &endpoint, &cluster_id, &attr_id);
        if (ret != ESP_OK) {
            zb_reporting_result_t result = {
                .success = false,
                .status = ZB_REPORTING_STATUS_ERROR,
            };
            strncpy(result.error_message, "Invalid request format",
                    sizeof(result.error_message) - 1);
            zb_reporting_publish_read_response(&result);
            return ret;
        }

        /* Resolve device */
        uint16_t short_addr = 0;
        uint8_t default_ep = 1;
        uint64_t device_ieee = resolve_device_ieee(device_id, &short_addr, &default_ep);

        if (device_ieee == 0 || short_addr == 0) {
            zb_reporting_result_t result = {
                .success = false,
                .status = ZB_REPORTING_STATUS_DEVICE_NOT_FOUND,
            };
            strncpy(result.device_name, device_id, sizeof(result.device_name) - 1);
            snprintf(result.error_message, sizeof(result.error_message),
                     "Device not found: %s", device_id);
            zb_reporting_publish_read_response(&result);
            return ESP_ERR_NOT_FOUND;
        }

        if (endpoint == 0) {
            endpoint = default_ep;
        }

        /* Read reporting configuration */
        ret = zb_reporting_read(short_addr, endpoint, cluster_id, attr_id);
        if (ret != ESP_OK) {
            zb_reporting_result_t result = {
                .success = false,
                .status = ZB_REPORTING_STATUS_ERROR,
                .endpoint = endpoint,
            };
            strncpy(result.device_name, device_id, sizeof(result.device_name) - 1);
            result.config.cluster_id = cluster_id;
            result.config.attr_id = attr_id;
            snprintf(result.error_message, sizeof(result.error_message),
                     "Read failed: %s", esp_err_to_name(ret));
            zb_reporting_publish_read_response(&result);
        }
        /* Response will be sent when we receive the device's response */
    }

    return ESP_OK;
}

esp_err_t zb_reporting_handle_configure_response(uint16_t short_addr,
                                                   uint8_t endpoint,
                                                   uint16_t cluster_id,
                                                   uint8_t status,
                                                   uint16_t attr_id)
{
    ESP_LOGI(TAG, "Configure Reporting Response: addr=0x%04X ep=%d cluster=0x%04X "
             "attr=0x%04X status=0x%02X",
             short_addr, endpoint, cluster_id, attr_id, status);

    zb_reporting_pending_t *pending = find_pending_request(short_addr, cluster_id, attr_id);
    if (pending == NULL) {
        ESP_LOGW(TAG, "No pending configure request for this response");
        return ESP_ERR_NOT_FOUND;
    }

    zb_reporting_status_t rpt_status;
    if (status == 0x00) {
        rpt_status = ZB_REPORTING_STATUS_SUCCESS;
        ESP_LOGI(TAG, "Configure Reporting successful");
    } else if (status == 0x86) {  /* UNSUPPORTED_ATTRIBUTE */
        rpt_status = ZB_REPORTING_STATUS_INVALID_ATTRIBUTE;
        ESP_LOGW(TAG, "Attribute does not support reporting");
    } else if (status == 0x8D) {  /* INVALID_DATA_TYPE */
        rpt_status = ZB_REPORTING_STATUS_INVALID_DATATYPE;
        ESP_LOGW(TAG, "Invalid data type for reporting");
    } else {
        rpt_status = ZB_REPORTING_STATUS_ERROR;
        ESP_LOGW(TAG, "Configure Reporting failed with status 0x%02X", status);
    }

    /* Get device info for response */
    zb_device_t *device = zb_device_get(short_addr);
    char device_name[ZB_REPORTING_NAME_MAX_LEN];
    if (device != NULL && strlen(device->friendly_name) > 0) {
        strncpy(device_name, device->friendly_name, sizeof(device_name) - 1);
    } else {
        snprintf(device_name, sizeof(device_name), "0x%04X", short_addr);
    }

    /* Publish result */
    zb_reporting_result_t result = {
        .success = (rpt_status == ZB_REPORTING_STATUS_SUCCESS),
        .status = rpt_status,
        .endpoint = endpoint,
    };
    strncpy(result.device_name, device_name, sizeof(result.device_name) - 1);
    memcpy(&result.config, &pending->config, sizeof(result.config));

    if (!result.success) {
        snprintf(result.error_message, sizeof(result.error_message),
                 "Device returned status 0x%02X", status);
    }

    zb_reporting_publish_configure_response(&result);

    /* Notify event */
    uint64_t device_ieee = 0;
    if (device != NULL) {
        memcpy(&device_ieee, device->ieee_addr, sizeof(uint64_t));
    }
    notify_event(result.success ? ZB_REPORTING_EVENT_CONFIGURED : ZB_REPORTING_EVENT_FAILED,
                 device_ieee, endpoint, &result.config, rpt_status);

    remove_pending_request(short_addr, cluster_id, attr_id);
    return ESP_OK;
}

esp_err_t zb_reporting_handle_read_response(uint16_t short_addr,
                                             uint8_t endpoint,
                                             uint16_t cluster_id,
                                             uint8_t status,
                                             uint8_t direction,
                                             uint16_t attr_id,
                                             uint8_t attr_type,
                                             uint16_t min_interval,
                                             uint16_t max_interval,
                                             uint16_t reportable_change)
{
    ESP_LOGI(TAG, "Read Reporting Config Response: addr=0x%04X ep=%d cluster=0x%04X "
             "attr=0x%04X status=0x%02X",
             short_addr, endpoint, cluster_id, attr_id, status);

    if (status == 0x00) {
        ESP_LOGI(TAG, "  direction=%d type=0x%02X min=%d max=%d change=%d",
                 direction, attr_type, min_interval, max_interval, reportable_change);
    }

    zb_reporting_pending_t *pending = find_pending_request(short_addr, cluster_id, attr_id);
    if (pending == NULL) {
        ESP_LOGW(TAG, "No pending read request for this response");
        return ESP_ERR_NOT_FOUND;
    }

    /* Get device info */
    zb_device_t *device = zb_device_get(short_addr);
    char device_name[ZB_REPORTING_NAME_MAX_LEN];
    if (device != NULL && strlen(device->friendly_name) > 0) {
        strncpy(device_name, device->friendly_name, sizeof(device_name) - 1);
    } else {
        snprintf(device_name, sizeof(device_name), "0x%04X", short_addr);
    }

    /* Build result */
    zb_reporting_status_t rpt_status;
    if (status == 0x00) {
        rpt_status = ZB_REPORTING_STATUS_SUCCESS;
    } else if (status == 0x86) {
        rpt_status = ZB_REPORTING_STATUS_INVALID_ATTRIBUTE;
    } else {
        rpt_status = ZB_REPORTING_STATUS_ERROR;
    }

    zb_reporting_result_t result = {
        .success = (rpt_status == ZB_REPORTING_STATUS_SUCCESS),
        .status = rpt_status,
        .endpoint = endpoint,
        .config = {
            .cluster_id = cluster_id,
            .attr_id = attr_id,
            .attr_type = attr_type,
            .min_interval = min_interval,
            .max_interval = max_interval,
            .reportable_change = reportable_change
        }
    };
    strncpy(result.device_name, device_name, sizeof(result.device_name) - 1);

    if (!result.success) {
        snprintf(result.error_message, sizeof(result.error_message),
                 "Device returned status 0x%02X", status);
    }

    zb_reporting_publish_read_response(&result);

    /* Notify event */
    uint64_t device_ieee = 0;
    if (device != NULL) {
        memcpy(&device_ieee, device->ieee_addr, sizeof(uint64_t));
    }
    notify_event(ZB_REPORTING_EVENT_READ, device_ieee, endpoint, &result.config, rpt_status);

    remove_pending_request(short_addr, cluster_id, attr_id);
    return ESP_OK;
}

esp_err_t zb_reporting_publish_configure_response(const zb_reporting_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Build response JSON */
    cJSON *data = cJSON_CreateObject();
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object for configure response data");
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(data, "id", result->device_name);
    cJSON_AddNumberToObject(data, "endpoint", result->endpoint);
    cJSON_AddStringToObject(data, "cluster",
                            zb_reporting_get_cluster_name(result->config.cluster_id));
    cJSON_AddStringToObject(data, "attribute",
                            zb_reporting_get_attr_name(result->config.cluster_id,
                                                       result->config.attr_id));
    cJSON_AddNumberToObject(data, "minimum_report_interval", result->config.min_interval);
    cJSON_AddNumberToObject(data, "maximum_report_interval", result->config.max_interval);
    cJSON_AddNumberToObject(data, "reportable_change", result->config.reportable_change);

    cJSON_AddItemToObject(json, "data", data);
    cJSON_AddStringToObject(json, "status", result->success ? "ok" : "error");

    if (!result->success && strlen(result->error_message) > 0) {
        cJSON_AddStringToObject(json, "error", result->error_message);
    }

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Publishing configure response to %s", TOPIC_REPORTING_CONFIGURE_RSP);
    ESP_LOGD(TAG, "Response: %s", json_str);

    esp_err_t ret = mqtt_client_publish(TOPIC_REPORTING_CONFIGURE_RSP, json_str, 0, 1, false);

    free(json_str);
    return ret;
}

esp_err_t zb_reporting_publish_read_response(const zb_reporting_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Build response JSON */
    cJSON *data = cJSON_CreateObject();
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object for read response data");
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(data, "id", result->device_name);
    cJSON_AddNumberToObject(data, "endpoint", result->endpoint);
    cJSON_AddStringToObject(data, "cluster",
                            zb_reporting_get_cluster_name(result->config.cluster_id));
    cJSON_AddStringToObject(data, "attribute",
                            zb_reporting_get_attr_name(result->config.cluster_id,
                                                       result->config.attr_id));

    if (result->success) {
        cJSON_AddNumberToObject(data, "minimum_report_interval", result->config.min_interval);
        cJSON_AddNumberToObject(data, "maximum_report_interval", result->config.max_interval);
        cJSON_AddNumberToObject(data, "reportable_change", result->config.reportable_change);
        cJSON_AddNumberToObject(data, "attribute_type", result->config.attr_type);
    }

    cJSON_AddItemToObject(json, "data", data);
    cJSON_AddStringToObject(json, "status", result->success ? "ok" : "error");

    if (!result->success && strlen(result->error_message) > 0) {
        cJSON_AddStringToObject(json, "error", result->error_message);
    }

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Publishing read response to %s", TOPIC_REPORTING_READ_RSP);
    ESP_LOGD(TAG, "Response: %s", json_str);

    esp_err_t ret = mqtt_client_publish(TOPIC_REPORTING_READ_RSP, json_str, 0, 1, false);

    free(json_str);
    return ret;
}

const zb_reporting_entry_t* zb_reporting_get_entry(size_t index)
{
    if (!s_initialized || index >= ZB_REPORTING_MAX_ENTRIES) {
        return NULL;
    }

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);
    const zb_reporting_entry_t *entry = NULL;
    if (s_entries[index].active) {
        entry = &s_entries[index];
    }
    xSemaphoreGive(s_reporting_mutex);

    return entry;
}

size_t zb_reporting_get_by_device(uint64_t device_ieee,
                                   zb_reporting_entry_t *entries,
                                   size_t max_count)
{
    if (!s_initialized || entries == NULL) {
        return 0;
    }

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    size_t copied = 0;
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES && copied < max_count; i++) {
        if (s_entries[i].active && s_entries[i].device_ieee == device_ieee) {
            memcpy(&entries[copied], &s_entries[i], sizeof(zb_reporting_entry_t));
            copied++;
        }
    }

    xSemaphoreGive(s_reporting_mutex);

    return copied;
}

size_t zb_reporting_get_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);
    size_t count = s_entry_count;
    xSemaphoreGive(s_reporting_mutex);

    return count;
}

esp_err_t zb_reporting_remove_device(uint64_t device_ieee)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Removing reporting configs for device 0x%016" PRIX64, device_ieee);

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    size_t removed = 0;
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        if (s_entries[i].active && s_entries[i].device_ieee == device_ieee) {
            s_entries[i].active = false;
            removed++;
        }
    }

    s_entry_count -= removed;

    xSemaphoreGive(s_reporting_mutex);

    if (removed > 0) {
        zb_reporting_save_to_nvs();
        ESP_LOGI(TAG, "Removed %zu reporting configurations", removed);
    }

    return ESP_OK;
}

esp_err_t zb_reporting_save_to_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving reporting configurations to NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_REPORTING_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    /* Save entry count */
    ret = nvs_set_u16(nvs_handle, NVS_KEY_ENTRY_COUNT, (uint16_t)s_entry_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save entry count: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_reporting_mutex);
        nvs_close(nvs_handle);
        return ret;
    }

    /* Save each active entry */
    uint8_t saved_idx = 0;
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        if (s_entries[i].active) {
            char key[16];
            snprintf(key, sizeof(key), NVS_KEY_ENTRY_DATA_FMT, saved_idx);

            ret = nvs_set_blob(nvs_handle, key, &s_entries[i], sizeof(zb_reporting_entry_t));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save entry %zu: %s", i, esp_err_to_name(ret));
                xSemaphoreGive(s_reporting_mutex);
                nvs_close(nvs_handle);
                return ret;
            }
            saved_idx++;
        }
    }

    xSemaphoreGive(s_reporting_mutex);

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %zu reporting configurations to NVS", s_entry_count);
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_reporting_load_from_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Loading reporting configurations from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_REPORTING_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found - no saved configurations");
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Load entry count */
    uint16_t count = 0;
    ret = nvs_get_u16(nvs_handle, NVS_KEY_ENTRY_COUNT, &count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load entry count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    xSemaphoreTake(s_reporting_mutex, portMAX_DELAY);

    /* Clear existing entries */
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        s_entries[i].active = false;
    }
    s_entry_count = 0;

    /* Load each entry */
    for (uint8_t i = 0; i < count && i < ZB_REPORTING_MAX_ENTRIES; i++) {
        char key[16];
        snprintf(key, sizeof(key), NVS_KEY_ENTRY_DATA_FMT, i);

        size_t required_size = sizeof(zb_reporting_entry_t);
        ret = nvs_get_blob(nvs_handle, key, &s_entries[i], &required_size);
        if (ret == ESP_OK) {
            s_entries[i].active = true;
            s_entry_count++;
            ESP_LOGD(TAG, "Loaded: device=0x%016" PRIX64 " cluster=0x%04X attr=0x%04X",
                     s_entries[i].device_ieee, s_entries[i].config.cluster_id,
                     s_entries[i].config.attr_id);
        } else {
            ESP_LOGW(TAG, "Failed to load entry %d: %s", i, esp_err_to_name(ret));
        }
    }

    xSemaphoreGive(s_reporting_mutex);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Loaded %zu reporting configurations from NVS", s_entry_count);
    return ESP_OK;
}

esp_err_t zb_reporting_clear_nvs(void)
{
    ESP_LOGI(TAG, "Clearing reporting configurations from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_REPORTING_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(nvs_handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Cleared all reporting configurations from NVS");
    }

    return ret;
}

esp_err_t zb_reporting_register_callback(zb_reporting_event_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

const char* zb_reporting_get_cluster_name(uint16_t cluster_id)
{
    for (const cluster_name_map_t *entry = s_cluster_names; entry->name != NULL; entry++) {
        if (entry->cluster_id == cluster_id) {
            return entry->name;
        }
    }

    static char unknown_cluster[16];
    snprintf(unknown_cluster, sizeof(unknown_cluster), "0x%04X", cluster_id);
    return unknown_cluster;
}

uint16_t zb_reporting_get_cluster_id(const char *name)
{
    if (name == NULL) {
        return 0xFFFF;
    }

    /* Check if it's a hex string */
    if (strncmp(name, "0x", 2) == 0 || strncmp(name, "0X", 2) == 0) {
        return (uint16_t)strtoul(name, NULL, 16);
    }

    /* Look up by name */
    for (const cluster_name_map_t *entry = s_cluster_names; entry->name != NULL; entry++) {
        if (strcasecmp(entry->name, name) == 0) {
            return entry->cluster_id;
        }
    }

    return 0xFFFF;
}

const char* zb_reporting_get_attr_name(uint16_t cluster_id, uint16_t attr_id)
{
    for (const attr_name_map_t *entry = s_attr_names; entry->name != NULL; entry++) {
        if (entry->cluster_id == cluster_id && entry->attr_id == attr_id) {
            return entry->name;
        }
    }

    static char unknown_attr[16];
    snprintf(unknown_attr, sizeof(unknown_attr), "0x%04X", attr_id);
    return unknown_attr;
}

uint16_t zb_reporting_get_attr_id(uint16_t cluster_id, const char *name)
{
    if (name == NULL) {
        return 0xFFFF;
    }

    /* Check if it's a hex string */
    if (strncmp(name, "0x", 2) == 0 || strncmp(name, "0X", 2) == 0) {
        return (uint16_t)strtoul(name, NULL, 16);
    }

    /* Look up by name */
    for (const attr_name_map_t *entry = s_attr_names; entry->name != NULL; entry++) {
        if (entry->cluster_id == cluster_id && strcasecmp(entry->name, name) == 0) {
            return entry->attr_id;
        }
    }

    return 0xFFFF;
}

const zb_reporting_default_t* zb_reporting_get_default(uint16_t cluster_id,
                                                        uint16_t attr_id)
{
    for (const zb_reporting_default_t *def = s_defaults; def->cluster_name != NULL; def++) {
        if (def->cluster_id == cluster_id && def->attr_id == attr_id) {
            return def;
        }
    }
    return NULL;
}

esp_err_t zb_reporting_test(void)
{
    ESP_LOGI(TAG, "Running reporting configuration self-test...");

    if (!s_initialized) {
        ESP_LOGE(TAG, "Reporting module not initialized");
        return ESP_FAIL;
    }

    /* Test 1: Cluster name lookup */
    const char *name = zb_reporting_get_cluster_name(0x0402);
    if (strcmp(name, "msTemperatureMeasurement") != 0) {
        ESP_LOGE(TAG, "Test 1 FAILED: Wrong cluster name: %s", name);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 1 PASSED: Cluster name = %s", name);

    /* Test 2: Cluster ID lookup */
    uint16_t cluster_id = zb_reporting_get_cluster_id("msTemperatureMeasurement");
    if (cluster_id != 0x0402) {
        ESP_LOGE(TAG, "Test 2 FAILED: Wrong cluster ID: 0x%04X", cluster_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 2 PASSED: Cluster ID = 0x%04X", cluster_id);

    /* Test 3: Attribute name lookup */
    const char *attr_name = zb_reporting_get_attr_name(0x0402, 0x0000);
    if (strcmp(attr_name, "measuredValue") != 0) {
        ESP_LOGE(TAG, "Test 3 FAILED: Wrong attr name: %s", attr_name);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 3 PASSED: Attribute name = %s", attr_name);

    /* Test 4: Attribute ID lookup */
    uint16_t attr_id = zb_reporting_get_attr_id(0x0402, "measuredValue");
    if (attr_id != 0x0000) {
        ESP_LOGE(TAG, "Test 4 FAILED: Wrong attr ID: 0x%04X", attr_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 4 PASSED: Attribute ID = 0x%04X", attr_id);

    /* Test 5: Default config lookup */
    const zb_reporting_default_t *def = zb_reporting_get_default(0x0402, 0x0000);
    if (def == NULL) {
        ESP_LOGE(TAG, "Test 5 FAILED: Default config not found");
        return ESP_FAIL;
    }
    if (def->min_interval != ZB_REPORT_MIN_INTERVAL_SEC || def->max_interval != ZB_REPORT_MAX_INTERVAL_SEC) {
        ESP_LOGE(TAG, "Test 5 FAILED: Wrong default values");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 5 PASSED: Default min=%d max=%d",
             def->min_interval, def->max_interval);

    /* Test 6: Get count */
    size_t count = zb_reporting_get_count();
    ESP_LOGI(TAG, "Test 6 PASSED: Current entry count = %zu", count);

    ESP_LOGI(TAG, "Reporting configuration self-test PASSED (all 6 tests)");
    return ESP_OK;
}

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

static zb_reporting_entry_t* find_entry(uint64_t device_ieee, uint8_t endpoint,
                                         uint16_t cluster_id, uint16_t attr_id)
{
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        if (s_entries[i].active &&
            s_entries[i].device_ieee == device_ieee &&
            s_entries[i].endpoint == endpoint &&
            s_entries[i].config.cluster_id == cluster_id &&
            s_entries[i].config.attr_id == attr_id) {
            return &s_entries[i];
        }
    }
    return NULL;
}

static zb_reporting_entry_t* find_free_slot(void)
{
    for (size_t i = 0; i < ZB_REPORTING_MAX_ENTRIES; i++) {
        if (!s_entries[i].active) {
            return &s_entries[i];
        }
    }
    return NULL;
}

static esp_err_t send_configure_reporting(uint16_t short_addr, uint8_t endpoint,
                                           zb_reporting_config_t *config)
{
    ESP_LOGD(TAG, "Sending Configure Reporting: addr=0x%04X ep=%d cluster=0x%04X attr=0x%04X",
             short_addr, endpoint, config->cluster_id, config->attr_id);

    /* Build the Configure Reporting command payload
     * ZCL Spec 2.5.7.1.3 Configure Reporting Command Payload:
     * - Direction (1 byte): 0x00 = attribute reported
     * - Attribute ID (2 bytes)
     * - Attribute Data Type (1 byte)
     * - Minimum Reporting Interval (2 bytes)
     * - Maximum Reporting Interval (2 bytes)
     * - Reportable Change (variable, depends on type)
     */

    /* Calculate reportable change size based on attribute type */
    size_t change_size = 0;
    switch (config->attr_type) {
        case ESP_ZB_ZCL_ATTR_TYPE_BOOL:
        case ESP_ZB_ZCL_ATTR_TYPE_8BIT:
        case ESP_ZB_ZCL_ATTR_TYPE_U8:
        case ESP_ZB_ZCL_ATTR_TYPE_S8:
        case ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM:
        case ESP_ZB_ZCL_ATTR_TYPE_8BITMAP:
            change_size = 1;
            break;
        case ESP_ZB_ZCL_ATTR_TYPE_16BIT:
        case ESP_ZB_ZCL_ATTR_TYPE_U16:
        case ESP_ZB_ZCL_ATTR_TYPE_S16:
        case ESP_ZB_ZCL_ATTR_TYPE_16BIT_ENUM:
        case ESP_ZB_ZCL_ATTR_TYPE_16BITMAP:
            change_size = 2;
            break;
        case ESP_ZB_ZCL_ATTR_TYPE_32BIT:
        case ESP_ZB_ZCL_ATTR_TYPE_U32:
        case ESP_ZB_ZCL_ATTR_TYPE_S32:
        case ESP_ZB_ZCL_ATTR_TYPE_SINGLE:
        case ESP_ZB_ZCL_ATTR_TYPE_32BITMAP:
            change_size = 4;
            break;
        case ESP_ZB_ZCL_ATTR_TYPE_U48:
        case ESP_ZB_ZCL_ATTR_TYPE_S48:
            change_size = 6;
            break;
        case ESP_ZB_ZCL_ATTR_TYPE_64BIT:
        case ESP_ZB_ZCL_ATTR_TYPE_U64:
        case ESP_ZB_ZCL_ATTR_TYPE_S64:
        case ESP_ZB_ZCL_ATTR_TYPE_DOUBLE:
        case ESP_ZB_ZCL_ATTR_TYPE_64BITMAP:
            change_size = 8;
            break;
        default:
            /* For discrete types, no reportable change */
            change_size = 0;
            break;
    }

    /* Use ESP-Zigbee SDK to send Configure Reporting command */
    esp_zb_zcl_config_report_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint = endpoint,
            .src_endpoint = 1,  /* Coordinator endpoint */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = config->cluster_id,
    };

    /* Create report configuration record */
    esp_zb_zcl_config_report_record_t record = {
        .direction = ZCL_REPORTING_DIRECTION_REPORTED,
        .attributeID = config->attr_id,
        .attrType = config->attr_type,
        .min_interval = config->min_interval,
        .max_interval = config->max_interval,
    };

    /* Set reportable change value */
    memcpy(&record.reportable_change, &config->reportable_change,
           change_size > sizeof(config->reportable_change) ?
           sizeof(config->reportable_change) : change_size);

    /* Create record list */
    esp_zb_zcl_config_report_record_t *records = &record;
    cmd.record_number = 1;
    cmd.record_field = records;

    /* Send command with thread-safety lock */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_config_report_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGD(TAG, "Configure Reporting command sent");
    return ESP_OK;
}

static esp_err_t send_read_reporting(uint16_t short_addr, uint8_t endpoint,
                                      uint16_t cluster_id, uint16_t attr_id)
{
    ESP_LOGD(TAG, "Sending Read Reporting Config: addr=0x%04X ep=%d cluster=0x%04X attr=0x%04X",
             short_addr, endpoint, cluster_id, attr_id);

    /* Use ESP-Zigbee SDK to send Read Reporting Configuration command */
    esp_zb_zcl_read_report_config_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint = endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster_id,
    };

    /* Create read reporting record */
    esp_zb_zcl_attribute_record_t record = {
        .report_direction = ZCL_REPORTING_DIRECTION_REPORTED,
        .attributeID = attr_id,
    };

    esp_zb_zcl_attribute_record_t *records = &record;
    cmd.record_number = 1;
    cmd.record_field = records;

    /* Send command with thread-safety lock */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_report_config_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGD(TAG, "Read Reporting Config command sent");
    return ESP_OK;
}

static void notify_event(zb_reporting_event_type_t event, uint64_t device_ieee,
                         uint8_t endpoint, const zb_reporting_config_t *config,
                         zb_reporting_status_t status)
{
    if (s_event_callback != NULL) {
        s_event_callback(event, device_ieee, endpoint, config, status);
    }
}

static esp_err_t parse_mqtt_configure_request(const char *payload,
                                               char *device_id, size_t device_len,
                                               uint8_t *endpoint,
                                               zb_reporting_config_t *config)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON payload");
        return ESP_FAIL;
    }

    /* Get device ID */
    cJSON *id = cJSON_GetObjectItem(json, "id");
    if (id == NULL || !cJSON_IsString(id)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'id' field");
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(device_id, id->valuestring, device_len - 1);
    device_id[device_len - 1] = '\0';

    /* Get endpoint (optional, default 0 means use device's default) */
    cJSON *ep = cJSON_GetObjectItem(json, "endpoint");
    *endpoint = (ep != NULL && cJSON_IsNumber(ep)) ? (uint8_t)ep->valueint : 0;

    /* Get cluster */
    cJSON *cluster = cJSON_GetObjectItem(json, "cluster");
    if (cluster == NULL || !cJSON_IsString(cluster)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'cluster' field");
        return ESP_ERR_INVALID_ARG;
    }
    config->cluster_id = zb_reporting_get_cluster_id(cluster->valuestring);
    if (config->cluster_id == 0xFFFF) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Unknown cluster: %s", cluster->valuestring);
        return ESP_ERR_INVALID_ARG;
    }

    /* Get attribute */
    cJSON *attr = cJSON_GetObjectItem(json, "attribute");
    if (attr == NULL || !cJSON_IsString(attr)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'attribute' field");
        return ESP_ERR_INVALID_ARG;
    }
    config->attr_id = zb_reporting_get_attr_id(config->cluster_id, attr->valuestring);
    if (config->attr_id == 0xFFFF) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Unknown attribute: %s", attr->valuestring);
        return ESP_ERR_INVALID_ARG;
    }

    /* Get attribute type from defaults or payload */
    const zb_reporting_default_t *def = zb_reporting_get_default(config->cluster_id,
                                                                  config->attr_id);
    if (def != NULL) {
        config->attr_type = def->attr_type;
    } else {
        /* Try to get from attribute name map */
        for (const attr_name_map_t *entry = s_attr_names; entry->name != NULL; entry++) {
            if (entry->cluster_id == config->cluster_id &&
                entry->attr_id == config->attr_id) {
                config->attr_type = entry->attr_type;
                break;
            }
        }
    }

    /* Override with explicit type if provided */
    cJSON *type = cJSON_GetObjectItem(json, "attribute_type");
    if (type != NULL && cJSON_IsNumber(type)) {
        config->attr_type = (uint8_t)type->valueint;
    }

    /* Get intervals */
    cJSON *min = cJSON_GetObjectItem(json, "minimum_report_interval");
    cJSON *max = cJSON_GetObjectItem(json, "maximum_report_interval");
    cJSON *change = cJSON_GetObjectItem(json, "reportable_change");

    if (min != NULL && cJSON_IsNumber(min)) {
        config->min_interval = (uint16_t)min->valueint;
    } else if (def != NULL) {
        config->min_interval = def->min_interval;
    } else {
        config->min_interval = ZB_REPORT_MIN_INTERVAL_SEC;  /* Default */
    }

    if (max != NULL && cJSON_IsNumber(max)) {
        config->max_interval = (uint16_t)max->valueint;
    } else if (def != NULL) {
        config->max_interval = def->max_interval;
    } else {
        config->max_interval = ZB_REPORT_MAX_INTERVAL_SEC;  /* Default */
    }

    if (change != NULL && cJSON_IsNumber(change)) {
        config->reportable_change = (uint16_t)change->valueint;
    } else if (def != NULL) {
        config->reportable_change = def->reportable_change;
    } else {
        config->reportable_change = 0;  /* Default */
    }

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t parse_mqtt_read_request(const char *payload,
                                          char *device_id, size_t device_len,
                                          uint8_t *endpoint,
                                          uint16_t *cluster_id, uint16_t *attr_id)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON payload");
        return ESP_FAIL;
    }

    /* Get device ID */
    cJSON *id = cJSON_GetObjectItem(json, "id");
    if (id == NULL || !cJSON_IsString(id)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'id' field");
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(device_id, id->valuestring, device_len - 1);
    device_id[device_len - 1] = '\0';

    /* Get endpoint */
    cJSON *ep = cJSON_GetObjectItem(json, "endpoint");
    *endpoint = (ep != NULL && cJSON_IsNumber(ep)) ? (uint8_t)ep->valueint : 0;

    /* Get cluster */
    cJSON *cluster = cJSON_GetObjectItem(json, "cluster");
    if (cluster == NULL || !cJSON_IsString(cluster)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'cluster' field");
        return ESP_ERR_INVALID_ARG;
    }
    *cluster_id = zb_reporting_get_cluster_id(cluster->valuestring);
    if (*cluster_id == 0xFFFF) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Unknown cluster: %s", cluster->valuestring);
        return ESP_ERR_INVALID_ARG;
    }

    /* Get attribute */
    cJSON *attr = cJSON_GetObjectItem(json, "attribute");
    if (attr == NULL || !cJSON_IsString(attr)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'attribute' field");
        return ESP_ERR_INVALID_ARG;
    }
    *attr_id = zb_reporting_get_attr_id(*cluster_id, attr->valuestring);
    if (*attr_id == 0xFFFF) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Unknown attribute: %s", attr->valuestring);
        return ESP_ERR_INVALID_ARG;
    }

    cJSON_Delete(json);
    return ESP_OK;
}

static uint64_t resolve_device_ieee(const char *device_str, uint16_t *short_addr,
                                     uint8_t *default_endpoint)
{
    if (device_str == NULL) {
        return 0;
    }

    *short_addr = 0;
    *default_endpoint = 1;

    /* Check if it's an IEEE address string */
    if (strncmp(device_str, "0x", 2) == 0 || strncmp(device_str, "0X", 2) == 0) {
        uint64_t ieee = strtoull(device_str, NULL, 16);

        /* Find device by IEEE to get short address */
        esp_zb_ieee_addr_t ieee_addr;
        memcpy(ieee_addr, &ieee, sizeof(esp_zb_ieee_addr_t));

        zb_device_t *device = zb_device_get_by_ieee(ieee_addr);
        if (device != NULL) {
            *short_addr = device->short_addr;
            *default_endpoint = device->endpoint;
        }
        return ieee;
    }

    /* Treat as friendly name (avoids stack allocation) */
    zb_device_t *device = zb_device_find_by_name(device_str);
    if (device != NULL) {
        *short_addr = device->short_addr;
        *default_endpoint = device->endpoint;
        uint64_t ieee = 0;
        memcpy(&ieee, device->ieee_addr, sizeof(uint64_t));
        return ieee;
    }

    ESP_LOGW(TAG, "Device '%s' not found", device_str);
    return 0;
}

static void add_pending_request(uint16_t short_addr, uint8_t endpoint,
                                uint16_t cluster_id, uint16_t attr_id,
                                bool is_configure, const zb_reporting_config_t *config)
{
    /* Find free slot or oldest entry */
    int oldest_idx = 0;
    TickType_t oldest_time = portMAX_DELAY;

    for (int i = 0; i < ZB_REPORTING_MAX_PENDING; i++) {
        if (!s_pending_requests[i].pending) {
            s_pending_requests[i].short_addr = short_addr;
            s_pending_requests[i].endpoint = endpoint;
            s_pending_requests[i].cluster_id = cluster_id;
            s_pending_requests[i].attr_id = attr_id;
            s_pending_requests[i].is_configure = is_configure;
            s_pending_requests[i].pending = true;
            s_pending_requests[i].timestamp = xTaskGetTickCount();
            if (config != NULL) {
                memcpy(&s_pending_requests[i].config, config, sizeof(zb_reporting_config_t));
            }
            return;
        }

        if (s_pending_requests[i].timestamp < oldest_time) {
            oldest_time = s_pending_requests[i].timestamp;
            oldest_idx = i;
        }
    }

    /* Overwrite oldest */
    s_pending_requests[oldest_idx].short_addr = short_addr;
    s_pending_requests[oldest_idx].endpoint = endpoint;
    s_pending_requests[oldest_idx].cluster_id = cluster_id;
    s_pending_requests[oldest_idx].attr_id = attr_id;
    s_pending_requests[oldest_idx].is_configure = is_configure;
    s_pending_requests[oldest_idx].pending = true;
    s_pending_requests[oldest_idx].timestamp = xTaskGetTickCount();
    if (config != NULL) {
        memcpy(&s_pending_requests[oldest_idx].config, config, sizeof(zb_reporting_config_t));
    }
}

static zb_reporting_pending_t* find_pending_request(uint16_t short_addr,
                                                     uint16_t cluster_id,
                                                     uint16_t attr_id)
{
    for (int i = 0; i < ZB_REPORTING_MAX_PENDING; i++) {
        if (s_pending_requests[i].pending &&
            s_pending_requests[i].short_addr == short_addr &&
            s_pending_requests[i].cluster_id == cluster_id &&
            s_pending_requests[i].attr_id == attr_id) {
            return &s_pending_requests[i];
        }
    }
    return NULL;
}

static void remove_pending_request(uint16_t short_addr, uint16_t cluster_id,
                                    uint16_t attr_id)
{
    for (int i = 0; i < ZB_REPORTING_MAX_PENDING; i++) {
        if (s_pending_requests[i].pending &&
            s_pending_requests[i].short_addr == short_addr &&
            s_pending_requests[i].cluster_id == cluster_id &&
            s_pending_requests[i].attr_id == attr_id) {
            s_pending_requests[i].pending = false;
            return;
        }
    }
}
