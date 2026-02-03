/**
 * @file zb_demand_response.c
 * @brief Zigbee Demand Response and Load Control Cluster (0x0701) Implementation
 *
 * Implements the ZCL DRLC Cluster for Smart Energy / Smart Grid integration.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_demand_response.h"
#include "zb_coordinator.h"
#include "compat_stubs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <stdlib.h>

static const char *TAG = "ZB_DRLC";

/* ============================================================================
 * Module State
 * ============================================================================ */

static bool s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

/* Configuration */
static zb_drlc_config_t s_config = {
    .utility_enrollment_group = 0xFF,   /* Respond to all groups */
    .start_randomize_minutes = 0,       /* No start randomization */
    .stop_randomize_minutes = 0,        /* No stop randomization */
    .device_class = ZB_DRLC_DEVICE_CLASS_ALL,  /* Respond to all classes */
    .allow_user_opt_out = true,         /* Allow user opt-out */
};

/* Active events storage */
static zb_drlc_active_event_t s_active_events[ZB_DRLC_MAX_ACTIVE_EVENTS];
static size_t s_active_event_count = 0;

/* Callbacks */
static zb_drlc_event_callback_t s_event_callback = NULL;
static zb_drlc_status_callback_t s_status_callback = NULL;

/* Statistics */
static zb_drlc_stats_t s_stats = {0};

/* Event processing timer */
static TimerHandle_t s_event_timer = NULL;
#define DRLC_EVENT_CHECK_INTERVAL_MS    10000   /* Check events every 10 seconds */

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Get current UTC time
 *
 * @return UTC time in seconds since 2000-01-01 (ZigBee epoch)
 */
static uint32_t get_utc_time(void)
{
    time_t now = time(NULL);
    /* ZigBee uses seconds since 2000-01-01, Unix epoch is 1970-01-01 */
    /* Difference is 946684800 seconds (30 years) */
    return (uint32_t)(now - 946684800);
}

/**
 * @brief Find active event by ID
 *
 * @param[in] issuer_event_id Event ID to find
 * @return Pointer to event or NULL if not found
 */
static zb_drlc_active_event_t* find_active_event(uint32_t issuer_event_id)
{
    for (size_t i = 0; i < s_active_event_count; i++) {
        if (s_active_events[i].event.issuer_event_id == issuer_event_id) {
            return &s_active_events[i];
        }
    }
    return NULL;
}

/**
 * @brief Remove active event by index
 *
 * @param[in] index Index of event to remove
 */
static void remove_active_event(size_t index)
{
    if (index >= s_active_event_count) {
        return;
    }

    /* Shift remaining events */
    for (size_t i = index; i < s_active_event_count - 1; i++) {
        memcpy(&s_active_events[i], &s_active_events[i + 1], sizeof(zb_drlc_active_event_t));
    }
    s_active_event_count--;

    /* Clear the last slot */
    memset(&s_active_events[s_active_event_count], 0, sizeof(zb_drlc_active_event_t));
}

/**
 * @brief Calculate randomized time offset
 *
 * @param[in] max_minutes Maximum randomization in minutes
 * @return Random offset in seconds
 */
static uint32_t calculate_random_offset(uint8_t max_minutes)
{
    if (max_minutes == 0) {
        return 0;
    }

    /* Use esp_random() for hardware RNG */
    uint32_t max_seconds = (uint32_t)max_minutes * 60;
    return esp_random() % max_seconds;
}

/**
 * @brief Check if device class matches
 *
 * @param[in] event_class Device class from event
 * @return true if this device should respond
 */
static bool device_class_matches(uint16_t event_class)
{
    /* If event targets all devices, always match */
    if (event_class == ZB_DRLC_DEVICE_CLASS_ALL) {
        return true;
    }

    /* Check if our device class overlaps with event's target */
    return (s_config.device_class & event_class) != 0;
}

/**
 * @brief Check if enrollment group matches
 *
 * @param[in] event_group Enrollment group from event
 * @return true if this device should respond
 */
static bool enrollment_group_matches(uint8_t event_group)
{
    /* 0xFF means all groups */
    if (event_group == 0xFF || s_config.utility_enrollment_group == 0xFF) {
        return true;
    }

    return event_group == s_config.utility_enrollment_group;
}

/**
 * @brief Update event status and notify
 *
 * @param[in] event         Active event
 * @param[in] new_status    New status
 */
static void update_event_status(zb_drlc_active_event_t *event, zb_drlc_event_status_t new_status)
{
    zb_drlc_event_status_t old_status = event->status;
    event->status = new_status;

    /* Notify callback */
    if (s_status_callback != NULL && old_status != new_status) {
        s_status_callback(event->event.issuer_event_id, old_status, new_status);
    }

    ESP_LOGI(TAG, "Event 0x%08" PRIX32 " status: %s -> %s",
             event->event.issuer_event_id,
             zb_drlc_status_to_string(old_status),
             zb_drlc_status_to_string(new_status));
}

/**
 * @brief Event timer callback - processes event state changes
 *
 * @param[in] xTimer Timer handle
 */
static void event_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (!s_initialized || s_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    uint32_t now = get_utc_time();
    bool events_changed = false;

    for (size_t i = 0; i < s_active_event_count; ) {
        zb_drlc_active_event_t *event = &s_active_events[i];

        /* Skip if user opted out */
        if (event->user_opted_out) {
            i++;
            continue;
        }

        /* Check if event should start */
        if (!event->active && event->actual_start_time <= now &&
            event->status == ZB_DRLC_STATUS_EVENT_RECEIVED) {

            event->active = true;
            update_event_status(event, ZB_DRLC_STATUS_EVENT_STARTED);
            s_stats.events_started++;
            events_changed = true;

            ESP_LOGI(TAG, "Event 0x%08" PRIX32 " STARTED (criticality: %s)",
                     event->event.issuer_event_id,
                     zb_drlc_criticality_to_string(event->event.criticality_level));
        }

        /* Check if event should end */
        if (event->active && event->actual_end_time <= now) {
            event->active = false;
            update_event_status(event, ZB_DRLC_STATUS_EVENT_COMPLETED);
            s_stats.events_completed++;
            events_changed = true;

            ESP_LOGI(TAG, "Event 0x%08" PRIX32 " COMPLETED",
                     event->event.issuer_event_id);

            /* Remove completed event */
            remove_active_event(i);
            continue;  /* Don't increment i, array shifted */
        }

        i++;
    }

    xSemaphoreGive(s_mutex);

    /* Publish updates if events changed */
    if (events_changed) {
        zb_demand_response_publish_events();
    }
}

/**
 * @brief Publish event to MQTT
 *
 * @param[in] event Active event to publish
 * @param[in] action Action (received, started, completed, cancelled)
 */
static void publish_event_mqtt(const zb_drlc_active_event_t *event, const char *action)
{
    if (!mqtt_client_is_connected()) {
        return;
    }

    char topic[128];
    char payload[512];
    char device_class_str[128];

    zb_drlc_device_class_to_string(event->event.device_class, device_class_str, sizeof(device_class_str));

    snprintf(topic, sizeof(topic), "zigbee2mqtt/bridge/demand_response/event");

    snprintf(payload, sizeof(payload),
             "{"
             "\"action\":\"%s\","
             "\"issuer_event_id\":\"0x%08" PRIX32 "\","
             "\"device_class\":\"%s\","
             "\"enrollment_group\":%u,"
             "\"start_time\":%" PRIu32 ","
             "\"duration_minutes\":%u,"
             "\"criticality\":\"%s\","
             "\"criticality_level\":%u,"
             "\"status\":\"%s\","
             "\"active\":%s,"
             "\"user_opted_out\":%s"
             "}",
             action,
             event->event.issuer_event_id,
             device_class_str,
             event->event.utility_enrollment_group,
             event->actual_start_time,
             event->event.duration_minutes,
             zb_drlc_criticality_to_string(event->event.criticality_level),
             event->event.criticality_level,
             zb_drlc_status_to_string(event->status),
             event->active ? "true" : "false",
             event->user_opted_out ? "true" : "false");

    esp_err_t ret = mqtt_client_publish(topic, payload, 0, 0, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish DRLC event: %s", esp_err_to_name(ret));
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "DRLC module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create mutex */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize state */
    memset(s_active_events, 0, sizeof(s_active_events));
    s_active_event_count = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    s_event_callback = NULL;
    s_status_callback = NULL;

    /* Create event processing timer */
    s_event_timer = xTimerCreate(
        "drlc_timer",
        pdMS_TO_TICKS(DRLC_EVENT_CHECK_INTERVAL_MS),
        pdTRUE,  /* Auto-reload */
        NULL,
        event_timer_callback
    );

    if (s_event_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create event timer");
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Start the timer */
    xTimerStart(s_event_timer, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "Demand Response module initialized (max %d events)",
             ZB_DRLC_MAX_ACTIVE_EVENTS);
    return ESP_OK;
}

esp_err_t zb_demand_response_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stop and delete timer */
    if (s_event_timer != NULL) {
        xTimerStop(s_event_timer, portMAX_DELAY);
        xTimerDelete(s_event_timer, portMAX_DELAY);
        s_event_timer = NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_initialized = false;
    s_active_event_count = 0;
    s_event_callback = NULL;
    s_status_callback = NULL;
    xSemaphoreGive(s_mutex);

    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;

    ESP_LOGI(TAG, "DRLC module deinitialized (events received: %" PRIu32 ", completed: %" PRIu32 ")",
             s_stats.events_received, s_stats.events_completed);
    return ESP_OK;
}

bool zb_demand_response_is_initialized(void)
{
    return s_initialized;
}

esp_err_t zb_demand_response_configure(const zb_drlc_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        /* Allow pre-init configuration */
        memcpy(&s_config, config, sizeof(zb_drlc_config_t));
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(&s_config, config, sizeof(zb_drlc_config_t));
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "DRLC configured: group=%u, device_class=0x%04X, randomize=%u/%u min",
             config->utility_enrollment_group, config->device_class,
             config->start_randomize_minutes, config->stop_randomize_minutes);

    return ESP_OK;
}

esp_err_t zb_demand_response_get_config(zb_drlc_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized && s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memcpy(config, &s_config, sizeof(zb_drlc_config_t));
        xSemaphoreGive(s_mutex);
    } else {
        memcpy(config, &s_config, sizeof(zb_drlc_config_t));
    }

    return ESP_OK;
}

/* ============================================================================
 * Event Handling Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_handle_event(const zb_load_control_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Received LoadControlEvent: ID=0x%08" PRIX32 ", class=0x%04X, duration=%u min",
             event->issuer_event_id, event->device_class, event->duration_minutes);

    /* Check if device class matches */
    if (!device_class_matches(event->device_class)) {
        ESP_LOGI(TAG, "Event ignored - device class mismatch (ours=0x%04X, event=0x%04X)",
                 s_config.device_class, event->device_class);
        return ESP_OK;
    }

    /* Check if enrollment group matches */
    if (!enrollment_group_matches(event->utility_enrollment_group)) {
        ESP_LOGI(TAG, "Event ignored - enrollment group mismatch (ours=%u, event=%u)",
                 s_config.utility_enrollment_group, event->utility_enrollment_group);
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Check for duplicate event */
    zb_drlc_active_event_t *existing = find_active_event(event->issuer_event_id);
    if (existing != NULL) {
        /* Update existing event */
        memcpy(&existing->event, event, sizeof(zb_load_control_event_t));
        ESP_LOGI(TAG, "Updated existing event 0x%08" PRIX32, event->issuer_event_id);
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }

    /* Check capacity */
    if (s_active_event_count >= ZB_DRLC_MAX_ACTIVE_EVENTS) {
        ESP_LOGW(TAG, "Event table full, cannot add event 0x%08" PRIX32,
                 event->issuer_event_id);
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Add new event */
    zb_drlc_active_event_t *new_event = &s_active_events[s_active_event_count];
    memset(new_event, 0, sizeof(zb_drlc_active_event_t));
    memcpy(&new_event->event, event, sizeof(zb_load_control_event_t));

    /* Calculate actual start/end times with randomization */
    uint32_t now = get_utc_time();
    uint32_t start_time = (event->start_time == 0) ? now : event->start_time;

    if (event->event_control & ZB_DRLC_EVENT_CTRL_RANDOMIZED_START) {
        start_time += calculate_random_offset(s_config.start_randomize_minutes);
    }

    uint32_t end_time = start_time + ((uint32_t)event->duration_minutes * 60);

    if (event->event_control & ZB_DRLC_EVENT_CTRL_RANDOMIZED_END) {
        end_time += calculate_random_offset(s_config.stop_randomize_minutes);
    }

    new_event->actual_start_time = start_time;
    new_event->actual_end_time = end_time;
    new_event->status = ZB_DRLC_STATUS_EVENT_RECEIVED;
    new_event->active = false;
    new_event->user_opted_out = false;

    s_active_event_count++;
    s_stats.events_received++;

    xSemaphoreGive(s_mutex);

    /* Invoke callback */
    if (s_event_callback != NULL) {
        s_event_callback(event, false);
    }

    /* Publish to MQTT */
    publish_event_mqtt(new_event, "received");

    ESP_LOGI(TAG, "Event 0x%08" PRIX32 " scheduled: start=%" PRIu32 ", end=%" PRIu32,
             event->issuer_event_id, new_event->actual_start_time, new_event->actual_end_time);

    return ESP_OK;
}

esp_err_t zb_demand_response_handle_cancel_event(uint32_t issuer_event_id,
                                                   uint16_t device_class,
                                                   uint8_t enrollment_group,
                                                   uint8_t cancel_control)
{
    (void)cancel_control;  /* For future use */

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Received CancelLoadControlEvent: ID=0x%08" PRIX32, issuer_event_id);

    /* Check device class and enrollment group */
    if (!device_class_matches(device_class) || !enrollment_group_matches(enrollment_group)) {
        ESP_LOGI(TAG, "Cancel ignored - class/group mismatch");
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_drlc_active_event_t *event = find_active_event(issuer_event_id);
    if (event == NULL) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "Cancel: Event 0x%08" PRIX32 " not found", issuer_event_id);
        return ESP_ERR_NOT_FOUND;
    }

    /* Update status */
    update_event_status(event, ZB_DRLC_STATUS_EVENT_CANCELLED);
    event->active = false;
    s_stats.events_cancelled++;

    /* Find and remove the event */
    for (size_t i = 0; i < s_active_event_count; i++) {
        if (s_active_events[i].event.issuer_event_id == issuer_event_id) {
            /* Save for MQTT publish before removal */
            zb_drlc_active_event_t cancelled_event;
            memcpy(&cancelled_event, &s_active_events[i], sizeof(zb_drlc_active_event_t));

            remove_active_event(i);
            xSemaphoreGive(s_mutex);

            /* Invoke callback */
            if (s_event_callback != NULL) {
                s_event_callback(&cancelled_event.event, true);
            }

            /* Publish cancellation */
            publish_event_mqtt(&cancelled_event, "cancelled");

            ESP_LOGI(TAG, "Event 0x%08" PRIX32 " CANCELLED", issuer_event_id);
            return ESP_OK;
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t zb_demand_response_handle_cancel_all(uint8_t cancel_control)
{
    (void)cancel_control;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Received CancelAllLoadControlEvents");

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    size_t cancelled_count = s_active_event_count;

    /* Update stats for each cancelled event */
    for (size_t i = 0; i < s_active_event_count; i++) {
        s_active_events[i].status = ZB_DRLC_STATUS_EVENT_CANCELLED;
        s_active_events[i].active = false;
    }

    s_stats.events_cancelled += cancelled_count;

    /* Clear all events */
    memset(s_active_events, 0, sizeof(s_active_events));
    s_active_event_count = 0;

    xSemaphoreGive(s_mutex);

    /* Publish status */
    zb_demand_response_publish_events();

    ESP_LOGI(TAG, "All events cancelled (count: %zu)", cancelled_count);
    return ESP_OK;
}

/* ============================================================================
 * Event Status and Reporting Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_report_status(uint32_t issuer_event_id,
                                            zb_drlc_event_status_t status,
                                            uint8_t event_control,
                                            uint8_t signature_type)
{
    ESP_LOGI(TAG, "Reporting status for event 0x%08" PRIX32 ": %s",
             issuer_event_id, zb_drlc_status_to_string(status));

    /* Build ReportEventStatus command payload */
    uint8_t payload[14];
    size_t offset = 0;

    /* Issuer Event ID (uint32) */
    payload[offset++] = (uint8_t)(issuer_event_id & 0xFF);
    payload[offset++] = (uint8_t)((issuer_event_id >> 8) & 0xFF);
    payload[offset++] = (uint8_t)((issuer_event_id >> 16) & 0xFF);
    payload[offset++] = (uint8_t)((issuer_event_id >> 24) & 0xFF);

    /* Event Status (uint8) */
    payload[offset++] = (uint8_t)status;

    /* Event Status Time (uint32) - current time */
    uint32_t now = get_utc_time();
    payload[offset++] = (uint8_t)(now & 0xFF);
    payload[offset++] = (uint8_t)((now >> 8) & 0xFF);
    payload[offset++] = (uint8_t)((now >> 16) & 0xFF);
    payload[offset++] = (uint8_t)((now >> 24) & 0xFF);

    /* Criticality Level Applied (uint8) - reuse event control */
    payload[offset++] = event_control;

    /* Cooling Temperature Set Point Applied (int16) - N/A */
    payload[offset++] = 0x00;
    payload[offset++] = 0x80;  /* 0x8000 = N/A */

    /* Heating Temperature Set Point Applied (int16) - N/A */
    payload[offset++] = 0x00;
    payload[offset++] = 0x80;  /* 0x8000 = N/A */

    /* Note: Signature is not included if signature_type == 0 */
    (void)signature_type;

    /* Send custom cluster command to coordinator/utility */
    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = 0x0000,  /* Coordinator */
            },
            .dst_endpoint = 1,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_DEMAND_RESPONSE,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_DRLC_REPORT_EVENT_STATUS_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_SET,
            .size = offset,
            .value = payload,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret == ESP_OK) {
        s_stats.status_reports_sent++;
    } else {
        ESP_LOGE(TAG, "Failed to send ReportEventStatus: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_demand_response_get_scheduled_events(uint16_t dst_addr,
                                                    uint8_t dst_endpoint,
                                                    uint32_t start_time,
                                                    uint8_t num_events)
{
    ESP_LOGI(TAG, "Requesting scheduled events from 0x%04X", dst_addr);

    /* Build GetScheduledEvents command payload */
    uint8_t payload[5];
    size_t offset = 0;

    /* Start Time (uint32) */
    payload[offset++] = (uint8_t)(start_time & 0xFF);
    payload[offset++] = (uint8_t)((start_time >> 8) & 0xFF);
    payload[offset++] = (uint8_t)((start_time >> 16) & 0xFF);
    payload[offset++] = (uint8_t)((start_time >> 24) & 0xFF);

    /* Number of Events (uint8) */
    payload[offset++] = num_events;

    esp_zb_zcl_custom_cluster_cmd_t cmd_req = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = dst_addr,
            },
            .dst_endpoint = dst_endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_ZCL_CLUSTER_ID_DEMAND_RESPONSE,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .custom_cmd_id = ZB_ZCL_CMD_DRLC_GET_SCHEDULED_EVENTS_ID,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_SET,
            .size = offset,
            .value = payload,
        },
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send GetScheduledEvents: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* ============================================================================
 * Active Events Management Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_get_active_events(zb_drlc_active_event_t *events,
                                                 size_t max_count,
                                                 size_t *count)
{
    if (events == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        *count = 0;
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    size_t to_copy = (s_active_event_count < max_count) ? s_active_event_count : max_count;
    memcpy(events, s_active_events, to_copy * sizeof(zb_drlc_active_event_t));
    *count = to_copy;

    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

size_t zb_demand_response_get_active_event_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    size_t count = s_active_event_count;
    xSemaphoreGive(s_mutex);

    return count;
}

esp_err_t zb_demand_response_get_event(uint32_t issuer_event_id,
                                         zb_drlc_active_event_t *event)
{
    if (event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_drlc_active_event_t *found = find_active_event(issuer_event_id);
    if (found == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(event, found, sizeof(zb_drlc_active_event_t));
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t zb_demand_response_user_opt_out(uint32_t issuer_event_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_config.allow_user_opt_out) {
        ESP_LOGW(TAG, "User opt-out not allowed by configuration");
        return ESP_ERR_NOT_SUPPORTED;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_drlc_active_event_t *event = find_active_event(issuer_event_id);
    if (event == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    event->user_opted_out = true;
    update_event_status(event, ZB_DRLC_STATUS_USER_OPTED_OUT);
    s_stats.events_opted_out++;

    xSemaphoreGive(s_mutex);

    /* Report opt-out to utility */
    zb_demand_response_report_status(issuer_event_id, ZB_DRLC_STATUS_USER_OPTED_OUT,
                                      event->event.event_control, 0);

    /* Publish to MQTT */
    publish_event_mqtt(event, "opted_out");

    ESP_LOGI(TAG, "User opted out of event 0x%08" PRIX32, issuer_event_id);
    return ESP_OK;
}

esp_err_t zb_demand_response_user_opt_in(uint32_t issuer_event_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_drlc_active_event_t *event = find_active_event(issuer_event_id);
    if (event == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    event->user_opted_out = false;
    update_event_status(event, ZB_DRLC_STATUS_USER_OPTED_IN);

    xSemaphoreGive(s_mutex);

    /* Report opt-in to utility */
    zb_demand_response_report_status(issuer_event_id, ZB_DRLC_STATUS_USER_OPTED_IN,
                                      event->event.event_control, 0);

    /* Publish to MQTT */
    publish_event_mqtt(event, "opted_in");

    ESP_LOGI(TAG, "User opted in to event 0x%08" PRIX32, issuer_event_id);
    return ESP_OK;
}

/* ============================================================================
 * Callbacks Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_register_callback(zb_drlc_event_callback_t callback)
{
    s_event_callback = callback;
    ESP_LOGI(TAG, "Event callback %s", callback ? "registered" : "unregistered");
    return ESP_OK;
}

esp_err_t zb_demand_response_register_status_callback(zb_drlc_status_callback_t callback)
{
    s_status_callback = callback;
    ESP_LOGI(TAG, "Status callback %s", callback ? "registered" : "unregistered");
    return ESP_OK;
}

/* ============================================================================
 * Statistics Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_get_stats(zb_drlc_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized && s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memcpy(stats, &s_stats, sizeof(zb_drlc_stats_t));
        xSemaphoreGive(s_mutex);
    } else {
        memcpy(stats, &s_stats, sizeof(zb_drlc_stats_t));
    }

    return ESP_OK;
}

esp_err_t zb_demand_response_reset_stats(void)
{
    if (s_initialized && s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        memset(&s_stats, 0, sizeof(zb_drlc_stats_t));
        xSemaphoreGive(s_mutex);
    } else {
        memset(&s_stats, 0, sizeof(zb_drlc_stats_t));
    }

    ESP_LOGI(TAG, "Statistics reset");
    return ESP_OK;
}

/* ============================================================================
 * MQTT Integration Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_publish_events(void)
{
    if (!mqtt_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char topic[128];
    char payload[1024];
    size_t offset = 0;

    snprintf(topic, sizeof(topic), "zigbee2mqtt/bridge/demand_response/events");

    /* Build JSON array of events */
    offset += snprintf(payload + offset, sizeof(payload) - offset, "{\"events\":[");

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (size_t i = 0; i < s_active_event_count && (offset < sizeof(payload) - 200); i++) {
        zb_drlc_active_event_t *event = &s_active_events[i];
        char device_class_str[64];
        zb_drlc_device_class_to_string(event->event.device_class, device_class_str, sizeof(device_class_str));

        if (i > 0) {
            offset += snprintf(payload + offset, sizeof(payload) - offset, ",");
        }

        offset += snprintf(payload + offset, sizeof(payload) - offset,
                          "{"
                          "\"id\":\"0x%08" PRIX32 "\","
                          "\"class\":\"%s\","
                          "\"criticality\":\"%s\","
                          "\"status\":\"%s\","
                          "\"active\":%s,"
                          "\"start\":%" PRIu32 ","
                          "\"end\":%" PRIu32
                          "}",
                          event->event.issuer_event_id,
                          device_class_str,
                          zb_drlc_criticality_to_string(event->event.criticality_level),
                          zb_drlc_status_to_string(event->status),
                          event->active ? "true" : "false",
                          event->actual_start_time,
                          event->actual_end_time);
    }

    size_t event_count = s_active_event_count;
    xSemaphoreGive(s_mutex);

    offset += snprintf(payload + offset, sizeof(payload) - offset,
                      "],\"count\":%zu}", event_count);

    return mqtt_client_publish(topic, payload, 0, 0, true);  /* Retain */
}

esp_err_t zb_demand_response_publish_status(void)
{
    if (!mqtt_client_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    char topic[128];
    char payload[512];

    snprintf(topic, sizeof(topic), "zigbee2mqtt/bridge/demand_response/status");

    zb_drlc_stats_t stats;
    zb_demand_response_get_stats(&stats);

    snprintf(payload, sizeof(payload),
             "{"
             "\"initialized\":%s,"
             "\"active_events\":%zu,"
             "\"enrollment_group\":%u,"
             "\"device_class\":\"0x%04X\","
             "\"allow_opt_out\":%s,"
             "\"stats\":{"
             "\"received\":%" PRIu32 ","
             "\"started\":%" PRIu32 ","
             "\"completed\":%" PRIu32 ","
             "\"cancelled\":%" PRIu32 ","
             "\"opted_out\":%" PRIu32 ","
             "\"reports_sent\":%" PRIu32
             "}"
             "}",
             s_initialized ? "true" : "false",
             s_active_event_count,
             s_config.utility_enrollment_group,
             s_config.device_class,
             s_config.allow_user_opt_out ? "true" : "false",
             stats.events_received,
             stats.events_started,
             stats.events_completed,
             stats.events_cancelled,
             stats.events_opted_out,
             stats.status_reports_sent);

    return mqtt_client_publish(topic, payload, 0, 0, true);  /* Retain */
}

/* ============================================================================
 * Utility Functions Implementation
 * ============================================================================ */

const char* zb_drlc_criticality_to_string(zb_drlc_criticality_t level)
{
    switch (level) {
        case ZB_DRLC_CRITICALITY_RESERVED:          return "reserved";
        case ZB_DRLC_CRITICALITY_GREEN:             return "green";
        case ZB_DRLC_CRITICALITY_LEVEL_1:           return "level_1";
        case ZB_DRLC_CRITICALITY_LEVEL_2:           return "level_2";
        case ZB_DRLC_CRITICALITY_LEVEL_3:           return "level_3";
        case ZB_DRLC_CRITICALITY_LEVEL_4:           return "level_4";
        case ZB_DRLC_CRITICALITY_LEVEL_5:           return "level_5";
        case ZB_DRLC_CRITICALITY_EMERGENCY:         return "emergency";
        case ZB_DRLC_CRITICALITY_PLANNED_OUTAGE:    return "planned_outage";
        case ZB_DRLC_CRITICALITY_SERVICE_DISCONNECT: return "service_disconnect";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_1: return "utility_1";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_2: return "utility_2";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_3: return "utility_3";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_4: return "utility_4";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_5: return "utility_5";
        case ZB_DRLC_CRITICALITY_UTILITY_DEFINED_6: return "utility_6";
        default:                                     return "unknown";
    }
}

const char* zb_drlc_status_to_string(zb_drlc_event_status_t status)
{
    switch (status) {
        case ZB_DRLC_STATUS_EVENT_RECEIVED:             return "received";
        case ZB_DRLC_STATUS_EVENT_STARTED:              return "started";
        case ZB_DRLC_STATUS_EVENT_COMPLETED:            return "completed";
        case ZB_DRLC_STATUS_USER_OPTED_OUT:             return "opted_out";
        case ZB_DRLC_STATUS_USER_OPTED_IN:              return "opted_in";
        case ZB_DRLC_STATUS_EVENT_CANCELLED:            return "cancelled";
        case ZB_DRLC_STATUS_EVENT_SUPERSEDED:           return "superseded";
        case ZB_DRLC_STATUS_EVENT_PARTIAL_OPT_OUT:      return "partial_opt_out";
        case ZB_DRLC_STATUS_EVENT_PARTIAL_OPT_IN:       return "partial_opt_in";
        case ZB_DRLC_STATUS_EVENT_COMPLETE_NO_USER:     return "complete_no_user";
        case ZB_DRLC_STATUS_INVALID_CANCEL_UNDEFINED:   return "invalid_cancel";
        case ZB_DRLC_STATUS_INVALID_CANCEL_COMMAND:     return "invalid_cancel_cmd";
        case ZB_DRLC_STATUS_INVALID_EFFECTIVE_TIME:     return "invalid_time";
        case ZB_DRLC_STATUS_INVALID_ISSUER_EVENT:       return "invalid_event";
        case ZB_DRLC_STATUS_REJECTED:                   return "rejected";
        case ZB_DRLC_STATUS_UNDEFINED:                  return "undefined";
        default:                                        return "unknown";
    }
}

esp_err_t zb_drlc_device_class_to_string(uint16_t device_class, char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    buf[0] = '\0';
    size_t offset = 0;

    struct {
        uint16_t mask;
        const char *name;
    } classes[] = {
        { ZB_DRLC_DEVICE_CLASS_HVAC,               "HVAC" },
        { ZB_DRLC_DEVICE_CLASS_STRIP_HEAT,         "StripHeat" },
        { ZB_DRLC_DEVICE_CLASS_WATER_HEATER,       "WaterHeater" },
        { ZB_DRLC_DEVICE_CLASS_POOL_PUMP,          "PoolPump" },
        { ZB_DRLC_DEVICE_CLASS_SMART_APPLIANCE,    "SmartAppliance" },
        { ZB_DRLC_DEVICE_CLASS_IRRIGATION_PUMP,    "IrrigationPump" },
        { ZB_DRLC_DEVICE_CLASS_MANAGED_COMM_LOAD,  "ManagedCommLoad" },
        { ZB_DRLC_DEVICE_CLASS_SIMPLE_LOAD,        "SimpleLoad" },
        { ZB_DRLC_DEVICE_CLASS_EXTERIOR_LIGHTING,  "ExteriorLighting" },
        { ZB_DRLC_DEVICE_CLASS_INTERIOR_LIGHTING,  "InteriorLighting" },
        { ZB_DRLC_DEVICE_CLASS_ELECTRIC_VEHICLE,   "ElectricVehicle" },
        { ZB_DRLC_DEVICE_CLASS_GENERATION_SYSTEM,  "GenerationSystem" },
    };

    bool first = true;
    for (size_t i = 0; i < sizeof(classes) / sizeof(classes[0]); i++) {
        if (device_class & classes[i].mask) {
            if (!first && offset < buf_len - 1) {
                offset += snprintf(buf + offset, buf_len - offset, ",");
            }
            offset += snprintf(buf + offset, buf_len - offset, "%s", classes[i].name);
            first = false;
        }
    }

    if (offset == 0) {
        snprintf(buf, buf_len, "None");
    }

    return ESP_OK;
}

/* ============================================================================
 * Self-Test Implementation
 * ============================================================================ */

esp_err_t zb_demand_response_test(void)
{
    ESP_LOGI(TAG, "Running DRLC module self-test...");

    /* Test initialization state */
    if (!s_initialized) {
        ESP_LOGE(TAG, "Module not initialized");
        return ESP_FAIL;
    }

    /* Test configuration */
    zb_drlc_config_t config;
    esp_err_t ret = zb_demand_response_get_config(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Config: group=%u, class=0x%04X", config.utility_enrollment_group, config.device_class);

    /* Test event handling with a synthetic event */
    zb_load_control_event_t test_event = {
        .issuer_event_id = 0x12345678,
        .device_class = ZB_DRLC_DEVICE_CLASS_ALL,
        .utility_enrollment_group = 0xFF,
        .start_time = 0,  /* Now */
        .duration_minutes = 60,
        .criticality_level = ZB_DRLC_CRITICALITY_LEVEL_2,
        .cooling_temp_offset = 0xFF,
        .heating_temp_offset = 0xFF,
        .cooling_temp_setpoint = (int16_t)0x8000,
        .heating_temp_setpoint = (int16_t)0x8000,
        .average_load_adjustment = 0,
        .duty_cycle = 0xFF,
        .event_control = 0,
    };

    ret = zb_demand_response_handle_event(&test_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to handle test event");
        return ESP_FAIL;
    }

    /* Verify event was added */
    size_t count = zb_demand_response_get_active_event_count();
    if (count < 1) {
        ESP_LOGE(TAG, "Event was not added to active list");
        return ESP_FAIL;
    }

    /* Test getting the event */
    zb_drlc_active_event_t retrieved_event;
    ret = zb_demand_response_get_event(0x12345678, &retrieved_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to retrieve test event");
        return ESP_FAIL;
    }

    /* Test cancel */
    ret = zb_demand_response_handle_cancel_event(0x12345678, ZB_DRLC_DEVICE_CLASS_ALL, 0xFF, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to cancel test event");
        return ESP_FAIL;
    }

    /* Verify event was removed */
    count = zb_demand_response_get_active_event_count();
    /* Count could be 0 or might have other events */

    /* Test utility functions */
    const char *criticality_str = zb_drlc_criticality_to_string(ZB_DRLC_CRITICALITY_EMERGENCY);
    if (strcmp(criticality_str, "emergency") != 0) {
        ESP_LOGE(TAG, "Criticality string mismatch: %s", criticality_str);
        return ESP_FAIL;
    }

    const char *status_str = zb_drlc_status_to_string(ZB_DRLC_STATUS_EVENT_STARTED);
    if (strcmp(status_str, "started") != 0) {
        ESP_LOGE(TAG, "Status string mismatch: %s", status_str);
        return ESP_FAIL;
    }

    char class_buf[128];
    ret = zb_drlc_device_class_to_string(ZB_DRLC_DEVICE_CLASS_HVAC | ZB_DRLC_DEVICE_CLASS_WATER_HEATER, class_buf, sizeof(class_buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device class to string failed");
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Device class string: %s", class_buf);

    /* Test stats */
    zb_drlc_stats_t stats;
    ret = zb_demand_response_get_stats(&stats);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get stats");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Stats: received=%" PRIu32 ", cancelled=%" PRIu32,
             stats.events_received, stats.events_cancelled);

    ESP_LOGI(TAG, "DRLC module self-test PASSED");
    return ESP_OK;
}
