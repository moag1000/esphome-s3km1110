/**
 * @file compat_stubs.h
 * @brief Compatibility Stubs for Removed Modules
 *
 * Provides no-op stubs for MQTT, WiFi, BLE, HA Discovery, and other
 * modules that are not present in the UART-only coordinator firmware.
 *
 * This allows the copied zigbee source files to compile with minimal
 * modifications — just changing their includes to this file.
 */

#ifndef COMPAT_STUBS_H
#define COMPAT_STUBS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * MQTT Client Stubs (gateway_mqtt.h)
 * ============================================================================ */

static inline bool mqtt_client_is_connected(void) { return false; }
static inline esp_err_t mqtt_client_publish(const char *topic, const char *data,
                                             size_t len, int qos, bool retain)
{
    (void)topic; (void)data; (void)len; (void)qos; (void)retain;
    return ESP_OK;
}

/* ============================================================================
 * MQTT Topics Stubs (mqtt/mqtt_topics.h)
 * ============================================================================ */

static inline esp_err_t mqtt_topic_device_state(const char *name, char *buf, size_t len)
{
    (void)name; if (buf && len > 0) buf[0] = '\0';
    return ESP_OK;
}

static inline esp_err_t mqtt_topic_device_availability(const char *name, char *buf, size_t len)
{
    (void)name; if (buf && len > 0) buf[0] = '\0';
    return ESP_OK;
}

static inline esp_err_t mqtt_topic_ha_discovery(const char *component, const char *device_id,
                                                  char *buf, size_t len)
{
    (void)component; (void)device_id;
    if (buf && len > 0) buf[0] = '\0';
    return ESP_OK;
}

static inline esp_err_t mqtt_topic_extract_friendly_name(const char *topic, char *friendly_name, size_t buf_len)
{
    (void)topic; if (friendly_name && buf_len > 0) friendly_name[0] = '\0';
    return ESP_ERR_NOT_FOUND;
}

/* Stub topic definitions */
#define MQTT_TOPIC_STATUS              "stub/status"
#define MQTT_TOPIC_ROUTER_STATE        "stub/router"
#define ZB_TOPOLOGY_MQTT_TOPIC_RESPONSE "stub/topology"
#define TOPIC_BRIDGE_SCENES            "stub/scenes"
#define TOPIC_REPORTING_CONFIGURE_RSP  "stub/reporting"
#define RESPONSE_TOPIC_BACKUP          "stub/backup"
#define TOPIC_BRIDGE_GROUPS            "stub/groups"
#define TOPIC_BRIDGE_DEVICES           "stub/devices"
#define TOPIC_BRIDGE_INFO              "stub/info"
#define TOPIC_BRIDGE_LOG               "stub/log"
#define TOPIC_BRIDGE_STATE             "stub/state"

/* MQTT guard macro — always returns early since MQTT is not available */
#define REQUIRE_MQTT_CONNECTED(ret_val) do { return (ret_val); } while(0)
#define REQUIRE_MQTT_CONNECTED_VOID() do { return; } while(0)

/* MQTT buffer sizes */
#define MQTT_TOPIC_MAX_LEN             256
#define MQTT_PAYLOAD_MAX_LEN           4096
#define MQTT_IEEE_ADDR_STR_LEN         19
#define MQTT_UNIQUE_ID_MAX_LEN         80
#define MQTT_LONG_STR_MAX_LEN          128

/* ============================================================================
 * MQTT Helpers Stubs (mqtt/mqtt_helpers.h)
 * ============================================================================ */

/* No-op — mqtt_helpers had formatting utilities that aren't needed */

/* ============================================================================
 * Bridge Events Stubs (bridge_events.h)
 * ============================================================================ */

static inline esp_err_t bridge_events_init(void) { return ESP_OK; }

static inline esp_err_t bridge_event_device_joined(uint64_t ieee, const char *name)
{
    (void)ieee; (void)name; return ESP_OK;
}

static inline esp_err_t bridge_event_device_leave(uint64_t ieee, const char *name)
{
    (void)ieee; (void)name; return ESP_OK;
}

static inline esp_err_t bridge_event_device_interview(uint64_t ieee, const char *status)
{
    (void)ieee; (void)status; return ESP_OK;
}

static inline esp_err_t bridge_event_device_announce(uint64_t ieee)
{
    (void)ieee; return ESP_OK;
}

static inline esp_err_t bridge_event_device_announce_full(uint64_t ieee, uint16_t nwk_addr)
{
    (void)ieee; (void)nwk_addr; return ESP_OK;
}

static inline esp_err_t bridge_event_permit_join(bool enabled, uint8_t time)
{
    (void)enabled; (void)time; return ESP_OK;
}

static inline esp_err_t bridge_event_network_started(void) { return ESP_OK; }
static inline esp_err_t bridge_event_network_stopped(void) { return ESP_OK; }

static inline esp_err_t bridge_event_nlme_status(uint16_t addr, uint8_t status)
{
    (void)addr; (void)status; return ESP_OK;
}

static inline esp_err_t bridge_event_device_bind(uint64_t src, const char *src_name,
                                                   uint64_t dst, const char *dst_name,
                                                   const char *cluster)
{
    (void)src; (void)src_name; (void)dst; (void)dst_name; (void)cluster;
    return ESP_OK;
}

static inline esp_err_t bridge_event_device_unbind(uint64_t src, const char *src_name,
                                                     uint64_t dst, const char *dst_name,
                                                     const char *cluster)
{
    (void)src; (void)src_name; (void)dst; (void)dst_name; (void)cluster;
    return ESP_OK;
}

static inline esp_err_t bridge_event_group_member_added(const char *gn, uint16_t gid,
                                                          uint64_t ieee, const char *dn)
{
    (void)gn; (void)gid; (void)ieee; (void)dn; return ESP_OK;
}

static inline esp_err_t bridge_event_group_member_removed(const char *gn, uint16_t gid,
                                                            uint64_t ieee, const char *dn)
{
    (void)gn; (void)gid; (void)ieee; (void)dn; return ESP_OK;
}

/* ============================================================================
 * Bridge Response Stubs (bridge_response.h)
 * ============================================================================ */

static inline esp_err_t bridge_response_publish_error(const char *topic,
                                                        const char *error_msg,
                                                        const char *transaction_id)
{
    (void)topic; (void)error_msg; (void)transaction_id; return ESP_OK;
}

static inline const char* bridge_request_get_transaction(const void *request)
{
    (void)request; return NULL;
}

/* ============================================================================
 * HA Discovery Stubs (ha_discovery.h)
 * ============================================================================ */

/* Forward declare zb_device_t if needed */
struct zb_device;

static inline esp_err_t ha_discovery_publish_device(const void *device)
{
    (void)device; return ESP_OK;
}

/* ============================================================================
 * Device State Publisher Stubs (device_state_publisher.h)
 * ============================================================================
 * Note: The real device_state module (core/device_state.h) provides
 * device_state_publish_by_addr() via UART. These stubs are for the
 * old MQTT-based API calls in files that haven't been adapted yet.
 */

static inline esp_err_t device_state_publish(const void *device)
{
    (void)device; return ESP_OK;
}

static inline esp_err_t device_state_publish_availability(const void *device, bool available)
{
    (void)device; (void)available; return ESP_OK;
}

static inline esp_err_t device_state_publish_availability_by_addr(uint16_t addr, bool available)
{
    (void)addr; (void)available; return ESP_OK;
}

static inline esp_err_t device_state_publish_all_availability(bool available)
{
    (void)available; return ESP_OK;
}

static inline esp_err_t device_state_publish_battery(uint16_t short_addr)
{
    (void)short_addr; return ESP_OK;
}

/* ============================================================================
 * BLE Scanner Stubs (bluetooth/ble_scanner.h)
 * ============================================================================ */

static inline bool ble_scanner_is_running(void) { return false; }
static inline esp_err_t ble_scanner_start(void) { return ESP_OK; }
static inline esp_err_t ble_scanner_stop(void) { return ESP_OK; }

/* ============================================================================
 * LED Status Manager Stubs (core/led_status_manager.h)
 * ============================================================================ */

typedef enum {
    LED_COND_PERMIT_JOIN = 0,
    LED_COND_MQTT_CONNECTED,
    LED_COND_WIFI_CONNECTED,
} led_condition_t;

static inline void led_status_manager_set_condition(led_condition_t cond, bool value)
{
    (void)cond; (void)value;
}

static inline void led_status_manager_notify(int notify_type)
{
    (void)notify_type;
}

/* ============================================================================
 * Command Handler Stubs (core/command_handler.h)
 * ============================================================================ */

#ifndef CMD_RETRY_MAX_PENDING
#define CMD_RETRY_MAX_PENDING 16
#endif

/* ============================================================================
 * Interview Status Strings (from bridge_events.h)
 * ============================================================================ */

#ifndef BRIDGE_INTERVIEW_STARTED
#define BRIDGE_INTERVIEW_STARTED     "started"
#define BRIDGE_INTERVIEW_SUCCESSFUL  "successful"
#define BRIDGE_INTERVIEW_FAILED      "failed"
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMPAT_STUBS_H */
