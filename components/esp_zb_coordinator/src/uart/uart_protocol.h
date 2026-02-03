/**
 * @file uart_protocol.h
 * @brief UART JSON Lines Protocol Encoder/Decoder
 *
 * Handles encoding Zigbee events to JSON and decoding
 * commands received from the ESP32-S3.
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Outgoing Messages (C5 → S3)
 * ============================================================================ */

/**
 * @brief Build "ready" message
 * @param pan_id Network PAN ID
 * @param channel Zigbee channel
 * @return Allocated JSON string (caller must free), or NULL on error
 */
char *uart_proto_build_ready(uint16_t pan_id, uint8_t channel);

/**
 * @brief Build "device_joined" message
 * @param ieee IEEE address as hex string (e.g., "00124B001234ABCD")
 * @param short_addr Device short address
 * @return Allocated JSON string (caller must free)
 */
char *uart_proto_build_device_joined(const char *ieee, uint16_t short_addr);

/**
 * @brief Build "device_left" message
 */
char *uart_proto_build_device_left(const char *ieee);

/**
 * @brief Build "interview_done" message
 * @param ieee IEEE address
 * @param model Device model string
 * @param manufacturer Device manufacturer string
 * @param endpoint_count Number of endpoints
 * @return Allocated JSON string (caller must free)
 */
char *uart_proto_build_interview_done(const char *ieee, const char *model,
                                       const char *manufacturer,
                                       uint8_t endpoint_count);

/**
 * @brief Build "attribute" report message
 * @param ieee IEEE address
 * @param ep Endpoint
 * @param cluster Cluster ID
 * @param attr Attribute ID
 * @param value Attribute value (as double for simplicity)
 * @return Allocated JSON string (caller must free)
 */
char *uart_proto_build_attribute_report(const char *ieee, uint8_t ep,
                                         uint16_t cluster, uint16_t attr,
                                         double value);

/**
 * @brief Build "attribute_str" report message (for string values)
 */
char *uart_proto_build_attribute_str_report(const char *ieee, uint8_t ep,
                                             uint16_t cluster, uint16_t attr,
                                             const char *value);

/**
 * @brief Build "response" message for command acknowledgment
 * @param id Command ID being responded to
 * @param status Status string ("ok" or "error")
 * @param error_msg Optional error message (NULL for success)
 * @return Allocated JSON string (caller must free)
 */
char *uart_proto_build_response(uint32_t id, const char *status,
                                 const char *error_msg);

/**
 * @brief Build "device_state" message with full device JSON
 * @param ieee IEEE address
 * @param state_json cJSON object with device state (not consumed)
 * @return Allocated JSON string (caller must free)
 */
char *uart_proto_build_device_state(const char *ieee, const cJSON *state_json);

/**
 * @brief Build "network_info" response
 */
char *uart_proto_build_network_info(uint16_t pan_id, uint8_t channel,
                                     uint8_t device_count);

/**
 * @brief Build "permit_join" event
 */
char *uart_proto_build_permit_join(bool enabled, uint8_t duration);

/* ============================================================================
 * Incoming Commands (S3 → C5)
 * ============================================================================ */

/**
 * @brief Parsed command structure
 */
typedef struct {
    char cmd[32];               /**< Command name */
    uint32_t id;                /**< Command ID for response matching */
    char ieee[20];              /**< IEEE address (if applicable) */
    uint8_t ep;                 /**< Endpoint (if applicable) */
    uint16_t cluster;           /**< Cluster ID (if applicable) */
    uint16_t attr;              /**< Attribute ID (if applicable) */
    int32_t value_int;          /**< Integer value */
    double value_double;        /**< Double value */
    char value_str[64];         /**< String value */
    uint16_t duration;          /**< Duration (for permit_join) */
    bool has_value;             /**< True if value field was present */
} uart_command_t;

/**
 * @brief Parse a JSON command line into a command structure
 * @param json_line NULL-terminated JSON string
 * @param cmd Output command structure
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on parse error
 */
esp_err_t uart_proto_parse_command(const char *json_line, uart_command_t *cmd);

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

/**
 * @brief Format IEEE address as hex string
 * @param ieee_addr 8-byte IEEE address (little-endian)
 * @param buf Output buffer (at least 17 bytes)
 * @param buf_len Buffer length
 */
void uart_proto_ieee_to_str(const uint8_t *ieee_addr, char *buf, size_t buf_len);

/**
 * @brief Parse hex IEEE string back to 8-byte array
 * @param str IEEE hex string (16 chars, e.g., "00124B001234ABCD")
 * @param ieee_addr Output 8-byte array (little-endian)
 * @return ESP_OK on success
 */
esp_err_t uart_proto_str_to_ieee(const char *str, uint8_t *ieee_addr);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
