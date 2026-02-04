/**
 * @file uart_bridge.h
 * @brief UART Bridge - Zigbee to UART Translation Layer
 *
 * Replaces mqtt_bridge for UART-only coordinator.
 * Provides the same callback API so zigbee modules can call
 * uart_bridge_on_device_join() etc. instead of mqtt_bridge_on_device_join().
 *
 * All output is JSON Lines over UART1 (IO6=RX, IO7=TX).
 */

#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * UART Pin Configuration
 * ============================================================================
 * Note: TX and RX pins are obtained from HAL at runtime.
 * These defaults are for C5 but may be overridden by zb_hal_get_uart_*_pin().
 */

#define UART_BRIDGE_PORT_NUM    1       /**< UART1 for S3 communication */
#define UART_BRIDGE_BAUD_RATE   115200
#define UART_BRIDGE_BUF_SIZE    2048    /**< RX/TX ring buffer size */

/* C5-specific pin defaults (used if HAL not available) */
#ifndef UART_BRIDGE_TX_PIN
#define UART_BRIDGE_TX_PIN      7       /**< IO7 → S3 RX (GPIO15) */
#endif
#ifndef UART_BRIDGE_RX_PIN
#define UART_BRIDGE_RX_PIN      6       /**< IO6 ← S3 TX (GPIO16) */
#endif

/* ============================================================================
 * Bridge Statistics
 * ============================================================================ */

typedef struct {
    bool enabled;
    uint32_t publish_count;
    uint32_t command_count;
    uint32_t error_count;
    uint32_t device_join_count;
    uint32_t device_leave_count;
} bridge_stats_t;

/* ============================================================================
 * Public API (mirrors mqtt_bridge.h)
 * ============================================================================ */

/**
 * @brief Initialize UART bridge
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_init(void);

/**
 * @brief Start UART bridge (sends ready message)
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_start(void);

/**
 * @brief Stop UART bridge
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_stop(void);

/**
 * @brief Check if bridge is enabled and running
 */
bool uart_bridge_is_enabled(void);

/**
 * @brief Get bridge statistics
 */
bridge_stats_t uart_bridge_get_stats(void);

/**
 * @brief Publish device state as JSON over UART
 * @param short_addr Device short address
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_publish_device_state(uint16_t short_addr);

/**
 * @brief Publish device list as JSON over UART
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_publish_device_list(void);

/* ============================================================================
 * Internal API (called by zb_callbacks.c — mirrors mqtt_bridge_internal.h)
 * ============================================================================ */

/**
 * @brief Handle Zigbee device join event
 */
esp_err_t uart_bridge_on_device_join(uint16_t short_addr);

/**
 * @brief Handle Zigbee device leave event
 */
esp_err_t uart_bridge_on_device_leave(uint16_t short_addr);

/**
 * @brief Handle Zigbee attribute change event
 */
esp_err_t uart_bridge_on_attribute_change(uint16_t short_addr,
                                           uint16_t cluster_id,
                                           uint16_t attr_id,
                                           const void *value,
                                           size_t value_len);

/* ============================================================================
 * UART RX Command Processing
 * ============================================================================ */

/**
 * @brief Process a received JSON command line from S3
 * @param json_line NULL-terminated JSON string (without \n)
 * @return ESP_OK on success
 */
esp_err_t uart_bridge_process_command(const char *json_line);

/**
 * @brief UART RX task — reads lines from UART and dispatches commands
 * @param pvParameters unused
 */
void uart_bridge_rx_task(void *pvParameters);

/* ============================================================================
 * Event Publishing (replaces bridge_events.h for UART)
 * ============================================================================ */

/**
 * @brief Publish a generic event over UART
 * @param type Event type string (e.g., "device_joined", "permit_join")
 * @param json_data JSON data string (will be embedded in event)
 */
esp_err_t uart_bridge_publish_event(const char *type, const char *json_data);

/**
 * @brief Send a JSON line over UART (low-level)
 * @param json_str Complete JSON string to send (newline appended automatically)
 */
esp_err_t uart_bridge_send_line(const char *json_str);

#ifdef __cplusplus
}
#endif

#endif /* UART_BRIDGE_H */
