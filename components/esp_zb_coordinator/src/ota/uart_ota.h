/**
 * @file uart_ota.h
 * @brief UART OTA Receiver for ESP32-C5/H2 Zigbee Coordinator
 *
 * Receives firmware updates from ESP32-S3 via UART bridge protocol.
 * Supports chunked transfer with Base64 encoding and MD5 validation.
 */

#ifndef UART_OTA_H
#define UART_OTA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

/** Default chunk size for OTA data (can be overridden by ota_start) */
#define UART_OTA_DEFAULT_CHUNK_SIZE     4096

/** Maximum chunk size supported */
#define UART_OTA_MAX_CHUNK_SIZE         8192

/** Timeout for receiving next chunk (ms) */
#define UART_OTA_TIMEOUT_MS             30000

/** Delay before reboot after successful OTA (seconds) */
#define UART_OTA_REBOOT_DELAY_SEC       3

/* ============================================================================
 * OTA State
 * ============================================================================ */

typedef enum {
    UART_OTA_STATE_IDLE,        /**< No OTA in progress */
    UART_OTA_STATE_READY,       /**< ota_start received, waiting for data */
    UART_OTA_STATE_RECEIVING,   /**< Receiving chunks */
    UART_OTA_STATE_VALIDATING,  /**< All chunks received, validating MD5 */
    UART_OTA_STATE_COMPLETE,    /**< OTA successful, rebooting soon */
    UART_OTA_STATE_ERROR,       /**< OTA failed */
} uart_ota_state_t;

/* ============================================================================
 * OTA Statistics
 * ============================================================================ */

typedef struct {
    uart_ota_state_t state;
    size_t total_size;          /**< Expected firmware size */
    size_t received_size;       /**< Bytes received so far */
    uint32_t chunks_received;   /**< Number of chunks received */
    uint32_t expected_seq;      /**< Next expected sequence number */
    uint8_t progress_percent;   /**< 0-100 */
    char error_msg[64];         /**< Last error message */
} uart_ota_status_t;

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Initialize UART OTA receiver
 * @return ESP_OK on success
 */
esp_err_t uart_ota_init(void);

/**
 * @brief Deinitialize UART OTA receiver
 */
void uart_ota_deinit(void);

/**
 * @brief Handle incoming OTA command from UART bridge
 *
 * Called by uart_bridge when a command with "cmd":"ota_*" is received.
 * Dispatches to appropriate handler based on command type.
 *
 * @param cmd Command string ("ota_start", "ota_data", "ota_end", "ota_abort")
 * @param json Full JSON object from UART
 * @return ESP_OK on success
 */
esp_err_t uart_ota_handle_command(const char *cmd, const cJSON *json);

/**
 * @brief Check if OTA is currently in progress
 * @return true if OTA is active
 */
bool uart_ota_is_in_progress(void);

/**
 * @brief Get current OTA status
 * @param status Output status structure
 */
void uart_ota_get_status(uart_ota_status_t *status);

/**
 * @brief Abort any ongoing OTA
 * @param reason Reason for abort (for logging)
 */
void uart_ota_abort(const char *reason);

/**
 * @brief Check for OTA timeout (call periodically from main loop)
 *
 * If OTA is in progress and no data received for UART_OTA_TIMEOUT_MS,
 * the OTA will be automatically aborted.
 */
void uart_ota_check_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_OTA_H */
