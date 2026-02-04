/**
 * @file zb_cmd_retry.h
 * @brief ZCL Command Retry on Device Unavailable
 *
 * Automatic command retry when ESP_ZB_ZDO_DEVICE_UNAVAILABLE fires or a
 * timeout expires without a device response. Stores the last command per
 * device in a small PSRAM-backed table. Confirmed delivery (attribute
 * report/response from the device) clears the entry; otherwise a single
 * retry is attempted.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum concurrent pending commands (PSRAM, ~320 bytes) */
#define CMD_RETRY_MAX_PENDING   8

/** @brief Timeout before retry (milliseconds) */
#define CMD_RETRY_TIMEOUT_MS    29000

/** @brief Maximum retry attempts per command */
#define CMD_RETRY_MAX_ATTEMPTS  1

/**
 * @brief Command types that can be retried
 */
typedef enum {
    CMD_RETRY_NONE = 0,
    CMD_RETRY_ON_OFF,
    CMD_RETRY_LEVEL,
    CMD_RETRY_COLOR,
    CMD_RETRY_TUYA_DP,
} cmd_retry_type_t;

/** @brief Maximum Tuya DP data length stored for retry */
#define CMD_RETRY_TUYA_DATA_MAX 8

/**
 * @brief Pending command entry (~40 bytes per slot)
 */
typedef struct {
    uint16_t short_addr;          /**< Target device */
    uint8_t  endpoint;            /**< Target endpoint */
    cmd_retry_type_t type;        /**< Command type */
    uint8_t  retries;             /**< Retries remaining (starts at CMD_RETRY_MAX_ATTEMPTS) */
    uint32_t timestamp;           /**< When command was sent (xTaskGetTickCount) */
    bool     active;              /**< Slot in use */
    union {
        struct { bool on; } on_off;
        struct { uint8_t level; uint16_t transition; } level;
        struct { uint16_t x, y; uint16_t transition; } color;
        struct {
            uint8_t dp_id;
            uint8_t dp_type;
            uint8_t data[CMD_RETRY_TUYA_DATA_MAX];
            uint8_t data_len;
        } tuya_dp;
    } params;
} pending_cmd_t;

/**
 * @brief Initialize the command retry subsystem
 *
 * Allocates the pending command table in PSRAM and creates the mutex.
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if PSRAM allocation fails
 */
esp_err_t cmd_retry_init(void);

/**
 * @brief Deinitialize the command retry subsystem
 *
 * Frees the pending command table and mutex.
 */
void cmd_retry_deinit(void);

/**
 * @brief Store a pending on/off command for retry
 *
 * @param addr  Target device short address
 * @param ep    Target endpoint
 * @param on    true for ON, false for OFF
 */
void cmd_retry_store_on_off(uint16_t addr, uint8_t ep, bool on);

/**
 * @brief Store a pending level command for retry
 *
 * @param addr  Target device short address
 * @param ep    Target endpoint
 * @param level Brightness level (0-254)
 * @param trans Transition time in 1/10 second units
 */
void cmd_retry_store_level(uint16_t addr, uint8_t ep, uint8_t level, uint16_t trans);

/**
 * @brief Store a pending color command for retry
 *
 * @param addr  Target device short address
 * @param ep    Target endpoint
 * @param x     X color coordinate (0-65535)
 * @param y     Y color coordinate (0-65535)
 * @param trans Transition time in 1/10 second units
 */
void cmd_retry_store_color(uint16_t addr, uint8_t ep, uint16_t x, uint16_t y, uint16_t trans);

/**
 * @brief Store a pending Tuya DP command for retry
 *
 * @param addr     Target device short address
 * @param ep       Target endpoint
 * @param dp_id    Tuya datapoint ID
 * @param dp_type  Tuya datapoint type
 * @param data     Datapoint value data
 * @param data_len Length of data (max CMD_RETRY_TUYA_DATA_MAX)
 */
void cmd_retry_store_tuya_dp(uint16_t addr, uint8_t ep, uint8_t dp_id, uint8_t dp_type,
                              const uint8_t *data, uint8_t data_len);

/**
 * @brief Confirm delivery — clears pending entry for this device
 *
 * Called when any incoming callback (report_attr, read_attr_resp,
 * default_resp, tuya_handle_command) is received from the device.
 *
 * @param short_addr Device short address
 */
void cmd_retry_confirm(uint16_t short_addr);

/**
 * @brief Called from DEVICE_UNAVAILABLE handler — immediate retry
 *
 * Finds the pending entry for the device, attempts retry if retries
 * remain, then clears the entry.
 *
 * @param short_addr Device short address
 */
void cmd_retry_on_unavailable(uint16_t short_addr);

/**
 * @brief Check for timed-out pending commands and retry them
 *
 * Called periodically (e.g., from system_monitor every ~30s).
 * Scans all active entries and retries those that have exceeded
 * CMD_RETRY_TIMEOUT_MS.
 */
void cmd_retry_check_timeouts(void);

#ifdef __cplusplus
}
#endif
