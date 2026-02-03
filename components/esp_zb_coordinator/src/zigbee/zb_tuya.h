/**
 * @file zb_tuya.h
 * @brief Tuya Private Cluster (0xEF00) Support
 *
 * This module handles Tuya's proprietary cluster 0xEF00 which uses a datapoint
 * (DP) protocol for device-specific settings. Supports devices like Fingerbot,
 * Tuya switches, sensors, and other devices that use Tuya's DP protocol.
 *
 * Tuya DP Protocol Format (Cluster 0xEF00):
 *
 * OUTGOING (Coordinator → Device) — Verified with _TZ3210 Fingerbot Plus:
 *   Command:     0x04 (sendData) — NOT 0x00 (dataRequest)!
 *   Payload:     [seq:2][dp_id:1][type:1][len:2][data:N]  (NO status byte)
 *   Total:       6 + N bytes
 *
 *   Example: Set mode to switch (DP 101 = enum 1)
 *     Raw CMD hex: 0400016504000101
 *     Breakdown:   04=cmd, 0001=seq, 65=dp101, 04=enum, 0001=len, 01=switch
 *
 * INCOMING cmd 0x05 (activeStatusReportAlt) — 6-byte header:
 *   Payload:     [seq:2][dp_id:1][type:1][len:2][data:N]  (NO status byte)
 *   ACK (short): [status:1][seq:2][result:2]  (5 bytes, result=0xFFFF=unsupported)
 *
 * INCOMING cmd 0x01/0x02 (dataResponse/dataReport) — 7-byte header:
 *   Payload:     [status:1][seq:2][dp_id:1][type:1][len:2][data:N]
 *   Note:        _TZ3210 Fingerbot uses cmd 0x05, not 0x01/0x02
 *
 * Key findings from hardware testing (2026-01-29, _TZ3210_j4pdtz9v):
 *   - _TZ3210 Fingerbot Plus responds ONLY to cmd 0x04 (sendData)
 *   - cmd 0x00 (dataRequest): ignored or Default Response only
 *   - cmd 0x03 (dataQuery): Default Response OK but no DPs returned
 *   - cmd 0x10 (mcuVersionRequest): Default Response OK, no version
 *   - cmd 0x25 (gatewayConnectionStatus): Default Response OK, no effect
 *   - Device reports spontaneously via cmd 0x05 after pairing only
 *   - Mode change (DP 101) triggers full DP dump — use as "query all DPs"
 *   - Outgoing payload must NOT include status byte (causes frame shift)
 *   - Adding status byte: device accepts frame but ignores DP content
 *   - DP 102 (down_movement): range 51-100, values <51 rejected silently
 *   - DP 103 (sustain_time): unit is SECONDS (not 100ms as documented)
 *   - DP 104 (reverse): reported as enum type (not bool!)
 *   - Sustain + down_movement not combinable in one session; set sustain=0 first
 *   - Time sync (cmd 0x06/0x24) fixed: uses Unix timestamps, not year/month/day
 *   - Program mode (DP 101=2) requires working time sync
 *   - Without time sync, program mode blocks normal push operation
 *   - DP 106 (up_movement): verified, ~25% = arm doesn't return fully to top
 *   - DP 107 (touch_control): verified OFF disables touch surface on top
 *   - DP 104 (reverse): no observable effect when toggled (needs more testing)
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_TUYA_H
#define ZB_TUYA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Tuya Cluster Constants
 * ============================================================================ */

/** @brief Tuya private cluster ID */
#define ZB_TUYA_CLUSTER_ID              0xEF00

/** @brief Maximum number of Tuya devices to track state for */
#define ZB_TUYA_MAX_DEVICES             10

/** @brief Maximum raw DP payload size */
#define ZB_TUYA_DP_MAX_RAW_SIZE         64

/** @brief Tuya DP header size for incoming cmd 0x01/0x02 (status + seq + dp_id + type + len) */
#define ZB_TUYA_DP_HEADER_SIZE          7

/** @brief Tuya DP header size for outgoing cmd 0x04 and incoming cmd 0x05 (seq + dp_id + type + len) */
#define ZB_TUYA_DP_HEADER_SIZE_NO_STATUS 6

/** @brief Tuya outgoing command: sendData (verified working with _TZ3210) */
#define ZB_TUYA_CMD_SEND_DATA           0x04

/** @brief Tuya outgoing command: dataRequest (NOT working with _TZ3210) */
#define ZB_TUYA_CMD_DATA_REQUEST        0x00

/** @brief Tuya outgoing command: dataQuery */
#define ZB_TUYA_CMD_DATA_QUERY          0x03

/* ============================================================================
 * Verified Raw CMD Reference (_TZ3210_j4pdtz9v Fingerbot Plus, 2026-01)
 *
 * Format for HA "Raw CMD" text entity: hex[:src_ep:dst_ep:cluster]
 * First byte = Tuya cmd ID, rest = ZCL payload (no status byte).
 *
 * Tuya DP commands (cmd 0x04, cluster 0xEF00) — ALL VERIFIED WORKING:
 *   DP 1   ON  (switch):     0400010101000101
 *   DP 1   OFF (switch):     0400010101000100
 *   DP 101 push mode:        0400016504000100  (triggers full DP dump!)
 *   DP 101 switch mode:      0400016504000101  (arm stays down on ZCL On)
 *   DP 101 program mode:     0400016504000102  (needs time sync!)
 *   DP 102 down_movement=N:  04000166020004000000XX  (XX = hex, range 33-64 / 51-100 dec)
 *   DP 103 sustain_time=N:   04000167020004000000XX  (XX = hex, unit=SECONDS)
 *   DP 104 reverse ON:       0400016804000101  (enum, not bool!)
 *   DP 104 reverse OFF:      0400016804000100
 *   DP 117 repeat ON:        0400017501000101  (only affects program mode)
 *   DP 117 repeat OFF:       0400017501000100
 *   DP 121 program_en ON:    0400017901000101
 *   DP 121 program_en OFF:   0400017901000100
 *
 * Standard ZCL commands (via cluster override) — VERIFIED WORKING:
 *   ZCL On:                  01:1:1:6          (triggers push cycle)
 *   ZCL Off:                 00:1:1:6
 *   ZCL Toggle:              02:1:1:6
 *
 * Tuya queries (no DP response from _TZ3210, use mode change instead):
 *   Data Query:              030001            (Default Response OK only)
 *   MCU Version:             100001            (Default Response OK only)
 *   Gateway Status:          2501              (Default Response OK only)
 *
 * Complete DP map from device dump (after mode change):
 *   DP 101 (0x65) mode           enum    0=push, 1=switch, 2=program
 *   DP 102 (0x66) down_movement  value   range 51-100 (%)
 *   DP 103 (0x67) sustain_time   value   unit: seconds
 *   DP 104 (0x68) reverse        enum    0=normal, 1=reversed
 *   DP 105 (0x69) battery        value   0-100 (%)
 *   DP 106 (0x6A) up_movement    value   0-49 (%), limits return travel
 *   DP 107 (0x6B) touch_control  bool    physical touch enable
 *   DP 108 (0x6C) click_control  bool    physical button enable
 *   DP 109 (0x6D) program        raw     program sequence data
 *   DP 112 (0x70) unknown        raw     5 bytes, always 00 00 00 00 00
 *   DP 116 (0x74) unknown        raw     5 bytes, always 00 00 00 00 00
 *   DP 117 (0x75) repeat_forever bool    endless program loop
 *   DP 121 (0x79) program_enable bool    activate program mode
 *
 * Program data (DP 109) observed format:
 *   Empty:   ff ff ff 00        (4 bytes, 0 steps)
 *   2-step:  ff ff 00 02 55 00 01 00 00 05  (10 bytes)
 *            [ff ff] = header, [00] = unknown, [02] = step count
 *            step1: [55=85%] [00 01=delay 1], step2: [00=0%] [00 05=delay 5]
 * ============================================================================ */

/* ============================================================================
 * Tuya DP Status Codes
 * ============================================================================ */

/** @brief Tuya DP status: Report (device -> coordinator) */
#define ZB_TUYA_DP_STATUS_REPORT        0x00

/** @brief Tuya DP status: Set (coordinator -> device) */
#define ZB_TUYA_DP_STATUS_SET           0x01

/** @brief Tuya DP status: Query (coordinator -> device) */
#define ZB_TUYA_DP_STATUS_QUERY         0x02

/* ============================================================================
 * Tuya DP Data Types
 * ============================================================================ */

/**
 * @brief Tuya DP data type enumeration
 */
typedef enum {
    TUYA_DP_TYPE_RAW    = 0x00,     /**< Raw data bytes */
    TUYA_DP_TYPE_BOOL   = 0x01,     /**< Boolean (1 byte: 0=false, 1=true) */
    TUYA_DP_TYPE_VALUE  = 0x02,     /**< 32-bit signed integer (big-endian) */
    TUYA_DP_TYPE_STRING = 0x03,     /**< String data */
    TUYA_DP_TYPE_ENUM   = 0x04,     /**< Enumeration (1 byte) */
    TUYA_DP_TYPE_BITMAP = 0x05,     /**< Bitmap (1-4 bytes, big-endian) */
} tuya_dp_type_t;

/* ============================================================================
 * Fingerbot DP Definitions (for _TZ3210 Fingerbot Plus)
 *
 * Reference: https://github.com/Koenkk/zigbee-herdsman-converters
 * Note: _TZ3210 variants use DPs 101-106, older variants may use DPs 1-8
 * ============================================================================ */

/** @brief Fingerbot DP: Switch (trigger push) - same for all variants */
#define TUYA_DP_FINGERBOT_SWITCH        1

/** @brief Fingerbot DP: Mode (click/switch/program) - _TZ3210 uses 101 */
#define TUYA_DP_FINGERBOT_MODE          101

/** @brief Fingerbot DP: Down/lower movement percentage
 *  Verified range: 51-100 (values <51 rejected silently by _TZ3210) */
#define TUYA_DP_FINGERBOT_DOWN_MOVEMENT 102

/** @brief Fingerbot DP: Sustain/hold time in SECONDS (not 100ms!)
 *  Verified: value=2 → 2 seconds, value=10 → 10 seconds
 *  Note: Set sustain=0 before changing down_movement */
#define TUYA_DP_FINGERBOT_SUSTAIN_TIME  103

/** @brief Fingerbot DP: Reverse direction (reported as enum, not bool!) */
#define TUYA_DP_FINGERBOT_REVERSE       104

/** @brief Fingerbot DP: Battery percentage */
#define TUYA_DP_FINGERBOT_BATTERY       105

/** @brief Fingerbot DP: Up/upper movement percentage (0-100) */
#define TUYA_DP_FINGERBOT_UP_MOVEMENT   106

/** @brief Fingerbot DP: Touch control enable/disable */
#define TUYA_DP_FINGERBOT_TOUCH_CONTROL 107

/** @brief Fingerbot Plus DP: Click control (physical button) enable/disable
 *  Verified on _TZ3210_j4pdtz9v: DP 108 (0x6C), bool type.
 *  Note: Commented out in herdsman-converters (0x6c), mapped to onOff */
#define TUYA_DP_FINGERBOT_CLICK_CONTROL 108

/** @brief Fingerbot Plus DP: Program sequence (raw data)
 *  Ref: herdsman-converters 0x6d = "program", tuya.valueConverter.raw
 *
 *  BLE-Integration (ha_tuya_ble) verwendet DP 121 mit Text-Format:
 *    "position[/time];position[/time];..."
 *    z.B. "100/2;0/1" = 100% für 2s, dann 0% für 1s
 *
 *  Verified Zigbee Raw-Format (from device dump, 2026-01-29):
 *
 *  Empty program (4 bytes):
 *    ff ff ff 00
 *    [ff ff] = header, [ff] = unknown, [00] = 0 steps
 *
 *  2-step program (10 bytes):
 *    ff ff 00 02 55 00 01 00 00 05
 *    [ff ff] = header (always ff ff)
 *    [00]    = unknown byte (00 when steps present, ff when empty)
 *    [02]    = step count
 *    Per step (3 bytes each):
 *      [position:1][delay_hi:1][delay_lo:1]
 *    Step 1: [55=85%] [00 01=delay 1]
 *    Step 2: [00=0%]  [00 05=delay 5]
 *
 *  Related DPs:
 *    - DP 121 (program_enable): Must be true for program to run
 *    - DP 117 (repeat_forever): Endless loop of program sequence
 *    - DP 101 = 2: Program mode (triggers time sync request!)
 */
#define TUYA_DP_FINGERBOT_PROGRAM       109

/** @brief Fingerbot Plus DP: Lifetime click counter (value/uint32)
 *  Not in herdsman-converters, observed on _TZ3210_j4pdtz9v.
 *  Read-only counter that increments with each physical actuation.
 *  Note: NOT confirmed in 2026-01 dump — may be DP 112 instead */
#define TUYA_DP_FINGERBOT_COUNT         111

/** @brief Fingerbot Plus DP: Unknown (raw, 5 bytes, always 00 00 00 00 00)
 *  Observed on _TZ3210_j4pdtz9v during full DP dump (2026-01-29).
 *  Possibly click counter or scheduling data. */
#define TUYA_DP_FINGERBOT_UNKNOWN_112   112

/** @brief Fingerbot Plus DP: Unknown (raw, 5 bytes, always 00 00 00 00 00)
 *  Observed on _TZ3210_j4pdtz9v during full DP dump (2026-01-29).
 *  Possibly scheduling or timer data. */
#define TUYA_DP_FINGERBOT_UNKNOWN_116   116

/** @brief Fingerbot Plus DP: Repeat forever (bool)
 *  When true, program sequence repeats indefinitely.
 *  Observed on _TZ3210_j4pdtz9v as DP 117 (0x75). */
#define TUYA_DP_FINGERBOT_REPEAT_FOREVER 117

/** @brief Fingerbot Plus DP: Program enable (bool)
 *  Activates/deactivates program mode. Same DP number (121/0x79) on both
 *  BLE and Zigbee. Set to true before sending program data (DP 109).
 *  Observed on _TZ3210_j4pdtz9v. */
#define TUYA_DP_FINGERBOT_PROGRAM_ENABLE 121

/**
 * @brief Fingerbot mode enumeration
 *
 * Note: _TZ3210 Fingerbot Plus uses different naming:
 * - CLICK (0) = push/momentary
 * - SWITCH (1) = toggle state
 * - PROGRAM (2) = custom sequence
 */
typedef enum {
    FINGERBOT_MODE_PUSH    = 0,     /**< Push/Click mode - momentary action */
    FINGERBOT_MODE_SWITCH  = 1,     /**< Switch mode - toggle state */
    FINGERBOT_MODE_PROGRAM = 2,     /**< Program mode - custom sequence */
} fingerbot_mode_t;

/* ============================================================================
 * Tuya DP Structures
 * ============================================================================ */

/**
 * @brief Parsed Tuya DP structure
 */
typedef struct {
    uint8_t dp_id;                              /**< Datapoint ID */
    tuya_dp_type_t type;                        /**< Data type */
    uint16_t length;                            /**< Payload length */
    union {
        bool bool_value;                        /**< Boolean value (type=BOOL) */
        int32_t int_value;                      /**< Integer value (type=VALUE) */
        uint8_t enum_value;                     /**< Enum value (type=ENUM) */
        uint32_t bitmap_value;                  /**< Bitmap value (type=BITMAP) */
        uint8_t raw[ZB_TUYA_DP_MAX_RAW_SIZE];   /**< Raw/string data */
    } value;
} tuya_dp_t;

/**
 * @brief Fingerbot state structure
 */
typedef struct {
    bool state;                     /**< Switch state (ON/OFF) */
    fingerbot_mode_t mode;          /**< Operating mode */
    uint8_t down_movement;          /**< Down movement percentage (51-100) */
    uint8_t up_movement;            /**< Up movement percentage (0-49) */
    uint8_t sustain_time;           /**< Sustain time in seconds */
    bool reverse;                   /**< Reverse direction flag */
    uint8_t battery;                /**< Battery percentage (0-100) */
    bool touch_control;             /**< Touch control enabled */
    bool click_control;             /**< Click control (physical button) enabled */
    uint32_t click_count;           /**< Lifetime click counter */
    bool repeat_forever;            /**< Program repeats indefinitely */
    bool program_enable;            /**< Program mode active (DP 121) */
    char program_text[128];         /**< Program as user-readable string "100/2;0/1" */
    bool valid;                     /**< State has been received */
    uint16_t short_addr;            /**< Device short address */
    int64_t last_update;            /**< Last update timestamp (microseconds) */
} tuya_fingerbot_state_t;

/* ============================================================================
 * Tuya Module API
 * ============================================================================ */

/**
 * @brief Initialize Tuya module
 *
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_init(void);

/**
 * @brief Deinitialize Tuya module
 *
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_deinit(void);

/**
 * @brief Parse Tuya DP data from raw cluster data
 *
 * Parses the raw Tuya DP protocol data into a structured format.
 * Note: For VALUE type, the int_value is converted from big-endian.
 *
 * @param[in] data Raw DP data (starting from status byte)
 * @param[in] len Data length
 * @param[out] dp Parsed DP structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_SIZE if data is too short
 */
esp_err_t zb_tuya_parse_dp(const uint8_t *data, size_t len, tuya_dp_t *dp);

/**
 * @brief Handle Tuya cluster command from device
 *
 * Handles various Tuya cluster commands:
 * - 0x00/0x01/0x02/0x05: DP reports/queries/status sync
 * - 0x06: Time sync request
 * - 0x24: Time sync response
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Source endpoint
 * @param[in] cmd_id Tuya command ID
 * @param[in] data Raw command data
 * @param[in] len Data length
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_handle_command(uint16_t short_addr, uint8_t endpoint,
                                  uint8_t cmd_id, const uint8_t *data, size_t len);

/**
 * @brief Send Tuya DP set command to device
 *
 * Builds and sends a Tuya DP set command to the specified device.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] dp DP structure with ID, type, and value to send
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_send_dp(uint16_t short_addr, uint8_t endpoint, const tuya_dp_t *dp);

/**
 * @brief Send a Tuya DP with a specific Tuya command ID
 *
 * Allows choosing the Tuya command type:
 *   0x00 = Datapoint Set (default)
 *   0x04 = Datapoint Command (action trigger)
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] dp DP structure with ID, type, and value
 * @param[in] tuya_cmd_id Tuya command ID (0x00, 0x04, etc.)
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_send_dp_with_cmd(uint16_t short_addr, uint8_t endpoint,
                                    const tuya_dp_t *dp, uint8_t tuya_cmd_id);

/**
 * @brief Get Fingerbot state for a device
 *
 * Returns pointer to the cached Fingerbot state for the given device.
 * The pointer is valid until the device is removed or module is deinitialized.
 *
 * @param[in] short_addr Device short address
 * @return Pointer to Fingerbot state, or NULL if not found
 */
const tuya_fingerbot_state_t* zb_tuya_get_fingerbot_state(uint16_t short_addr);

/**
 * @brief Check if device is a Tuya device (has 0xEF00 cluster)
 *
 * @param[in] short_addr Device short address
 * @return true if device has the Tuya private cluster
 */
bool zb_device_is_tuya(uint16_t short_addr);

/**
 * @brief Check if device is a Fingerbot based on model string
 *
 * @param[in] short_addr Device short address
 * @return true if device is identified as a Fingerbot
 */
bool zb_tuya_is_fingerbot(uint16_t short_addr);

/* ============================================================================
 * Fingerbot Command Helpers
 * ============================================================================ */

/**
 * @brief Set Fingerbot switch state (trigger push)
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] state true=ON (push), false=OFF
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_switch(uint16_t short_addr, uint8_t endpoint, bool state);

/**
 * @brief Set Fingerbot operating mode
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] mode Operating mode (push/switch/program)
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_mode(uint16_t short_addr, uint8_t endpoint,
                                      fingerbot_mode_t mode);

/**
 * @brief Set Fingerbot down movement percentage
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] percent Movement percentage (51-100, values <51 rejected by device)
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_down_movement(uint16_t short_addr, uint8_t endpoint,
                                               uint8_t percent);

/**
 * @brief Set Fingerbot sustain time
 *
 * @note Set sustain to 0 before changing down_movement to avoid conflicts.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] seconds Sustain time in seconds (e.g., 2 = 2 seconds)
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_sustain_time(uint16_t short_addr, uint8_t endpoint,
                                              uint8_t seconds);

/**
 * @brief Set Fingerbot reverse direction
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] reverse true to reverse direction
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_reverse(uint16_t short_addr, uint8_t endpoint,
                                         bool reverse);

/**
 * @brief Set Fingerbot up movement percentage (DP 106)
 *
 * Controls how far the arm returns upward. Lower values mean the arm
 * doesn't return fully to the top position.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] percent Up movement percentage (0-50, clamped)
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_up_movement(uint16_t short_addr, uint8_t endpoint,
                                             uint8_t percent);

/**
 * @brief Set Fingerbot touch control (DP 107)
 *
 * Enables or disables the touch surface on top of the Fingerbot.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] enable true to enable touch control, false to disable
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_touch_control(uint16_t short_addr, uint8_t endpoint,
                                               bool enable);

/**
 * @brief Query all Tuya DPs from a device
 *
 * Sends a DP query command (0x03). The device responds with all its
 * current DP values, which will be processed through the normal
 * DP report handler.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_query_dp(uint16_t short_addr, uint8_t endpoint);

/**
 * @brief Set Fingerbot program enable (DP 121)
 *
 * Activates or deactivates program mode. Must be set to true before
 * sending program data (DP 109).
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] enable true to enable program mode, false to disable
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_program_enable(uint16_t short_addr, uint8_t endpoint,
                                                bool enable);

/**
 * @brief Set Fingerbot repeat forever (DP 117)
 *
 * When enabled, program sequence repeats indefinitely.
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] repeat true to repeat forever, false for single run
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_set_repeat_forever(uint16_t short_addr, uint8_t endpoint,
                                                bool repeat);

/**
 * @brief Send Fingerbot program sequence
 *
 * Encodes a BLE-format program string (e.g. "100/2;0/1;50/3") as raw bytes
 * and sends it via DP 109 (program). Also sets mode to PROGRAM (DP 101).
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] program_str Program string in "position[/time];..." format
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_send_program(uint16_t short_addr, uint8_t endpoint,
                                          const char *program_str);

/**
 * @brief Send Fingerbot program as raw binary bytes via DP 109
 *
 * @param[in] short_addr Device short address
 * @param[in] endpoint Target endpoint
 * @param[in] data Raw binary program data
 * @param[in] len Data length
 * @return ESP_OK on success
 */
esp_err_t zb_tuya_fingerbot_send_program_raw(uint16_t short_addr, uint8_t endpoint,
                                              const uint8_t *data, size_t len);

/**
 * @brief Get Fingerbot mode as string
 *
 * @param[in] mode Fingerbot mode enum value
 * @return Static string representation ("push", "switch", or "program")
 */
const char* zb_tuya_fingerbot_mode_to_string(fingerbot_mode_t mode);

/**
 * @brief Parse Fingerbot mode from string
 *
 * @param[in] str Mode string ("push", "switch", or "program")
 * @param[out] mode Output mode enum value
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if string not recognized
 */
esp_err_t zb_tuya_fingerbot_mode_from_string(const char *str, fingerbot_mode_t *mode);

/* ============================================================================
 * Tuya Driver Registry
 * ============================================================================ */

/**
 * @brief Publish Tuya device state via driver
 *
 * Uses the driver registry to build and publish device-specific state JSON.
 *
 * @param[in] short_addr Device short address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no driver bound or no state available
 */
esp_err_t device_state_publish_tuya(uint16_t short_addr);

/* Backward-compatibility includes */
#include "tuya_fingerbot.h"

#ifdef __cplusplus
}
#endif

#endif /* ZB_TUYA_H */
