/**
 * @file zb_install_codes.h
 * @brief Zigbee Install Code Management API
 *
 * This module provides support for Zigbee Install Codes, which enable
 * secure device joining using pre-configured link keys. Install codes
 * are used to derive unique link keys via HMAC-SHA-256, providing
 * enhanced security over the default Trust Center link key.
 *
 * Features:
 * - Install code storage in NVS (up to 50 codes)
 * - Install code to link key derivation (HMAC-SHA-256)
 * - Install code validation with CRC check
 * - Pre-configured install codes for specific devices
 * - MQTT commands for install code management
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_INSTALL_CODES_H
#define ZB_INSTALL_CODES_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of install codes that can be stored
 */
#define ZB_INSTALL_CODE_MAX_ENTRIES     50

/**
 * @brief Install code length options
 *
 * Zigbee supports install codes of various lengths.
 * The CRC is included in the length.
 */
#define ZB_INSTALL_CODE_LEN_48_BIT      (6 + 2)   /**< 48-bit code + 16-bit CRC */
#define ZB_INSTALL_CODE_LEN_64_BIT      (8 + 2)   /**< 64-bit code + 16-bit CRC */
#define ZB_INSTALL_CODE_LEN_96_BIT      (12 + 2)  /**< 96-bit code + 16-bit CRC */
#define ZB_INSTALL_CODE_LEN_128_BIT     (16 + 2)  /**< 128-bit code + 16-bit CRC */

/**
 * @brief Maximum install code length (128-bit + CRC)
 */
#define ZB_INSTALL_CODE_MAX_LEN         18

/**
 * @brief Derived link key length (always 128-bit)
 */
#define ZB_INSTALL_CODE_KEY_LEN         16

/**
 * @brief Install code entry structure
 *
 * Represents a single install code entry with associated IEEE address,
 * the install code itself, and the derived link key.
 */
typedef struct {
    uint64_t ieee_addr;                             /**< Device IEEE address (64-bit) */
    uint8_t install_code[ZB_INSTALL_CODE_MAX_LEN];  /**< Install code (including CRC) */
    uint8_t install_code_len;                       /**< Actual install code length */
    uint8_t derived_key[ZB_INSTALL_CODE_KEY_LEN];   /**< Derived link key from install code */
    bool is_active;                                 /**< Whether this entry is active */
    uint32_t added_timestamp;                       /**< Timestamp when entry was added */
} zb_install_code_entry_t;

/**
 * @brief Install code status enumeration
 */
typedef enum {
    ZB_INSTALL_CODE_STATUS_OK = 0,              /**< Install code is valid */
    ZB_INSTALL_CODE_STATUS_INVALID_LENGTH,      /**< Invalid code length */
    ZB_INSTALL_CODE_STATUS_CRC_ERROR,           /**< CRC check failed */
    ZB_INSTALL_CODE_STATUS_STORAGE_FULL,        /**< No more storage space */
    ZB_INSTALL_CODE_STATUS_NOT_FOUND,           /**< Install code not found */
    ZB_INSTALL_CODE_STATUS_ALREADY_EXISTS,      /**< Entry already exists for IEEE address */
    ZB_INSTALL_CODE_STATUS_ERROR                /**< Generic error */
} zb_install_code_status_t;

/**
 * @brief Initialize the install codes module
 *
 * Initializes internal data structures and loads stored install codes
 * from NVS. Must be called before using other install code functions.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_NVS_* on NVS errors
 */
esp_err_t zb_install_codes_init(void);

/**
 * @brief Deinitialize the install codes module
 *
 * Frees all resources used by the install codes module.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_install_codes_deinit(void);

/**
 * @brief Add an install code for a device
 *
 * Adds an install code for the specified IEEE address. The install code
 * is validated (CRC check) and the link key is derived using HMAC-SHA-256.
 * The entry is persisted to NVS.
 *
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @param[in] install_code Install code data (including CRC)
 * @param[in] code_len Length of install code in bytes
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_CRC if CRC validation fails
 * @return ESP_ERR_NO_MEM if storage is full
 * @return ESP_ERR_INVALID_STATE if entry already exists
 */
esp_err_t zb_install_codes_add(uint64_t ieee_addr, const uint8_t *install_code,
                                size_t code_len);

/**
 * @brief Remove an install code entry
 *
 * Removes the install code entry for the specified IEEE address.
 * The change is persisted to NVS.
 *
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if entry does not exist
 */
esp_err_t zb_install_codes_remove(uint64_t ieee_addr);

/**
 * @brief Get the derived link key for a device
 *
 * Retrieves the derived link key for the specified IEEE address.
 * Returns the pre-computed key if an install code exists for this device.
 *
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @param[out] out_key Buffer to store the 16-byte derived key
 * @return ESP_OK if key found and copied
 * @return ESP_ERR_NOT_FOUND if no install code exists for this device
 * @return ESP_ERR_INVALID_ARG if out_key is NULL
 */
esp_err_t zb_install_codes_get_key(uint64_t ieee_addr, uint8_t *out_key);

/**
 * @brief Check if an install code exists for a device
 *
 * Checks whether an install code entry exists for the specified IEEE address.
 *
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @return true if install code exists
 * @return false if no install code exists
 */
bool zb_install_codes_has_entry(uint64_t ieee_addr);

/**
 * @brief Get install code entry by IEEE address
 *
 * Retrieves the complete install code entry for the specified IEEE address.
 *
 * @param[in] ieee_addr Device IEEE address (64-bit)
 * @param[out] out_entry Pointer to store the entry
 * @return ESP_OK if entry found
 * @return ESP_ERR_NOT_FOUND if no entry exists
 * @return ESP_ERR_INVALID_ARG if out_entry is NULL
 */
esp_err_t zb_install_codes_get_entry(uint64_t ieee_addr, zb_install_code_entry_t *out_entry);

/**
 * @brief List all install code entries
 *
 * Retrieves all stored install code entries.
 *
 * @param[out] out_entries Array to store entries
 * @param[in] max_entries Maximum number of entries to retrieve
 * @return Number of entries copied to the array
 */
size_t zb_install_codes_list(zb_install_code_entry_t *out_entries, size_t max_entries);

/**
 * @brief Get the number of stored install codes
 *
 * @return Number of active install code entries
 */
size_t zb_install_codes_count(void);

/**
 * @brief Clear all install code entries
 *
 * Removes all stored install codes from memory and NVS.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_install_codes_clear_all(void);

/**
 * @brief Remove expired install code entries
 *
 * Scans all entries and removes those that have exceeded TTL (30 days).
 * Legacy entries (with timestamp = 0) are not expired.
 *
 * @return Number of entries removed
 */
size_t zb_install_codes_cleanup_expired(void);

/**
 * @brief Validate an install code
 *
 * Validates the install code format and CRC without adding it.
 *
 * @param[in] install_code Install code data (including CRC)
 * @param[in] code_len Length of install code in bytes
 * @return ZB_INSTALL_CODE_STATUS_OK if valid
 * @return ZB_INSTALL_CODE_STATUS_INVALID_LENGTH if length is invalid
 * @return ZB_INSTALL_CODE_STATUS_CRC_ERROR if CRC check fails
 */
zb_install_code_status_t zb_install_codes_validate(const uint8_t *install_code,
                                                    size_t code_len);

/**
 * @brief Derive link key from install code
 *
 * Derives a 128-bit link key from an install code using HMAC-SHA-256.
 * The derivation follows the Zigbee specification using "ZigBeeAlliance09"
 * as the HMAC key.
 *
 * @param[in] install_code Install code data (including CRC)
 * @param[in] code_len Length of install code in bytes
 * @param[out] out_key Buffer for 16-byte derived key
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_FAIL on crypto error
 */
esp_err_t zb_install_codes_derive_key(const uint8_t *install_code, size_t code_len,
                                       uint8_t *out_key);

/**
 * @brief Calculate CRC-16 for install code
 *
 * Calculates the CRC-16/X-25 checksum for an install code.
 * This is the standard CRC used for Zigbee install codes.
 *
 * @param[in] data Install code data (without CRC)
 * @param[in] len Length of data in bytes
 * @return CRC-16 value
 */
uint16_t zb_install_codes_calculate_crc(const uint8_t *data, size_t len);

/**
 * @brief Set install code in Zigbee stack
 *
 * Configures the Zigbee stack to use the derived link key for the
 * specified device during the join process.
 *
 * @param[in] ieee_addr Device IEEE address
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no install code exists for this device
 * @return ESP_FAIL on stack error
 */
esp_err_t zb_install_codes_apply_to_stack(uint64_t ieee_addr);

/**
 * @brief Parse IEEE address from hex string
 *
 * Parses an IEEE address string like "0x00124b001234abcd" to uint64_t.
 *
 * @param[in] ieee_str IEEE address string
 * @param[out] out_addr Parsed 64-bit address
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if string format is invalid
 */
esp_err_t zb_install_codes_parse_ieee(const char *ieee_str, uint64_t *out_addr);

/**
 * @brief Parse install code from hex string
 *
 * Parses an install code hex string like "83FED3407A939723A5C639B26916D505C3B5"
 * to a byte array.
 *
 * @param[in] code_str Install code hex string (with or without separators)
 * @param[out] out_code Buffer for parsed install code
 * @param[in] buf_len Size of output buffer
 * @param[out] out_len Actual parsed length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if string format is invalid
 * @return ESP_ERR_NO_MEM if buffer is too small
 */
esp_err_t zb_install_codes_parse_code(const char *code_str, uint8_t *out_code,
                                       size_t buf_len, size_t *out_len);

/**
 * @brief Format install code as hex string
 *
 * Formats an install code byte array as a hex string.
 *
 * @param[in] install_code Install code data
 * @param[in] code_len Length of install code
 * @param[out] out_str Output string buffer
 * @param[in] str_len Size of output buffer (needs at least code_len*2 + 1)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 */
esp_err_t zb_install_codes_format_code(const uint8_t *install_code, size_t code_len,
                                        char *out_str, size_t str_len);

/**
 * @brief Self-test function for install codes module
 *
 * Tests CRC calculation, key derivation, and basic operations.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_install_codes_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_INSTALL_CODES_H */
