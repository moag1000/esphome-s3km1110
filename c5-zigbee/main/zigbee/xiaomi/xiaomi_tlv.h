/**
 * @file xiaomi_tlv.h
 * @brief Xiaomi Proprietary TLV Parser (Attribute 0xFF01)
 *
 * Parses the Xiaomi/Aqara proprietary 0xFF01 attribute format used
 * in Cluster 0x0000 (Basic). Format per entry: [tag:1][zcl_type:1][data:N]
 *
 * Reusable for all Xiaomi/Aqara Zigbee devices.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef XIAOMI_TLV_H
#define XIAOMI_TLV_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Well-known Xiaomi TLV tags */
#define XIAOMI_TAG_BATTERY_MV       0x01  /**< Battery voltage in mV (uint16) */
#define XIAOMI_TAG_TEMPERATURE      0x03  /**< Device temperature (int16, °C * 100) */
#define XIAOMI_TAG_BATTERY_PCT      0x0A  /**< Battery percentage (uint8) */

/** @brief Xiaomi manufacturer-specific attribute ID */
#define XIAOMI_ATTR_SPECIAL         0xFF01

/** @brief Xiaomi manufacturer code */
#define XIAOMI_MANUFACTURER_CODE    0x115F

/**
 * @brief Parsed TLV entry
 */
typedef struct {
    uint8_t  tag;         /**< Tag identifier */
    uint8_t  zcl_type;    /**< ZCL data type */
    uint16_t data_len;    /**< Length of data in bytes */
    const uint8_t *data;  /**< Pointer into original buffer (not copied) */
} xiaomi_tlv_entry_t;

/**
 * @brief Callback invoked for each parsed TLV entry
 *
 * @param entry Parsed entry (pointer valid only during callback)
 * @param user_data Opaque context pointer
 * @return true to continue parsing, false to stop
 */
typedef bool (*xiaomi_tlv_cb_t)(const xiaomi_tlv_entry_t *entry, void *user_data);

/**
 * @brief Parse Xiaomi 0xFF01 TLV payload
 *
 * Iterates over TLV entries in the raw attribute data and invokes
 * the callback for each entry.
 *
 * @param data Raw attribute data bytes
 * @param len Length of data
 * @param cb Callback function
 * @param user_data Opaque pointer passed to callback
 * @return Number of entries parsed, or -1 on format error
 */
int xiaomi_tlv_parse(const uint8_t *data, size_t len,
                     xiaomi_tlv_cb_t cb, void *user_data);

/**
 * @brief Get data size for a ZCL attribute type
 *
 * Returns the fixed size for known ZCL types, or 0 for variable/unknown.
 *
 * @param zcl_type ZCL attribute type ID
 * @return Size in bytes, or 0 if variable/unknown
 */
uint16_t xiaomi_zcl_type_size(uint8_t zcl_type);

#ifdef __cplusplus
}
#endif

#endif /* XIAOMI_TLV_H */
