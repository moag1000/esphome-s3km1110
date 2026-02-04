/**
 * @file xiaomi_tlv.c
 * @brief Xiaomi Proprietary TLV Parser Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "xiaomi_tlv.h"
#include "esp_log.h"

static const char *TAG = "XIAOMI_TLV";

uint16_t xiaomi_zcl_type_size(uint8_t zcl_type)
{
    switch (zcl_type) {
        case 0x10: return 1;  /* boolean */
        case 0x18: return 1;  /* 8-bit bitmap */
        case 0x19: return 2;  /* 16-bit bitmap */
        case 0x20: return 1;  /* uint8 */
        case 0x21: return 2;  /* uint16 */
        case 0x22: return 3;  /* uint24 */
        case 0x23: return 4;  /* uint32 */
        case 0x24: return 5;  /* uint40 */
        case 0x25: return 6;  /* uint48 */
        case 0x28: return 1;  /* int8 */
        case 0x29: return 2;  /* int16 */
        case 0x2A: return 3;  /* int24 */
        case 0x2B: return 4;  /* int32 */
        case 0x39: return 4;  /* float (single) */
        case 0x3A: return 8;  /* double */
        default:   return 0;  /* unknown or variable-length */
    }
}

int xiaomi_tlv_parse(const uint8_t *data, size_t len,
                     xiaomi_tlv_cb_t cb, void *user_data)
{
    if (data == NULL || len == 0 || cb == NULL) {
        return -1;
    }

    size_t offset = 0;
    int count = 0;

    while (offset + 2 <= len) {
        uint8_t tag = data[offset];
        uint8_t zcl_type = data[offset + 1];
        offset += 2;

        uint16_t data_size = xiaomi_zcl_type_size(zcl_type);

        if (data_size == 0) {
            /* Variable-length types: string (0x42) or octet string (0x41)
             * have a 1-byte length prefix */
            if (zcl_type == 0x41 || zcl_type == 0x42) {
                if (offset >= len) {
                    ESP_LOGW(TAG, "TLV truncated at tag 0x%02X (string length)", tag);
                    break;
                }
                data_size = data[offset];
                offset += 1;  /* skip length byte */
            } else {
                ESP_LOGW(TAG, "TLV unknown ZCL type 0x%02X at tag 0x%02X, stopping", zcl_type, tag);
                break;
            }
        }

        if (offset + data_size > len) {
            ESP_LOGW(TAG, "TLV truncated at tag 0x%02X: need %u bytes, have %zu",
                     tag, data_size, len - offset);
            break;
        }

        xiaomi_tlv_entry_t entry = {
            .tag = tag,
            .zcl_type = zcl_type,
            .data_len = data_size,
            .data = &data[offset],
        };

        count++;
        if (!cb(&entry, user_data)) {
            break;
        }

        offset += data_size;
    }

    ESP_LOGD(TAG, "Parsed %d TLV entries from %zu bytes", count, len);
    return count;
}
