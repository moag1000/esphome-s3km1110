/**
 * @file zb_install_codes.c
 * @brief Zigbee Install Code Management Implementation
 *
 * Implements secure device joining using pre-configured install codes.
 * Install codes are validated with CRC-16/X-25 and converted to link keys
 * using HMAC-SHA-256 as specified in the Zigbee specification.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_install_codes.h"
#include "zb_constants.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mbedtls/md.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <inttypes.h>

static const char *TAG = "ZB_IC";

/* NVS namespace and keys */
#define NVS_NAMESPACE           "zb_ic"
#define NVS_KEY_COUNT           "count"
#define NVS_KEY_ENTRY_PREFIX    "ic_"

/* HMAC key for link key derivation (Zigbee specification) */
static const uint8_t HMAC_KEY[] = "ZigBeeAlliance09";
#define HMAC_KEY_LEN            16

/* CRC-16/X-25 polynomial and initial value */
#define CRC16_POLYNOMIAL        0x1021
#define CRC16_INIT              0xFFFF

/* Install code time-to-live (default: 30 days) */
#define ZB_INSTALL_CODE_TTL_SECONDS  (30 * 24 * 60 * 60)

/* Install code storage - allocated in PSRAM to save internal RAM */
static zb_install_code_entry_t *s_install_codes = NULL;
static size_t s_entry_count = 0;
static bool s_initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

/* Forward declarations */
static esp_err_t load_from_nvs(void);
static esp_err_t save_to_nvs(void);
static esp_err_t save_entry_to_nvs(size_t index);
static int find_entry_index(uint64_t ieee_addr);
static int find_free_slot(void);

/* ============================================================================
 * CRC-16/X-25 Implementation
 * ============================================================================ */

/**
 * @brief Calculate CRC-16/X-25 checksum
 *
 * This is the CRC algorithm used for Zigbee install codes.
 * Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * Initial value: 0xFFFF
 * Input/Output reflection: Yes
 * Final XOR: 0xFFFF
 */
uint16_t zb_install_codes_calculate_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = CRC16_INIT;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408; /* Reflected polynomial */
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFF;
}

/* ============================================================================
 * Link Key Derivation (HMAC-SHA-256)
 * ============================================================================ */

esp_err_t zb_install_codes_derive_key(const uint8_t *install_code, size_t code_len,
                                       uint8_t *out_key)
{
    if (install_code == NULL || out_key == NULL || code_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Use HMAC-SHA-256 with "ZigBeeAlliance09" as the key */
    uint8_t hmac_output[32]; /* SHA-256 produces 32 bytes */

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);

    int ret = mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_setup failed: %d", ret);
        mbedtls_md_free(&ctx);
        return ESP_FAIL;
    }

    ret = mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_hmac_starts failed: %d", ret);
        mbedtls_md_free(&ctx);
        return ESP_FAIL;
    }

    ret = mbedtls_md_hmac_update(&ctx, install_code, code_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_hmac_update failed: %d", ret);
        mbedtls_md_free(&ctx);
        return ESP_FAIL;
    }

    ret = mbedtls_md_hmac_finish(&ctx, hmac_output);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_hmac_finish failed: %d", ret);
        mbedtls_md_free(&ctx);
        return ESP_FAIL;
    }

    mbedtls_md_free(&ctx);

    /* Take first 16 bytes of HMAC-SHA-256 output as the link key */
    memcpy(out_key, hmac_output, ZB_INSTALL_CODE_KEY_LEN);

    /* Clear sensitive data */
    memset(hmac_output, 0, sizeof(hmac_output));

    return ESP_OK;
}

/* ============================================================================
 * Validation Functions
 * ============================================================================ */

zb_install_code_status_t zb_install_codes_validate(const uint8_t *install_code,
                                                    size_t code_len)
{
    if (install_code == NULL) {
        return ZB_INSTALL_CODE_STATUS_ERROR;
    }

    /* Check valid lengths: 8, 10, 14, or 18 bytes (including 2-byte CRC) */
    if (code_len != ZB_INSTALL_CODE_LEN_48_BIT &&
        code_len != ZB_INSTALL_CODE_LEN_64_BIT &&
        code_len != ZB_INSTALL_CODE_LEN_96_BIT &&
        code_len != ZB_INSTALL_CODE_LEN_128_BIT) {
        ESP_LOGW(TAG, "Invalid install code length: %d", code_len);
        return ZB_INSTALL_CODE_STATUS_INVALID_LENGTH;
    }

    /* Extract the CRC from the install code (last 2 bytes, little-endian) */
    uint16_t provided_crc = ((uint16_t)install_code[code_len - 1] << 8) |
                            ((uint16_t)install_code[code_len - 2]);

    /* Calculate CRC over the code (without the CRC bytes) */
    uint16_t calculated_crc = zb_install_codes_calculate_crc(install_code, code_len - 2);

    if (provided_crc != calculated_crc) {
        ESP_LOGW(TAG, "CRC mismatch: provided=0x%04X, calculated=0x%04X",
                 provided_crc, calculated_crc);
        return ZB_INSTALL_CODE_STATUS_CRC_ERROR;
    }

    return ZB_INSTALL_CODE_STATUS_OK;
}

/* ============================================================================
 * NVS Persistence Functions
 * ============================================================================ */

static esp_err_t load_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No stored install codes found");
        return ESP_OK; /* No stored data is OK */
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Read entry count */
    uint8_t count = 0;
    ret = nvs_get_u8(nvs_handle, NVS_KEY_COUNT, &count);
    if (ret != ESP_OK) {
        nvs_close(nvs_handle);
        return ESP_OK; /* No count = no entries */
    }

    if (count > ZB_INSTALL_CODE_MAX_ENTRIES) {
        count = ZB_INSTALL_CODE_MAX_ENTRIES;
    }

    ESP_LOGI(TAG, "Loading %d install codes from NVS", count);

    /* Read each entry */
    char key[16];
    for (size_t i = 0; i < count; i++) {
        snprintf(key, sizeof(key), "%s%d", NVS_KEY_ENTRY_PREFIX, (int)i);

        size_t required_size = sizeof(zb_install_code_entry_t);
        ret = nvs_get_blob(nvs_handle, key, &s_install_codes[i], &required_size);
        if (ret == ESP_OK && s_install_codes[i].is_active) {
            s_entry_count++;
            ESP_LOGD(TAG, "Loaded install code for 0x%016" PRIX64,
                     s_install_codes[i].ieee_addr);
        }
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded %d active install codes", s_entry_count);
    return ESP_OK;
}

static esp_err_t save_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Count active entries */
    uint8_t active_count = 0;
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES; i++) {
        if (s_install_codes[i].is_active) {
            active_count++;
        }
    }

    /* Save count */
    ret = nvs_set_u8(nvs_handle, NVS_KEY_COUNT, active_count);
    if (ret != ESP_OK) {
        nvs_close(nvs_handle);
        return ret;
    }

    /* Save each active entry */
    char key[16];
    size_t save_index = 0;
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES && save_index < active_count; i++) {
        if (s_install_codes[i].is_active) {
            snprintf(key, sizeof(key), "%s%d", NVS_KEY_ENTRY_PREFIX, (int)save_index);
            ret = nvs_set_blob(nvs_handle, key, &s_install_codes[i],
                              sizeof(zb_install_code_entry_t));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save entry %d: %s", (int)i, esp_err_to_name(ret));
            }
            save_index++;
        }
    }

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %d install codes to NVS", active_count);
    }

    return ret;
}

static esp_err_t save_entry_to_nvs(size_t index)
{
    /* For simplicity, save all entries (could be optimized for single entry) */
    return save_to_nvs();
}

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Check if an install code entry has expired
 *
 * @param entry Pointer to install code entry
 * @return true if expired or invalid, false if still valid
 */
static bool is_entry_expired(const zb_install_code_entry_t *entry)
{
    if (entry == NULL || !entry->is_active) {
        return true;
    }

    /* If timestamp is 0, entry was created before expiration was implemented */
    if (entry->added_timestamp == 0) {
        return false;  /* Don't expire legacy entries */
    }

    time_t now = time(NULL);
    if (now == (time_t)-1) {
        return false;  /* Time not available, don't expire */
    }

    return ((uint32_t)now - entry->added_timestamp) > ZB_INSTALL_CODE_TTL_SECONDS;
}

static int find_entry_index(uint64_t ieee_addr)
{
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES; i++) {
        if (s_install_codes[i].is_active &&
            s_install_codes[i].ieee_addr == ieee_addr &&
            !is_entry_expired(&s_install_codes[i])) {
            return (int)i;
        }
    }
    return -1;
}

static int find_free_slot(void)
{
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES; i++) {
        if (!s_install_codes[i].is_active) {
            return (int)i;
        }
    }
    return -1;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_install_codes_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Install codes module already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing install codes module...");

    /* Create mutex for thread-safe cleanup operations */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate install code storage in PSRAM (saves ~1.2KB internal RAM) */
    if (s_install_codes == NULL) {
        s_install_codes = heap_caps_calloc(ZB_INSTALL_CODE_MAX_ENTRIES,
                                            sizeof(zb_install_code_entry_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_install_codes) {
            ESP_LOGW(TAG, "PSRAM alloc failed, falling back to internal RAM");
            s_install_codes = calloc(ZB_INSTALL_CODE_MAX_ENTRIES, sizeof(zb_install_code_entry_t));
        }
        if (!s_install_codes) {
            ESP_LOGE(TAG, "Failed to allocate install code storage");
            vSemaphoreDelete(s_mutex);
            s_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    } else {
        memset(s_install_codes, 0,
               ZB_INSTALL_CODE_MAX_ENTRIES * sizeof(zb_install_code_entry_t));
    }
    s_entry_count = 0;

    /* Load from NVS */
    esp_err_t ret = load_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load install codes from NVS: %s", esp_err_to_name(ret));
        /* Continue anyway with empty storage */
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Install codes module initialized with %d entries", s_entry_count);
    return ESP_OK;
}

esp_err_t zb_install_codes_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    /* Clear sensitive data */
    memset(s_install_codes, 0, ZB_INSTALL_CODE_MAX_ENTRIES * sizeof(zb_install_code_entry_t));
    s_entry_count = 0;
    s_initialized = false;

    /* Delete mutex */
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    ESP_LOGI(TAG, "Install codes module deinitialized");
    return ESP_OK;
}

esp_err_t zb_install_codes_add(uint64_t ieee_addr, const uint8_t *install_code,
                                size_t code_len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (install_code == NULL || code_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate install code */
    zb_install_code_status_t status = zb_install_codes_validate(install_code, code_len);
    if (status == ZB_INSTALL_CODE_STATUS_INVALID_LENGTH) {
        ESP_LOGE(TAG, "Invalid install code length: %d", code_len);
        return ESP_ERR_INVALID_ARG;
    }
    if (status == ZB_INSTALL_CODE_STATUS_CRC_ERROR) {
        ESP_LOGE(TAG, "Install code CRC validation failed");
        return ESP_ERR_INVALID_CRC;
    }
    if (status != ZB_INSTALL_CODE_STATUS_OK) {
        return ESP_FAIL;
    }

    /* Check if entry already exists */
    int existing = find_entry_index(ieee_addr);
    if (existing >= 0) {
        ESP_LOGW(TAG, "Install code already exists for 0x%016" PRIX64 ", updating",
                 ieee_addr);
        /* Update existing entry */
        memcpy(s_install_codes[existing].install_code, install_code, code_len);
        s_install_codes[existing].install_code_len = code_len;
        s_install_codes[existing].added_timestamp = (uint32_t)time(NULL);

        /* Derive new key */
        esp_err_t ret = zb_install_codes_derive_key(install_code, code_len,
                                                     s_install_codes[existing].derived_key);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to derive link key");
            return ret;
        }

        return save_entry_to_nvs((size_t)existing);
    }

    /* Find free slot */
    int slot = find_free_slot();
    if (slot < 0) {
        ESP_LOGE(TAG, "Install code storage full (%d entries)", ZB_INSTALL_CODE_MAX_ENTRIES);
        return ESP_ERR_NO_MEM;
    }

    /* Create new entry */
    zb_install_code_entry_t *entry = &s_install_codes[slot];
    memset(entry, 0, sizeof(zb_install_code_entry_t));

    entry->ieee_addr = ieee_addr;
    memcpy(entry->install_code, install_code, code_len);
    entry->install_code_len = code_len;
    entry->is_active = true;
    entry->added_timestamp = (uint32_t)time(NULL);

    /* Derive link key */
    esp_err_t ret = zb_install_codes_derive_key(install_code, code_len, entry->derived_key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to derive link key");
        entry->is_active = false;
        return ret;
    }

    s_entry_count++;

    ESP_LOGI(TAG, "Added install code for 0x%016" PRIX64 " (slot %d, total %d)",
             ieee_addr, slot, s_entry_count);

    /* Log derived key for debugging (first 4 bytes only) */
    ESP_LOGD(TAG, "Derived key: %02X%02X%02X%02X...",
             entry->derived_key[0], entry->derived_key[1],
             entry->derived_key[2], entry->derived_key[3]);

    return save_entry_to_nvs((size_t)slot);
}

esp_err_t zb_install_codes_remove(uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int index = find_entry_index(ieee_addr);
    if (index < 0) {
        ESP_LOGW(TAG, "No install code found for 0x%016" PRIX64, ieee_addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* Clear entry */
    memset(&s_install_codes[index], 0, sizeof(zb_install_code_entry_t));
    s_entry_count--;

    ESP_LOGI(TAG, "Removed install code for 0x%016" PRIX64 " (total %d)",
             ieee_addr, s_entry_count);

    return save_to_nvs();
}

esp_err_t zb_install_codes_get_key(uint64_t ieee_addr, uint8_t *out_key)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int index = find_entry_index(ieee_addr);
    if (index < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(out_key, s_install_codes[index].derived_key, ZB_INSTALL_CODE_KEY_LEN);
    return ESP_OK;
}

bool zb_install_codes_has_entry(uint64_t ieee_addr)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Install codes module not initialized");
        return false;
    }
    return find_entry_index(ieee_addr) >= 0;
}

esp_err_t zb_install_codes_get_entry(uint64_t ieee_addr, zb_install_code_entry_t *out_entry)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_entry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int index = find_entry_index(ieee_addr);
    if (index < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(out_entry, &s_install_codes[index], sizeof(zb_install_code_entry_t));
    return ESP_OK;
}

size_t zb_install_codes_list(zb_install_code_entry_t *out_entries, size_t max_entries)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Install codes module not initialized");
        return 0;
    }
    if (out_entries == NULL || max_entries == 0) {
        return 0;
    }

    size_t copied = 0;
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES && copied < max_entries; i++) {
        if (s_install_codes[i].is_active) {
            memcpy(&out_entries[copied], &s_install_codes[i],
                   sizeof(zb_install_code_entry_t));
            copied++;
        }
    }

    return copied;
}

size_t zb_install_codes_count(void)
{
    if (!s_initialized) {
        ESP_LOGD(TAG, "Install codes module not initialized");
        return 0;
    }
    return s_entry_count;
}

esp_err_t zb_install_codes_clear_all(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Clearing all install codes!");

    /* Clear memory */
    memset(s_install_codes, 0, ZB_INSTALL_CODE_MAX_ENTRIES * sizeof(zb_install_code_entry_t));
    s_entry_count = 0;

    /* Clear NVS */
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    ESP_LOGI(TAG, "All install codes cleared");
    return ESP_OK;
}

/* ============================================================================
 * Zigbee Stack Integration
 * ============================================================================ */

esp_err_t zb_install_codes_apply_to_stack(uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int index = find_entry_index(ieee_addr);
    if (index < 0) {
        return ESP_ERR_NOT_FOUND;
    }

    zb_install_code_entry_t *entry = &s_install_codes[index];

    /* Convert uint64_t to esp_zb_ieee_addr_t (8-byte array, little-endian) */
    esp_zb_ieee_addr_t ieee_array;
    zb_u64_to_ieee(ieee_addr, ieee_array);

    /* Add the link key to the Trust Center for this device */
    esp_err_t ret = esp_zb_secur_ic_add(ieee_array, entry->install_code_len,
                                         entry->install_code);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add install code to stack: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Applied install code to stack for 0x%016" PRIX64, ieee_addr);
    return ESP_OK;
}

/* ============================================================================
 * Parsing Functions
 * ============================================================================ */

esp_err_t zb_install_codes_parse_ieee(const char *ieee_str, uint64_t *out_addr)
{
    if (ieee_str == NULL || out_addr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Skip "0x" prefix if present */
    const char *ptr = ieee_str;
    if (ptr[0] == '0' && (ptr[1] == 'x' || ptr[1] == 'X')) {
        ptr += 2;
    }

    /* Parse as hexadecimal */
    char *endptr;
    uint64_t addr = strtoull(ptr, &endptr, 16);

    /* Check for parse errors */
    if (endptr == ptr || (*endptr != '\0' && !isspace((unsigned char)*endptr))) {
        ESP_LOGW(TAG, "Invalid IEEE address format: %s", ieee_str);
        return ESP_ERR_INVALID_ARG;
    }

    *out_addr = addr;
    return ESP_OK;
}

esp_err_t zb_install_codes_parse_code(const char *code_str, uint8_t *out_code,
                                       size_t buf_len, size_t *out_len)
{
    if (code_str == NULL || out_code == NULL || out_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t str_len = strlen(code_str);
    size_t byte_count = 0;

    /* Parse hex string, skipping separators like '-', ':', ' ' */
    for (size_t i = 0; i < str_len && byte_count < buf_len; ) {
        /* Skip separators */
        while (i < str_len && (code_str[i] == '-' || code_str[i] == ':' ||
               code_str[i] == ' ' || code_str[i] == '\t')) {
            i++;
        }

        if (i >= str_len) {
            break;
        }

        /* Parse two hex characters */
        if (i + 1 >= str_len) {
            ESP_LOGW(TAG, "Odd number of hex characters");
            return ESP_ERR_INVALID_ARG;
        }

        char hex[3] = { code_str[i], code_str[i + 1], '\0' };

        if (!isxdigit((unsigned char)hex[0]) || !isxdigit((unsigned char)hex[1])) {
            ESP_LOGW(TAG, "Invalid hex character at position %d", (int)i);
            return ESP_ERR_INVALID_ARG;
        }

        out_code[byte_count++] = (uint8_t)strtol(hex, NULL, 16);
        i += 2;
    }

    *out_len = byte_count;
    return ESP_OK;
}

esp_err_t zb_install_codes_format_code(const uint8_t *install_code, size_t code_len,
                                        char *out_str, size_t str_len)
{
    if (install_code == NULL || out_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (str_len < code_len * 2 + 1) {
        return ESP_ERR_NO_MEM;
    }

    char *ptr = out_str;
    size_t remaining = str_len;
    for (size_t i = 0; i < code_len && remaining > 2; i++) {
        int written = snprintf(ptr, remaining, "%02X", install_code[i]);
        if (written < 0 || (size_t)written >= remaining) break;
        ptr += written;
        remaining -= written;
    }
    *ptr = '\0';

    return ESP_OK;
}

/* ============================================================================
 * Expired Entry Cleanup
 * ============================================================================ */

size_t zb_install_codes_cleanup_expired(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    size_t removed = 0;
    for (size_t i = 0; i < ZB_INSTALL_CODE_MAX_ENTRIES; i++) {
        if (s_install_codes[i].is_active && is_entry_expired(&s_install_codes[i])) {
            ESP_LOGI(TAG, "Removing expired install code for 0x%016" PRIX64,
                     s_install_codes[i].ieee_addr);
            memset(&s_install_codes[i], 0, sizeof(zb_install_code_entry_t));
            s_entry_count--;
            removed++;
        }
    }

    if (removed > 0) {
        save_to_nvs();
        ESP_LOGI(TAG, "Cleaned up %zu expired install code entries", removed);
    }

    xSemaphoreGive(s_mutex);
    return removed;
}

/* ============================================================================
 * Self-Test Function
 * ============================================================================ */

esp_err_t zb_install_codes_test(void)
{
    ESP_LOGI(TAG, "Running install codes self-test...");

    /* Test CRC calculation with known values */
    /* Example: 48-bit install code: 83 FE D3 40 7A 93 */
    uint8_t test_code_48[] = { 0x83, 0xFE, 0xD3, 0x40, 0x7A, 0x93 };
    uint16_t crc_48 = zb_install_codes_calculate_crc(test_code_48, sizeof(test_code_48));
    ESP_LOGI(TAG, "CRC for 48-bit test code: 0x%04X", crc_48);

    /* Test validation with valid code (including CRC) */
    uint8_t test_code_with_crc[8];
    memcpy(test_code_with_crc, test_code_48, 6);
    test_code_with_crc[6] = (uint8_t)(crc_48 & 0xFF);       /* CRC low byte */
    test_code_with_crc[7] = (uint8_t)((crc_48 >> 8) & 0xFF); /* CRC high byte */

    zb_install_code_status_t status = zb_install_codes_validate(test_code_with_crc, 8);
    if (status != ZB_INSTALL_CODE_STATUS_OK) {
        ESP_LOGE(TAG, "FAILED: Valid code rejected (status=%d)", status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASSED: Valid code accepted");

    /* Test validation with invalid CRC */
    test_code_with_crc[7] ^= 0xFF; /* Corrupt CRC */
    status = zb_install_codes_validate(test_code_with_crc, 8);
    if (status != ZB_INSTALL_CODE_STATUS_CRC_ERROR) {
        ESP_LOGE(TAG, "FAILED: Invalid CRC not detected (status=%d)", status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASSED: Invalid CRC detected");

    /* Test key derivation */
    uint8_t derived_key[16];
    test_code_with_crc[7] ^= 0xFF; /* Restore CRC */
    esp_err_t ret = zb_install_codes_derive_key(test_code_with_crc, 8, derived_key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAILED: Key derivation error: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASSED: Key derivation successful");
    ESP_LOGI(TAG, "Derived key: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
             derived_key[0], derived_key[1], derived_key[2], derived_key[3],
             derived_key[4], derived_key[5], derived_key[6], derived_key[7],
             derived_key[8], derived_key[9], derived_key[10], derived_key[11],
             derived_key[12], derived_key[13], derived_key[14], derived_key[15]);

    /* Test parsing functions */
    uint64_t parsed_ieee;
    ret = zb_install_codes_parse_ieee("0x00124b001234ABCD", &parsed_ieee);
    if (ret != ESP_OK || parsed_ieee != 0x00124b001234ABCDULL) {
        ESP_LOGE(TAG, "FAILED: IEEE parsing error");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASSED: IEEE parsing successful: 0x%016" PRIX64, parsed_ieee);

    /* Test install code parsing */
    uint8_t parsed_code[18];
    size_t parsed_len;
    ret = zb_install_codes_parse_code("83-FE-D3-40-7A-93-AB-CD", parsed_code,
                                       sizeof(parsed_code), &parsed_len);
    if (ret != ESP_OK || parsed_len != 8) {
        ESP_LOGE(TAG, "FAILED: Install code parsing error");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "PASSED: Install code parsing successful (len=%d)", parsed_len);

    ESP_LOGI(TAG, "Install codes self-test PASSED");
    return ESP_OK;
}
