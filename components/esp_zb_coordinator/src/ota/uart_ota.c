/**
 * @file uart_ota.c
 * @brief UART OTA Receiver Implementation
 */

#include "uart_ota.h"
#include "uart_bridge.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbedtls/md5.h"
#include "mbedtls/base64.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "UART_OTA";

/* ============================================================================
 * OTA Context
 * ============================================================================ */

typedef struct {
    uart_ota_state_t state;
    esp_ota_handle_t handle;
    const esp_partition_t *update_partition;

    size_t total_size;
    size_t received_size;
    uint32_t chunk_seq;
    uint32_t chunk_size;

    char expected_md5[33];
    mbedtls_md5_context md5_ctx;

    uint32_t last_activity_ms;
    char error_msg[64];
} uart_ota_ctx_t;

static uart_ota_ctx_t s_ota_ctx;
static bool s_initialized = false;

/* ============================================================================
 * Helper Functions
 * ============================================================================ */

static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void send_response(const char *type, const cJSON *data)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return;

    cJSON_AddStringToObject(root, "type", type);

    if (data != NULL) {
        cJSON *item;
        cJSON_ArrayForEach(item, data) {
            cJSON_AddItemToObject(root, item->string, cJSON_Duplicate(item, true));
        }
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str != NULL) {
        uart_bridge_send_line(json_str);
        free(json_str);
    }
}

static void send_ota_ready(void)
{
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "chunk_size", s_ota_ctx.chunk_size);

    const char *partition_label = s_ota_ctx.update_partition ?
                                  s_ota_ctx.update_partition->label : "unknown";
    cJSON_AddStringToObject(data, "partition", partition_label);

    send_response("ota_ready", data);
    cJSON_Delete(data);
}

static void send_ota_ack(uint32_t seq, uint8_t progress)
{
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "seq", seq);
    cJSON_AddNumberToObject(data, "progress", progress);

    send_response("ota_ack", data);
    cJSON_Delete(data);
}

static void send_ota_complete(bool md5_ok, int reboot_in)
{
    cJSON *data = cJSON_CreateObject();
    cJSON_AddBoolToObject(data, "md5_ok", md5_ok);
    cJSON_AddNumberToObject(data, "rebooting_in", reboot_in);

    send_response("ota_complete", data);
    cJSON_Delete(data);
}

static void send_ota_error(const char *error, const char *message)
{
    cJSON *data = cJSON_CreateObject();
    cJSON_AddStringToObject(data, "error", error);
    cJSON_AddStringToObject(data, "message", message);

    send_response("ota_error", data);
    cJSON_Delete(data);
}

static void reset_ota_state(void)
{
    if (s_ota_ctx.state != UART_OTA_STATE_IDLE && s_ota_ctx.handle != 0) {
        esp_ota_abort(s_ota_ctx.handle);
    }

    memset(&s_ota_ctx, 0, sizeof(s_ota_ctx));
    s_ota_ctx.state = UART_OTA_STATE_IDLE;
    s_ota_ctx.chunk_size = UART_OTA_DEFAULT_CHUNK_SIZE;
}

/* ============================================================================
 * OTA Command Handlers
 * ============================================================================ */

static esp_err_t handle_ota_start(const cJSON *json)
{
    if (s_ota_ctx.state != UART_OTA_STATE_IDLE) {
        ESP_LOGW(TAG, "OTA already in progress, aborting previous");
        reset_ota_state();
    }

    /* Parse parameters */
    cJSON *size_item = cJSON_GetObjectItem(json, "size");
    cJSON *md5_item = cJSON_GetObjectItem(json, "md5");

    if (size_item == NULL || !cJSON_IsNumber(size_item)) {
        send_ota_error("invalid_params", "Missing or invalid 'size' parameter");
        return ESP_ERR_INVALID_ARG;
    }

    s_ota_ctx.total_size = (size_t)size_item->valuedouble;

    if (md5_item != NULL && cJSON_IsString(md5_item)) {
        strncpy(s_ota_ctx.expected_md5, md5_item->valuestring,
                sizeof(s_ota_ctx.expected_md5) - 1);
    }

    ESP_LOGI(TAG, "OTA start: size=%zu, md5=%s",
             s_ota_ctx.total_size, s_ota_ctx.expected_md5);

    /* Find update partition */
    s_ota_ctx.update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_ota_ctx.update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        send_ota_error("no_partition", "No OTA partition available");
        reset_ota_state();
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Writing to partition: %s (offset=0x%lx, size=%lu)",
             s_ota_ctx.update_partition->label,
             s_ota_ctx.update_partition->address,
             s_ota_ctx.update_partition->size);

    /* Check if firmware fits */
    if (s_ota_ctx.total_size > s_ota_ctx.update_partition->size) {
        ESP_LOGE(TAG, "Firmware too large: %zu > %lu",
                 s_ota_ctx.total_size, s_ota_ctx.update_partition->size);
        send_ota_error("too_large", "Firmware larger than partition");
        reset_ota_state();
        return ESP_ERR_INVALID_SIZE;
    }

    /* Begin OTA */
    esp_err_t err = esp_ota_begin(s_ota_ctx.update_partition,
                                   s_ota_ctx.total_size,
                                   &s_ota_ctx.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_ota_error("ota_begin_failed", esp_err_to_name(err));
        reset_ota_state();
        return err;
    }

    /* Initialize MD5 context */
    mbedtls_md5_init(&s_ota_ctx.md5_ctx);
    mbedtls_md5_starts(&s_ota_ctx.md5_ctx);

    s_ota_ctx.state = UART_OTA_STATE_READY;
    s_ota_ctx.received_size = 0;
    s_ota_ctx.chunk_seq = 0;
    s_ota_ctx.last_activity_ms = get_time_ms();

    send_ota_ready();

    return ESP_OK;
}

static esp_err_t handle_ota_data(const cJSON *json)
{
    if (s_ota_ctx.state != UART_OTA_STATE_READY &&
        s_ota_ctx.state != UART_OTA_STATE_RECEIVING) {
        send_ota_error("invalid_state", "Not ready for OTA data");
        return ESP_ERR_INVALID_STATE;
    }

    /* Parse parameters */
    cJSON *seq_item = cJSON_GetObjectItem(json, "seq");
    cJSON *data_item = cJSON_GetObjectItem(json, "data");

    if (seq_item == NULL || !cJSON_IsNumber(seq_item) ||
        data_item == NULL || !cJSON_IsString(data_item)) {
        send_ota_error("invalid_params", "Missing seq or data");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t seq = (uint32_t)seq_item->valuedouble;
    const char *b64_data = data_item->valuestring;
    size_t b64_len = strlen(b64_data);

    /* Check sequence number */
    if (seq != s_ota_ctx.chunk_seq) {
        ESP_LOGW(TAG, "Sequence mismatch: got %lu, expected %lu",
                 seq, s_ota_ctx.chunk_seq);
        send_ota_error("seq_mismatch", "Unexpected sequence number");
        return ESP_ERR_INVALID_ARG;
    }

    /* Decode Base64 */
    size_t decoded_len = 0;
    uint8_t *decoded = malloc(b64_len);  /* Decoded is always smaller than encoded */
    if (decoded == NULL) {
        send_ota_error("no_memory", "Failed to allocate decode buffer");
        return ESP_ERR_NO_MEM;
    }

    int ret = mbedtls_base64_decode(decoded, b64_len, &decoded_len,
                                     (const unsigned char *)b64_data, b64_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "Base64 decode failed: %d", ret);
        free(decoded);
        send_ota_error("decode_failed", "Base64 decode error");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Chunk %lu: %zu bytes (b64: %zu)", seq, decoded_len, b64_len);

    /* Write to flash */
    esp_err_t err = esp_ota_write(s_ota_ctx.handle, decoded, decoded_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        free(decoded);
        send_ota_error("write_failed", esp_err_to_name(err));
        uart_ota_abort("Flash write failed");
        return err;
    }

    /* Update MD5 */
    mbedtls_md5_update(&s_ota_ctx.md5_ctx, decoded, decoded_len);

    free(decoded);

    /* Update state */
    s_ota_ctx.received_size += decoded_len;
    s_ota_ctx.chunk_seq++;
    s_ota_ctx.state = UART_OTA_STATE_RECEIVING;
    s_ota_ctx.last_activity_ms = get_time_ms();

    /* Calculate progress */
    uint8_t progress = 0;
    if (s_ota_ctx.total_size > 0) {
        progress = (uint8_t)((s_ota_ctx.received_size * 100) / s_ota_ctx.total_size);
    }

    ESP_LOGI(TAG, "OTA progress: %u%% (%zu/%zu bytes)",
             progress, s_ota_ctx.received_size, s_ota_ctx.total_size);

    send_ota_ack(seq, progress);

    return ESP_OK;
}

static esp_err_t handle_ota_end(const cJSON *json)
{
    (void)json;

    if (s_ota_ctx.state != UART_OTA_STATE_RECEIVING) {
        send_ota_error("invalid_state", "No OTA data received");
        return ESP_ERR_INVALID_STATE;
    }

    /* Check received size */
    if (s_ota_ctx.received_size != s_ota_ctx.total_size) {
        ESP_LOGE(TAG, "Size mismatch: received %zu, expected %zu",
                 s_ota_ctx.received_size, s_ota_ctx.total_size);
        send_ota_error("size_mismatch", "Incomplete transfer");
        uart_ota_abort("Size mismatch");
        return ESP_ERR_INVALID_SIZE;
    }

    s_ota_ctx.state = UART_OTA_STATE_VALIDATING;

    /* Finish MD5 calculation */
    uint8_t md5_hash[16];
    mbedtls_md5_finish(&s_ota_ctx.md5_ctx, md5_hash);
    mbedtls_md5_free(&s_ota_ctx.md5_ctx);

    char calculated_md5[33];
    for (int i = 0; i < 16; i++) {
        snprintf(calculated_md5 + i * 2, 3, "%02x", md5_hash[i]);
    }

    ESP_LOGI(TAG, "MD5: calculated=%s, expected=%s",
             calculated_md5, s_ota_ctx.expected_md5);

    /* Validate MD5 if provided */
    bool md5_ok = true;
    if (s_ota_ctx.expected_md5[0] != '\0') {
        md5_ok = (strcasecmp(calculated_md5, s_ota_ctx.expected_md5) == 0);
        if (!md5_ok) {
            ESP_LOGE(TAG, "MD5 mismatch!");
            send_ota_complete(false, 0);
            uart_ota_abort("MD5 mismatch");
            return ESP_ERR_INVALID_CRC;
        }
    }

    /* Finish OTA */
    esp_err_t err = esp_ota_end(s_ota_ctx.handle);
    s_ota_ctx.handle = 0;

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        send_ota_error("ota_end_failed", esp_err_to_name(err));
        uart_ota_abort("OTA end failed");
        return err;
    }

    /* Set boot partition */
    err = esp_ota_set_boot_partition(s_ota_ctx.update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        send_ota_error("set_boot_failed", esp_err_to_name(err));
        uart_ota_abort("Set boot partition failed");
        return err;
    }

    s_ota_ctx.state = UART_OTA_STATE_COMPLETE;

    ESP_LOGI(TAG, "OTA complete! Rebooting in %d seconds...", UART_OTA_REBOOT_DELAY_SEC);
    send_ota_complete(true, UART_OTA_REBOOT_DELAY_SEC);

    /* Schedule reboot */
    vTaskDelay(pdMS_TO_TICKS(UART_OTA_REBOOT_DELAY_SEC * 1000));
    esp_restart();

    return ESP_OK;  /* Never reached */
}

static esp_err_t handle_ota_abort(const cJSON *json)
{
    const char *reason = "User requested";

    cJSON *reason_item = cJSON_GetObjectItem(json, "reason");
    if (reason_item != NULL && cJSON_IsString(reason_item)) {
        reason = reason_item->valuestring;
    }

    uart_ota_abort(reason);
    return ESP_OK;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t uart_ota_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    memset(&s_ota_ctx, 0, sizeof(s_ota_ctx));
    s_ota_ctx.state = UART_OTA_STATE_IDLE;
    s_ota_ctx.chunk_size = UART_OTA_DEFAULT_CHUNK_SIZE;

    s_initialized = true;
    ESP_LOGI(TAG, "UART OTA receiver initialized");

    return ESP_OK;
}

void uart_ota_deinit(void)
{
    if (s_ota_ctx.state != UART_OTA_STATE_IDLE) {
        uart_ota_abort("Deinitializing");
    }
    s_initialized = false;
}

esp_err_t uart_ota_handle_command(const char *cmd, const cJSON *json)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "OTA not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "OTA command: %s", cmd);

    if (strcmp(cmd, "ota_start") == 0) {
        return handle_ota_start(json);
    } else if (strcmp(cmd, "ota_data") == 0) {
        return handle_ota_data(json);
    } else if (strcmp(cmd, "ota_end") == 0) {
        return handle_ota_end(json);
    } else if (strcmp(cmd, "ota_abort") == 0) {
        return handle_ota_abort(json);
    } else {
        ESP_LOGW(TAG, "Unknown OTA command: %s", cmd);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

bool uart_ota_is_in_progress(void)
{
    return s_ota_ctx.state != UART_OTA_STATE_IDLE &&
           s_ota_ctx.state != UART_OTA_STATE_COMPLETE &&
           s_ota_ctx.state != UART_OTA_STATE_ERROR;
}

void uart_ota_get_status(uart_ota_status_t *status)
{
    if (status == NULL) return;

    status->state = s_ota_ctx.state;
    status->total_size = s_ota_ctx.total_size;
    status->received_size = s_ota_ctx.received_size;
    status->chunks_received = s_ota_ctx.chunk_seq;
    status->expected_seq = s_ota_ctx.chunk_seq;

    if (s_ota_ctx.total_size > 0) {
        status->progress_percent = (uint8_t)((s_ota_ctx.received_size * 100) /
                                              s_ota_ctx.total_size);
    } else {
        status->progress_percent = 0;
    }

    strncpy(status->error_msg, s_ota_ctx.error_msg, sizeof(status->error_msg) - 1);
}

void uart_ota_abort(const char *reason)
{
    if (s_ota_ctx.state == UART_OTA_STATE_IDLE) {
        return;
    }

    ESP_LOGW(TAG, "OTA aborted: %s", reason ? reason : "unknown");

    if (s_ota_ctx.handle != 0) {
        esp_ota_abort(s_ota_ctx.handle);
        s_ota_ctx.handle = 0;
    }

    mbedtls_md5_free(&s_ota_ctx.md5_ctx);

    strncpy(s_ota_ctx.error_msg, reason ? reason : "unknown",
            sizeof(s_ota_ctx.error_msg) - 1);

    s_ota_ctx.state = UART_OTA_STATE_ERROR;

    /* Reset to idle after brief delay */
    s_ota_ctx.state = UART_OTA_STATE_IDLE;
}

void uart_ota_check_timeout(void)
{
    if (!uart_ota_is_in_progress()) {
        return;
    }

    uint32_t now = get_time_ms();
    uint32_t elapsed = now - s_ota_ctx.last_activity_ms;

    if (elapsed > UART_OTA_TIMEOUT_MS) {
        ESP_LOGW(TAG, "OTA timeout: no data for %lu ms", elapsed);
        send_ota_error("timeout", "No data received within timeout");
        uart_ota_abort("Timeout");
    }
}
