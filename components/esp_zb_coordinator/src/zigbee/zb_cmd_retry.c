/**
 * @file zb_cmd_retry.c
 * @brief ZCL Command Retry on Device Unavailable
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_cmd_retry.h"
#include "compat_stubs.h"
#include "command_handler.h"
#include "zb_tuya.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "CMD_RETRY";

/** @brief PSRAM-allocated array of pending command slots */
static pending_cmd_t *s_pending_cmds = NULL;

/** @brief Mutex protecting table access */
static SemaphoreHandle_t s_retry_mutex = NULL;

/**
 * @brief Find slot for a given short_addr (existing or empty, LRU evict if full)
 *
 * @param short_addr Device to find or allocate for
 * @return Pointer to the slot, or NULL on error
 */
static pending_cmd_t *find_or_alloc_slot(uint16_t short_addr)
{
    pending_cmd_t *empty_slot = NULL;
    pending_cmd_t *oldest_slot = NULL;
    uint32_t oldest_tick = UINT32_MAX;

    for (int i = 0; i < CMD_RETRY_MAX_PENDING; i++) {
        pending_cmd_t *slot = &s_pending_cmds[i];

        /* Existing entry for same device — overwrite */
        if (slot->active && slot->short_addr == short_addr) {
            return slot;
        }

        /* Track first empty slot */
        if (!slot->active && empty_slot == NULL) {
            empty_slot = slot;
        }

        /* Track oldest active slot for LRU eviction */
        if (slot->active && slot->timestamp < oldest_tick) {
            oldest_tick = slot->timestamp;
            oldest_slot = slot;
        }
    }

    if (empty_slot != NULL) {
        return empty_slot;
    }

    /* Table full — evict oldest entry */
    if (oldest_slot != NULL) {
        ESP_LOGW(TAG, "Table full, evicting entry for 0x%04X", oldest_slot->short_addr);
        oldest_slot->active = false;
        return oldest_slot;
    }

    return NULL;
}

/**
 * @brief Execute retry for a pending command entry
 *
 * Calls the appropriate command_send_*() function. These functions
 * already handle Zigbee lock acquisition internally.
 *
 * @param entry Pending command entry to retry
 * @param reason Description for logging ("unavailable" or "timeout")
 */
static void execute_retry(pending_cmd_t *entry, const char *reason)
{
    esp_err_t ret = ESP_FAIL;

    switch (entry->type) {
        case CMD_RETRY_ON_OFF:
            ESP_LOGI(TAG, "Retrying on_off for 0x%04X EP%d (%s): %s",
                     entry->short_addr, entry->endpoint, reason,
                     entry->params.on_off.on ? "ON" : "OFF");
            ret = command_send_on_off(entry->short_addr, entry->endpoint,
                                       entry->params.on_off.on);
            break;

        case CMD_RETRY_LEVEL:
            ESP_LOGI(TAG, "Retrying level for 0x%04X EP%d (%s): %d",
                     entry->short_addr, entry->endpoint, reason,
                     entry->params.level.level);
            ret = command_send_level(entry->short_addr, entry->endpoint,
                                      entry->params.level.level,
                                      entry->params.level.transition);
            break;

        case CMD_RETRY_COLOR:
            ESP_LOGI(TAG, "Retrying color for 0x%04X EP%d (%s): x=%d y=%d",
                     entry->short_addr, entry->endpoint, reason,
                     entry->params.color.x, entry->params.color.y);
            ret = command_send_color(entry->short_addr, entry->endpoint,
                                      entry->params.color.x,
                                      entry->params.color.y,
                                      entry->params.color.transition);
            break;

        case CMD_RETRY_TUYA_DP: {
            ESP_LOGI(TAG, "Retrying tuya_dp for 0x%04X EP%d (%s): dp=%d",
                     entry->short_addr, entry->endpoint, reason,
                     entry->params.tuya_dp.dp_id);
            tuya_dp_t dp = {0};
            dp.dp_id = entry->params.tuya_dp.dp_id;
            dp.type = (tuya_dp_type_t)entry->params.tuya_dp.dp_type;
            dp.length = entry->params.tuya_dp.data_len;
            /* Reconstruct value from stored data bytes */
            if (dp.type == TUYA_DP_TYPE_BOOL && entry->params.tuya_dp.data_len >= 1) {
                dp.value.bool_value = entry->params.tuya_dp.data[0] ? true : false;
            } else if (dp.type == TUYA_DP_TYPE_ENUM && entry->params.tuya_dp.data_len >= 1) {
                dp.value.enum_value = entry->params.tuya_dp.data[0];
            } else if (dp.type == TUYA_DP_TYPE_VALUE && entry->params.tuya_dp.data_len >= 4) {
                dp.value.int_value = (int32_t)(entry->params.tuya_dp.data[0] |
                                     ((uint32_t)entry->params.tuya_dp.data[1] << 8) |
                                     ((uint32_t)entry->params.tuya_dp.data[2] << 16) |
                                     ((uint32_t)entry->params.tuya_dp.data[3] << 24));
            } else {
                /* Raw/bitmap — copy into raw buffer */
                uint8_t copy_len = entry->params.tuya_dp.data_len;
                if (copy_len > sizeof(dp.value.raw)) {
                    copy_len = sizeof(dp.value.raw);
                }
                memcpy(dp.value.raw, entry->params.tuya_dp.data, copy_len);
            }
            ret = zb_tuya_send_dp(entry->short_addr, entry->endpoint, &dp);
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown retry type %d for 0x%04X", entry->type, entry->short_addr);
            break;
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Retry failed for 0x%04X: %s", entry->short_addr, esp_err_to_name(ret));
    }
}

esp_err_t cmd_retry_init(void)
{
    if (s_pending_cmds != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    /* Allocate table in PSRAM */
    size_t table_size = CMD_RETRY_MAX_PENDING * sizeof(pending_cmd_t);
    s_pending_cmds = heap_caps_calloc(CMD_RETRY_MAX_PENDING, sizeof(pending_cmd_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_pending_cmds == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes in PSRAM", table_size);
        return ESP_ERR_NO_MEM;
    }

    s_retry_mutex = xSemaphoreCreateMutex();
    if (s_retry_mutex == NULL) {
        heap_caps_free(s_pending_cmds);
        s_pending_cmds = NULL;
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initialized (%d slots, %zu bytes PSRAM)", CMD_RETRY_MAX_PENDING, table_size);
    return ESP_OK;
}

void cmd_retry_deinit(void)
{
    if (s_retry_mutex != NULL) {
        vSemaphoreDelete(s_retry_mutex);
        s_retry_mutex = NULL;
    }
    if (s_pending_cmds != NULL) {
        heap_caps_free(s_pending_cmds);
        s_pending_cmds = NULL;
    }
    ESP_LOGI(TAG, "Deinitialized");
}

void cmd_retry_store_on_off(uint16_t addr, uint8_t ep, bool on)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    pending_cmd_t *slot = find_or_alloc_slot(addr);
    if (slot != NULL) {
        slot->short_addr = addr;
        slot->endpoint = ep;
        slot->type = CMD_RETRY_ON_OFF;
        slot->retries = CMD_RETRY_MAX_ATTEMPTS;
        slot->timestamp = xTaskGetTickCount();
        slot->active = true;
        slot->params.on_off.on = on;
        ESP_LOGD(TAG, "Stored on_off for 0x%04X: %s", addr, on ? "ON" : "OFF");
    }

    xSemaphoreGive(s_retry_mutex);
}

void cmd_retry_store_level(uint16_t addr, uint8_t ep, uint8_t level, uint16_t trans)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    pending_cmd_t *slot = find_or_alloc_slot(addr);
    if (slot != NULL) {
        slot->short_addr = addr;
        slot->endpoint = ep;
        slot->type = CMD_RETRY_LEVEL;
        slot->retries = CMD_RETRY_MAX_ATTEMPTS;
        slot->timestamp = xTaskGetTickCount();
        slot->active = true;
        slot->params.level.level = level;
        slot->params.level.transition = trans;
        ESP_LOGD(TAG, "Stored level for 0x%04X: %d", addr, level);
    }

    xSemaphoreGive(s_retry_mutex);
}

void cmd_retry_store_color(uint16_t addr, uint8_t ep, uint16_t x, uint16_t y, uint16_t trans)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    pending_cmd_t *slot = find_or_alloc_slot(addr);
    if (slot != NULL) {
        slot->short_addr = addr;
        slot->endpoint = ep;
        slot->type = CMD_RETRY_COLOR;
        slot->retries = CMD_RETRY_MAX_ATTEMPTS;
        slot->timestamp = xTaskGetTickCount();
        slot->active = true;
        slot->params.color.x = x;
        slot->params.color.y = y;
        slot->params.color.transition = trans;
        ESP_LOGD(TAG, "Stored color for 0x%04X: x=%d y=%d", addr, x, y);
    }

    xSemaphoreGive(s_retry_mutex);
}

void cmd_retry_store_tuya_dp(uint16_t addr, uint8_t ep, uint8_t dp_id, uint8_t dp_type,
                              const uint8_t *data, uint8_t data_len)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    pending_cmd_t *slot = find_or_alloc_slot(addr);
    if (slot != NULL) {
        slot->short_addr = addr;
        slot->endpoint = ep;
        slot->type = CMD_RETRY_TUYA_DP;
        slot->retries = CMD_RETRY_MAX_ATTEMPTS;
        slot->timestamp = xTaskGetTickCount();
        slot->active = true;
        slot->params.tuya_dp.dp_id = dp_id;
        slot->params.tuya_dp.dp_type = dp_type;
        uint8_t copy_len = (data_len > CMD_RETRY_TUYA_DATA_MAX) ? CMD_RETRY_TUYA_DATA_MAX : data_len;
        if (data != NULL && copy_len > 0) {
            memcpy(slot->params.tuya_dp.data, data, copy_len);
        }
        slot->params.tuya_dp.data_len = copy_len;
        ESP_LOGD(TAG, "Stored tuya_dp for 0x%04X: dp=%d", addr, dp_id);
    }

    xSemaphoreGive(s_retry_mutex);
}

void cmd_retry_confirm(uint16_t short_addr)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    for (int i = 0; i < CMD_RETRY_MAX_PENDING; i++) {
        pending_cmd_t *slot = &s_pending_cmds[i];
        if (slot->active && slot->short_addr == short_addr) {
            ESP_LOGD(TAG, "Confirmed 0x%04X (type=%d)", short_addr, slot->type);
            slot->active = false;
            break;
        }
    }

    xSemaphoreGive(s_retry_mutex);
}

void cmd_retry_on_unavailable(uint16_t short_addr)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    pending_cmd_t entry_copy;
    bool found = false;

    for (int i = 0; i < CMD_RETRY_MAX_PENDING; i++) {
        pending_cmd_t *slot = &s_pending_cmds[i];
        if (slot->active && slot->short_addr == short_addr) {
            if (slot->retries > 0) {
                slot->retries--;
                /* Copy entry before releasing mutex (command send may take time) */
                memcpy(&entry_copy, slot, sizeof(pending_cmd_t));
                found = true;
            }
            /* Clear regardless — single retry, then done */
            slot->active = false;
            break;
        }
    }

    xSemaphoreGive(s_retry_mutex);

    /* Execute retry outside mutex to avoid holding lock during Zigbee API calls */
    if (found) {
        execute_retry(&entry_copy, "unavailable");
    }
}

void cmd_retry_check_timeouts(void)
{
    if (s_pending_cmds == NULL || s_retry_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_retry_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    uint32_t now = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(CMD_RETRY_TIMEOUT_MS);

    /* Collect entries to retry (max all slots) */
    pending_cmd_t retry_list[CMD_RETRY_MAX_PENDING];
    int retry_count = 0;

    for (int i = 0; i < CMD_RETRY_MAX_PENDING; i++) {
        pending_cmd_t *slot = &s_pending_cmds[i];
        if (!slot->active) {
            continue;
        }

        uint32_t elapsed = now - slot->timestamp;
        if (elapsed >= timeout_ticks) {
            if (slot->retries > 0) {
                slot->retries--;
                memcpy(&retry_list[retry_count], slot, sizeof(pending_cmd_t));
                retry_count++;
            }
            /* Clear after timeout — no further retries */
            slot->active = false;
        }
    }

    xSemaphoreGive(s_retry_mutex);

    /* Execute retries outside mutex */
    for (int i = 0; i < retry_count; i++) {
        execute_retry(&retry_list[i], "timeout");
    }
}
