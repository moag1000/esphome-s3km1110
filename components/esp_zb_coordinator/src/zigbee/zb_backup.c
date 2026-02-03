/**
 * @file zb_backup.c
 * @brief Zigbee Network Backup and Restore Implementation
 *
 * Implements comprehensive network backup and restore functionality for
 * the ESP32-C5 Zigbee2MQTT Gateway including coordinator state, device
 * database, groups, and bindings.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_backup.h"
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "zb_groups.h"
#include "zb_binding.h"
#include "zb_network.h"
#include "compat_stubs.h"
#include "json_utils.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <inttypes.h>

static const char *TAG = "ZB_BACKUP";

/* Backup magic number: "ZBK1" */
#define ZB_BACKUP_MAGIC 0x5A424B31

/* Module state */
static bool s_initialized = false;
static SemaphoreHandle_t s_backup_mutex = NULL;
static zb_backup_event_cb_t s_event_callback = NULL;

/* MQTT response topics */
#define RESPONSE_TOPIC_BACKUP       "zigbee2mqtt/bridge/response/backup"
#define RESPONSE_TOPIC_BACKUP_STATE "zigbee2mqtt/bridge/response/backup/state"
#define RESPONSE_TOPIC_BACKUP_DEVICES "zigbee2mqtt/bridge/response/backup/devices"
#define RESPONSE_TOPIC_RESTORE      "zigbee2mqtt/bridge/response/restore"
#define RESPONSE_TOPIC_BACKUP_LIST  "zigbee2mqtt/bridge/response/backup/list"

/* NVS keys for backup metadata */
#define NVS_KEY_BACKUP_COUNT        "backup_cnt"
#define NVS_KEY_LATEST_BACKUP       "latest_bkp"

/* Forward declarations */
static esp_err_t mount_spiffs(void);
static esp_err_t unmount_spiffs(void);
static esp_err_t collect_coordinator_state(zb_backup_coordinator_t *coord);
static esp_err_t collect_devices(zb_backup_device_t *devices, uint16_t *count);
static esp_err_t collect_groups(zb_backup_group_t *groups, uint16_t *count);
static esp_err_t collect_bindings(zb_backup_binding_t *bindings, uint16_t *count);
static esp_err_t restore_coordinator_state(const zb_backup_coordinator_t *coord);
static esp_err_t restore_devices(const zb_backup_device_t *devices, uint16_t count,
                                  zb_backup_restore_result_t *result);
static esp_err_t restore_groups(const zb_backup_group_t *groups, uint16_t count,
                                 zb_backup_restore_result_t *result);
static esp_err_t restore_bindings(const zb_backup_binding_t *bindings, uint16_t count,
                                   zb_backup_restore_result_t *result);
static void cleanup_old_backups(void);
static esp_err_t generate_backup_filename(char *filename, size_t len);
static cJSON* coordinator_to_json(const zb_backup_coordinator_t *coord);
static cJSON* devices_to_json(const zb_backup_device_t *devices, uint16_t count);
static cJSON* groups_to_json(const zb_backup_group_t *groups, uint16_t count);
static cJSON* bindings_to_json(const zb_backup_binding_t *bindings, uint16_t count);

esp_err_t zb_backup_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Backup module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee backup management...");

    /* Create mutex */
    s_backup_mutex = xSemaphoreCreateMutex();
    if (s_backup_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create backup mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Mount SPIFFS for backup storage */
    esp_err_t ret = mount_spiffs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS mount failed: %s (backups will use NVS fallback)",
                 esp_err_to_name(ret));
        /* Continue anyway - can still do backups via MQTT without file storage */
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Backup management initialized");
    return ESP_OK;
}

esp_err_t zb_backup_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing backup management...");

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    unmount_spiffs();

    s_initialized = false;
    s_event_callback = NULL;

    xSemaphoreGive(s_backup_mutex);
    vSemaphoreDelete(s_backup_mutex);
    s_backup_mutex = NULL;

    ESP_LOGI(TAG, "Backup management deinitialized");
    return ESP_OK;
}

esp_err_t zb_backup_create(zb_backup_type_t backup_type, zb_backup_t *backup)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Backup module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Creating backup (type: %d)...", backup_type);

    /* Allocate backup structure if not provided */
    zb_backup_t *bkp = backup;
    bool allocated = false;
    if (bkp == NULL) {
        bkp = (zb_backup_t *)calloc(1, sizeof(zb_backup_t));
        if (bkp == NULL) {
            ESP_LOGE(TAG, "Failed to allocate backup structure");
            return ESP_ERR_NO_MEM;
        }
        allocated = true;
    }

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_OK;

    /* Initialize backup header */
    memset(bkp, 0, sizeof(zb_backup_t));
    bkp->magic = ZB_BACKUP_MAGIC;
    bkp->version = ZB_BACKUP_VERSION;
    bkp->timestamp = (uint32_t)time(NULL);
    bkp->backup_type = backup_type;

    /* Collect coordinator state */
    if (backup_type == ZB_BACKUP_TYPE_FULL || backup_type == ZB_BACKUP_TYPE_STATE) {
        ret = collect_coordinator_state(&bkp->coordinator);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to collect coordinator state: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        ESP_LOGI(TAG, "Collected coordinator state");
    }

    /* Collect device database */
    if (backup_type == ZB_BACKUP_TYPE_FULL || backup_type == ZB_BACKUP_TYPE_DEVICES) {
        ret = collect_devices(bkp->devices, &bkp->device_count);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to collect devices: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        ESP_LOGI(TAG, "Collected %d devices", bkp->device_count);
    }

    /* Collect groups */
    if (backup_type == ZB_BACKUP_TYPE_FULL || backup_type == ZB_BACKUP_TYPE_GROUPS) {
        ret = collect_groups(bkp->groups, &bkp->group_count);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to collect groups: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        ESP_LOGI(TAG, "Collected %d groups", bkp->group_count);
    }

    /* Collect bindings */
    if (backup_type == ZB_BACKUP_TYPE_FULL || backup_type == ZB_BACKUP_TYPE_BINDINGS) {
        ret = collect_bindings(bkp->bindings, &bkp->binding_count);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to collect bindings: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        ESP_LOGI(TAG, "Collected %d bindings", bkp->binding_count);
    }

    /* Calculate checksum (exclude checksum field itself) */
    bkp->checksum = 0;
    bkp->checksum = zb_backup_calc_checksum(bkp, sizeof(zb_backup_t));

    ESP_LOGI(TAG, "Backup created: %d devices, %d groups, %d bindings",
             bkp->device_count, bkp->group_count, bkp->binding_count);

cleanup:
    xSemaphoreGive(s_backup_mutex);

    if (ret != ESP_OK && allocated) {
        free(bkp);
    }

    return ret;
}

esp_err_t zb_backup_save(const zb_backup_t *backup, char *filename, size_t filename_len)
{
    if (!s_initialized || backup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving backup to SPIFFS...");

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    /* Generate filename */
    char filepath[128];
    esp_err_t ret = generate_backup_filename(filepath, sizeof(filepath));
    if (ret != ESP_OK) {
        xSemaphoreGive(s_backup_mutex);
        return ret;
    }

    /* Open file for writing */
    FILE *file = fopen(filepath, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        xSemaphoreGive(s_backup_mutex);
        return ESP_FAIL;
    }

    /* Write backup data */
    size_t written = fwrite(backup, 1, sizeof(zb_backup_t), file);
    fclose(file);

    if (written != sizeof(zb_backup_t)) {
        ESP_LOGE(TAG, "Failed to write complete backup (%zu of %zu bytes)",
                 written, sizeof(zb_backup_t));
        remove(filepath);
        xSemaphoreGive(s_backup_mutex);
        return ESP_FAIL;
    }

    /* Copy filename to output if provided */
    if (filename != NULL && filename_len > 0) {
        /* Extract just the filename from path */
        const char *basename = strrchr(filepath, '/');
        if (basename != NULL) {
            basename++; /* Skip the slash */
        } else {
            basename = filepath;
        }
        strncpy(filename, basename, filename_len - 1);
        filename[filename_len - 1] = '\0';
    }

    /* Update NVS with latest backup info */
    nvs_handle_t nvs_handle;
    ret = nvs_open(ZB_BACKUP_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_set_str(nvs_handle, NVS_KEY_LATEST_BACKUP, filepath);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    /* Cleanup old backups to maintain maximum count */
    cleanup_old_backups();

    xSemaphoreGive(s_backup_mutex);

    ESP_LOGI(TAG, "Backup saved: %s (%zu bytes)", filepath, sizeof(zb_backup_t));
    return ESP_OK;
}

esp_err_t zb_backup_load(const char *filename, zb_backup_t *backup)
{
    if (!s_initialized || backup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char filepath[128];

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    /* If no filename provided, get the latest backup */
    if (filename == NULL || strlen(filename) == 0) {
        nvs_handle_t nvs_handle;
        esp_err_t ret = nvs_open(ZB_BACKUP_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
        if (ret != ESP_OK) {
            xSemaphoreGive(s_backup_mutex);
            ESP_LOGE(TAG, "No backups found");
            return ESP_ERR_NOT_FOUND;
        }

        size_t path_len = sizeof(filepath);
        ret = nvs_get_str(nvs_handle, NVS_KEY_LATEST_BACKUP, filepath, &path_len);
        nvs_close(nvs_handle);

        if (ret != ESP_OK) {
            xSemaphoreGive(s_backup_mutex);
            ESP_LOGE(TAG, "No latest backup found");
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        /* Build full path from filename */
        if (filename[0] == '/') {
            strncpy(filepath, filename, sizeof(filepath) - 1);
        } else {
            snprintf(filepath, sizeof(filepath), "%s/%s", ZB_BACKUP_FILE_PATH, filename);
        }
    }

    ESP_LOGI(TAG, "Loading backup from: %s", filepath);

    /* Open file for reading */
    FILE *file = fopen(filepath, "rb");
    if (file == NULL) {
        xSemaphoreGive(s_backup_mutex);
        ESP_LOGE(TAG, "Failed to open backup file: %s", filepath);
        return ESP_ERR_NOT_FOUND;
    }

    /* Read backup data */
    size_t read = fread(backup, 1, sizeof(zb_backup_t), file);
    fclose(file);

    if (read != sizeof(zb_backup_t)) {
        xSemaphoreGive(s_backup_mutex);
        ESP_LOGE(TAG, "Incomplete backup file (%zu of %zu bytes)", read, sizeof(zb_backup_t));
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreGive(s_backup_mutex);

    ESP_LOGI(TAG, "Backup loaded: %d devices, %d groups, %d bindings",
             backup->device_count, backup->group_count, backup->binding_count);
    return ESP_OK;
}

esp_err_t zb_backup_validate(const zb_backup_t *backup)
{
    if (backup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check magic number */
    if (backup->magic != ZB_BACKUP_MAGIC) {
        ESP_LOGE(TAG, "Invalid backup magic: 0x%08" PRIX32 " (expected 0x%08X)",
                 backup->magic, ZB_BACKUP_MAGIC);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check version */
    if (backup->version > ZB_BACKUP_VERSION) {
        ESP_LOGE(TAG, "Backup version too new: %lu (max supported: %d)",
                 (unsigned long)backup->version, ZB_BACKUP_VERSION);
        return ESP_ERR_INVALID_VERSION;
    }

    /* Verify checksum */
    uint32_t stored_checksum = backup->checksum;
    zb_backup_t temp_backup;
    memcpy(&temp_backup, backup, sizeof(zb_backup_t));
    temp_backup.checksum = 0;
    uint32_t calc_checksum = zb_backup_calc_checksum(&temp_backup, sizeof(zb_backup_t));

    if (calc_checksum != stored_checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: 0x%08" PRIX32 " (expected 0x%08" PRIX32 ")",
                 calc_checksum, stored_checksum);
        return ESP_ERR_INVALID_CRC;
    }

    /* Validate counts */
    if (backup->device_count > ZB_BACKUP_MAX_DEVICES) {
        ESP_LOGE(TAG, "Invalid device count: %d (max %d)",
                 backup->device_count, ZB_BACKUP_MAX_DEVICES);
        return ESP_ERR_INVALID_ARG;
    }

    if (backup->group_count > ZB_BACKUP_MAX_GROUPS) {
        ESP_LOGE(TAG, "Invalid group count: %d (max %d)",
                 backup->group_count, ZB_BACKUP_MAX_GROUPS);
        return ESP_ERR_INVALID_ARG;
    }

    if (backup->binding_count > ZB_BACKUP_MAX_BINDINGS) {
        ESP_LOGE(TAG, "Invalid binding count: %d (max %d)",
                 backup->binding_count, ZB_BACKUP_MAX_BINDINGS);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Backup validation passed (version %lu, timestamp %lu)",
             (unsigned long)backup->version, (unsigned long)backup->timestamp);
    return ESP_OK;
}

esp_err_t zb_backup_restore(const zb_backup_t *backup, zb_backup_restore_result_t *result)
{
    if (!s_initialized || backup == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize result */
    zb_backup_restore_result_t local_result = {0};

    /* Validate backup first */
    esp_err_t ret = zb_backup_validate(backup);
    if (ret != ESP_OK) {
        local_result.success = false;
        local_result.status = ZB_BACKUP_STATUS_INVALID;
        snprintf(local_result.warnings, sizeof(local_result.warnings),
                 "Backup validation failed");
        if (result != NULL) {
            *result = local_result;
        }
        return ret;
    }

    /* Check if network is running - should be stopped for restore */
    if (zb_coordinator_is_running()) {
        ESP_LOGW(TAG, "Network is running - proceeding with restore (may cause issues)");
        local_result.warning_count++;
        strncat(local_result.warnings, "Network was running during restore",
                sizeof(local_result.warnings) - strlen(local_result.warnings) - 1);
    }

    ESP_LOGI(TAG, "Starting network restore...");

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    /* Restore coordinator state */
    if (backup->backup_type == ZB_BACKUP_TYPE_FULL ||
        backup->backup_type == ZB_BACKUP_TYPE_STATE) {
        ret = restore_coordinator_state(&backup->coordinator);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to restore coordinator state: %s", esp_err_to_name(ret));
            local_result.warning_count++;
        }
    }

    /* Restore devices */
    if (backup->backup_type == ZB_BACKUP_TYPE_FULL ||
        backup->backup_type == ZB_BACKUP_TYPE_DEVICES) {
        ret = restore_devices(backup->devices, backup->device_count, &local_result);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Some devices failed to restore");
        }
    }

    /* Restore groups */
    if (backup->backup_type == ZB_BACKUP_TYPE_FULL ||
        backup->backup_type == ZB_BACKUP_TYPE_GROUPS) {
        ret = restore_groups(backup->groups, backup->group_count, &local_result);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Some groups failed to restore");
        }
    }

    /* Restore bindings */
    if (backup->backup_type == ZB_BACKUP_TYPE_FULL ||
        backup->backup_type == ZB_BACKUP_TYPE_BINDINGS) {
        ret = restore_bindings(backup->bindings, backup->binding_count, &local_result);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Some bindings failed to restore");
        }
    }

    xSemaphoreGive(s_backup_mutex);

    local_result.success = (local_result.restored_devices > 0 ||
                            local_result.restored_groups > 0 ||
                            local_result.restored_bindings > 0);
    local_result.status = local_result.success ? ZB_BACKUP_STATUS_SUCCESS :
                          ZB_BACKUP_STATUS_VALIDATION_FAILED;

    if (result != NULL) {
        *result = local_result;
    }

    ESP_LOGI(TAG, "Restore completed: %d devices, %d groups, %d bindings, %d warnings",
             local_result.restored_devices, local_result.restored_groups,
             local_result.restored_bindings, local_result.warning_count);

    return local_result.success ? ESP_OK : ESP_FAIL;
}

esp_err_t zb_backup_restore_from_file(const char *filename, zb_backup_restore_result_t *result)
{
    /* Allocate backup structure */
    zb_backup_t *backup = (zb_backup_t *)malloc(sizeof(zb_backup_t));
    if (backup == NULL) {
        ESP_LOGE(TAG, "Failed to allocate backup structure for restore");
        return ESP_ERR_NO_MEM;
    }

    /* Load backup */
    esp_err_t ret = zb_backup_load(filename, backup);
    if (ret != ESP_OK) {
        free(backup);
        return ret;
    }

    /* Restore from backup */
    ret = zb_backup_restore(backup, result);

    free(backup);
    return ret;
}

esp_err_t zb_backup_delete(const char *filename)
{
    if (!s_initialized || filename == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char filepath[128];
    if (filename[0] == '/') {
        strncpy(filepath, filename, sizeof(filepath) - 1);
    } else {
        snprintf(filepath, sizeof(filepath), "%s/%s", ZB_BACKUP_FILE_PATH, filename);
    }

    ESP_LOGI(TAG, "Deleting backup: %s", filepath);

    if (remove(filepath) != 0) {
        ESP_LOGE(TAG, "Failed to delete backup file");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t zb_backup_list(zb_backup_list_t *list)
{
    if (!s_initialized || list == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(list, 0, sizeof(zb_backup_list_t));

    xSemaphoreTake(s_backup_mutex, portMAX_DELAY);

    DIR *dir = opendir(ZB_BACKUP_FILE_PATH);
    if (dir == NULL) {
        ESP_LOGW(TAG, "Backup directory not found");
        xSemaphoreGive(s_backup_mutex);
        return ESP_OK; /* Not an error, just no backups */
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && list->count < ZB_BACKUP_MAX_FILES) {
        /* Check for backup file extension */
        const char *ext = strrchr(entry->d_name, '.');
        if (ext == NULL || strcmp(ext, ZB_BACKUP_FILE_EXT) != 0) {
            continue;
        }

        /* Build full path */
        char filepath[512];
        snprintf(filepath, sizeof(filepath), "%s/%s", ZB_BACKUP_FILE_PATH, entry->d_name);

        /* Get file info */
        struct stat file_stat;
        if (stat(filepath, &file_stat) != 0) {
            continue;
        }

        /* Read backup header to get metadata */
        FILE *file = fopen(filepath, "rb");
        if (file == NULL) {
            continue;
        }

        zb_backup_t backup_header;
        size_t read = fread(&backup_header, 1, sizeof(zb_backup_t), file);
        fclose(file);

        if (read != sizeof(zb_backup_t) || backup_header.magic != ZB_BACKUP_MAGIC) {
            continue;
        }

        /* Add to list */
        zb_backup_info_t *info = &list->backups[list->count];
        info->timestamp = backup_header.timestamp;
        info->version = backup_header.version;
        info->backup_type = backup_header.backup_type;
        info->device_count = backup_header.device_count;
        info->group_count = backup_header.group_count;
        info->binding_count = backup_header.binding_count;
        info->file_size = (uint32_t)file_stat.st_size;
        strncpy(info->filename, entry->d_name, sizeof(info->filename) - 1);

        list->count++;
    }

    closedir(dir);
    xSemaphoreGive(s_backup_mutex);

    ESP_LOGI(TAG, "Found %d backups", list->count);
    return ESP_OK;
}

esp_err_t zb_backup_get_latest(zb_backup_info_t *info)
{
    if (!s_initialized || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    zb_backup_list_t list;
    esp_err_t ret = zb_backup_list(&list);
    if (ret != ESP_OK) {
        return ret;
    }

    if (list.count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Find most recent by timestamp */
    uint32_t latest_timestamp = 0;
    int latest_idx = 0;

    for (int i = 0; i < list.count; i++) {
        if (list.backups[i].timestamp > latest_timestamp) {
            latest_timestamp = list.backups[i].timestamp;
            latest_idx = i;
        }
    }

    *info = list.backups[latest_idx];
    return ESP_OK;
}

esp_err_t zb_backup_process_mqtt_backup(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized || topic == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing backup request: %s", topic);

    /* Parse transaction ID from payload if present */
    const char *transaction_id = NULL;
    cJSON *json = NULL;
    if (payload != NULL && len > 0) {
        char *payload_str = (char *)malloc(len + 1);
        if (payload_str != NULL) {
            memcpy(payload_str, payload, len);
            payload_str[len] = '\0';
            json = cJSON_Parse(payload_str);
            if (json != NULL) {
                transaction_id = bridge_request_get_transaction(json);
            }
            free(payload_str);
        }
    }

    /* Determine backup type from topic */
    zb_backup_type_t backup_type = ZB_BACKUP_TYPE_FULL;
    const char *response_topic = RESPONSE_TOPIC_BACKUP;

    if (strstr(topic, "/backup/state") != NULL) {
        backup_type = ZB_BACKUP_TYPE_STATE;
        response_topic = RESPONSE_TOPIC_BACKUP_STATE;
    } else if (strstr(topic, "/backup/devices") != NULL) {
        backup_type = ZB_BACKUP_TYPE_DEVICES;
        response_topic = RESPONSE_TOPIC_BACKUP_DEVICES;
    }

    /* Create backup */
    zb_backup_t *backup = (zb_backup_t *)malloc(sizeof(zb_backup_t));
    if (backup == NULL) {
        bridge_response_publish_error(response_topic,
                                     "Failed to allocate backup memory",
                                     transaction_id);
        if (json != NULL) cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = zb_backup_create(backup_type, backup);
    if (ret != ESP_OK) {
        bridge_response_publish_error(response_topic,
                                     "Failed to create backup",
                                     transaction_id);
        free(backup);
        if (json != NULL) cJSON_Delete(json);
        return ret;
    }

    /* Save backup to SPIFFS */
    char filename[64] = {0};
    ret = zb_backup_save(backup, filename, sizeof(filename));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save backup to file (will publish via MQTT only)");
        filename[0] = '\0';
    }

    /* Publish response */
    ret = zb_backup_publish_response(backup, ZB_BACKUP_STATUS_SUCCESS, transaction_id);

    free(backup);
    if (json != NULL) cJSON_Delete(json);

    return ret;
}

esp_err_t zb_backup_process_mqtt_restore(const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Processing restore request");

    /* Parse payload */
    const char *transaction_id = NULL;
    const char *filename = NULL;
    cJSON *json = NULL;

    if (payload != NULL && len > 0) {
        char *payload_str = (char *)malloc(len + 1);
        if (payload_str != NULL) {
            memcpy(payload_str, payload, len);
            payload_str[len] = '\0';
            json = cJSON_Parse(payload_str);
            if (json != NULL) {
                transaction_id = bridge_request_get_transaction(json);
                cJSON *file_item = cJSON_GetObjectItem(json, "filename");
                if (file_item != NULL && cJSON_IsString(file_item)) {
                    filename = file_item->valuestring;
                }
            }
            free(payload_str);
        }
    }

    /* Perform restore */
    zb_backup_restore_result_t result = {0};
    esp_err_t ret = zb_backup_restore_from_file(filename, &result);

    /* Publish response */
    zb_backup_publish_restore_response(&result, transaction_id);

    if (json != NULL) cJSON_Delete(json);

    return ret;
}

esp_err_t zb_backup_process_mqtt_list(const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Processing backup list request");

    /* Parse transaction ID */
    const char *transaction_id = NULL;
    cJSON *json = NULL;

    if (payload != NULL && len > 0) {
        char *payload_str = (char *)malloc(len + 1);
        if (payload_str != NULL) {
            memcpy(payload_str, payload, len);
            payload_str[len] = '\0';
            json = cJSON_Parse(payload_str);
            if (json != NULL) {
                transaction_id = bridge_request_get_transaction(json);
            }
            free(payload_str);
        }
    }

    /* Get backup list */
    zb_backup_list_t list;
    esp_err_t ret = zb_backup_list(&list);

    /* Publish response */
    zb_backup_publish_list_response(&list, transaction_id);

    if (json != NULL) cJSON_Delete(json);

    return ret;
}

esp_err_t zb_backup_publish_response(const zb_backup_t *backup,
                                      zb_backup_status_t status,
                                      const char *transaction_id)
{
    if (backup == NULL && status == ZB_BACKUP_STATUS_SUCCESS) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *response = cJSON_CreateObject();
    if (response == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (status == ZB_BACKUP_STATUS_SUCCESS && backup != NULL) {
        /* Convert backup to JSON */
        cJSON *data = zb_backup_to_json(backup);
        if (data != NULL) {
            cJSON_AddItemToObject(response, "data", data);
        }
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", zb_backup_status_str(status));
    }

    if (transaction_id != NULL) {
        cJSON_AddStringToObject(response, "transaction", transaction_id);
    }

    char *json_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_client_publish(RESPONSE_TOPIC_BACKUP, json_str, 0, 1, false);
    free(json_str);

    return ret;
}

esp_err_t zb_backup_publish_restore_response(const zb_backup_restore_result_t *result,
                                              const char *transaction_id)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *response = cJSON_CreateObject();
    if (response == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON *data = cJSON_CreateObject();
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object for restore response data");
        cJSON_Delete(response);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddNumberToObject(data, "restored_devices", result->restored_devices);
    cJSON_AddNumberToObject(data, "restored_groups", result->restored_groups);
    cJSON_AddNumberToObject(data, "restored_bindings", result->restored_bindings);

    cJSON *warnings = cJSON_CreateArray();
    if (warnings == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON array for warnings");
        cJSON_Delete(data);
        cJSON_Delete(response);
        return ESP_ERR_NO_MEM;
    }
    if (result->warning_count > 0 && strlen(result->warnings) > 0) {
        cJSON_AddItemToArray(warnings, cJSON_CreateString(result->warnings));
    }
    cJSON_AddItemToObject(data, "warnings", warnings);

    cJSON_AddItemToObject(response, "data", data);
    cJSON_AddStringToObject(response, "status", result->success ? "ok" : "error");

    if (!result->success) {
        cJSON_AddStringToObject(response, "error", zb_backup_status_str(result->status));
    }

    if (transaction_id != NULL) {
        cJSON_AddStringToObject(response, "transaction", transaction_id);
    }

    char *json_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_client_publish(RESPONSE_TOPIC_RESTORE, json_str, 0, 1, false);
    free(json_str);

    return ret;
}

esp_err_t zb_backup_publish_list_response(const zb_backup_list_t *list,
                                           const char *transaction_id)
{
    if (list == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *response = cJSON_CreateObject();
    if (response == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON *data = cJSON_CreateArray();
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON array for backup list");
        cJSON_Delete(response);
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < list->count; i++) {
        cJSON *backup_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(backup_obj, "filename", list->backups[i].filename);
        cJSON_AddNumberToObject(backup_obj, "timestamp", list->backups[i].timestamp);
        cJSON_AddNumberToObject(backup_obj, "version", list->backups[i].version);
        cJSON_AddNumberToObject(backup_obj, "device_count", list->backups[i].device_count);
        cJSON_AddNumberToObject(backup_obj, "group_count", list->backups[i].group_count);
        cJSON_AddNumberToObject(backup_obj, "binding_count", list->backups[i].binding_count);
        cJSON_AddNumberToObject(backup_obj, "file_size", list->backups[i].file_size);
        cJSON_AddItemToArray(data, backup_obj);
    }

    cJSON_AddItemToObject(response, "data", data);
    cJSON_AddStringToObject(response, "status", "ok");

    if (transaction_id != NULL) {
        cJSON_AddStringToObject(response, "transaction", transaction_id);
    }

    char *json_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_client_publish(RESPONSE_TOPIC_BACKUP_LIST, json_str, 0, 1, false);
    free(json_str);

    return ret;
}

cJSON* zb_backup_to_json(const zb_backup_t *backup)
{
    if (backup == NULL) {
        return NULL;
    }

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return NULL;
    }

    /* Version info */
    cJSON_AddStringToObject(json, "version", "1.0");
    cJSON_AddNumberToObject(json, "timestamp", backup->timestamp);

    /* Coordinator */
    cJSON *coord = coordinator_to_json(&backup->coordinator);
    if (coord != NULL) {
        cJSON_AddItemToObject(json, "coordinator", coord);
    }

    /* Devices */
    cJSON *devices = devices_to_json(backup->devices, backup->device_count);
    if (devices != NULL) {
        cJSON_AddItemToObject(json, "devices", devices);
    }

    /* Groups */
    cJSON *groups = groups_to_json(backup->groups, backup->group_count);
    if (groups != NULL) {
        cJSON_AddItemToObject(json, "groups", groups);
    }

    /* Bindings */
    cJSON *bindings = bindings_to_json(backup->bindings, backup->binding_count);
    if (bindings != NULL) {
        cJSON_AddItemToObject(json, "bindings", bindings);
    }

    return json;
}

esp_err_t zb_backup_register_callback(zb_backup_event_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

const char* zb_backup_status_str(zb_backup_status_t status)
{
    switch (status) {
        case ZB_BACKUP_STATUS_SUCCESS:          return "success";
        case ZB_BACKUP_STATUS_NOT_FOUND:        return "backup_not_found";
        case ZB_BACKUP_STATUS_INVALID:          return "invalid_backup";
        case ZB_BACKUP_STATUS_VERSION_MISMATCH: return "version_mismatch";
        case ZB_BACKUP_STATUS_STORAGE_FULL:     return "storage_full";
        case ZB_BACKUP_STATUS_IO_ERROR:         return "io_error";
        case ZB_BACKUP_STATUS_VALIDATION_FAILED: return "validation_failed";
        case ZB_BACKUP_STATUS_NETWORK_ACTIVE:   return "network_active";
        case ZB_BACKUP_STATUS_ERROR:            return "error";
        default:                                return "unknown";
    }
}

uint32_t zb_backup_calc_checksum(const void *data, size_t len)
{
    /* Simple CRC32-like checksum */
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < len; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF;
}

esp_err_t zb_backup_test(void)
{
    ESP_LOGI(TAG, "Running backup self-test...");

    if (!s_initialized) {
        ESP_LOGE(TAG, "Backup module not initialized");
        return ESP_FAIL;
    }

    /* Test 1: Create backup */
    zb_backup_t *backup = (zb_backup_t *)malloc(sizeof(zb_backup_t));
    if (backup == NULL) {
        ESP_LOGE(TAG, "Test 1 FAILED: Memory allocation");
        return ESP_FAIL;
    }

    esp_err_t ret = zb_backup_create(ZB_BACKUP_TYPE_FULL, backup);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test 1 FAILED: Create backup");
        free(backup);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 1 PASSED: Created backup");

    /* Test 2: Validate backup */
    ret = zb_backup_validate(backup);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Test 2 FAILED: Validate backup");
        free(backup);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 2 PASSED: Validated backup");

    /* Test 3: Convert to JSON */
    cJSON *json = zb_backup_to_json(backup);
    if (json == NULL) {
        ESP_LOGE(TAG, "Test 3 FAILED: Convert to JSON");
        free(backup);
        return ESP_FAIL;
    }
    cJSON_Delete(json);
    ESP_LOGI(TAG, "Test 3 PASSED: Converted to JSON");

    /* Test 4: Checksum calculation */
    uint32_t checksum = zb_backup_calc_checksum("test", 4);
    if (checksum == 0) {
        ESP_LOGE(TAG, "Test 4 FAILED: Checksum calculation");
        free(backup);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 4 PASSED: Checksum = 0x%08" PRIX32, checksum);

    free(backup);
    ESP_LOGI(TAG, "Backup self-test PASSED (all 4 tests)");
    return ESP_OK;
}

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

static esp_err_t mount_spiffs(void)
{
    ESP_LOGI(TAG, "Mounting SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    /* Create backup directory if it doesn't exist */
    struct stat st;
    if (stat(ZB_BACKUP_FILE_PATH, &st) != 0) {
        ESP_LOGI(TAG, "Creating backup directory: %s", ZB_BACKUP_FILE_PATH);
        mkdir(ZB_BACKUP_FILE_PATH, 0755);
    }

    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS mounted: %zu/%zu bytes used", used, total);

    return ESP_OK;
}

static esp_err_t unmount_spiffs(void)
{
    return esp_vfs_spiffs_unregister(NULL);
}

static esp_err_t collect_coordinator_state(zb_backup_coordinator_t *coord)
{
    if (coord == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(coord, 0, sizeof(zb_backup_coordinator_t));

    /* Get coordinator IEEE address */
    esp_zb_ieee_addr_t ieee_addr;
    esp_read_mac(ieee_addr, ESP_MAC_IEEE802154);
    memcpy(&coord->ieee_addr, ieee_addr, sizeof(uint64_t));

    /* Get network configuration from network manager */
    coord->pan_id = zb_network_get_pan_id_config();
    coord->channel = zb_network_get_channel_config();

    /* Extended PAN ID (same as coordinator IEEE) */
    coord->ext_pan_id = coord->ieee_addr;

    /* Network key - note: we can't read this back from the stack for security */
    /* In a real implementation, this should be stored separately in secure storage */
    memset(coord->network_key, 0, sizeof(coord->network_key));

    /* TC link key - default Zigbee HA key */
    const uint8_t ha_link_key[] = {
        0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C,
        0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39
    };
    memcpy(coord->tc_link_key, ha_link_key, sizeof(coord->tc_link_key));

    /* Other state */
    coord->network_key_seq = 0;
    coord->network_frame_counter = 0;
    coord->permit_join = false;

    ESP_LOGI(TAG, "Coordinator state collected: IEEE=0x%016" PRIX64 ", PAN=0x%04X, Ch=%d",
             coord->ieee_addr, coord->pan_id, coord->channel);

    return ESP_OK;
}

static esp_err_t collect_devices(zb_backup_device_t *devices, uint16_t *count)
{
    if (devices == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;

    /* Iterate devices by index (avoids stack allocation) */
    size_t dev_count = zb_device_get_count();

    for (size_t i = 0; i < dev_count && *count < ZB_BACKUP_MAX_DEVICES; i++) {
        zb_device_t *dev = zb_device_get_by_index(i);
        if (dev == NULL) {
            continue;
        }

        zb_backup_device_t *bkp_dev = &devices[*count];

        /* Copy IEEE address (convert from array to uint64) */
        memcpy(&bkp_dev->ieee_addr, dev->ieee_addr, sizeof(uint64_t));
        bkp_dev->short_addr = dev->short_addr;
        bkp_dev->endpoint = dev->endpoint;
        strncpy(bkp_dev->friendly_name, dev->friendly_name, ZB_BACKUP_FRIENDLY_NAME_LEN - 1);
        strncpy(bkp_dev->model, dev->model, ZB_BACKUP_MODEL_LEN - 1);
        strncpy(bkp_dev->manufacturer, dev->manufacturer, ZB_BACKUP_MANUFACTURER_LEN - 1);
        bkp_dev->device_type = (uint8_t)dev->device_type;
        bkp_dev->cluster_count = dev->cluster_count;

        /* Copy clusters */
        size_t cluster_copy = (dev->cluster_count < ZB_BACKUP_MAX_CLUSTERS) ?
                              dev->cluster_count : ZB_BACKUP_MAX_CLUSTERS;
        memcpy(bkp_dev->clusters, dev->clusters, cluster_copy * sizeof(uint16_t));

        (*count)++;
    }

    return ESP_OK;
}

static esp_err_t collect_groups(zb_backup_group_t *groups, uint16_t *count)
{
    if (groups == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;

    /* Get all groups from groups module */
    zb_group_t grp_list[ZB_GROUPS_MAX_COUNT];
    size_t grp_count = zb_groups_get_all(grp_list, ZB_GROUPS_MAX_COUNT);

    for (size_t i = 0; i < grp_count && *count < ZB_BACKUP_MAX_GROUPS; i++) {
        zb_backup_group_t *bkp_grp = &groups[*count];
        zb_group_t *grp = &grp_list[i];

        if (!grp->active) {
            continue;
        }

        bkp_grp->group_id = grp->group_id;
        strncpy(bkp_grp->name, grp->name, ZB_BACKUP_GROUP_NAME_LEN - 1);
        bkp_grp->member_count = grp->member_count;

        /* Copy members */
        size_t member_copy = (grp->member_count < ZB_BACKUP_GROUP_MAX_MEMBERS) ?
                              grp->member_count : ZB_BACKUP_GROUP_MAX_MEMBERS;
        memcpy(bkp_grp->members, grp->members, member_copy * sizeof(uint64_t));

        (*count)++;
    }

    return ESP_OK;
}

static esp_err_t collect_bindings(zb_backup_binding_t *bindings, uint16_t *count)
{
    if (bindings == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;

    /* Get all bindings from binding module */
    zb_binding_entry_t bind_list[ZB_BINDING_MAX_ENTRIES];
    size_t bind_count = zb_binding_get_all(bind_list, ZB_BINDING_MAX_ENTRIES);

    for (size_t i = 0; i < bind_count && *count < ZB_BACKUP_MAX_BINDINGS; i++) {
        zb_backup_binding_t *bkp_bind = &bindings[*count];
        zb_binding_entry_t *bind = &bind_list[i];

        if (!bind->active) {
            continue;
        }

        bkp_bind->source_ieee = bind->source_ieee;
        bkp_bind->source_endpoint = bind->source_endpoint;
        bkp_bind->cluster_id = bind->cluster_id;
        bkp_bind->dest_ieee = bind->dest_ieee;
        bkp_bind->dest_endpoint = bind->dest_endpoint;

        (*count)++;
    }

    return ESP_OK;
}

static esp_err_t restore_coordinator_state(const zb_backup_coordinator_t *coord)
{
    if (coord == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Restoring coordinator state: PAN=0x%04X, Ch=%d",
             coord->pan_id, coord->channel);

    /* Store network configuration in NVS for next startup */
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("zb_network", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_set_u16(nvs_handle, "pan_id", coord->pan_id);
        nvs_set_u8(nvs_handle, "channel", coord->channel);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    /* Note: Network key and other security parameters would need
     * special handling through the Zigbee stack's secure key management.
     * This is a simplified implementation. */

    return ESP_OK;
}

static esp_err_t restore_devices(const zb_backup_device_t *devices, uint16_t count,
                                  zb_backup_restore_result_t *result)
{
    if (devices == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Restoring %d devices...", count);

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_device_t *dev = &devices[i];

        /* Convert IEEE address back to array format */
        esp_zb_ieee_addr_t ieee_addr;
        memcpy(ieee_addr, &dev->ieee_addr, sizeof(esp_zb_ieee_addr_t));

        /* Add device to registry */
        esp_err_t ret = zb_device_add(ieee_addr, dev->short_addr);
        if (ret == ESP_OK) {
            /* Set device properties */
            zb_device_set_friendly_name(dev->short_addr, dev->friendly_name);
            zb_device_update_info(dev->short_addr, dev->manufacturer, dev->model);

            /* Add clusters */
            for (uint16_t c = 0; c < dev->cluster_count && c < ZB_BACKUP_MAX_CLUSTERS; c++) {
                zb_device_add_cluster(dev->short_addr, dev->clusters[c]);
            }

            /* Determine device type from clusters */
            zb_device_determine_type(dev->short_addr);

            result->restored_devices++;
        } else {
            ESP_LOGW(TAG, "Failed to restore device 0x%04X", dev->short_addr);
        }
    }

    return ESP_OK;
}

static esp_err_t restore_groups(const zb_backup_group_t *groups, uint16_t count,
                                 zb_backup_restore_result_t *result)
{
    if (groups == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Restoring %d groups...", count);

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_group_t *grp = &groups[i];

        /* Create group */
        uint16_t group_id;
        esp_err_t ret = zb_groups_create(grp->name, &group_id);
        if (ret == ESP_OK) {
            /* Add members */
            for (uint8_t m = 0; m < grp->member_count && m < ZB_BACKUP_GROUP_MAX_MEMBERS; m++) {
                zb_groups_add_member(group_id, grp->members[m]);
            }
            result->restored_groups++;
        } else if (ret == ESP_ERR_INVALID_ARG) {
            /* Group might already exist */
            ESP_LOGW(TAG, "Group '%s' may already exist", grp->name);
        } else {
            ESP_LOGW(TAG, "Failed to restore group '%s'", grp->name);
        }
    }

    return ESP_OK;
}

static esp_err_t restore_bindings(const zb_backup_binding_t *bindings, uint16_t count,
                                   zb_backup_restore_result_t *result)
{
    if (bindings == NULL || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Restoring %d bindings...", count);

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_binding_t *bind = &bindings[i];

        esp_err_t ret = zb_binding_create(bind->source_ieee, bind->source_endpoint,
                                          bind->cluster_id,
                                          bind->dest_ieee, bind->dest_endpoint);
        if (ret == ESP_OK) {
            result->restored_bindings++;
        } else if (ret == ESP_ERR_INVALID_ARG) {
            /* Binding might already exist */
            ESP_LOGW(TAG, "Binding may already exist");
        } else {
            ESP_LOGW(TAG, "Failed to restore binding");
        }
    }

    return ESP_OK;
}

static void cleanup_old_backups(void)
{
    zb_backup_list_t list;
    esp_err_t ret = zb_backup_list(&list);
    if (ret != ESP_OK || list.count <= ZB_BACKUP_MAX_FILES) {
        return; /* No cleanup needed */
    }

    ESP_LOGI(TAG, "Cleaning up old backups (have %d, max %d)", list.count, ZB_BACKUP_MAX_FILES);

    /* Sort by timestamp (oldest first) - simple bubble sort */
    for (int i = 0; i < list.count - 1; i++) {
        for (int j = 0; j < list.count - i - 1; j++) {
            if (list.backups[j].timestamp > list.backups[j + 1].timestamp) {
                zb_backup_info_t temp = list.backups[j];
                list.backups[j] = list.backups[j + 1];
                list.backups[j + 1] = temp;
            }
        }
    }

    /* Delete oldest backups */
    int to_delete = list.count - ZB_BACKUP_MAX_FILES;
    for (int i = 0; i < to_delete; i++) {
        ESP_LOGI(TAG, "Deleting old backup: %s", list.backups[i].filename);
        zb_backup_delete(list.backups[i].filename);
    }
}

static esp_err_t generate_backup_filename(char *filename, size_t len)
{
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);

    snprintf(filename, len, "%s/backup_%04d%02d%02d_%02d%02d%02d%s",
             ZB_BACKUP_FILE_PATH,
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             ZB_BACKUP_FILE_EXT);

    return ESP_OK;
}

static cJSON* coordinator_to_json(const zb_backup_coordinator_t *coord)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return NULL;
    }

    char ieee_str[24];
    snprintf(ieee_str, sizeof(ieee_str), "0x%016" PRIx64, coord->ieee_addr);
    cJSON_AddStringToObject(json, "ieee_address", ieee_str);

    char pan_str[8];
    snprintf(pan_str, sizeof(pan_str), "0x%04X", coord->pan_id);
    cJSON_AddStringToObject(json, "pan_id", pan_str);

    char ext_pan_str[24];
    snprintf(ext_pan_str, sizeof(ext_pan_str), "0x%016" PRIx64, coord->ext_pan_id);
    cJSON_AddStringToObject(json, "ext_pan_id", ext_pan_str);

    cJSON_AddNumberToObject(json, "channel", coord->channel);

    /* Network key as hex string (NOTE: redacted for security in real implementation) */
    cJSON_AddStringToObject(json, "network_key", "**REDACTED**");

    return json;
}

static cJSON* devices_to_json(const zb_backup_device_t *devices, uint16_t count)
{
    cJSON *json = cJSON_CreateArray();
    if (json == NULL) {
        return NULL;
    }

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_device_t *dev = &devices[i];
        cJSON *dev_obj = cJSON_CreateObject();

        char ieee_str[24];
        snprintf(ieee_str, sizeof(ieee_str), "0x%016" PRIx64, dev->ieee_addr);
        cJSON_AddStringToObject(dev_obj, "ieee_address", ieee_str);

        cJSON_AddNumberToObject(dev_obj, "network_address", dev->short_addr);
        cJSON_AddStringToObject(dev_obj, "friendly_name", dev->friendly_name);
        cJSON_AddStringToObject(dev_obj, "manufacturer", dev->manufacturer);
        cJSON_AddStringToObject(dev_obj, "model", dev->model);

        cJSON *endpoints = cJSON_CreateArray();
        if (endpoints == NULL) {
            cJSON_Delete(dev_obj);
            cJSON_Delete(json);
            return NULL;
        }
        cJSON *ep_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(ep_obj, "endpoint", dev->endpoint);

        cJSON *clusters = cJSON_CreateArray();
        if (clusters == NULL) {
            cJSON_Delete(ep_obj);
            cJSON_Delete(endpoints);
            cJSON_Delete(dev_obj);
            cJSON_Delete(json);
            return NULL;
        }
        for (uint16_t c = 0; c < dev->cluster_count && c < ZB_BACKUP_MAX_CLUSTERS; c++) {
            cJSON_AddItemToArray(clusters, cJSON_CreateNumber(dev->clusters[c]));
        }
        cJSON_AddItemToObject(ep_obj, "clusters", clusters);
        cJSON_AddItemToArray(endpoints, ep_obj);
        cJSON_AddItemToObject(dev_obj, "endpoints", endpoints);

        cJSON_AddItemToArray(json, dev_obj);
    }

    return json;
}

static cJSON* groups_to_json(const zb_backup_group_t *groups, uint16_t count)
{
    cJSON *json = cJSON_CreateArray();
    if (json == NULL) {
        return NULL;
    }

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_group_t *grp = &groups[i];
        cJSON *grp_obj = cJSON_CreateObject();

        cJSON_AddNumberToObject(grp_obj, "id", grp->group_id);
        cJSON_AddStringToObject(grp_obj, "friendly_name", grp->name);

        cJSON *members = cJSON_CreateArray();
        if (members == NULL) {
            cJSON_Delete(grp_obj);
            cJSON_Delete(json);
            return NULL;
        }
        for (uint8_t m = 0; m < grp->member_count && m < ZB_BACKUP_GROUP_MAX_MEMBERS; m++) {
            char ieee_str[24];
            snprintf(ieee_str, sizeof(ieee_str), "0x%016" PRIx64, grp->members[m]);
            cJSON_AddItemToArray(members, cJSON_CreateString(ieee_str));
        }
        cJSON_AddItemToObject(grp_obj, "members", members);

        cJSON_AddItemToArray(json, grp_obj);
    }

    return json;
}

static cJSON* bindings_to_json(const zb_backup_binding_t *bindings, uint16_t count)
{
    cJSON *json = cJSON_CreateArray();
    if (json == NULL) {
        return NULL;
    }

    for (uint16_t i = 0; i < count; i++) {
        const zb_backup_binding_t *bind = &bindings[i];
        cJSON *bind_obj = cJSON_CreateObject();

        cJSON *source = cJSON_CreateObject();
        char src_ieee[24];
        snprintf(src_ieee, sizeof(src_ieee), "0x%016" PRIx64, bind->source_ieee);
        cJSON_AddStringToObject(source, "ieee_address", src_ieee);
        cJSON_AddNumberToObject(source, "endpoint", bind->source_endpoint);
        cJSON_AddItemToObject(bind_obj, "source", source);

        cJSON *target = cJSON_CreateObject();
        char dst_ieee[24];
        snprintf(dst_ieee, sizeof(dst_ieee), "0x%016" PRIx64, bind->dest_ieee);
        cJSON_AddStringToObject(target, "ieee_address", dst_ieee);
        cJSON_AddNumberToObject(target, "endpoint", bind->dest_endpoint);
        cJSON_AddStringToObject(target, "type", "endpoint");
        cJSON_AddItemToObject(bind_obj, "target", target);

        cJSON *clusters = cJSON_CreateArray();
        if (clusters == NULL) {
            cJSON_Delete(bind_obj);
            cJSON_Delete(json);
            return NULL;
        }
        cJSON_AddItemToArray(clusters, cJSON_CreateString(
            zb_binding_get_cluster_name(bind->cluster_id)));
        cJSON_AddItemToObject(bind_obj, "clusters", clusters);

        cJSON_AddItemToArray(json, bind_obj);
    }

    return json;
}
