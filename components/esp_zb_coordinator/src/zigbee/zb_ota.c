/**
 * @file zb_ota.c
 * @brief Zigbee OTA (Over-The-Air) Update Server Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_ota.h"
#include "zb_constants.h"
#include "gateway_defaults.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "compat_stubs.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

static const char *TAG = "ZB_OTA";

/* ============================================================================
 * Static Variables
 * ============================================================================ */

/** Module initialization flag */
static bool s_initialized = false;

/** Mutex for thread safety */
static SemaphoreHandle_t s_ota_mutex = NULL;

/** Stored OTA images */
static zb_ota_image_info_t s_images[ZB_OTA_MAX_IMAGES];

/** Number of stored images */
static size_t s_image_count = 0;

/** Active OTA transfers */
static zb_ota_transfer_t s_transfers[ZB_OTA_MAX_TRANSFERS];

/** Default block size */
static uint8_t s_block_size = ZB_OTA_DEFAULT_BLOCK_SIZE;

/** Rate limit (max blocks per second) */
static uint8_t s_max_blocks_per_sec = ZB_OTA_MAX_BLOCKS_PER_SEC;

/** Progress callback */
static zb_ota_progress_cb_t s_progress_callback = NULL;

/** OTA statistics */
static zb_ota_stats_t s_stats = {0};

/** SPIFFS mounted flag */
static bool s_spiffs_mounted = false;

/** Image upload buffer for MQTT */
static uint8_t *s_upload_buffer = NULL;
static size_t s_upload_buffer_size = 0;
static size_t s_upload_received = 0;

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static esp_err_t mount_spiffs(void);
static esp_err_t unmount_spiffs(void);
static esp_err_t load_metadata_from_nvs(void);
static esp_err_t save_metadata_to_nvs(void);
static esp_err_t parse_image_file(const char *file_path, zb_ota_image_info_t *info);
static zb_ota_transfer_t* find_transfer(const esp_zb_ieee_addr_t ieee_addr);
static zb_ota_transfer_t* create_transfer(const esp_zb_ieee_addr_t ieee_addr);
static void remove_transfer(const esp_zb_ieee_addr_t ieee_addr);
static esp_err_t send_query_next_image_response(uint16_t short_addr, uint8_t endpoint,
                                                 uint8_t tsn, uint8_t status,
                                                 const zb_ota_image_info_t *image);
static esp_err_t send_image_block_response(uint16_t short_addr, uint8_t endpoint,
                                            uint8_t tsn, zb_ota_transfer_t *transfer,
                                            uint32_t offset, uint8_t size);
static esp_err_t send_upgrade_end_response(uint16_t short_addr, uint8_t endpoint,
                                            uint8_t tsn, uint16_t mfg_code,
                                            uint16_t image_type, uint32_t file_version);
static bool check_rate_limit(zb_ota_transfer_t *transfer);
static void update_transfer_state(zb_ota_transfer_t *transfer, zb_ota_transfer_state_t state);
static uint8_t calculate_progress(const zb_ota_transfer_t *transfer);
static void ieee_addr_to_string(const esp_zb_ieee_addr_t ieee_addr, char *str, size_t len);
static esp_err_t read_image_block(const zb_ota_image_info_t *image, uint32_t offset,
                                   uint8_t *buffer, uint8_t size);

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

esp_err_t zb_ota_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee OTA server...");

    /* Create mutex */
    s_ota_mutex = xSemaphoreCreateMutex();
    if (s_ota_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize arrays */
    memset(s_images, 0, sizeof(s_images));
    memset(s_transfers, 0, sizeof(s_transfers));
    memset(&s_stats, 0, sizeof(s_stats));
    s_image_count = 0;

    /* Mount SPIFFS partition */
    esp_err_t ret = mount_spiffs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS mount failed, OTA storage unavailable: %s",
                 esp_err_to_name(ret));
        /* Continue without SPIFFS - images can still be added via data */
    }

    /* Load metadata from NVS */
    ret = load_metadata_from_nvs();
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to load metadata: %s", esp_err_to_name(ret));
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Zigbee OTA server initialized, %d images available", s_image_count);

    /* Optimize APS fragmentation for OTA transfers (API-011)
     *
     * The APS fragmentation settings control how large messages are split
     * for transmission over the Zigbee network:
     * - Window size: Number of fragments that can be in flight (1-8)
     * - Interframe delay: Minimum delay between consecutive fragments
     *
     * Larger window sizes improve throughput for OTA transfers by allowing
     * multiple fragments to be transmitted before waiting for acknowledgments.
     */
    esp_err_t aps_ret = esp_zb_aps_set_fragment_max_window_size(
        ZB_OTA_APS_FRAGMENT_WINDOW_SIZE);
    if (aps_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set APS fragment window size: %s", esp_err_to_name(aps_ret));
    }

    aps_ret = esp_zb_aps_set_fragment_interframe_delay(
        ZB_OTA_APS_INTERFRAME_DELAY_MS);
    if (aps_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set APS fragment interframe delay: %s", esp_err_to_name(aps_ret));
    }

    ESP_LOGI(TAG, "APS fragmentation configured for OTA: window_size=%d, interframe_delay=%dms",
             ZB_OTA_APS_FRAGMENT_WINDOW_SIZE, ZB_OTA_APS_INTERFRAME_DELAY_MS);

    return ESP_OK;
}

esp_err_t zb_ota_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Zigbee OTA server...");

    /* Abort all transfers */
    zb_ota_abort_all_transfers();

    /* Save metadata */
    save_metadata_to_nvs();

    /* Unmount SPIFFS */
    if (s_spiffs_mounted) {
        unmount_spiffs();
    }

    /* Free upload buffer if allocated */
    if (s_upload_buffer != NULL) {
        free(s_upload_buffer);
        s_upload_buffer = NULL;
        s_upload_buffer_size = 0;
        s_upload_received = 0;
    }

    /* Delete mutex */
    if (s_ota_mutex != NULL) {
        vSemaphoreDelete(s_ota_mutex);
        s_ota_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Zigbee OTA server deinitialized");

    return ESP_OK;
}

esp_err_t zb_ota_add_image(const char *file_path, zb_ota_image_info_t *image_info)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (file_path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Adding OTA image from file: %s", file_path);

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_OK;

    /* Check if max images reached */
    if (s_image_count >= ZB_OTA_MAX_IMAGES) {
        ESP_LOGE(TAG, "Maximum number of images reached (%d)", ZB_OTA_MAX_IMAGES);
        ret = ESP_ERR_NO_MEM;
        goto done;
    }

    /* Check if file exists */
    struct stat st;
    if (stat(file_path, &st) != 0) {
        ESP_LOGE(TAG, "File not found: %s", file_path);
        ret = ESP_ERR_NOT_FOUND;
        goto done;
    }

    /* Check file size */
    if (st.st_size > ZB_OTA_MAX_IMAGE_SIZE) {
        ESP_LOGE(TAG, "Image too large: %ld bytes (max %d)", st.st_size, ZB_OTA_MAX_IMAGE_SIZE);
        ret = ESP_ERR_INVALID_SIZE;
        goto done;
    }

    /* Parse image header */
    zb_ota_image_info_t info = {0};
    ret = parse_image_file(file_path, &info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse image: %s", esp_err_to_name(ret));
        goto done;
    }

    /* Check for duplicate */
    for (size_t i = 0; i < s_image_count; i++) {
        if (s_images[i].valid &&
            s_images[i].manufacturer_code == info.manufacturer_code &&
            s_images[i].image_type == info.image_type &&
            s_images[i].file_version == info.file_version) {
            ESP_LOGW(TAG, "Image already exists, replacing");
            /* Remove old image */
            s_images[i].valid = false;
            break;
        }
    }

    /* Find empty slot */
    int slot = -1;
    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (!s_images[i].valid) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        ESP_LOGE(TAG, "No empty slot available");
        ret = ESP_ERR_NO_MEM;
        goto done;
    }

    /* Copy file path */
    strncpy(info.file_path, file_path, sizeof(info.file_path) - 1);
    info.valid = true;
    info.add_timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS / GW_MS_PER_SECOND;

    /* Store image info */
    memcpy(&s_images[slot], &info, sizeof(zb_ota_image_info_t));
    s_image_count++;

    ESP_LOGI(TAG, "OTA image added: mfg=0x%04X type=0x%04X ver=0x%08lX size=%lu",
             info.manufacturer_code, info.image_type, info.file_version, info.image_size);

    /* Save metadata */
    save_metadata_to_nvs();

    /* Return image info if requested */
    if (image_info != NULL) {
        memcpy(image_info, &info, sizeof(zb_ota_image_info_t));
    }

done:
    xSemaphoreGive(s_ota_mutex);
    return ret;
}

esp_err_t zb_ota_add_image_data(const uint8_t *data, size_t size,
                                 zb_ota_image_info_t *image_info)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (size < ZB_OTA_HEADER_MIN_LEN) {
        ESP_LOGE(TAG, "Data too small for OTA header");
        return ESP_ERR_INVALID_SIZE;
    }

    if (size > ZB_OTA_MAX_IMAGE_SIZE) {
        ESP_LOGE(TAG, "Image too large: %u bytes", size);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Adding OTA image from data (%u bytes)", size);

    /* Parse header */
    zb_ota_file_header_t header;
    esp_err_t ret = zb_ota_parse_header(data, size, &header);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse header");
        return ret;
    }

    /* Generate file name */
    char file_path[64];
    snprintf(file_path, sizeof(file_path), "%s/%04X_%04X_%08lX.ota",
             ZB_OTA_SPIFFS_MOUNT_POINT,
             header.manufacturer_code,
             header.image_type,
             header.file_version);

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    /* Write to SPIFFS */
    if (s_spiffs_mounted) {
        FILE *f = fopen(file_path, "wb");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to create file: %s", file_path);
            xSemaphoreGive(s_ota_mutex);
            return ESP_FAIL;
        }

        size_t written = fwrite(data, 1, size, f);
        fclose(f);

        if (written != size) {
            ESP_LOGE(TAG, "Write incomplete: %u/%u bytes", written, size);
            unlink(file_path);
            xSemaphoreGive(s_ota_mutex);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "OTA image written to: %s", file_path);
    } else {
        ESP_LOGE(TAG, "SPIFFS not mounted, cannot store image");
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreGive(s_ota_mutex);

    /* Add using file-based method */
    return zb_ota_add_image(file_path, image_info);
}

esp_err_t zb_ota_remove_image(uint16_t manufacturer_code, uint16_t image_type,
                               uint32_t file_version)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Removing OTA image: mfg=0x%04X type=0x%04X ver=0x%08lX",
             manufacturer_code, image_type, file_version);

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_ERR_NOT_FOUND;

    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (s_images[i].valid &&
            s_images[i].manufacturer_code == manufacturer_code &&
            s_images[i].image_type == image_type &&
            s_images[i].file_version == file_version) {

            /* Delete file if it exists */
            if (strlen(s_images[i].file_path) > 0) {
                unlink(s_images[i].file_path);
            }

            /* Mark slot as invalid */
            s_images[i].valid = false;
            s_image_count--;

            ESP_LOGI(TAG, "OTA image removed");
            ret = ESP_OK;
            break;
        }
    }

    if (ret == ESP_OK) {
        save_metadata_to_nvs();
    }

    xSemaphoreGive(s_ota_mutex);
    return ret;
}

esp_err_t zb_ota_remove_all_images(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Removing all OTA images");

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (s_images[i].valid) {
            if (strlen(s_images[i].file_path) > 0) {
                unlink(s_images[i].file_path);
            }
            s_images[i].valid = false;
        }
    }
    s_image_count = 0;

    save_metadata_to_nvs();

    xSemaphoreGive(s_ota_mutex);

    ESP_LOGI(TAG, "All OTA images removed");
    return ESP_OK;
}

size_t zb_ota_get_images(zb_ota_image_info_t *images, size_t max_count)
{
    if (!s_initialized || images == NULL || max_count == 0) {
        return 0;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    size_t count = 0;
    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES && count < max_count; i++) {
        if (s_images[i].valid) {
            memcpy(&images[count], &s_images[i], sizeof(zb_ota_image_info_t));
            count++;
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return count;
}

size_t zb_ota_get_image_count(void)
{
    return s_image_count;
}

const zb_ota_image_info_t* zb_ota_find_image(uint16_t manufacturer_code,
                                              uint16_t image_type,
                                              uint32_t current_version,
                                              uint16_t hw_version)
{
    if (!s_initialized) {
        return NULL;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    const zb_ota_image_info_t *best_match = NULL;
    uint32_t best_version = current_version;

    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (!s_images[i].valid) {
            continue;
        }

        /* Check manufacturer and image type */
        if (s_images[i].manufacturer_code != manufacturer_code ||
            s_images[i].image_type != image_type) {
            continue;
        }

        /* Check version (must be newer) */
        if (s_images[i].file_version <= current_version) {
            continue;
        }

        /* Check hardware version range if specified */
        if (hw_version != 0) {
            if (s_images[i].min_hw_version != 0 && hw_version < s_images[i].min_hw_version) {
                continue;
            }
            if (s_images[i].max_hw_version != 0xFFFF && hw_version > s_images[i].max_hw_version) {
                continue;
            }
        }

        /* Select newest version */
        if (s_images[i].file_version > best_version) {
            best_version = s_images[i].file_version;
            best_match = &s_images[i];
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return best_match;
}

esp_err_t zb_ota_notify_device(const esp_zb_ieee_addr_t ieee_addr,
                                uint8_t endpoint,
                                uint8_t payload_type)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char ieee_str[24];
    ieee_addr_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    ESP_LOGI(TAG, "Sending Image Notify to %s endpoint %d", ieee_str, endpoint);

    /* Build Image Notify payload per ZCL OTA spec
     * Payload Type 0: Query Jitter only (1 byte)
     */
    uint8_t payload[1];
    size_t payload_len = 0;

    (void)payload_type;  /* Currently only supporting jitter-only payload type */

    /* Query jitter (50% jitter) */
    payload[payload_len++] = 50;

    esp_zb_zcl_custom_cluster_cmd_req_t req = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_long = {0},
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_IMAGE_NOTIFY,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    memcpy(req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_addr, 8);

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
    return ESP_OK;
}

esp_err_t zb_ota_notify_device_short(uint16_t short_addr, uint8_t endpoint)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending Image Notify to 0x%04X endpoint %d", short_addr, endpoint);

    /* Build Image Notify payload per ZCL OTA spec
     * Payload Type 0: Query Jitter only (1 byte)
     */
    uint8_t payload[1];
    size_t payload_len = 0;

    /* Query jitter (50% jitter) */
    payload[payload_len++] = 50;

    esp_zb_zcl_custom_cluster_cmd_req_t req = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_IMAGE_NOTIFY,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
    return ESP_OK;
}

esp_err_t zb_ota_notify_broadcast(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Broadcasting Image Notify");

    /* Build Image Notify payload per ZCL OTA spec
     * Payload Type 0: Query Jitter only (1 byte)
     */
    uint8_t payload[1];
    size_t payload_len = 0;

    /* Query jitter (100% jitter for broadcast) */
    payload[payload_len++] = ZB_OTA_BROADCAST_QUERY_JITTER;

    esp_zb_zcl_custom_cluster_cmd_req_t req = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_short = 0xFFFF,  /* Broadcast */
            },
            .dst_endpoint = 0xFF,  /* All endpoints */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_IMAGE_NOTIFY,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
    return ESP_OK;
}

bool zb_ota_check_update_available(const esp_zb_ieee_addr_t ieee_addr,
                                    uint16_t manufacturer_code,
                                    uint16_t image_type,
                                    uint32_t current_version,
                                    uint32_t *new_version)
{
    const zb_ota_image_info_t *image = zb_ota_find_image(manufacturer_code,
                                                          image_type,
                                                          current_version, 0);
    if (image != NULL) {
        if (new_version != NULL) {
            *new_version = image->file_version;
        }
        return true;
    }
    return false;
}

esp_err_t zb_ota_get_transfer_status(const esp_zb_ieee_addr_t ieee_addr,
                                      zb_ota_transfer_t *transfer)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    zb_ota_transfer_t *t = find_transfer(ieee_addr);
    if (t == NULL) {
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    if (transfer != NULL) {
        memcpy(transfer, t, sizeof(zb_ota_transfer_t));
    }

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_abort_transfer(const esp_zb_ieee_addr_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char ieee_str[24];
    ieee_addr_to_string(ieee_addr, ieee_str, sizeof(ieee_str));
    ESP_LOGI(TAG, "Aborting transfer to %s", ieee_str);

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    zb_ota_transfer_t *transfer = find_transfer(ieee_addr);
    if (transfer == NULL) {
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    update_transfer_state(transfer, ZB_OTA_TRANSFER_ABORTED);
    s_stats.aborted_upgrades++;

    /* Publish status */
    zb_ota_publish_status(ieee_addr);

    remove_transfer(ieee_addr);

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_abort_all_transfers(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Aborting all transfers");

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (s_transfers[i].state != ZB_OTA_TRANSFER_IDLE) {
            s_transfers[i].state = ZB_OTA_TRANSFER_ABORTED;
            s_stats.aborted_upgrades++;
            memset(&s_transfers[i], 0, sizeof(zb_ota_transfer_t));
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_get_stats(zb_ota_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
    memcpy(stats, &s_stats, sizeof(zb_ota_stats_t));

    /* Count active transfers */
    stats->active_transfers = 0;
    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (s_transfers[i].state == ZB_OTA_TRANSFER_DOWNLOADING ||
            s_transfers[i].state == ZB_OTA_TRANSFER_COMPLETE) {
            stats->active_transfers++;
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_reset_stats(void)
{
    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
    memset(&s_stats, 0, sizeof(s_stats));
    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_register_callback(zb_ota_progress_cb_t callback)
{
    s_progress_callback = callback;
    return ESP_OK;
}

esp_err_t zb_ota_set_block_size(uint8_t block_size)
{
    if (block_size < ZB_OTA_MIN_BLOCK_SIZE || block_size > ZB_OTA_MAX_BLOCK_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    s_block_size = block_size;
    ESP_LOGI(TAG, "Block size set to %d bytes", block_size);
    return ESP_OK;
}

uint8_t zb_ota_get_block_size(void)
{
    return s_block_size;
}

esp_err_t zb_ota_set_rate_limit(uint8_t max_blocks_per_sec)
{
    s_max_blocks_per_sec = max_blocks_per_sec;
    ESP_LOGI(TAG, "Rate limit set to %d blocks/sec", max_blocks_per_sec);
    return ESP_OK;
}

/* ============================================================================
 * ZCL Command Handlers
 * ============================================================================ */

esp_err_t zb_ota_handle_query_next_image_req(const esp_zb_zcl_custom_cluster_command_message_t *message)
{
    if (!s_initialized || message == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t src_short_addr = message->info.src_address.u.short_addr;
    uint8_t src_endpoint = message->info.src_endpoint;

    ESP_LOGI(TAG, "Query Next Image Request from 0x%04X", src_short_addr);

    /* Parse request payload */
    const uint8_t *data = message->data.value;
    uint8_t field_control = data[0];
    uint16_t mfg_code = (data[2] << 8) | data[1];
    uint16_t image_type = (data[4] << 8) | data[3];
    uint32_t current_version = (data[8] << 24) | (data[7] << 16) | (data[6] << 8) | data[5];
    uint16_t hw_version = 0;

    if (field_control & 0x01) {
        /* Hardware version present */
        hw_version = (data[10] << 8) | data[9];
    }

    ESP_LOGI(TAG, "  Mfg: 0x%04X, Type: 0x%04X, Ver: 0x%08lX, HW: 0x%04X",
             mfg_code, image_type, current_version, hw_version);

    /* Find matching image */
    const zb_ota_image_info_t *image = zb_ota_find_image(mfg_code, image_type,
                                                          current_version, hw_version);

    if (image == NULL) {
        ESP_LOGI(TAG, "No update available");
        return send_query_next_image_response(src_short_addr,
                                               src_endpoint,
                                               0,  /* TSN handled by SDK */
                                               ZB_OTA_STATUS_NO_IMAGE_AVAILABLE, NULL);
    }

    ESP_LOGI(TAG, "Update available: version 0x%08lX", image->file_version);

    /* Create or update transfer record */
    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    esp_zb_ieee_addr_t ieee_addr;
    /* Get IEEE address from short address - would need device lookup */
    /* For now, use placeholder */
    memset(ieee_addr, 0, sizeof(ieee_addr));
    ieee_addr[0] = src_short_addr & 0xFF;
    ieee_addr[1] = (src_short_addr >> 8) & 0xFF;

    zb_ota_transfer_t *transfer = find_transfer(ieee_addr);
    if (transfer == NULL) {
        transfer = create_transfer(ieee_addr);
    }

    if (transfer != NULL) {
        transfer->short_addr = src_short_addr;
        transfer->manufacturer_code = mfg_code;
        transfer->image_type = image_type;
        transfer->file_version = image->file_version;
        transfer->image_size = image->image_size;
        transfer->block_size = s_block_size;
        transfer->file_offset = 0;
        transfer->blocks_sent = 0;
        transfer->start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        transfer->endpoint = src_endpoint;
        transfer->image = (zb_ota_image_info_t *)image;
        update_transfer_state(transfer, ZB_OTA_TRANSFER_QUERY);
    }

    xSemaphoreGive(s_ota_mutex);

    return send_query_next_image_response(src_short_addr,
                                           src_endpoint,
                                           0,  /* TSN handled by SDK */
                                           ZB_OTA_STATUS_SUCCESS, image);
}

esp_err_t zb_ota_handle_image_block_req(const esp_zb_zcl_custom_cluster_command_message_t *message)
{
    if (!s_initialized || message == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t src_short_addr = message->info.src_address.u.short_addr;
    uint8_t src_endpoint = message->info.src_endpoint;

    /* Parse request payload */
    const uint8_t *data = message->data.value;
    uint8_t field_control = data[0];
    uint16_t mfg_code = (data[2] << 8) | data[1];
    uint16_t image_type = (data[4] << 8) | data[3];
    uint32_t file_version = (data[8] << 24) | (data[7] << 16) | (data[6] << 8) | data[5];
    uint32_t file_offset = (data[12] << 24) | (data[11] << 16) | (data[10] << 8) | data[9];
    uint8_t max_data_size = data[13];

    ESP_LOGD(TAG, "Image Block Request from 0x%04X: offset=%lu, size=%d",
             src_short_addr, file_offset, max_data_size);

    (void)field_control;  /* Unused for now */
    (void)mfg_code;
    (void)image_type;
    (void)file_version;

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    /* Find transfer */
    esp_zb_ieee_addr_t ieee_addr;
    memset(ieee_addr, 0, sizeof(ieee_addr));
    ieee_addr[0] = src_short_addr & 0xFF;
    ieee_addr[1] = (src_short_addr >> 8) & 0xFF;

    zb_ota_transfer_t *transfer = find_transfer(ieee_addr);
    if (transfer == NULL) {
        ESP_LOGW(TAG, "No active transfer for device 0x%04X", src_short_addr);
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Update state to downloading */
    if (transfer->state == ZB_OTA_TRANSFER_QUERY) {
        update_transfer_state(transfer, ZB_OTA_TRANSFER_DOWNLOADING);
    }

    /* Rate limiting */
    if (!check_rate_limit(transfer)) {
        ESP_LOGD(TAG, "Rate limited, deferring response");
        /* Send wait response */
        xSemaphoreGive(s_ota_mutex);
        return ESP_OK;
    }

    /* Limit block size */
    uint8_t block_size = (max_data_size > s_block_size) ? s_block_size : max_data_size;

    /* Send response */
    esp_err_t ret = send_image_block_response(src_short_addr,
                                               src_endpoint,
                                               0,  /* TSN handled by SDK */
                                               transfer, file_offset, block_size);

    if (ret == ESP_OK) {
        transfer->file_offset = file_offset + block_size;
        transfer->blocks_sent++;
        transfer->last_block_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        s_stats.total_blocks_sent++;
        s_stats.total_bytes_sent += block_size;

        /* Check if transfer complete */
        if (transfer->file_offset >= transfer->image_size) {
            update_transfer_state(transfer, ZB_OTA_TRANSFER_COMPLETE);
        }

        /* Publish progress periodically */
        if (transfer->blocks_sent % 10 == 0) {
            zb_ota_publish_status(ieee_addr);
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return ret;
}

esp_err_t zb_ota_handle_upgrade_end_req(const esp_zb_zcl_custom_cluster_command_message_t *message)
{
    if (!s_initialized || message == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t src_short_addr = message->info.src_address.u.short_addr;
    uint8_t src_endpoint = message->info.src_endpoint;

    /* Parse request payload */
    const uint8_t *data = message->data.value;
    uint8_t status = data[0];
    uint16_t mfg_code = (data[2] << 8) | data[1];
    uint16_t image_type = (data[4] << 8) | data[3];
    uint32_t file_version = (data[8] << 24) | (data[7] << 16) | (data[6] << 8) | data[5];

    ESP_LOGI(TAG, "Upgrade End Request from 0x%04X: status=0x%02X, ver=0x%08lX",
             src_short_addr, status, file_version);

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    esp_zb_ieee_addr_t ieee_addr;
    memset(ieee_addr, 0, sizeof(ieee_addr));
    ieee_addr[0] = src_short_addr & 0xFF;
    ieee_addr[1] = (src_short_addr >> 8) & 0xFF;

    zb_ota_transfer_t *transfer = find_transfer(ieee_addr);

    if (status == ZB_OTA_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "OTA upgrade successful for device 0x%04X", src_short_addr);

        if (transfer != NULL) {
            update_transfer_state(transfer, ZB_OTA_TRANSFER_SUCCESS);
            s_stats.total_upgrades++;
        }

        /* Send response to trigger reboot */
        send_upgrade_end_response(src_short_addr,
                                   src_endpoint,
                                   0,  /* TSN handled by SDK */
                                   mfg_code, image_type, file_version);

        /* Publish final status */
        zb_ota_publish_status(ieee_addr);

        /* Clean up transfer */
        if (transfer != NULL) {
            remove_transfer(ieee_addr);
        }
    } else {
        ESP_LOGE(TAG, "OTA upgrade failed for device 0x%04X: status=0x%02X",
                 src_short_addr, status);

        if (transfer != NULL) {
            update_transfer_state(transfer, ZB_OTA_TRANSFER_FAILED);
            s_stats.failed_upgrades++;
            zb_ota_publish_status(ieee_addr);
            remove_transfer(ieee_addr);
        }
    }

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

/* ============================================================================
 * MQTT Integration
 * ============================================================================ */

esp_err_t zb_ota_process_mqtt_request(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing MQTT request: %s", topic);

    /* Check topic type */
    if (strstr(topic, "/ota/update/") != NULL) {
        /* Extract IEEE address from topic */
        const char *ieee_start = strstr(topic, "/ota/update/");
        if (ieee_start != NULL) {
            ieee_start += strlen("/ota/update/");
            ESP_LOGI(TAG, "OTA update requested for: %s", ieee_start);

            /* Parse IEEE address and trigger notify */
            /* TODO: Implement IEEE address parsing and device lookup */
        }
    } else if (strstr(topic, "/ota/check/") != NULL) {
        /* Check for updates */
        const char *ieee_start = strstr(topic, "/ota/check/");
        if (ieee_start != NULL) {
            ieee_start += strlen("/ota/check/");
            ESP_LOGI(TAG, "OTA check requested for: %s", ieee_start);

            /* TODO: Implement update check and response */
        }
    }

    return ESP_OK;
}

esp_err_t zb_ota_mqtt_receive_image(const uint8_t *data, size_t len, bool final)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    /* First chunk - allocate buffer (prefer PSRAM for large buffer) */
    if (s_upload_buffer == NULL) {
        s_upload_buffer_size = ZB_OTA_MAX_IMAGE_SIZE;
        /* Try PSRAM first for large OTA buffer */
#if CONFIG_SPIRAM
        s_upload_buffer = heap_caps_malloc(s_upload_buffer_size,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_upload_buffer != NULL) {
            ESP_LOGI(TAG, "Allocated %u bytes in PSRAM for OTA", s_upload_buffer_size);
        } else
#endif
        {
            /* Fallback to internal RAM */
            s_upload_buffer = malloc(s_upload_buffer_size);
            if (s_upload_buffer != NULL) {
                ESP_LOGI(TAG, "Allocated %u bytes in internal RAM for OTA", s_upload_buffer_size);
            }
        }
        if (s_upload_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate upload buffer (%u bytes)", s_upload_buffer_size);
            xSemaphoreGive(s_ota_mutex);
            return ESP_ERR_NO_MEM;
        }
        s_upload_received = 0;
        ESP_LOGI(TAG, "Starting image upload");
    }

    /* Check buffer space */
    if (s_upload_received + len > s_upload_buffer_size) {
        ESP_LOGE(TAG, "Image too large for buffer");
        free(s_upload_buffer);
        s_upload_buffer = NULL;
        s_upload_buffer_size = 0;
        s_upload_received = 0;
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    /* Copy data to buffer */
    memcpy(s_upload_buffer + s_upload_received, data, len);
    s_upload_received += len;

    ESP_LOGI(TAG, "Received %u bytes (total: %u)", len, s_upload_received);

    /* Final chunk - process image */
    if (final) {
        ESP_LOGI(TAG, "Image upload complete, processing %u bytes", s_upload_received);

        xSemaphoreGive(s_ota_mutex);

        /* Add image */
        esp_err_t ret = zb_ota_add_image_data(s_upload_buffer, s_upload_received, NULL);

        xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

        /* Free buffer */
        free(s_upload_buffer);
        s_upload_buffer = NULL;
        s_upload_buffer_size = 0;
        s_upload_received = 0;

        xSemaphoreGive(s_ota_mutex);

        if (ret == ESP_OK) {
            /* Publish updated image list */
            zb_ota_publish_images();
        }

        return ret;
    }

    xSemaphoreGive(s_ota_mutex);
    return ESP_OK;
}

esp_err_t zb_ota_publish_images(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Publishing OTA images list");

    cJSON *root = cJSON_CreateArray();
    if (root == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (!s_images[i].valid) {
            continue;
        }

        cJSON *image = cJSON_CreateObject();
        if (image == NULL) {
            continue;
        }

        cJSON_AddNumberToObject(image, "manufacturerCode", s_images[i].manufacturer_code);
        cJSON_AddNumberToObject(image, "imageType", s_images[i].image_type);
        cJSON_AddNumberToObject(image, "fileVersion", s_images[i].file_version);
        cJSON_AddNumberToObject(image, "fileSize", s_images[i].image_size);
        cJSON_AddStringToObject(image, "description", s_images[i].header_string);

        cJSON_AddItemToArray(root, image);
    }

    xSemaphoreGive(s_ota_mutex);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mqtt_client_publish("zigbee2mqtt/bridge/ota/images",
                                         json_str, 0, 0, false);
    free(json_str);

    return ret;
}

esp_err_t zb_ota_publish_status(const esp_zb_ieee_addr_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char ieee_str[24];
    ieee_addr_to_string(ieee_addr, ieee_str, sizeof(ieee_str));

    xSemaphoreTake(s_ota_mutex, portMAX_DELAY);

    zb_ota_transfer_t *transfer = find_transfer(ieee_addr);
    if (transfer == NULL) {
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        xSemaphoreGive(s_ota_mutex);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "state", zb_ota_get_state_string(transfer->state));
    cJSON_AddNumberToObject(root, "progress", calculate_progress(transfer));
    cJSON_AddNumberToObject(root, "remaining",
                            transfer->image_size - transfer->file_offset);

    xSemaphoreGive(s_ota_mutex);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char topic[128];
    snprintf(topic, sizeof(topic), "zigbee2mqtt/bridge/ota/status/%s", ieee_str);

    esp_err_t ret = mqtt_client_publish(topic, json_str, 0, 0, false);
    free(json_str);

    return ret;
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

esp_err_t zb_ota_parse_header(const uint8_t *data, size_t len,
                               zb_ota_file_header_t *header)
{
    if (data == NULL || header == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len < ZB_OTA_HEADER_MIN_LEN) {
        ESP_LOGE(TAG, "Data too small for OTA header: %u < %d", len, ZB_OTA_HEADER_MIN_LEN);
        return ESP_ERR_INVALID_SIZE;
    }

    /* Parse fixed fields */
    size_t offset = 0;

    header->magic = (data[offset + 3] << 24) | (data[offset + 2] << 16) |
                    (data[offset + 1] << 8) | data[offset];
    offset += 4;

    if (header->magic != ZB_OTA_HEADER_MAGIC) {
        ESP_LOGE(TAG, "Invalid OTA header magic: 0x%08lX", header->magic);
        return ESP_ERR_INVALID_ARG;
    }

    header->header_version = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    header->header_length = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    header->field_control = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    header->manufacturer_code = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    header->image_type = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    header->file_version = (data[offset + 3] << 24) | (data[offset + 2] << 16) |
                           (data[offset + 1] << 8) | data[offset];
    offset += 4;

    header->zigbee_stack_version = (data[offset + 1] << 8) | data[offset];
    offset += 2;

    /* Copy header string (32 bytes) */
    memcpy(header->header_string, &data[offset], ZB_OTA_HEADER_STRING_MAX_LEN);
    header->header_string[ZB_OTA_HEADER_STRING_MAX_LEN - 1] = '\0';
    offset += ZB_OTA_HEADER_STRING_MAX_LEN;

    header->total_image_size = (data[offset + 3] << 24) | (data[offset + 2] << 16) |
                                (data[offset + 1] << 8) | data[offset];
    offset += 4;

    /* Parse optional fields based on field_control */
    header->security_credential_version = 0;
    header->upgrade_file_destination = 0;
    header->min_hw_version = 0;
    header->max_hw_version = 0xFFFF;

    if (header->field_control & ZB_OTA_HEADER_FC_SECURITY_CREDENTIAL) {
        if (offset < len) {
            header->security_credential_version = data[offset++];
        }
    }

    if (header->field_control & ZB_OTA_HEADER_FC_DEVICE_SPECIFIC) {
        if (offset + 8 <= len) {
            memcpy(&header->upgrade_file_destination, &data[offset], 8);
            offset += 8;
        }
    }

    if (header->field_control & ZB_OTA_HEADER_FC_HW_VERSION) {
        if (offset + 4 <= len) {
            header->min_hw_version = (data[offset + 1] << 8) | data[offset];
            offset += 2;
            header->max_hw_version = (data[offset + 1] << 8) | data[offset];
            offset += 2;
        }
    }

    ESP_LOGI(TAG, "Parsed OTA header: mfg=0x%04X type=0x%04X ver=0x%08lX size=%lu",
             header->manufacturer_code, header->image_type,
             header->file_version, header->total_image_size);

    return ESP_OK;
}

bool zb_ota_validate_header(const zb_ota_file_header_t *header)
{
    if (header == NULL) {
        return false;
    }

    if (header->magic != ZB_OTA_HEADER_MAGIC) {
        ESP_LOGE(TAG, "Invalid magic number");
        return false;
    }

    if (header->header_length < ZB_OTA_HEADER_MIN_LEN) {
        ESP_LOGE(TAG, "Invalid header length: %d", header->header_length);
        return false;
    }

    if (header->total_image_size < header->header_length) {
        ESP_LOGE(TAG, "Invalid image size: %lu < %d",
                 header->total_image_size, header->header_length);
        return false;
    }

    if (header->total_image_size > ZB_OTA_MAX_IMAGE_SIZE) {
        ESP_LOGE(TAG, "Image too large: %lu > %d",
                 header->total_image_size, ZB_OTA_MAX_IMAGE_SIZE);
        return false;
    }

    return true;
}

const char* zb_ota_get_state_string(zb_ota_transfer_state_t state)
{
    switch (state) {
        case ZB_OTA_TRANSFER_IDLE:        return "idle";
        case ZB_OTA_TRANSFER_QUERY:       return "queried";
        case ZB_OTA_TRANSFER_DOWNLOADING: return "downloading";
        case ZB_OTA_TRANSFER_COMPLETE:    return "complete";
        case ZB_OTA_TRANSFER_UPGRADING:   return "upgrading";
        case ZB_OTA_TRANSFER_SUCCESS:     return "success";
        case ZB_OTA_TRANSFER_FAILED:      return "failed";
        case ZB_OTA_TRANSFER_ABORTED:     return "aborted";
        default:                          return "unknown";
    }
}

esp_err_t zb_ota_test(void)
{
    ESP_LOGI(TAG, "Running OTA self-test...");

    /* Test header parsing */
    uint8_t test_header[64] = {
        0x1E, 0xF1, 0xEE, 0x0B,  /* Magic */
        0x00, 0x01,              /* Header version */
        0x38, 0x00,              /* Header length (56) */
        0x00, 0x00,              /* Field control */
        0x15, 0x10,              /* Manufacturer code (0x1015 = LUMI) */
        0x01, 0x00,              /* Image type */
        0x01, 0x00, 0x00, 0x00,  /* File version */
        0x02, 0x00,              /* Stack version */
        /* Header string (32 bytes) */
        'T', 'e', 's', 't', ' ', 'I', 'm', 'a', 'g', 'e', 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0x00, 0x10, 0x00, 0x00,  /* Total size (4096) */
    };

    zb_ota_file_header_t header;
    esp_err_t ret = zb_ota_parse_header(test_header, sizeof(test_header), &header);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Header parse test FAILED");
        return ESP_FAIL;
    }

    if (header.manufacturer_code != 0x1015 || header.image_type != 0x0001) {
        ESP_LOGE(TAG, "Header values incorrect");
        return ESP_FAIL;
    }

    if (!zb_ota_validate_header(&header)) {
        ESP_LOGE(TAG, "Header validation test FAILED");
        return ESP_FAIL;
    }

    /* Test state string */
    const char *state_str = zb_ota_get_state_string(ZB_OTA_TRANSFER_DOWNLOADING);
    if (strcmp(state_str, "downloading") != 0) {
        ESP_LOGE(TAG, "State string test FAILED");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA self-test PASSED");
    return ESP_OK;
}

/* ============================================================================
 * Internal Functions
 * ============================================================================ */

static esp_err_t mount_spiffs(void)
{
    ESP_LOGI(TAG, "Mounting SPIFFS partition...");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = ZB_OTA_SPIFFS_MOUNT_POINT,
        .partition_label = "ota_storage",
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info("ota_storage", &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total=%u, used=%u", total, used);
    }

    s_spiffs_mounted = true;
    return ESP_OK;
}

static esp_err_t unmount_spiffs(void)
{
    if (!s_spiffs_mounted) {
        return ESP_OK;
    }

    esp_vfs_spiffs_unregister("ota_storage");
    s_spiffs_mounted = false;
    ESP_LOGI(TAG, "SPIFFS unmounted");
    return ESP_OK;
}

static esp_err_t load_metadata_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(ZB_OTA_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Load image count */
    uint8_t count = 0;
    ret = nvs_get_u8(nvs, "image_count", &count);
    if (ret != ESP_OK) {
        nvs_close(nvs);
        return ret;
    }

    /* Load each image metadata */
    for (uint8_t i = 0; i < count && i < ZB_OTA_MAX_IMAGES; i++) {
        char key[16];
        snprintf(key, sizeof(key), "image_%d", i);

        size_t size = sizeof(zb_ota_image_info_t);
        ret = nvs_get_blob(nvs, key, &s_images[i], &size);
        if (ret == ESP_OK && s_images[i].valid) {
            /* Verify file still exists */
            struct stat st;
            if (strlen(s_images[i].file_path) > 0 &&
                stat(s_images[i].file_path, &st) != 0) {
                ESP_LOGW(TAG, "Image file missing: %s", s_images[i].file_path);
                s_images[i].valid = false;
            } else {
                s_image_count++;
            }
        }
    }

    nvs_close(nvs);
    ESP_LOGI(TAG, "Loaded %d images from NVS", s_image_count);
    return ESP_OK;
}

static esp_err_t save_metadata_to_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(ZB_OTA_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Save image count */
    ret = nvs_set_u8(nvs, "image_count", s_image_count);
    if (ret != ESP_OK) {
        nvs_close(nvs);
        return ret;
    }

    /* Save each image metadata */
    uint8_t saved = 0;
    for (size_t i = 0; i < ZB_OTA_MAX_IMAGES; i++) {
        if (s_images[i].valid) {
            char key[16];
            snprintf(key, sizeof(key), "image_%d", saved);
            ret = nvs_set_blob(nvs, key, &s_images[i], sizeof(zb_ota_image_info_t));
            if (ret == ESP_OK) {
                saved++;
            }
        }
    }

    ret = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Saved %d images to NVS", saved);
    return ret;
}

static esp_err_t parse_image_file(const char *file_path, zb_ota_image_info_t *info)
{
    FILE *f = fopen(file_path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", file_path);
        return ESP_ERR_NOT_FOUND;
    }

    /* Read header */
    uint8_t header_data[128];
    size_t read_size = fread(header_data, 1, sizeof(header_data), f);

    /* Get file size */
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fclose(f);

    if (read_size < ZB_OTA_HEADER_MIN_LEN) {
        ESP_LOGE(TAG, "File too small for OTA header");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Parse header */
    zb_ota_file_header_t header;
    esp_err_t ret = zb_ota_parse_header(header_data, read_size, &header);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Validate */
    if (!zb_ota_validate_header(&header)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check file size matches header */
    if (file_size != header.total_image_size) {
        ESP_LOGW(TAG, "File size mismatch: %ld != %lu", file_size, header.total_image_size);
    }

    /* Fill info structure */
    info->manufacturer_code = header.manufacturer_code;
    info->image_type = header.image_type;
    info->file_version = header.file_version;
    info->image_size = header.total_image_size;
    info->header_length = header.header_length;
    info->min_hw_version = header.min_hw_version;
    info->max_hw_version = header.max_hw_version;
    strncpy(info->header_string, header.header_string, ZB_OTA_HEADER_STRING_MAX_LEN);

    return ESP_OK;
}

static zb_ota_transfer_t* find_transfer(const esp_zb_ieee_addr_t ieee_addr)
{
    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (s_transfers[i].state != ZB_OTA_TRANSFER_IDLE &&
            memcmp(s_transfers[i].ieee_addr, ieee_addr, 8) == 0) {
            return &s_transfers[i];
        }
    }
    return NULL;
}

static zb_ota_transfer_t* create_transfer(const esp_zb_ieee_addr_t ieee_addr)
{
    /* Find empty slot */
    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (s_transfers[i].state == ZB_OTA_TRANSFER_IDLE) {
            memset(&s_transfers[i], 0, sizeof(zb_ota_transfer_t));
            memcpy(s_transfers[i].ieee_addr, ieee_addr, 8);
            return &s_transfers[i];
        }
    }

    /* No empty slot - find oldest completed/failed transfer */
    uint32_t oldest_time = UINT32_MAX;
    int oldest_idx = -1;

    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (s_transfers[i].state == ZB_OTA_TRANSFER_SUCCESS ||
            s_transfers[i].state == ZB_OTA_TRANSFER_FAILED ||
            s_transfers[i].state == ZB_OTA_TRANSFER_ABORTED) {
            if (s_transfers[i].last_block_time < oldest_time) {
                oldest_time = s_transfers[i].last_block_time;
                oldest_idx = i;
            }
        }
    }

    if (oldest_idx >= 0) {
        memset(&s_transfers[oldest_idx], 0, sizeof(zb_ota_transfer_t));
        memcpy(s_transfers[oldest_idx].ieee_addr, ieee_addr, 8);
        return &s_transfers[oldest_idx];
    }

    ESP_LOGE(TAG, "No available transfer slots");
    return NULL;
}

static void remove_transfer(const esp_zb_ieee_addr_t ieee_addr)
{
    for (size_t i = 0; i < ZB_OTA_MAX_TRANSFERS; i++) {
        if (memcmp(s_transfers[i].ieee_addr, ieee_addr, 8) == 0) {
            memset(&s_transfers[i], 0, sizeof(zb_ota_transfer_t));
            break;
        }
    }
}

static esp_err_t send_query_next_image_response(uint16_t short_addr, uint8_t endpoint,
                                                 uint8_t tsn, uint8_t status,
                                                 const zb_ota_image_info_t *image)
{
    (void)tsn;  /* TSN is handled by the custom cluster response API */

    ESP_LOGD(TAG, "Sending Query Next Image Response to 0x%04X, status=0x%02X",
             short_addr, status);

    /* Build Query Next Image Response payload per ZCL OTA spec */
    uint8_t payload[13];
    size_t payload_len = 1;  /* At minimum, status byte */

    payload[0] = status;

    if (status == ZB_OTA_STATUS_SUCCESS && image != NULL) {
        /* Manufacturer code (2 bytes, little-endian) */
        payload[1] = image->manufacturer_code & 0xFF;
        payload[2] = (image->manufacturer_code >> 8) & 0xFF;
        /* Image type (2 bytes, little-endian) */
        payload[3] = image->image_type & 0xFF;
        payload[4] = (image->image_type >> 8) & 0xFF;
        /* File version (4 bytes, little-endian) */
        payload[5] = image->file_version & 0xFF;
        payload[6] = (image->file_version >> 8) & 0xFF;
        payload[7] = (image->file_version >> 16) & 0xFF;
        payload[8] = (image->file_version >> 24) & 0xFF;
        /* Image size (4 bytes, little-endian) */
        payload[9] = image->image_size & 0xFF;
        payload[10] = (image->image_size >> 8) & 0xFF;
        payload[11] = (image->image_size >> 16) & 0xFF;
        payload[12] = (image->image_size >> 24) & 0xFF;
        payload_len = 13;
    }

    esp_zb_zcl_custom_cluster_cmd_resp_t resp = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_QUERY_NEXT_IMAGE_RSP,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    resp.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_zcl_custom_cluster_cmd_resp(&resp);
    return ESP_OK;
}

static esp_err_t send_image_block_response(uint16_t short_addr, uint8_t endpoint,
                                            uint8_t tsn, zb_ota_transfer_t *transfer,
                                            uint32_t offset, uint8_t size)
{
    (void)tsn;  /* TSN is handled by the custom cluster response API */

    if (transfer == NULL || transfer->image == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read block from file */
    uint8_t block_data[ZB_OTA_MAX_BLOCK_SIZE];
    esp_err_t ret = read_image_block(transfer->image, offset, block_data, size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read block at offset %lu", offset);
        return ret;
    }

    ESP_LOGD(TAG, "Sending Image Block Response to 0x%04X, offset=%lu, size=%d",
             short_addr, offset, size);

    /* Build Image Block Response payload per ZCL OTA spec
     * Format: Status(1) + ManufCode(2) + ImageType(2) + FileVer(4) + FileOffset(4) + DataSize(1) + Data(N)
     */
    uint8_t payload[14 + ZB_OTA_MAX_BLOCK_SIZE];
    size_t payload_len = 0;

    /* Status */
    payload[payload_len++] = ZB_OTA_STATUS_SUCCESS;
    /* Manufacturer code (2 bytes, little-endian) */
    payload[payload_len++] = transfer->manufacturer_code & 0xFF;
    payload[payload_len++] = (transfer->manufacturer_code >> 8) & 0xFF;
    /* Image type (2 bytes, little-endian) */
    payload[payload_len++] = transfer->image_type & 0xFF;
    payload[payload_len++] = (transfer->image_type >> 8) & 0xFF;
    /* File version (4 bytes, little-endian) */
    payload[payload_len++] = transfer->file_version & 0xFF;
    payload[payload_len++] = (transfer->file_version >> 8) & 0xFF;
    payload[payload_len++] = (transfer->file_version >> 16) & 0xFF;
    payload[payload_len++] = (transfer->file_version >> 24) & 0xFF;
    /* File offset (4 bytes, little-endian) */
    payload[payload_len++] = offset & 0xFF;
    payload[payload_len++] = (offset >> 8) & 0xFF;
    payload[payload_len++] = (offset >> 16) & 0xFF;
    payload[payload_len++] = (offset >> 24) & 0xFF;
    /* Data size (1 byte) */
    payload[payload_len++] = size;
    /* Image data */
    memcpy(&payload[payload_len], block_data, size);
    payload_len += size;

    esp_zb_zcl_custom_cluster_cmd_resp_t resp = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_IMAGE_BLOCK_RSP,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    resp.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_zcl_custom_cluster_cmd_resp(&resp);
    return ESP_OK;
}

static esp_err_t send_upgrade_end_response(uint16_t short_addr, uint8_t endpoint,
                                            uint8_t tsn, uint16_t mfg_code,
                                            uint16_t image_type, uint32_t file_version)
{
    (void)tsn;  /* TSN is handled by the custom cluster response API */

    ESP_LOGI(TAG, "Sending Upgrade End Response to 0x%04X", short_addr);

    /* Build Upgrade End Response payload per ZCL OTA spec
     * Format: ManufCode(2) + ImageType(2) + FileVer(4) + CurrentTime(4) + UpgradeTime(4)
     */
    uint8_t payload[16];
    size_t payload_len = 0;

    /* Manufacturer code (2 bytes, little-endian) */
    payload[payload_len++] = mfg_code & 0xFF;
    payload[payload_len++] = (mfg_code >> 8) & 0xFF;
    /* Image type (2 bytes, little-endian) */
    payload[payload_len++] = image_type & 0xFF;
    payload[payload_len++] = (image_type >> 8) & 0xFF;
    /* File version (4 bytes, little-endian) */
    payload[payload_len++] = file_version & 0xFF;
    payload[payload_len++] = (file_version >> 8) & 0xFF;
    payload[payload_len++] = (file_version >> 16) & 0xFF;
    payload[payload_len++] = (file_version >> 24) & 0xFF;
    /* Current time (4 bytes, little-endian) - 0 means upgrade immediately */
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;
    /* Upgrade time (4 bytes, little-endian) - 0 means upgrade immediately */
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;
    payload[payload_len++] = 0;

    esp_zb_zcl_custom_cluster_cmd_resp_t resp = {
        .zcl_basic_cmd = {
            .src_endpoint = 1,
            .dst_addr_u = {
                .addr_short = short_addr,
            },
            .dst_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = ZB_OTA_CLUSTER_ID,
        .custom_cmd_id = ZB_OTA_CMD_UPGRADE_END_RSP,
        .data = {
            .type = ESP_ZB_ZCL_ATTR_TYPE_INVALID,
            .size = payload_len,
            .value = payload,
        },
    };
    resp.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_zcl_custom_cluster_cmd_resp(&resp);
    return ESP_OK;
}

static bool check_rate_limit(zb_ota_transfer_t *transfer)
{
    if (transfer == NULL || s_max_blocks_per_sec == 0) {
        return true;
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t min_interval = 1000 / s_max_blocks_per_sec;

    if (now - transfer->last_block_time < min_interval) {
        return false;
    }

    return true;
}

static void update_transfer_state(zb_ota_transfer_t *transfer, zb_ota_transfer_state_t state)
{
    if (transfer == NULL) {
        return;
    }

    zb_ota_transfer_state_t old_state = transfer->state;
    transfer->state = state;

    ESP_LOGI(TAG, "Transfer state: %s -> %s",
             zb_ota_get_state_string(old_state),
             zb_ota_get_state_string(state));

    /* Call progress callback */
    if (s_progress_callback != NULL) {
        uint8_t progress = calculate_progress(transfer);
        s_progress_callback(transfer->ieee_addr, state, progress);
    }
}

static uint8_t calculate_progress(const zb_ota_transfer_t *transfer)
{
    if (transfer == NULL || transfer->image_size == 0) {
        return 0;
    }

    uint32_t progress = (transfer->file_offset * GW_PERCENTAGE_MAX) / transfer->image_size;
    return (progress > GW_PERCENTAGE_MAX) ? GW_PERCENTAGE_MAX : (uint8_t)progress;
}

static void ieee_addr_to_string(const esp_zb_ieee_addr_t ieee_addr, char *str, size_t len)
{
    if (str == NULL || len < ZB_OTA_VERSION_STRING_MIN_SIZE) {
        return;
    }

    snprintf(str, len, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
             ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
}

static esp_err_t read_image_block(const zb_ota_image_info_t *image, uint32_t offset,
                                   uint8_t *buffer, uint8_t size)
{
    if (image == NULL || buffer == NULL || strlen(image->file_path) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(image->file_path, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open image file: %s", image->file_path);
        return ESP_ERR_NOT_FOUND;
    }

    /* Seek to offset */
    if (fseek(f, offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to seek to offset %lu", offset);
        fclose(f);
        return ESP_FAIL;
    }

    /* Read block */
    size_t read_size = fread(buffer, 1, size, f);
    fclose(f);

    if (read_size != size) {
        /* End of file - return what we have */
        if (read_size > 0) {
            memset(buffer + read_size, 0xFF, size - read_size);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Failed to read block: %u/%d bytes", read_size, size);
        return ESP_FAIL;
    }

    return ESP_OK;
}
