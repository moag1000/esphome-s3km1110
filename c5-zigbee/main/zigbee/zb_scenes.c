/**
 * @file zb_scenes.c
 * @brief Zigbee Scene Management Implementation
 *
 * Implements Zigbee scene management including creation, store/recall,
 * NVS persistence, and MQTT integration for Zigbee2MQTT compatibility.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "zb_scenes.h"
#include "zb_constants.h"
#include "zb_groups.h"
#include "zb_device_handler.h"
#include "core/compat_stubs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ZB_SCENES";

/* Module state */
static zb_scene_t *s_scenes = NULL;
static size_t s_scene_count = 0;
static bool s_initialized = false;
static SemaphoreHandle_t s_scenes_mutex = NULL;
static zb_scene_event_cb_t s_event_callback = NULL;

/* NVS keys */
#define NVS_KEY_SCENE_COUNT "scn_count"
#define NVS_KEY_SCENE_PREFIX "scn_"
#define NVS_KEY_SCENE_DATA_FMT "scn_%02d"

/* MQTT topic patterns */
#define TOPIC_SCENE_ADD "zigbee2mqtt/bridge/request/scene/add"
#define TOPIC_SCENE_REMOVE "zigbee2mqtt/bridge/request/scene/remove"
#define TOPIC_SCENE_STORE "zigbee2mqtt/bridge/request/scene/store"
#define TOPIC_SCENE_RECALL "zigbee2mqtt/bridge/request/scene/recall"
#define TOPIC_BRIDGE_SCENES "zigbee2mqtt/bridge/scenes"
#define TOPIC_SCENE_RECALL_SUFFIX "/scene_recall"

/* Forward declarations */
static zb_scene_t* find_scene_by_id(uint8_t scene_id);
static zb_scene_t* find_scene_by_name(const char *name);
static zb_scene_t* find_free_scene_slot(void);
static uint8_t allocate_scene_id(void);
static int find_device_index(const zb_scene_t *scene, uint64_t ieee_addr, uint8_t endpoint);
static esp_err_t apply_device_state(const zb_scene_device_state_t *state, uint16_t transition);
static esp_err_t capture_group_device_states(zb_scene_t *scene);
static void notify_event(uint8_t scene_id, zb_scene_event_type_t event_type);
static esp_err_t parse_scene_add_request(const char *payload, uint16_t *group_id,
                                          char *name, size_t name_len);
static esp_err_t parse_scene_remove_request(const char *payload, char *name, size_t name_len);
static esp_err_t parse_scene_store_request(const char *payload, char *name, size_t name_len);
static esp_err_t parse_scene_recall_request(const char *payload, char *name, size_t name_len,
                                             uint16_t *transition);
static esp_err_t extract_group_name_from_topic(const char *topic, char *name, size_t name_len);

esp_err_t zb_scenes_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Scenes module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee scene management...");

    /* Create mutex */
    s_scenes_mutex = xSemaphoreCreateMutex();
    if (s_scenes_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create scenes mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate scene storage - prefer PSRAM to free internal RAM for WiFi */
    s_scenes = heap_caps_calloc(ZB_SCENES_MAX_COUNT, sizeof(zb_scene_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_scenes == NULL) {
        /* Fallback to internal RAM if PSRAM not available */
        s_scenes = calloc(ZB_SCENES_MAX_COUNT, sizeof(zb_scene_t));
    }
    if (s_scenes == NULL) {
        ESP_LOGE(TAG, "Failed to allocate scene storage");
        vSemaphoreDelete(s_scenes_mutex);
        s_scenes_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Scene storage allocated: %zu bytes",
             ZB_SCENES_MAX_COUNT * sizeof(zb_scene_t));

    /* Initialize all scenes as inactive */
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        s_scenes[i].active = false;
        s_scenes[i].scene_id = ZB_SCENE_ID_INVALID;
        s_scenes[i].device_count = 0;
        s_scenes[i].transition_time = ZB_SCENE_DEFAULT_TRANSITION;
    }

    s_scene_count = 0;
    s_initialized = true;

    /* Try to load scenes from NVS */
    esp_err_t ret = zb_scenes_load_from_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded %d scenes from NVS", s_scene_count);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved scenes found in NVS");
    } else {
        ESP_LOGW(TAG, "Failed to load scenes from NVS: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Scene management initialized (max: %d scenes, %d devices each)",
             ZB_SCENES_MAX_COUNT, ZB_SCENE_MAX_DEVICES);
    return ESP_OK;
}

esp_err_t zb_scenes_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Zigbee scene management...");

    /* Save scenes before shutdown */
    zb_scenes_save_to_nvs();

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    if (s_scenes != NULL) {
        free(s_scenes);
        s_scenes = NULL;
    }

    s_scene_count = 0;
    s_initialized = false;
    s_event_callback = NULL;

    xSemaphoreGive(s_scenes_mutex);
    vSemaphoreDelete(s_scenes_mutex);
    s_scenes_mutex = NULL;

    ESP_LOGI(TAG, "Scene management deinitialized");
    return ESP_OK;
}

esp_err_t zb_scenes_create(uint16_t group_id, const char *name, uint8_t *scene_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Scenes module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (name == NULL || strlen(name) == 0) {
        ESP_LOGE(TAG, "Invalid scene name");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(name) >= ZB_SCENE_NAME_MAX_LEN) {
        ESP_LOGE(TAG, "Scene name too long: %d chars (max %d)",
                 strlen(name), ZB_SCENE_NAME_MAX_LEN - 1);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    /* Check if name already exists */
    if (find_scene_by_name(name) != NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene with name '%s' already exists", name);
        return ESP_ERR_INVALID_ARG;
    }

    /* Find free slot */
    zb_scene_t *scene = find_free_scene_slot();
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGE(TAG, "Maximum number of scenes reached (%d)", ZB_SCENES_MAX_COUNT);
        return ESP_ERR_NO_MEM;
    }

    /* Allocate scene ID */
    uint8_t new_id = allocate_scene_id();
    if (new_id == ZB_SCENE_ID_INVALID) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGE(TAG, "Failed to allocate scene ID");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize scene */
    scene->scene_id = new_id;
    scene->group_id = group_id;
    strncpy(scene->name, name, ZB_SCENE_NAME_MAX_LEN - 1);
    scene->name[ZB_SCENE_NAME_MAX_LEN - 1] = '\0';
    scene->device_count = 0;
    scene->transition_time = ZB_SCENE_DEFAULT_TRANSITION;
    memset(scene->devices, 0, sizeof(scene->devices));
    scene->active = true;

    s_scene_count++;

    if (scene_id != NULL) {
        *scene_id = new_id;
    }

    xSemaphoreGive(s_scenes_mutex);

    ESP_LOGI(TAG, "Created scene '%s' with ID %d for group 0x%04X (total: %d)",
             name, new_id, group_id, s_scene_count);

    /* Save to NVS */
    zb_scenes_save_to_nvs();

    /* Notify event */
    notify_event(new_id, ZB_SCENE_EVENT_CREATED);

    return ESP_OK;
}

esp_err_t zb_scenes_remove(uint8_t scene_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene %d not found", scene_id);
        return ESP_ERR_NOT_FOUND;
    }

    char scene_name[ZB_SCENE_NAME_MAX_LEN];
    strncpy(scene_name, scene->name, ZB_SCENE_NAME_MAX_LEN);

    /* Mark scene as inactive */
    scene->active = false;
    scene->scene_id = ZB_SCENE_ID_INVALID;
    scene->device_count = 0;
    memset(scene->name, 0, ZB_SCENE_NAME_MAX_LEN);
    memset(scene->devices, 0, sizeof(scene->devices));

    s_scene_count--;

    xSemaphoreGive(s_scenes_mutex);

    ESP_LOGI(TAG, "Removed scene '%s' (ID: %d, remaining: %d)",
             scene_name, scene_id, s_scene_count);

    /* Save to NVS */
    zb_scenes_save_to_nvs();

    /* Notify event */
    notify_event(scene_id, ZB_SCENE_EVENT_REMOVED);

    return ESP_OK;
}

esp_err_t zb_scenes_remove_by_name(const char *name)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_name(name);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene '%s' not found", name);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t scene_id = scene->scene_id;
    xSemaphoreGive(s_scenes_mutex);

    return zb_scenes_remove(scene_id);
}

esp_err_t zb_scenes_store(uint8_t scene_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene %d not found", scene_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Storing current device states in scene '%s' (ID: %d)",
             scene->name, scene_id);

    /* Capture states from group devices */
    esp_err_t ret = capture_group_device_states(scene);

    xSemaphoreGive(s_scenes_mutex);

    if (ret == ESP_OK) {
        /* Save to NVS */
        zb_scenes_save_to_nvs();

        /* Notify event */
        notify_event(scene_id, ZB_SCENE_EVENT_STORED);

        ESP_LOGI(TAG, "Stored %d device states in scene '%s'",
                 scene->device_count, scene->name);
    } else {
        ESP_LOGE(TAG, "Failed to store device states: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_scenes_store_device(uint8_t scene_id, uint64_t ieee_addr,
                                  uint8_t endpoint, const zb_scene_device_state_t *state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene %d not found", scene_id);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if device already exists in scene */
    int dev_idx = find_device_index(scene, ieee_addr, endpoint);

    if (dev_idx >= 0) {
        /* Update existing device state */
        memcpy(&scene->devices[dev_idx], state, sizeof(zb_scene_device_state_t));
        ESP_LOGD(TAG, "Updated device 0x%016llX:%d in scene %d",
                 (unsigned long long)ieee_addr, endpoint, scene_id);
    } else {
        /* Add new device */
        if (scene->device_count >= ZB_SCENE_MAX_DEVICES) {
            xSemaphoreGive(s_scenes_mutex);
            ESP_LOGE(TAG, "Scene %d device limit reached (%d)", scene_id, ZB_SCENE_MAX_DEVICES);
            return ESP_ERR_NO_MEM;
        }

        memcpy(&scene->devices[scene->device_count], state, sizeof(zb_scene_device_state_t));
        scene->devices[scene->device_count].ieee_addr = ieee_addr;
        scene->devices[scene->device_count].endpoint = endpoint;
        scene->device_count++;

        ESP_LOGD(TAG, "Added device 0x%016llX:%d to scene %d",
                 (unsigned long long)ieee_addr, endpoint, scene_id);
    }

    xSemaphoreGive(s_scenes_mutex);

    /* Save to NVS */
    zb_scenes_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_scenes_recall(uint8_t scene_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene %d not found", scene_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Recalling scene '%s' (ID: %d, %d devices, transition: %d ms)",
             scene->name, scene_id, scene->device_count, scene->transition_time * 100);

    esp_err_t ret = ESP_OK;
    uint16_t transition = scene->transition_time;

    /* Apply state to each device in the scene */
    for (uint8_t i = 0; i < scene->device_count; i++) {
        esp_err_t dev_ret = apply_device_state(&scene->devices[i], transition);
        if (dev_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to apply state to device 0x%016llX:%d: %s",
                     (unsigned long long)scene->devices[i].ieee_addr,
                     scene->devices[i].endpoint, esp_err_to_name(dev_ret));
            ret = dev_ret; /* Report last error */
        }
    }

    xSemaphoreGive(s_scenes_mutex);

    /* Notify event */
    notify_event(scene_id, ZB_SCENE_EVENT_RECALLED);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Scene '%s' recalled successfully", scene->name);
    }

    return ret;
}

esp_err_t zb_scenes_recall_by_name(const char *name)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_name(name);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene '%s' not found", name);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t scene_id = scene->scene_id;
    xSemaphoreGive(s_scenes_mutex);

    return zb_scenes_recall(scene_id);
}

const zb_scene_t* zb_scenes_get(uint8_t scene_id)
{
    if (!s_initialized) {
        return NULL;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);
    zb_scene_t *scene = find_scene_by_id(scene_id);
    xSemaphoreGive(s_scenes_mutex);

    return scene;
}

const zb_scene_t* zb_scenes_get_by_name(const char *name)
{
    if (!s_initialized || name == NULL) {
        return NULL;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);
    zb_scene_t *scene = find_scene_by_name(name);
    xSemaphoreGive(s_scenes_mutex);

    return scene;
}

size_t zb_scenes_get_by_group(uint16_t group_id, zb_scene_t *scenes, size_t max_count)
{
    if (!s_initialized || scenes == NULL) {
        return 0;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    size_t copied = 0;
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT && copied < max_count; i++) {
        if (s_scenes[i].active && s_scenes[i].group_id == group_id) {
            memcpy(&scenes[copied], &s_scenes[i], sizeof(zb_scene_t));
            copied++;
        }
    }

    xSemaphoreGive(s_scenes_mutex);

    return copied;
}

size_t zb_scenes_get_all(zb_scene_t *scenes, size_t max_count)
{
    if (!s_initialized || scenes == NULL) {
        return 0;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    size_t copied = 0;
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT && copied < max_count; i++) {
        if (s_scenes[i].active) {
            memcpy(&scenes[copied], &s_scenes[i], sizeof(zb_scene_t));
            copied++;
        }
    }

    xSemaphoreGive(s_scenes_mutex);

    return copied;
}

size_t zb_scenes_get_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);
    size_t count = s_scene_count;
    xSemaphoreGive(s_scenes_mutex);

    return count;
}

esp_err_t zb_scenes_rename(uint8_t scene_id, const char *new_name)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (new_name == NULL || strlen(new_name) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(new_name) >= ZB_SCENE_NAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if new name already exists */
    zb_scene_t *existing = find_scene_by_name(new_name);
    if (existing != NULL && existing != scene) {
        xSemaphoreGive(s_scenes_mutex);
        ESP_LOGW(TAG, "Scene with name '%s' already exists", new_name);
        return ESP_ERR_INVALID_ARG;
    }

    char old_name[ZB_SCENE_NAME_MAX_LEN];
    strncpy(old_name, scene->name, ZB_SCENE_NAME_MAX_LEN);

    strncpy(scene->name, new_name, ZB_SCENE_NAME_MAX_LEN - 1);
    scene->name[ZB_SCENE_NAME_MAX_LEN - 1] = '\0';

    xSemaphoreGive(s_scenes_mutex);

    ESP_LOGI(TAG, "Renamed scene %d from '%s' to '%s'", scene_id, old_name, new_name);

    /* Save to NVS */
    zb_scenes_save_to_nvs();

    /* Notify event */
    notify_event(scene_id, ZB_SCENE_EVENT_UPDATED);

    return ESP_OK;
}

esp_err_t zb_scenes_set_transition(uint8_t scene_id, uint16_t transition_time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    zb_scene_t *scene = find_scene_by_id(scene_id);
    if (scene == NULL) {
        xSemaphoreGive(s_scenes_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    scene->transition_time = transition_time;

    xSemaphoreGive(s_scenes_mutex);

    ESP_LOGI(TAG, "Set scene %d transition time to %d (1/10 sec)", scene_id, transition_time);

    /* Save to NVS */
    zb_scenes_save_to_nvs();

    return ESP_OK;
}

esp_err_t zb_scenes_save_to_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving scenes to NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_SCENES_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    /* Save scene count */
    ret = nvs_set_u16(nvs_handle, NVS_KEY_SCENE_COUNT, (uint16_t)s_scene_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save scene count: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_scenes_mutex);
        nvs_close(nvs_handle);
        return ret;
    }

    /* Save each active scene */
    uint8_t saved_idx = 0;
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        if (s_scenes[i].active) {
            char key[16];
            snprintf(key, sizeof(key), NVS_KEY_SCENE_DATA_FMT, saved_idx);

            ret = nvs_set_blob(nvs_handle, key, &s_scenes[i], sizeof(zb_scene_t));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save scene %d: %s", i, esp_err_to_name(ret));
                xSemaphoreGive(s_scenes_mutex);
                nvs_close(nvs_handle);
                return ret;
            }
            saved_idx++;
        }
    }

    xSemaphoreGive(s_scenes_mutex);

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %d scenes to NVS", s_scene_count);
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_scenes_load_from_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Loading scenes from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_SCENES_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found - no saved scenes");
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Load scene count */
    uint16_t count = 0;
    ret = nvs_get_u16(nvs_handle, NVS_KEY_SCENE_COUNT, &count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load scene count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    /* Clear existing scenes */
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        s_scenes[i].active = false;
        s_scenes[i].scene_id = ZB_SCENE_ID_INVALID;
    }
    s_scene_count = 0;

    /* Load each scene */
    for (uint8_t i = 0; i < count && i < ZB_SCENES_MAX_COUNT; i++) {
        char key[16];
        snprintf(key, sizeof(key), NVS_KEY_SCENE_DATA_FMT, i);

        size_t required_size = sizeof(zb_scene_t);
        ret = nvs_get_blob(nvs_handle, key, &s_scenes[i], &required_size);
        if (ret == ESP_OK) {
            s_scenes[i].active = true;
            s_scene_count++;
            ESP_LOGD(TAG, "Loaded scene '%s' (ID: %d, %d devices)",
                     s_scenes[i].name, s_scenes[i].scene_id, s_scenes[i].device_count);
        } else {
            ESP_LOGW(TAG, "Failed to load scene %d: %s", i, esp_err_to_name(ret));
        }
    }

    xSemaphoreGive(s_scenes_mutex);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Loaded %d scenes from NVS", s_scene_count);
    return ESP_OK;
}

esp_err_t zb_scenes_clear_nvs(void)
{
    ESP_LOGI(TAG, "Clearing scenes from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_SCENES_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(nvs_handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Cleared all scenes from NVS");
    }

    return ret;
}

esp_err_t zb_scenes_process_mqtt_request(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing scene request: %s", topic);
    ESP_LOGD(TAG, "Payload: %.*s", (int)len, payload);

    /* Null-terminate payload for parsing */
    char *payload_str = malloc(len + 1);
    if (payload_str == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(payload_str, payload, len);
    payload_str[len] = '\0';

    esp_err_t ret = ESP_FAIL;

    if (strcmp(topic, TOPIC_SCENE_ADD) == 0) {
        /* Create scene */
        uint16_t group_id = 0;
        char name[ZB_SCENE_NAME_MAX_LEN];
        ret = parse_scene_add_request(payload_str, &group_id, name, sizeof(name));
        if (ret == ESP_OK) {
            uint8_t scene_id;
            ret = zb_scenes_create(group_id, name, &scene_id);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Created scene '%s' via MQTT", name);
                zb_scenes_publish_list();
            }
        }
    } else if (strcmp(topic, TOPIC_SCENE_REMOVE) == 0) {
        /* Remove scene */
        char name[ZB_SCENE_NAME_MAX_LEN];
        ret = parse_scene_remove_request(payload_str, name, sizeof(name));
        if (ret == ESP_OK) {
            ret = zb_scenes_remove_by_name(name);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Removed scene '%s' via MQTT", name);
                zb_scenes_publish_list();
            }
        }
    } else if (strcmp(topic, TOPIC_SCENE_STORE) == 0) {
        /* Store current state in scene */
        char name[ZB_SCENE_NAME_MAX_LEN];
        ret = parse_scene_store_request(payload_str, name, sizeof(name));
        if (ret == ESP_OK) {
            const zb_scene_t *scene = zb_scenes_get_by_name(name);
            if (scene != NULL) {
                ret = zb_scenes_store(scene->scene_id);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Stored state in scene '%s' via MQTT", name);
                    zb_scenes_publish_list();
                }
            } else {
                ret = ESP_ERR_NOT_FOUND;
            }
        }
    } else if (strcmp(topic, TOPIC_SCENE_RECALL) == 0) {
        /* Recall scene */
        char name[ZB_SCENE_NAME_MAX_LEN];
        uint16_t transition = 0;
        ret = parse_scene_recall_request(payload_str, name, sizeof(name), &transition);
        if (ret == ESP_OK) {
            const zb_scene_t *scene = zb_scenes_get_by_name(name);
            if (scene != NULL) {
                /* Optionally set transition time */
                if (transition > 0) {
                    zb_scenes_set_transition(scene->scene_id, transition);
                }
                ret = zb_scenes_recall(scene->scene_id);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Recalled scene '%s' via MQTT", name);
                }
            } else {
                ret = ESP_ERR_NOT_FOUND;
            }
        }
    } else {
        ESP_LOGW(TAG, "Unknown scene request topic: %s", topic);
        ret = ESP_ERR_INVALID_ARG;
    }

    free(payload_str);
    return ret;
}

esp_err_t zb_scenes_process_mqtt_recall(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Extract group name from topic: zigbee2mqtt/[group_name]/scene_recall */
    char group_name[ZB_SCENE_NAME_MAX_LEN];
    esp_err_t ret = extract_group_name_from_topic(topic, group_name, sizeof(group_name));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to extract group name from topic: %s", topic);
        return ret;
    }

    ESP_LOGI(TAG, "Processing scene recall for group '%s'", group_name);
    ESP_LOGD(TAG, "Payload: %.*s", (int)len, payload);

    /* Null-terminate payload */
    char *scene_name = malloc(len + 1);
    if (scene_name == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(scene_name, payload, len);
    scene_name[len] = '\0';

    /* Try to parse as JSON first */
    cJSON *json = cJSON_Parse(scene_name);
    if (json != NULL) {
        cJSON *name_obj = cJSON_GetObjectItem(json, "scene");
        if (name_obj == NULL) {
            name_obj = cJSON_GetObjectItem(json, "name");
        }
        if (name_obj != NULL && cJSON_IsString(name_obj)) {
            free(scene_name);
            scene_name = strdup(name_obj->valuestring);
        }
        cJSON_Delete(json);
    }

    /* Find and recall scene */
    ret = zb_scenes_recall_by_name(scene_name);

    free(scene_name);
    return ret;
}

esp_err_t zb_scenes_publish_state(uint8_t scene_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const zb_scene_t *scene = zb_scenes_get(scene_id);
    if (scene == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Create state JSON */
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(json, "friendly_name", scene->name);
    cJSON_AddNumberToObject(json, "id", scene->scene_id);
    cJSON_AddNumberToObject(json, "group_id", scene->group_id);
    cJSON_AddNumberToObject(json, "device_count", scene->device_count);
    cJSON_AddNumberToObject(json, "transition_time", scene->transition_time);

    /* Add device list */
    cJSON *devices = cJSON_CreateArray();
    if (devices == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON array for scene devices");
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }
    for (uint8_t i = 0; i < scene->device_count; i++) {
        cJSON *dev_obj = cJSON_CreateObject();
        char ieee_str[20];
        snprintf(ieee_str, sizeof(ieee_str), "0x%016llx",
                 (unsigned long long)scene->devices[i].ieee_addr);
        cJSON_AddStringToObject(dev_obj, "ieee_address", ieee_str);
        cJSON_AddNumberToObject(dev_obj, "endpoint", scene->devices[i].endpoint);
        cJSON_AddBoolToObject(dev_obj, "state", scene->devices[i].on_off_state);
        cJSON_AddNumberToObject(dev_obj, "brightness", scene->devices[i].level);
        if (scene->devices[i].color_x > 0 || scene->devices[i].color_y > 0) {
            cJSON *color = cJSON_CreateObject();
            cJSON_AddNumberToObject(color, "x", scene->devices[i].color_x / 65535.0);
            cJSON_AddNumberToObject(color, "y", scene->devices[i].color_y / 65535.0);
            cJSON_AddItemToObject(dev_obj, "color", color);
        }
        if (scene->devices[i].color_temp > 0) {
            cJSON_AddNumberToObject(dev_obj, "color_temp", scene->devices[i].color_temp);
        }
        cJSON_AddItemToArray(devices, dev_obj);
    }
    cJSON_AddItemToObject(json, "devices", devices);

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Build topic: zigbee2mqtt/bridge/scene/[name] */
    char topic[128];
    snprintf(topic, sizeof(topic), "zigbee2mqtt/bridge/scene/%s", scene->name);

    ESP_LOGI(TAG, "Publishing scene state to %s", topic);
    ESP_LOGD(TAG, "State: %s", json_str);

    /* Publish via MQTT client */
    esp_err_t ret = ESP_OK;
    if (mqtt_client_is_connected()) {
        ret = mqtt_client_publish(topic, json_str, 0, 0, false);
    }

    free(json_str);
    return ret;
}

esp_err_t zb_scenes_publish_list(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Create scenes array JSON */
    cJSON *json = cJSON_CreateArray();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_scenes_mutex, portMAX_DELAY);

    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        if (s_scenes[i].active) {
            cJSON *scene_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(scene_obj, "id", s_scenes[i].scene_id);
            cJSON_AddStringToObject(scene_obj, "friendly_name", s_scenes[i].name);
            cJSON_AddNumberToObject(scene_obj, "group_id", s_scenes[i].group_id);
            cJSON_AddNumberToObject(scene_obj, "device_count", s_scenes[i].device_count);
            cJSON_AddNumberToObject(scene_obj, "transition_time", s_scenes[i].transition_time);

            /* Add device list */
            cJSON *devices = cJSON_CreateArray();
            if (devices == NULL) {
                cJSON_Delete(scene_obj);
                cJSON_Delete(json);
                xSemaphoreGive(s_scenes_mutex);
                return ESP_ERR_NO_MEM;
            }
            for (uint8_t j = 0; j < s_scenes[i].device_count; j++) {
                char ieee_str[20];
                snprintf(ieee_str, sizeof(ieee_str), "0x%016llx",
                         (unsigned long long)s_scenes[i].devices[j].ieee_addr);
                cJSON_AddItemToArray(devices, cJSON_CreateString(ieee_str));
            }
            cJSON_AddItemToObject(scene_obj, "devices", devices);

            cJSON_AddItemToArray(json, scene_obj);
        }
    }

    xSemaphoreGive(s_scenes_mutex);

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Publishing scenes list to %s", TOPIC_BRIDGE_SCENES);
    ESP_LOGD(TAG, "Scenes: %s", json_str);

    /* Publish via MQTT client */
    esp_err_t ret = ESP_OK;
    if (mqtt_client_is_connected()) {
        ret = mqtt_client_publish(TOPIC_BRIDGE_SCENES, json_str, 0, 0, true);
    }

    free(json_str);
    return ret;
}

esp_err_t zb_scenes_register_callback(zb_scene_event_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

esp_err_t zb_scenes_test(void)
{
    ESP_LOGI(TAG, "Running scenes self-test...");

    if (!s_initialized) {
        ESP_LOGE(TAG, "Scenes module not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret;

    /* Test 1: Create scene */
    uint8_t test_scene_id;
    ret = zb_scenes_create(0x0001, "Test_Scene", &test_scene_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create test scene");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 1 PASSED: Created scene with ID %d", test_scene_id);

    /* Test 2: Get scene by name */
    const zb_scene_t *scene = zb_scenes_get_by_name("Test_Scene");
    if (scene == NULL || scene->scene_id != test_scene_id) {
        ESP_LOGE(TAG, "Failed to get scene by name");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 2 PASSED: Retrieved scene by name");

    /* Test 3: Store device state */
    zb_scene_device_state_t test_state = {
        .ieee_addr = 0x00124B001234ABCD,
        .endpoint = 1,
        .on_off_state = true,
        .level = ZB_SCENE_TEST_LEVEL_VALUE,
        .color_x = ZB_COLOR_XY_DEFAULT,
        .color_y = ZB_COLOR_XY_DEFAULT,
        .color_temp = 350,
    };
    ret = zb_scenes_store_device(test_scene_id, test_state.ieee_addr,
                                  test_state.endpoint, &test_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to store device state");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 3 PASSED: Stored device state in scene");

    /* Test 4: Verify device count */
    scene = zb_scenes_get(test_scene_id);
    if (scene == NULL || scene->device_count != 1) {
        ESP_LOGE(TAG, "Device count mismatch");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 4 PASSED: Device count is correct");

    /* Test 5: Rename scene */
    ret = zb_scenes_rename(test_scene_id, "Renamed_Scene");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to rename scene");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 5 PASSED: Renamed scene");

    /* Test 6: Set transition time */
    ret = zb_scenes_set_transition(test_scene_id, ZB_SCENE_TRANSITION_TIME_TEST);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set transition time");
        return ESP_FAIL;
    }
    scene = zb_scenes_get(test_scene_id);
    if (scene == NULL || scene->transition_time != ZB_SCENE_TRANSITION_TIME_TEST) {
        ESP_LOGE(TAG, "Transition time mismatch");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 6 PASSED: Set transition time");

    /* Test 7: Remove scene */
    ret = zb_scenes_remove(test_scene_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove scene");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 7 PASSED: Removed scene");

    /* Test 8: Verify scene removed */
    if (zb_scenes_get(test_scene_id) != NULL) {
        ESP_LOGE(TAG, "Scene still exists after removal");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 8 PASSED: Scene no longer exists");

    ESP_LOGI(TAG, "Scenes self-test PASSED (all 8 tests)");
    return ESP_OK;
}

/* Internal helper functions */

static zb_scene_t* find_scene_by_id(uint8_t scene_id)
{
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        if (s_scenes[i].active && s_scenes[i].scene_id == scene_id) {
            return &s_scenes[i];
        }
    }
    return NULL;
}

static zb_scene_t* find_scene_by_name(const char *name)
{
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        if (s_scenes[i].active && strcmp(s_scenes[i].name, name) == 0) {
            return &s_scenes[i];
        }
    }
    return NULL;
}

static zb_scene_t* find_free_scene_slot(void)
{
    for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
        if (!s_scenes[i].active) {
            return &s_scenes[i];
        }
    }
    return NULL;
}

static uint8_t allocate_scene_id(void)
{
    /* Find lowest available scene ID */
    for (uint8_t id = ZB_SCENE_ID_START; id <= ZB_SCENE_ID_MAX; id++) {
        bool in_use = false;
        for (size_t i = 0; i < ZB_SCENES_MAX_COUNT; i++) {
            if (s_scenes[i].active && s_scenes[i].scene_id == id) {
                in_use = true;
                break;
            }
        }
        if (!in_use) {
            return id;
        }
    }
    return ZB_SCENE_ID_INVALID;
}

static int find_device_index(const zb_scene_t *scene, uint64_t ieee_addr, uint8_t endpoint)
{
    for (uint8_t i = 0; i < scene->device_count; i++) {
        if (scene->devices[i].ieee_addr == ieee_addr &&
            scene->devices[i].endpoint == endpoint) {
            return i;
        }
    }
    return -1;
}

static esp_err_t apply_device_state(const zb_scene_device_state_t *state, uint16_t transition)
{
    ESP_LOGD(TAG, "Applying state to device 0x%016llX:%d",
             (unsigned long long)state->ieee_addr, state->endpoint);

    /* Find device by IEEE address to get short address */
    esp_zb_ieee_addr_t ieee;
    memcpy(ieee, &state->ieee_addr, sizeof(esp_zb_ieee_addr_t));

    zb_device_t *device = zb_device_get_by_ieee(ieee);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device 0x%016llX not found in registry",
                 (unsigned long long)state->ieee_addr);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t ret = ESP_OK;

    /* Send On/Off command */
    esp_zb_zcl_on_off_cmd_t on_off_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u = {
                .addr_short = device->short_addr,
            },
            .dst_endpoint = state->endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = state->on_off_state ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID :
                                               ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    ret = esp_zb_zcl_on_off_cmd_req(&on_off_cmd);
    esp_zb_lock_release();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send On/Off command: %s", esp_err_to_name(ret));
    }

    /* Send Level command if device is on and level is specified */
    if (state->on_off_state && state->level > 0) {
        esp_zb_zcl_move_to_level_cmd_t level_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u = {
                    .addr_short = device->short_addr,
                },
                .dst_endpoint = state->endpoint,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .level = state->level,
            .transition_time = transition,
        };
        esp_zb_lock_acquire(portMAX_DELAY);
        ret = esp_zb_zcl_level_move_to_level_cmd_req(&level_cmd);
        esp_zb_lock_release();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send Level command: %s", esp_err_to_name(ret));
        }
    }

    /* Send Color command if color is specified */
    if (state->color_x > 0 || state->color_y > 0) {
        esp_zb_zcl_color_move_to_color_cmd_t color_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u = {
                    .addr_short = device->short_addr,
                },
                .dst_endpoint = state->endpoint,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .color_x = state->color_x,
            .color_y = state->color_y,
            .transition_time = transition,
        };
        esp_zb_lock_acquire(portMAX_DELAY);
        ret = esp_zb_zcl_color_move_to_color_cmd_req(&color_cmd);
        esp_zb_lock_release();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send Color command: %s", esp_err_to_name(ret));
        }
    }

    /* Send Color Temperature command if specified */
    if (state->color_temp > 0) {
        esp_zb_zcl_color_move_to_color_temperature_cmd_t ct_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u = {
                    .addr_short = device->short_addr,
                },
                .dst_endpoint = state->endpoint,
                .src_endpoint = 1,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .color_temperature = state->color_temp,
            .transition_time = transition,
        };
        esp_zb_lock_acquire(portMAX_DELAY);
        ret = esp_zb_zcl_color_move_to_color_temperature_cmd_req(&ct_cmd);
        esp_zb_lock_release();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send Color Temperature command: %s", esp_err_to_name(ret));
        }
    }

    return ret;
}

static esp_err_t capture_group_device_states(zb_scene_t *scene)
{
    if (scene->group_id == 0) {
        ESP_LOGW(TAG, "Scene has no associated group - cannot capture device states");
        return ESP_ERR_INVALID_STATE;
    }

    /* Get group info */
    const zb_group_t *group = zb_groups_get(scene->group_id);
    if (group == NULL) {
        ESP_LOGW(TAG, "Group 0x%04X not found", scene->group_id);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Capturing states from %d devices in group '%s'",
             group->member_count, group->name);

    scene->device_count = 0;

    /* Iterate through group members */
    for (uint8_t i = 0; i < group->member_count && scene->device_count < ZB_SCENE_MAX_DEVICES; i++) {
        uint64_t ieee_addr = group->members[i];

        /* Find device info */
        esp_zb_ieee_addr_t ieee;
        memcpy(ieee, &ieee_addr, sizeof(esp_zb_ieee_addr_t));
        zb_device_t *device = zb_device_get_by_ieee(ieee);

        if (device != NULL) {
            /* Store device state - in real implementation, would read current state */
            zb_scene_device_state_t *state = &scene->devices[scene->device_count];
            state->ieee_addr = ieee_addr;
            state->endpoint = device->endpoint;

            /* TODO: Read actual device state via ZCL Read Attributes
             * For now, store default values - actual implementation would
             * need to query the device for current On/Off, Level, Color states
             */
            state->on_off_state = true;     /* Placeholder */
            state->level = ZCL_LEVEL_MAX;   /* Placeholder */
            state->color_x = 0;
            state->color_y = 0;
            state->color_temp = 0;

            scene->device_count++;

            ESP_LOGD(TAG, "Captured state for device 0x%016llX:%d",
                     (unsigned long long)ieee_addr, device->endpoint);
        } else {
            ESP_LOGW(TAG, "Device 0x%016llX not found in registry",
                     (unsigned long long)ieee_addr);
        }
    }

    ESP_LOGI(TAG, "Captured %d device states", scene->device_count);
    return ESP_OK;
}

static void notify_event(uint8_t scene_id, zb_scene_event_type_t event_type)
{
    if (s_event_callback != NULL) {
        s_event_callback(scene_id, event_type);
    }
}

static esp_err_t parse_scene_add_request(const char *payload, uint16_t *group_id,
                                          char *name, size_t name_len)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse scene add request JSON");
        return ESP_FAIL;
    }

    /* Get scene name (required) */
    cJSON *name_obj = cJSON_GetObjectItem(json, "friendly_name");
    if (name_obj == NULL) {
        name_obj = cJSON_GetObjectItem(json, "name");
    }

    if (name_obj == NULL || !cJSON_IsString(name_obj)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'friendly_name' or 'name' in request");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(name, name_obj->valuestring, name_len - 1);
    name[name_len - 1] = '\0';

    /* Get group ID (optional) */
    *group_id = 0;
    cJSON *group_obj = cJSON_GetObjectItem(json, "group_id");
    if (group_obj == NULL) {
        group_obj = cJSON_GetObjectItem(json, "group");
    }

    if (group_obj != NULL) {
        if (cJSON_IsNumber(group_obj)) {
            *group_id = (uint16_t)group_obj->valueint;
        } else if (cJSON_IsString(group_obj)) {
            /* Try to find group by name */
            const zb_group_t *group = zb_groups_get_by_name(group_obj->valuestring);
            if (group != NULL) {
                *group_id = group->group_id;
            }
        }
    }

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t parse_scene_remove_request(const char *payload, char *name, size_t name_len)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse scene remove request JSON");
        return ESP_FAIL;
    }

    /* Try "friendly_name" first, then "name", then "id" */
    cJSON *name_obj = cJSON_GetObjectItem(json, "friendly_name");
    if (name_obj == NULL) {
        name_obj = cJSON_GetObjectItem(json, "name");
    }

    if (name_obj != NULL && cJSON_IsString(name_obj)) {
        strncpy(name, name_obj->valuestring, name_len - 1);
        name[name_len - 1] = '\0';
        cJSON_Delete(json);
        return ESP_OK;
    }

    /* Try by ID */
    cJSON *id_obj = cJSON_GetObjectItem(json, "id");
    if (id_obj != NULL && cJSON_IsNumber(id_obj)) {
        uint8_t scene_id = (uint8_t)id_obj->valueint;
        const zb_scene_t *scene = zb_scenes_get(scene_id);
        if (scene != NULL) {
            strncpy(name, scene->name, name_len - 1);
            name[name_len - 1] = '\0';
            cJSON_Delete(json);
            return ESP_OK;
        }
    }

    cJSON_Delete(json);
    ESP_LOGE(TAG, "Missing 'friendly_name', 'name', or 'id' in request");
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t parse_scene_store_request(const char *payload, char *name, size_t name_len)
{
    /* Same format as remove request */
    return parse_scene_remove_request(payload, name, name_len);
}

static esp_err_t parse_scene_recall_request(const char *payload, char *name, size_t name_len,
                                             uint16_t *transition)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse scene recall request JSON");
        return ESP_FAIL;
    }

    /* Get scene name or ID */
    cJSON *name_obj = cJSON_GetObjectItem(json, "friendly_name");
    if (name_obj == NULL) {
        name_obj = cJSON_GetObjectItem(json, "name");
    }
    if (name_obj == NULL) {
        name_obj = cJSON_GetObjectItem(json, "scene");
    }

    if (name_obj != NULL && cJSON_IsString(name_obj)) {
        strncpy(name, name_obj->valuestring, name_len - 1);
        name[name_len - 1] = '\0';
    } else {
        /* Try by ID */
        cJSON *id_obj = cJSON_GetObjectItem(json, "id");
        if (id_obj != NULL && cJSON_IsNumber(id_obj)) {
            uint8_t scene_id = (uint8_t)id_obj->valueint;
            const zb_scene_t *scene = zb_scenes_get(scene_id);
            if (scene != NULL) {
                strncpy(name, scene->name, name_len - 1);
                name[name_len - 1] = '\0';
            } else {
                cJSON_Delete(json);
                return ESP_ERR_NOT_FOUND;
            }
        } else {
            cJSON_Delete(json);
            ESP_LOGE(TAG, "Missing scene identifier in request");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* Get optional transition time */
    *transition = 0;
    cJSON *trans_obj = cJSON_GetObjectItem(json, "transition");
    if (trans_obj != NULL && cJSON_IsNumber(trans_obj)) {
        /* Convert from seconds to 1/10 seconds */
        *transition = (uint16_t)(trans_obj->valuedouble * 10);
    }

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t extract_group_name_from_topic(const char *topic, char *name, size_t name_len)
{
    /* Topic format: zigbee2mqtt/[group_name]/scene_recall */
    const char *prefix = "zigbee2mqtt/";
    const char *suffix = TOPIC_SCENE_RECALL_SUFFIX;

    if (strncmp(topic, prefix, strlen(prefix)) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *name_start = topic + strlen(prefix);
    const char *name_end = strstr(name_start, suffix);

    if (name_end == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t name_length = name_end - name_start;
    if (name_length >= name_len) {
        return ESP_ERR_NO_MEM;
    }

    strncpy(name, name_start, name_length);
    name[name_length] = '\0';

    return ESP_OK;
}
