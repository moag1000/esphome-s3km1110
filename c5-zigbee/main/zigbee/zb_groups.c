/**
 * @file zb_groups.c
 * @brief Zigbee Group Management Implementation
 *
 * Implements Zigbee group management including creation, member management,
 * group commands, NVS persistence, and MQTT integration.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

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

static const char *TAG = "ZB_GROUPS";

/* Module state */
static zb_group_t *s_groups = NULL;
static size_t s_group_count = 0;
static bool s_initialized = false;
static SemaphoreHandle_t s_groups_mutex = NULL;
static zb_group_event_cb_t s_event_callback = NULL;

/* NVS keys */
#define NVS_KEY_GROUP_COUNT "grp_count"
#define NVS_KEY_GROUP_PREFIX "grp_"
#define NVS_KEY_GROUP_DATA_FMT "grp_%02d"

/* MQTT topic patterns */
#define TOPIC_GROUP_ADD "zigbee2mqtt/bridge/request/group/add"
#define TOPIC_GROUP_REMOVE "zigbee2mqtt/bridge/request/group/remove"
#define TOPIC_GROUP_MEMBERS_ADD "zigbee2mqtt/bridge/request/group/members/add"
#define TOPIC_GROUP_MEMBERS_REMOVE "zigbee2mqtt/bridge/request/group/members/remove"
#define TOPIC_GROUP_SET_PREFIX "zigbee2mqtt/group/"
#define TOPIC_GROUP_SET_SUFFIX "/set"

/* Forward declarations */
static zb_group_t* find_group_by_id(uint16_t group_id);
static zb_group_t* find_group_by_name(const char *name);
static zb_group_t* find_free_group_slot(void);
static uint16_t allocate_group_id(void);
static int find_member_index(const zb_group_t *group, uint64_t ieee_addr);
static esp_err_t send_zcl_add_group(uint16_t group_id, uint64_t ieee_addr);
static esp_err_t send_zcl_remove_group(uint16_t group_id, uint64_t ieee_addr);
static void notify_event(uint16_t group_id, const char *event_type);
static esp_err_t parse_group_add_request(const char *payload, char *name, size_t name_len);
static esp_err_t parse_group_remove_request(const char *payload, char *name, size_t name_len);
static esp_err_t parse_group_member_request(const char *payload, char *group_name,
                                            size_t name_len, char *device_id, size_t device_len);
static esp_err_t extract_group_name_from_topic(const char *topic, char *name, size_t name_len);

esp_err_t zb_groups_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Groups module already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing Zigbee groups management...");

    /* Create mutex */
    s_groups_mutex = xSemaphoreCreateMutex();
    if (s_groups_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create groups mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Allocate group storage - prefer PSRAM to free internal RAM for WiFi */
    s_groups = heap_caps_calloc(ZB_GROUPS_MAX_COUNT, sizeof(zb_group_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_groups == NULL) {
        /* Fallback to internal RAM if PSRAM not available */
        s_groups = calloc(ZB_GROUPS_MAX_COUNT, sizeof(zb_group_t));
    }
    if (s_groups == NULL) {
        ESP_LOGE(TAG, "Failed to allocate group storage");
        vSemaphoreDelete(s_groups_mutex);
        s_groups_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Group storage allocated: %zu bytes",
             ZB_GROUPS_MAX_COUNT * sizeof(zb_group_t));

    /* Initialize all groups as inactive */
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        s_groups[i].active = false;
        s_groups[i].group_id = ZB_GROUP_ID_INVALID;
        s_groups[i].member_count = 0;
    }

    s_group_count = 0;
    s_initialized = true;

    /* Try to load groups from NVS */
    esp_err_t ret = zb_groups_load_from_nvs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded %d groups from NVS", s_group_count);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved groups found in NVS");
    } else {
        ESP_LOGW(TAG, "Failed to load groups from NVS: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Groups management initialized (max: %d groups, %d members each)",
             ZB_GROUPS_MAX_COUNT, ZB_GROUP_MAX_MEMBERS);
    return ESP_OK;
}

esp_err_t zb_groups_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing Zigbee groups management...");

    /* Save groups before shutdown */
    zb_groups_save_to_nvs();

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    if (s_groups != NULL) {
        free(s_groups);
        s_groups = NULL;
    }

    s_group_count = 0;
    s_initialized = false;
    s_event_callback = NULL;

    xSemaphoreGive(s_groups_mutex);
    vSemaphoreDelete(s_groups_mutex);
    s_groups_mutex = NULL;

    ESP_LOGI(TAG, "Groups management deinitialized");
    return ESP_OK;
}

esp_err_t zb_groups_create(const char *name, uint16_t *group_id)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Groups module not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (name == NULL || strlen(name) == 0) {
        ESP_LOGE(TAG, "Invalid group name");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(name) >= ZB_GROUP_NAME_MAX_LEN) {
        ESP_LOGE(TAG, "Group name too long: %d chars (max %d)",
                 strlen(name), ZB_GROUP_NAME_MAX_LEN - 1);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    /* Check if name already exists */
    if (find_group_by_name(name) != NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group with name '%s' already exists", name);
        return ESP_ERR_INVALID_ARG;
    }

    /* Find free slot */
    zb_group_t *group = find_free_group_slot();
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGE(TAG, "Maximum number of groups reached (%d)", ZB_GROUPS_MAX_COUNT);
        return ESP_ERR_NO_MEM;
    }

    /* Allocate group ID */
    uint16_t new_id = allocate_group_id();
    if (new_id == ZB_GROUP_ID_INVALID) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGE(TAG, "Failed to allocate group ID");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize group */
    group->group_id = new_id;
    strncpy(group->name, name, ZB_GROUP_NAME_MAX_LEN - 1);
    group->name[ZB_GROUP_NAME_MAX_LEN - 1] = '\0';
    group->member_count = 0;
    memset(group->members, 0, sizeof(group->members));
    group->active = true;

    s_group_count++;

    if (group_id != NULL) {
        *group_id = new_id;
    }

    xSemaphoreGive(s_groups_mutex);

    ESP_LOGI(TAG, "Created group '%s' with ID 0x%04X (total: %d)",
             name, new_id, s_group_count);

    /* Save to NVS */
    zb_groups_save_to_nvs();

    /* Notify event */
    notify_event(new_id, "group_added");

    return ESP_OK;
}

esp_err_t zb_groups_remove(uint16_t group_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group 0x%04X not found", group_id);
        return ESP_ERR_NOT_FOUND;
    }

    char group_name[ZB_GROUP_NAME_MAX_LEN];
    strncpy(group_name, group->name, ZB_GROUP_NAME_MAX_LEN);

    /* Remove all members from the Zigbee group */
    for (uint8_t i = 0; i < group->member_count; i++) {
        send_zcl_remove_group(group_id, group->members[i]);
    }

    /* Mark group as inactive */
    group->active = false;
    group->group_id = ZB_GROUP_ID_INVALID;
    group->member_count = 0;
    memset(group->name, 0, ZB_GROUP_NAME_MAX_LEN);
    memset(group->members, 0, sizeof(group->members));

    s_group_count--;

    xSemaphoreGive(s_groups_mutex);

    ESP_LOGI(TAG, "Removed group '%s' (ID: 0x%04X, remaining: %d)",
             group_name, group_id, s_group_count);

    /* Save to NVS */
    zb_groups_save_to_nvs();

    /* Notify event */
    notify_event(group_id, "group_removed");

    return ESP_OK;
}

esp_err_t zb_groups_remove_by_name(const char *name)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_name(name);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group '%s' not found", name);
        return ESP_ERR_NOT_FOUND;
    }

    uint16_t group_id = group->group_id;
    xSemaphoreGive(s_groups_mutex);

    return zb_groups_remove(group_id);
}

esp_err_t zb_groups_add_member(uint16_t group_id, uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group 0x%04X not found", group_id);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if already a member */
    if (find_member_index(group, ieee_addr) >= 0) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Device 0x%016llX already in group 0x%04X",
                 (unsigned long long)ieee_addr, group_id);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check if group is full */
    if (group->member_count >= ZB_GROUP_MAX_MEMBERS) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGE(TAG, "Group 0x%04X is full (%d members)", group_id, ZB_GROUP_MAX_MEMBERS);
        return ESP_ERR_NO_MEM;
    }

    /* Add member */
    group->members[group->member_count] = ieee_addr;
    group->member_count++;

    xSemaphoreGive(s_groups_mutex);

    /* Send ZCL Add Group command to device */
    esp_err_t ret = send_zcl_add_group(group_id, ieee_addr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Add Group to device: %s", esp_err_to_name(ret));
        /* Continue anyway - device might be offline */
    }

    ESP_LOGI(TAG, "Added device 0x%016llX to group 0x%04X (%d members)",
             (unsigned long long)ieee_addr, group_id, group->member_count);

    /* Save to NVS */
    zb_groups_save_to_nvs();

    /* Notify event */
    notify_event(group_id, "member_added");

    /* Publish bridge event: group_member_added */
    bridge_event_group_member_added(group->name, group_id, ieee_addr, NULL);

    return ESP_OK;
}

esp_err_t zb_groups_remove_member(uint16_t group_id, uint64_t ieee_addr)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group 0x%04X not found", group_id);
        return ESP_ERR_NOT_FOUND;
    }

    int member_idx = find_member_index(group, ieee_addr);
    if (member_idx < 0) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Device 0x%016llX not in group 0x%04X",
                 (unsigned long long)ieee_addr, group_id);
        return ESP_ERR_NOT_FOUND;
    }

    /* Remove member by shifting array */
    for (int i = member_idx; i < group->member_count - 1; i++) {
        group->members[i] = group->members[i + 1];
    }
    group->members[group->member_count - 1] = 0;
    group->member_count--;

    xSemaphoreGive(s_groups_mutex);

    /* Send ZCL Remove Group command to device */
    esp_err_t ret = send_zcl_remove_group(group_id, ieee_addr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send Remove Group to device: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Removed device 0x%016llX from group 0x%04X (%d members)",
             (unsigned long long)ieee_addr, group_id, group->member_count);

    /* Save to NVS */
    zb_groups_save_to_nvs();

    /* Notify event */
    notify_event(group_id, "member_removed");

    /* Publish bridge event: group_member_removed */
    bridge_event_group_member_removed(group->name, group_id, ieee_addr, NULL);

    return ESP_OK;
}

esp_err_t zb_groups_send_command(uint16_t group_id, const zb_group_cmd_t *cmd)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group 0x%04X not found", group_id);
        return ESP_ERR_NOT_FOUND;
    }

    xSemaphoreGive(s_groups_mutex);

    ESP_LOGI(TAG, "Sending command type %d to group 0x%04X", cmd->cmd_type, group_id);

    esp_err_t ret = ESP_OK;

    switch (cmd->cmd_type) {
        case ZB_GROUP_CMD_ON: {
            esp_zb_zcl_on_off_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF, /* Broadcast to all endpoints */
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        case ZB_GROUP_CMD_OFF: {
            esp_zb_zcl_on_off_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF,
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        case ZB_GROUP_CMD_TOGGLE: {
            esp_zb_zcl_on_off_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF,
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_on_off_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        case ZB_GROUP_CMD_LEVEL: {
            esp_zb_zcl_move_to_level_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF,
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .level = cmd->params.level.level,
                .transition_time = cmd->params.level.transition,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_level_move_to_level_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        case ZB_GROUP_CMD_COLOR: {
            esp_zb_zcl_color_move_to_color_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF,
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .color_x = cmd->params.color.color_x,
                .color_y = cmd->params.color.color_y,
                .transition_time = cmd->params.color.transition,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_color_move_to_color_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        case ZB_GROUP_CMD_COLOR_TEMP: {
            esp_zb_zcl_color_move_to_color_temperature_cmd_t cmd_req = {
                .zcl_basic_cmd = {
                    .dst_addr_u = {
                        .addr_short = group_id,
                    },
                    .dst_endpoint = 0xFF,
                    .src_endpoint = 1,
                },
                .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                .color_temperature = cmd->params.color_temp.color_temp,
                .transition_time = cmd->params.color_temp.transition,
            };
            esp_zb_lock_acquire(portMAX_DELAY);
            ret = esp_zb_zcl_color_move_to_color_temperature_cmd_req(&cmd_req);
            esp_zb_lock_release();
            break;
        }

        default:
            ESP_LOGW(TAG, "Unsupported command type: %d", cmd->cmd_type);
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send group command: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_groups_send_on_off(uint16_t group_id, bool on)
{
    zb_group_cmd_t cmd = {
        .cmd_type = on ? ZB_GROUP_CMD_ON : ZB_GROUP_CMD_OFF,
    };
    return zb_groups_send_command(group_id, &cmd);
}

esp_err_t zb_groups_send_toggle(uint16_t group_id)
{
    zb_group_cmd_t cmd = {
        .cmd_type = ZB_GROUP_CMD_TOGGLE,
    };
    return zb_groups_send_command(group_id, &cmd);
}

esp_err_t zb_groups_send_level(uint16_t group_id, uint8_t level, uint16_t transition_time)
{
    zb_group_cmd_t cmd = {
        .cmd_type = ZB_GROUP_CMD_LEVEL,
        .params.level = {
            .level = level,
            .transition = transition_time,
        },
    };
    return zb_groups_send_command(group_id, &cmd);
}

const zb_group_t* zb_groups_get(uint16_t group_id)
{
    if (!s_initialized) {
        return NULL;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);
    zb_group_t *group = find_group_by_id(group_id);
    xSemaphoreGive(s_groups_mutex);

    return group;
}

const zb_group_t* zb_groups_get_by_name(const char *name)
{
    if (!s_initialized || name == NULL) {
        return NULL;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);
    zb_group_t *group = find_group_by_name(name);
    xSemaphoreGive(s_groups_mutex);

    return group;
}

size_t zb_groups_get_all(zb_group_t *groups, size_t max_count)
{
    if (!s_initialized || groups == NULL) {
        return 0;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    size_t copied = 0;
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT && copied < max_count; i++) {
        if (s_groups[i].active) {
            memcpy(&groups[copied], &s_groups[i], sizeof(zb_group_t));
            copied++;
        }
    }

    xSemaphoreGive(s_groups_mutex);

    return copied;
}

size_t zb_groups_get_count(void)
{
    if (!s_initialized) {
        return 0;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);
    size_t count = s_group_count;
    xSemaphoreGive(s_groups_mutex);

    return count;
}

bool zb_groups_is_member(uint16_t group_id, uint64_t ieee_addr)
{
    if (!s_initialized) {
        return false;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    bool is_member = (group != NULL) && (find_member_index(group, ieee_addr) >= 0);

    xSemaphoreGive(s_groups_mutex);

    return is_member;
}

size_t zb_groups_get_device_groups(uint64_t ieee_addr, uint16_t *group_ids, size_t max_count)
{
    if (!s_initialized || group_ids == NULL) {
        return 0;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    size_t count = 0;
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT && count < max_count; i++) {
        if (s_groups[i].active && find_member_index(&s_groups[i], ieee_addr) >= 0) {
            group_ids[count++] = s_groups[i].group_id;
        }
    }

    xSemaphoreGive(s_groups_mutex);

    return count;
}

esp_err_t zb_groups_rename(uint16_t group_id, const char *new_name)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (new_name == NULL || strlen(new_name) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(new_name) >= ZB_GROUP_NAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    zb_group_t *group = find_group_by_id(group_id);
    if (group == NULL) {
        xSemaphoreGive(s_groups_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Check if new name already exists */
    zb_group_t *existing = find_group_by_name(new_name);
    if (existing != NULL && existing != group) {
        xSemaphoreGive(s_groups_mutex);
        ESP_LOGW(TAG, "Group with name '%s' already exists", new_name);
        return ESP_ERR_INVALID_ARG;
    }

    char old_name[ZB_GROUP_NAME_MAX_LEN];
    strncpy(old_name, group->name, ZB_GROUP_NAME_MAX_LEN);

    strncpy(group->name, new_name, ZB_GROUP_NAME_MAX_LEN - 1);
    group->name[ZB_GROUP_NAME_MAX_LEN - 1] = '\0';

    xSemaphoreGive(s_groups_mutex);

    ESP_LOGI(TAG, "Renamed group 0x%04X from '%s' to '%s'", group_id, old_name, new_name);

    /* Save to NVS */
    zb_groups_save_to_nvs();

    /* Notify event */
    notify_event(group_id, "group_renamed");

    return ESP_OK;
}

esp_err_t zb_groups_save_to_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving groups to NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_GROUPS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    /* Save group count */
    ret = nvs_set_u16(nvs_handle, NVS_KEY_GROUP_COUNT, (uint16_t)s_group_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save group count: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_groups_mutex);
        nvs_close(nvs_handle);
        return ret;
    }

    /* Save each active group */
    uint8_t saved_idx = 0;
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        if (s_groups[i].active) {
            char key[16];
            snprintf(key, sizeof(key), NVS_KEY_GROUP_DATA_FMT, saved_idx);

            ret = nvs_set_blob(nvs_handle, key, &s_groups[i], sizeof(zb_group_t));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save group %d: %s", i, esp_err_to_name(ret));
                xSemaphoreGive(s_groups_mutex);
                nvs_close(nvs_handle);
                return ret;
            }
            saved_idx++;
        }
    }

    xSemaphoreGive(s_groups_mutex);

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %d groups to NVS", s_group_count);
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t zb_groups_load_from_nvs(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Loading groups from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_GROUPS_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found - no saved groups");
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Load group count */
    uint16_t count = 0;
    ret = nvs_get_u16(nvs_handle, NVS_KEY_GROUP_COUNT, &count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load group count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    /* Clear existing groups */
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        s_groups[i].active = false;
        s_groups[i].group_id = ZB_GROUP_ID_INVALID;
    }
    s_group_count = 0;

    /* Load each group */
    for (uint8_t i = 0; i < count && i < ZB_GROUPS_MAX_COUNT; i++) {
        char key[16];
        snprintf(key, sizeof(key), NVS_KEY_GROUP_DATA_FMT, i);

        size_t required_size = sizeof(zb_group_t);
        ret = nvs_get_blob(nvs_handle, key, &s_groups[i], &required_size);
        if (ret == ESP_OK) {
            s_groups[i].active = true;
            s_group_count++;
            ESP_LOGD(TAG, "Loaded group '%s' (ID: 0x%04X, %d members)",
                     s_groups[i].name, s_groups[i].group_id, s_groups[i].member_count);
        } else {
            ESP_LOGW(TAG, "Failed to load group %d: %s", i, esp_err_to_name(ret));
        }
    }

    xSemaphoreGive(s_groups_mutex);
    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Loaded %d groups from NVS", s_group_count);
    return ESP_OK;
}

esp_err_t zb_groups_clear_nvs(void)
{
    ESP_LOGI(TAG, "Clearing groups from NVS...");

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(ZB_GROUPS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_all(nvs_handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Cleared all groups from NVS");
    }

    return ret;
}

esp_err_t zb_groups_process_mqtt_request(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Processing group request: %s", topic);
    ESP_LOGD(TAG, "Payload: %.*s", (int)len, payload);

    /* Null-terminate payload for parsing */
    char *payload_str = malloc(len + 1);
    if (payload_str == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(payload_str, payload, len);
    payload_str[len] = '\0';

    esp_err_t ret = ESP_FAIL;

    if (strcmp(topic, TOPIC_GROUP_ADD) == 0) {
        /* Create group */
        char name[ZB_GROUP_NAME_MAX_LEN];
        ret = parse_group_add_request(payload_str, name, sizeof(name));
        if (ret == ESP_OK) {
            uint16_t group_id;
            ret = zb_groups_create(name, &group_id);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Created group '%s' via MQTT", name);
                zb_groups_publish_list();
            }
        }
    } else if (strcmp(topic, TOPIC_GROUP_REMOVE) == 0) {
        /* Remove group */
        char name[ZB_GROUP_NAME_MAX_LEN];
        ret = parse_group_remove_request(payload_str, name, sizeof(name));
        if (ret == ESP_OK) {
            ret = zb_groups_remove_by_name(name);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Removed group '%s' via MQTT", name);
                zb_groups_publish_list();
            }
        }
    } else if (strcmp(topic, TOPIC_GROUP_MEMBERS_ADD) == 0) {
        /* Add member to group */
        char group_name[ZB_GROUP_NAME_MAX_LEN];
        char device_id[32];
        ret = parse_group_member_request(payload_str, group_name, sizeof(group_name),
                                         device_id, sizeof(device_id));
        if (ret == ESP_OK) {
            const zb_group_t *group = zb_groups_get_by_name(group_name);
            if (group != NULL) {
                /* Parse IEEE address from device_id */
                uint64_t ieee_addr = strtoull(device_id, NULL, 16);
                ret = zb_groups_add_member(group->group_id, ieee_addr);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Added member to group '%s' via MQTT", group_name);
                    zb_groups_publish_list();
                }
            } else {
                ret = ESP_ERR_NOT_FOUND;
            }
        }
    } else if (strcmp(topic, TOPIC_GROUP_MEMBERS_REMOVE) == 0) {
        /* Remove member from group */
        char group_name[ZB_GROUP_NAME_MAX_LEN];
        char device_id[32];
        ret = parse_group_member_request(payload_str, group_name, sizeof(group_name),
                                         device_id, sizeof(device_id));
        if (ret == ESP_OK) {
            const zb_group_t *group = zb_groups_get_by_name(group_name);
            if (group != NULL) {
                uint64_t ieee_addr = strtoull(device_id, NULL, 16);
                ret = zb_groups_remove_member(group->group_id, ieee_addr);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Removed member from group '%s' via MQTT", group_name);
                    zb_groups_publish_list();
                }
            } else {
                ret = ESP_ERR_NOT_FOUND;
            }
        }
    } else {
        ESP_LOGW(TAG, "Unknown group request topic: %s", topic);
        ret = ESP_ERR_INVALID_ARG;
    }

    free(payload_str);
    return ret;
}

esp_err_t zb_groups_process_mqtt_command(const char *topic, const char *payload, size_t len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == NULL || payload == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Extract group name from topic: zigbee2mqtt/group/[name]/set */
    char group_name[ZB_GROUP_NAME_MAX_LEN];
    esp_err_t ret = extract_group_name_from_topic(topic, group_name, sizeof(group_name));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to extract group name from topic: %s", topic);
        return ret;
    }

    ESP_LOGI(TAG, "Processing group command for '%s'", group_name);
    ESP_LOGD(TAG, "Payload: %.*s", (int)len, payload);

    const zb_group_t *group = zb_groups_get_by_name(group_name);
    if (group == NULL) {
        ESP_LOGW(TAG, "Group '%s' not found", group_name);
        return ESP_ERR_NOT_FOUND;
    }

    /* Null-terminate payload for parsing */
    char *payload_str = malloc(len + 1);
    if (payload_str == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(payload_str, payload, len);
    payload_str[len] = '\0';

    /* Parse JSON command */
    cJSON *json = cJSON_Parse(payload_str);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON payload");
        free(payload_str);
        return ESP_FAIL;
    }

    /* Check for state command */
    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (state != NULL && cJSON_IsString(state)) {
        if (strcasecmp(state->valuestring, "ON") == 0) {
            ret = zb_groups_send_on_off(group->group_id, true);
        } else if (strcasecmp(state->valuestring, "OFF") == 0) {
            ret = zb_groups_send_on_off(group->group_id, false);
        } else if (strcasecmp(state->valuestring, "TOGGLE") == 0) {
            ret = zb_groups_send_toggle(group->group_id);
        }
    }

    /* Check for brightness command */
    cJSON *brightness = cJSON_GetObjectItem(json, "brightness");
    if (brightness != NULL && cJSON_IsNumber(brightness)) {
        uint16_t transition = 0;
        cJSON *trans = cJSON_GetObjectItem(json, "transition");
        if (trans != NULL && cJSON_IsNumber(trans)) {
            transition = (uint16_t)(trans->valuedouble * 10); /* Convert to 1/10 sec */
        }
        ret = zb_groups_send_level(group->group_id, (uint8_t)brightness->valueint, transition);
    }

    /* Check for color command */
    cJSON *color = cJSON_GetObjectItem(json, "color");
    if (color != NULL && cJSON_IsObject(color)) {
        cJSON *x = cJSON_GetObjectItem(color, "x");
        cJSON *y = cJSON_GetObjectItem(color, "y");
        if (x != NULL && y != NULL && cJSON_IsNumber(x) && cJSON_IsNumber(y)) {
            uint16_t transition = 0;
            cJSON *trans = cJSON_GetObjectItem(json, "transition");
            if (trans != NULL && cJSON_IsNumber(trans)) {
                transition = (uint16_t)(trans->valuedouble * 10);
            }
            zb_group_cmd_t cmd = {
                .cmd_type = ZB_GROUP_CMD_COLOR,
                .params.color = {
                    .color_x = (uint16_t)(x->valuedouble * 65535),
                    .color_y = (uint16_t)(y->valuedouble * 65535),
                    .transition = transition,
                },
            };
            ret = zb_groups_send_command(group->group_id, &cmd);
        }
    }

    /* Check for color_temp command */
    cJSON *color_temp = cJSON_GetObjectItem(json, "color_temp");
    if (color_temp != NULL && cJSON_IsNumber(color_temp)) {
        uint16_t transition = 0;
        cJSON *trans = cJSON_GetObjectItem(json, "transition");
        if (trans != NULL && cJSON_IsNumber(trans)) {
            transition = (uint16_t)(trans->valuedouble * 10);
        }
        zb_group_cmd_t cmd = {
            .cmd_type = ZB_GROUP_CMD_COLOR_TEMP,
            .params.color_temp = {
                .color_temp = (uint16_t)color_temp->valueint,
                .transition = transition,
            },
        };
        ret = zb_groups_send_command(group->group_id, &cmd);
    }

    cJSON_Delete(json);
    free(payload_str);

    /* Publish updated state */
    zb_groups_publish_state(group->group_id);

    return ret;
}

esp_err_t zb_groups_publish_state(uint16_t group_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const zb_group_t *group = zb_groups_get(group_id);
    if (group == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Create state JSON */
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(json, "friendly_name", group->name);
    cJSON_AddNumberToObject(json, "id", group->group_id);
    cJSON_AddNumberToObject(json, "member_count", group->member_count);

    /* Add member list */
    cJSON *members = cJSON_CreateArray();
    if (members == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON array for group members");
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }
    for (uint8_t i = 0; i < group->member_count; i++) {
        char ieee_str[20];
        snprintf(ieee_str, sizeof(ieee_str), "0x%016llx", (unsigned long long)group->members[i]);
        cJSON_AddItemToArray(members, cJSON_CreateString(ieee_str));
    }
    cJSON_AddItemToObject(json, "members", members);

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Build topic: zigbee2mqtt/group/[name] */
    char topic[128];
    snprintf(topic, sizeof(topic), "zigbee2mqtt/group/%s", group->name);

    ESP_LOGI(TAG, "Publishing group state to %s", topic);
    ESP_LOGD(TAG, "State: %s", json_str);

    /* TODO: Actually publish via MQTT client
     * This would require integration with mqtt_client module
     * For now, just log the message
     */

    free(json_str);
    return ESP_OK;
}

esp_err_t zb_groups_publish_list(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Create groups array JSON */
    cJSON *json = cJSON_CreateArray();
    if (json == NULL) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_groups_mutex, portMAX_DELAY);

    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        if (s_groups[i].active) {
            cJSON *group_obj = cJSON_CreateObject();
            if (group_obj == NULL) {
                continue;  /* Skip this entry on allocation failure */
            }
            cJSON_AddNumberToObject(group_obj, "id", s_groups[i].group_id);
            cJSON_AddStringToObject(group_obj, "friendly_name", s_groups[i].name);
            cJSON_AddNumberToObject(group_obj, "members", s_groups[i].member_count);

            /* Add member list */
            cJSON *members = cJSON_CreateArray();
            if (members == NULL) {
                cJSON_Delete(group_obj);
                cJSON_Delete(json);
                xSemaphoreGive(s_groups_mutex);
                return ESP_ERR_NO_MEM;
            }
            for (uint8_t j = 0; j < s_groups[i].member_count; j++) {
                char ieee_str[20];
                snprintf(ieee_str, sizeof(ieee_str), "0x%016llx",
                         (unsigned long long)s_groups[i].members[j]);
                cJSON_AddItemToArray(members, cJSON_CreateString(ieee_str));
            }
            cJSON_AddItemToObject(group_obj, "member_list", members);

            cJSON_AddItemToArray(json, group_obj);
        }
    }

    xSemaphoreGive(s_groups_mutex);

    char *json_str = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (json_str == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Publishing groups list to %s", TOPIC_BRIDGE_GROUPS);
    ESP_LOGD(TAG, "Groups: %s", json_str);

    /* TODO: Actually publish via MQTT client */

    free(json_str);
    return ESP_OK;
}

esp_err_t zb_groups_register_callback(zb_group_event_cb_t callback)
{
    s_event_callback = callback;
    return ESP_OK;
}

esp_err_t zb_groups_test(void)
{
    ESP_LOGI(TAG, "Running groups self-test...");

    if (!s_initialized) {
        ESP_LOGE(TAG, "Groups module not initialized");
        return ESP_FAIL;
    }

    esp_err_t ret;

    /* Test 1: Create group */
    uint16_t test_group_id;
    ret = zb_groups_create("Test_Group", &test_group_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create test group");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 1 PASSED: Created group with ID 0x%04X", test_group_id);

    /* Test 2: Get group by name */
    const zb_group_t *group = zb_groups_get_by_name("Test_Group");
    if (group == NULL || group->group_id != test_group_id) {
        ESP_LOGE(TAG, "Failed to get group by name");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 2 PASSED: Retrieved group by name");

    /* Test 3: Add member */
    uint64_t test_ieee = 0x00124B001234ABCD;
    ret = zb_groups_add_member(test_group_id, test_ieee);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add member");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 3 PASSED: Added member to group");

    /* Test 4: Check membership */
    if (!zb_groups_is_member(test_group_id, test_ieee)) {
        ESP_LOGE(TAG, "Member not found in group");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 4 PASSED: Member found in group");

    /* Test 5: Remove member */
    ret = zb_groups_remove_member(test_group_id, test_ieee);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove member");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 5 PASSED: Removed member from group");

    /* Test 6: Remove group */
    ret = zb_groups_remove(test_group_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove group");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 6 PASSED: Removed group");

    /* Test 7: Verify group removed */
    if (zb_groups_get(test_group_id) != NULL) {
        ESP_LOGE(TAG, "Group still exists after removal");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Test 7 PASSED: Group no longer exists");

    ESP_LOGI(TAG, "Groups self-test PASSED (all 7 tests)");
    return ESP_OK;
}

/* Internal helper functions */

static zb_group_t* find_group_by_id(uint16_t group_id)
{
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        if (s_groups[i].active && s_groups[i].group_id == group_id) {
            return &s_groups[i];
        }
    }
    return NULL;
}

static zb_group_t* find_group_by_name(const char *name)
{
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        if (s_groups[i].active && strcmp(s_groups[i].name, name) == 0) {
            return &s_groups[i];
        }
    }
    return NULL;
}

static zb_group_t* find_free_group_slot(void)
{
    for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
        if (!s_groups[i].active) {
            return &s_groups[i];
        }
    }
    return NULL;
}

static uint16_t allocate_group_id(void)
{
    /* Find lowest available group ID */
    for (uint16_t id = ZB_GROUP_ID_START; id <= ZB_GROUP_ID_MAX; id++) {
        bool in_use = false;
        for (size_t i = 0; i < ZB_GROUPS_MAX_COUNT; i++) {
            if (s_groups[i].active && s_groups[i].group_id == id) {
                in_use = true;
                break;
            }
        }
        if (!in_use) {
            return id;
        }
    }
    return ZB_GROUP_ID_INVALID;
}

static int find_member_index(const zb_group_t *group, uint64_t ieee_addr)
{
    for (uint8_t i = 0; i < group->member_count; i++) {
        if (group->members[i] == ieee_addr) {
            return i;
        }
    }
    return -1;
}

static esp_err_t send_zcl_add_group(uint16_t group_id, uint64_t ieee_addr)
{
    ESP_LOGD(TAG, "Sending Add Group (0x%04X) to device 0x%016llX",
             group_id, (unsigned long long)ieee_addr);

    /* Find device by IEEE address to get short address */
    esp_zb_ieee_addr_t ieee;
    memcpy(ieee, &ieee_addr, sizeof(esp_zb_ieee_addr_t));

    zb_device_t *device = zb_device_get_by_ieee(ieee);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device not found in registry");
        return ESP_ERR_NOT_FOUND;
    }

    /* Create Add Group command */
    esp_zb_zcl_groups_add_group_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = device->short_addr,
            .dst_endpoint = device->endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .group_id = group_id,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_groups_add_group_cmd_req(&cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Add Group command: %s", esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t send_zcl_remove_group(uint16_t group_id, uint64_t ieee_addr)
{
    ESP_LOGD(TAG, "Sending Remove Group (0x%04X) to device 0x%016llX",
             group_id, (unsigned long long)ieee_addr);

    /* Find device by IEEE address to get short address */
    esp_zb_ieee_addr_t ieee;
    memcpy(ieee, &ieee_addr, sizeof(esp_zb_ieee_addr_t));

    zb_device_t *device = zb_device_get_by_ieee(ieee);
    if (device == NULL) {
        ESP_LOGW(TAG, "Device not found in registry");
        return ESP_ERR_NOT_FOUND;
    }

    /* Create Remove Group command - uses same struct type as Add Group */
    esp_zb_zcl_groups_add_group_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = device->short_addr,
            .dst_endpoint = device->endpoint,
            .src_endpoint = 1,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .group_id = group_id,
    };

    /* Thread-safety: Acquire Zigbee lock before API call */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_groups_remove_group_cmd_req(&cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Remove Group command: %s", esp_err_to_name(ret));
    }

    return ret;
}

static void notify_event(uint16_t group_id, const char *event_type)
{
    if (s_event_callback != NULL) {
        s_event_callback(group_id, event_type);
    }
}

static esp_err_t parse_group_add_request(const char *payload, char *name, size_t name_len)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse group add request JSON");
        return ESP_FAIL;
    }

    /* Try "friendly_name" first, then "name" */
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

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t parse_group_remove_request(const char *payload, char *name, size_t name_len)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse group remove request JSON");
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
        uint16_t group_id = (uint16_t)id_obj->valueint;
        const zb_group_t *group = zb_groups_get(group_id);
        if (group != NULL) {
            strncpy(name, group->name, name_len - 1);
            name[name_len - 1] = '\0';
            cJSON_Delete(json);
            return ESP_OK;
        }
    }

    cJSON_Delete(json);
    ESP_LOGE(TAG, "Missing 'friendly_name', 'name', or 'id' in request");
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t parse_group_member_request(const char *payload, char *group_name,
                                            size_t name_len, char *device_id, size_t device_len)
{
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse member request JSON");
        return ESP_FAIL;
    }

    /* Get group name */
    cJSON *group_obj = cJSON_GetObjectItem(json, "group");
    if (group_obj == NULL || !cJSON_IsString(group_obj)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'group' in member request");
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(group_name, group_obj->valuestring, name_len - 1);
    group_name[name_len - 1] = '\0';

    /* Get device ID (IEEE address or friendly name) */
    cJSON *device_obj = cJSON_GetObjectItem(json, "device");
    if (device_obj == NULL) {
        device_obj = cJSON_GetObjectItem(json, "ieee_address");
    }
    if (device_obj == NULL || !cJSON_IsString(device_obj)) {
        cJSON_Delete(json);
        ESP_LOGE(TAG, "Missing 'device' or 'ieee_address' in member request");
        return ESP_ERR_INVALID_ARG;
    }
    strncpy(device_id, device_obj->valuestring, device_len - 1);
    device_id[device_len - 1] = '\0';

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t extract_group_name_from_topic(const char *topic, char *name, size_t name_len)
{
    /* Topic format: zigbee2mqtt/group/[name]/set */
    const char *prefix = TOPIC_GROUP_SET_PREFIX;
    const char *suffix = TOPIC_GROUP_SET_SUFFIX;

    if (strncmp(topic, prefix, strlen(prefix)) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *name_start = topic + strlen(prefix);
    const char *name_end = strstr(name_start, suffix);

    if (name_end == NULL) {
        /* No /set suffix, use until end of string */
        strncpy(name, name_start, name_len - 1);
        name[name_len - 1] = '\0';
    } else {
        size_t name_length = name_end - name_start;
        if (name_length >= name_len) {
            return ESP_ERR_NO_MEM;
        }
        strncpy(name, name_start, name_length);
        name[name_length] = '\0';
    }

    return ESP_OK;
}
