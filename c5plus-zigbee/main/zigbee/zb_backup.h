/**
 * @file zb_backup.h
 * @brief Zigbee Network Backup and Restore API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides comprehensive network backup and restore functionality:
 * - Network state backup (PAN ID, extended PAN ID, network key, channel, TC link key)
 * - Device database backup (all paired devices with IEEE addresses, short addresses, endpoints, clusters)
 * - Group and binding backup
 * - Coordinator state backup
 * - SPIFFS-based backup storage with versioning and timestamps
 * - Restore validation before applying
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/backup - Create full network backup
 * - zigbee2mqtt/bridge/request/backup/state - Backup coordinator state only
 * - zigbee2mqtt/bridge/request/backup/devices - Backup device database only
 * - zigbee2mqtt/bridge/request/restore - Restore from backup (with network restart)
 * - zigbee2mqtt/bridge/request/backup/list - List available backups
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_BACKUP_H
#define ZB_BACKUP_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Backup version identifier
 */
#define ZB_BACKUP_VERSION               1

/**
 * @brief Maximum number of devices in backup
 */
#define ZB_BACKUP_MAX_DEVICES           50

/**
 * @brief Maximum number of groups in backup
 */
#define ZB_BACKUP_MAX_GROUPS            16

/**
 * @brief Maximum number of bindings in backup
 */
#define ZB_BACKUP_MAX_BINDINGS          32

/**
 * @brief Maximum number of backups to retain
 */
#define ZB_BACKUP_MAX_FILES             5

/**
 * @brief Backup file path base
 */
#define ZB_BACKUP_FILE_PATH             "/spiffs/backup"

/**
 * @brief Backup file extension
 */
#define ZB_BACKUP_FILE_EXT              ".zbk"

/**
 * @brief NVS namespace for backup metadata
 */
#define ZB_BACKUP_NVS_NAMESPACE         "zb_backup"

/**
 * @brief Network key length
 */
#define ZB_BACKUP_NETWORK_KEY_LEN       16

/**
 * @brief TC link key length
 */
#define ZB_BACKUP_TC_LINK_KEY_LEN       16

/**
 * @brief Maximum friendly name length in backup
 */
#define ZB_BACKUP_FRIENDLY_NAME_LEN     32

/**
 * @brief Maximum model name length in backup
 */
#define ZB_BACKUP_MODEL_LEN             32

/**
 * @brief Maximum manufacturer name length in backup
 */
#define ZB_BACKUP_MANUFACTURER_LEN      32

/**
 * @brief Maximum group name length in backup
 */
#define ZB_BACKUP_GROUP_NAME_LEN        32

/**
 * @brief Maximum group members
 */
#define ZB_BACKUP_GROUP_MAX_MEMBERS     32

/**
 * @brief Maximum clusters per device
 */
#define ZB_BACKUP_MAX_CLUSTERS          32

/**
 * @brief Backup type enumeration
 */
typedef enum {
    ZB_BACKUP_TYPE_FULL = 0,       /**< Full backup (coordinator + devices + groups + bindings) */
    ZB_BACKUP_TYPE_STATE,          /**< Coordinator state only */
    ZB_BACKUP_TYPE_DEVICES,        /**< Device database only */
    ZB_BACKUP_TYPE_GROUPS,         /**< Groups only */
    ZB_BACKUP_TYPE_BINDINGS        /**< Bindings only */
} zb_backup_type_t;

/**
 * @brief Backup status codes
 */
typedef enum {
    ZB_BACKUP_STATUS_SUCCESS = 0,      /**< Operation successful */
    ZB_BACKUP_STATUS_NOT_FOUND,        /**< Backup file not found */
    ZB_BACKUP_STATUS_INVALID,          /**< Invalid backup data */
    ZB_BACKUP_STATUS_VERSION_MISMATCH, /**< Backup version incompatible */
    ZB_BACKUP_STATUS_STORAGE_FULL,     /**< Storage space exhausted */
    ZB_BACKUP_STATUS_IO_ERROR,         /**< File I/O error */
    ZB_BACKUP_STATUS_VALIDATION_FAILED,/**< Restore validation failed */
    ZB_BACKUP_STATUS_NETWORK_ACTIVE,   /**< Cannot restore while network active */
    ZB_BACKUP_STATUS_ERROR             /**< General error */
} zb_backup_status_t;

/**
 * @brief Coordinator state structure in backup
 */
typedef struct {
    uint64_t ieee_addr;                                 /**< Coordinator IEEE address */
    uint16_t pan_id;                                    /**< PAN ID */
    uint64_t ext_pan_id;                                /**< Extended PAN ID */
    uint8_t network_key[ZB_BACKUP_NETWORK_KEY_LEN];     /**< Network encryption key */
    uint8_t channel;                                    /**< Radio channel (11-26) */
    uint8_t tc_link_key[ZB_BACKUP_TC_LINK_KEY_LEN];     /**< Trust Center link key */
    uint8_t network_key_seq;                            /**< Network key sequence number */
    uint32_t network_frame_counter;                     /**< Network frame counter */
    bool permit_join;                                   /**< Permit join status at backup time */
} zb_backup_coordinator_t;

/**
 * @brief Device entry structure in backup
 */
typedef struct {
    uint64_t ieee_addr;                                 /**< IEEE 64-bit address */
    uint16_t short_addr;                                /**< Network short address */
    uint8_t endpoint;                                   /**< Primary endpoint */
    char friendly_name[ZB_BACKUP_FRIENDLY_NAME_LEN];    /**< User-friendly name */
    char model[ZB_BACKUP_MODEL_LEN];                    /**< Model identifier */
    char manufacturer[ZB_BACKUP_MANUFACTURER_LEN];      /**< Manufacturer name */
    uint8_t device_type;                                /**< Device type enumeration */
    uint16_t cluster_count;                             /**< Number of supported clusters */
    uint16_t clusters[ZB_BACKUP_MAX_CLUSTERS];          /**< Supported cluster IDs */
} zb_backup_device_t;

/**
 * @brief Group entry structure in backup
 */
typedef struct {
    uint16_t group_id;                                  /**< Zigbee group ID */
    char name[ZB_BACKUP_GROUP_NAME_LEN];                /**< Group friendly name */
    uint64_t members[ZB_BACKUP_GROUP_MAX_MEMBERS];      /**< IEEE addresses of members */
    uint8_t member_count;                               /**< Number of members in group */
} zb_backup_group_t;

/**
 * @brief Binding entry structure in backup
 */
typedef struct {
    uint64_t source_ieee;           /**< Source device IEEE address */
    uint8_t source_endpoint;        /**< Source device endpoint */
    uint16_t cluster_id;            /**< Cluster ID */
    uint64_t dest_ieee;             /**< Destination device IEEE address */
    uint8_t dest_endpoint;          /**< Destination device endpoint */
} zb_backup_binding_t;

/**
 * @brief Complete backup data structure
 *
 * This structure contains all data needed for a full network backup.
 * It is serialized to SPIFFS for persistent storage.
 */
typedef struct {
    /* Header */
    uint32_t magic;                                     /**< Magic number for validation (0x5A424B31 = "ZBK1") */
    uint32_t version;                                   /**< Backup format version */
    uint32_t timestamp;                                 /**< Unix timestamp of backup creation */
    uint32_t checksum;                                  /**< CRC32 checksum of backup data */
    zb_backup_type_t backup_type;                       /**< Type of backup */

    /* Coordinator state */
    zb_backup_coordinator_t coordinator;                /**< Coordinator configuration */

    /* Device database */
    uint16_t device_count;                              /**< Number of devices in backup */
    zb_backup_device_t devices[ZB_BACKUP_MAX_DEVICES];  /**< Device entries */

    /* Groups */
    uint16_t group_count;                               /**< Number of groups in backup */
    zb_backup_group_t groups[ZB_BACKUP_MAX_GROUPS];     /**< Group entries */

    /* Bindings */
    uint16_t binding_count;                             /**< Number of bindings in backup */
    zb_backup_binding_t bindings[ZB_BACKUP_MAX_BINDINGS]; /**< Binding entries */
} zb_backup_t;

/**
 * @brief Backup metadata for listing
 */
typedef struct {
    uint32_t timestamp;             /**< Backup timestamp */
    uint32_t version;               /**< Backup format version */
    zb_backup_type_t backup_type;   /**< Type of backup */
    uint16_t device_count;          /**< Number of devices in backup */
    uint16_t group_count;           /**< Number of groups in backup */
    uint16_t binding_count;         /**< Number of bindings in backup */
    uint32_t file_size;             /**< File size in bytes */
    char filename[64];              /**< Backup filename */
} zb_backup_info_t;

/**
 * @brief Backup list structure
 */
typedef struct {
    uint8_t count;                              /**< Number of backups */
    zb_backup_info_t backups[ZB_BACKUP_MAX_FILES]; /**< Backup metadata array */
} zb_backup_list_t;

/**
 * @brief Restore result structure
 */
typedef struct {
    bool success;                   /**< Restore operation success */
    zb_backup_status_t status;      /**< Detailed status code */
    uint16_t restored_devices;      /**< Number of devices restored */
    uint16_t restored_groups;       /**< Number of groups restored */
    uint16_t restored_bindings;     /**< Number of bindings restored */
    uint16_t warning_count;         /**< Number of warnings during restore */
    char warnings[256];             /**< Warning messages (comma-separated) */
} zb_backup_restore_result_t;

/**
 * @brief Backup event callback type
 *
 * @param[in] status Backup/restore status
 * @param[in] message Status message
 */
typedef void (*zb_backup_event_cb_t)(zb_backup_status_t status, const char *message);

/**
 * @brief Initialize backup module
 *
 * Initializes the backup module, mounts SPIFFS partition if needed,
 * and loads backup metadata from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_backup_init(void);

/**
 * @brief Deinitialize backup module
 *
 * Releases resources used by the backup module.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_backup_deinit(void);

/**
 * @brief Create network backup
 *
 * Creates a backup of the current network state including coordinator
 * configuration, device database, groups, and bindings.
 *
 * @param[in] backup_type Type of backup to create
 * @param[out] backup Pointer to backup structure (optional, can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t zb_backup_create(zb_backup_type_t backup_type, zb_backup_t *backup);

/**
 * @brief Save backup to SPIFFS
 *
 * Serializes and saves the backup data to SPIFFS storage.
 * Automatically manages backup file rotation (keeps last N backups).
 *
 * @param[in] backup Pointer to backup structure
 * @param[out] filename Output: saved filename (optional, can be NULL)
 * @param[in] filename_len Length of filename buffer
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if storage is full
 * @return ESP_FAIL on I/O error
 */
esp_err_t zb_backup_save(const zb_backup_t *backup, char *filename, size_t filename_len);

/**
 * @brief Load backup from SPIFFS
 *
 * Loads and deserializes backup data from SPIFFS storage.
 *
 * @param[in] filename Backup filename (NULL for most recent)
 * @param[out] backup Pointer to backup structure
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if file not found
 * @return ESP_ERR_INVALID_SIZE if file corrupt
 * @return ESP_FAIL on I/O error
 */
esp_err_t zb_backup_load(const char *filename, zb_backup_t *backup);

/**
 * @brief Validate backup before restore
 *
 * Validates backup data integrity and compatibility before restore.
 * Checks checksum, version, and data consistency.
 *
 * @param[in] backup Pointer to backup structure
 * @return ESP_OK if backup is valid
 * @return ESP_ERR_INVALID_ARG if backup is NULL
 * @return ESP_ERR_INVALID_VERSION if version incompatible
 * @return ESP_ERR_INVALID_CRC if checksum mismatch
 */
esp_err_t zb_backup_validate(const zb_backup_t *backup);

/**
 * @brief Restore network from backup
 *
 * Restores network configuration, device database, groups, and bindings
 * from a backup file. Requires network restart after restore.
 *
 * IMPORTANT: Network must be stopped before calling this function.
 *
 * @param[in] backup Pointer to backup structure
 * @param[out] result Restore operation result (optional, can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if network is running
 * @return ESP_ERR_INVALID_ARG if backup is invalid
 */
esp_err_t zb_backup_restore(const zb_backup_t *backup, zb_backup_restore_result_t *result);

/**
 * @brief Restore from file with automatic validation
 *
 * Loads, validates, and restores from a backup file in one operation.
 * Handles network stop/restart automatically.
 *
 * @param[in] filename Backup filename (NULL for most recent)
 * @param[out] result Restore operation result (optional, can be NULL)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if file not found
 * @return ESP_ERR_INVALID_ARG if validation fails
 */
esp_err_t zb_backup_restore_from_file(const char *filename, zb_backup_restore_result_t *result);

/**
 * @brief Delete backup file
 *
 * Removes a backup file from SPIFFS storage.
 *
 * @param[in] filename Backup filename
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if file not found
 * @return ESP_FAIL on I/O error
 */
esp_err_t zb_backup_delete(const char *filename);

/**
 * @brief List available backups
 *
 * Returns a list of all available backup files with metadata.
 *
 * @param[out] list Pointer to backup list structure
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if list is NULL
 */
esp_err_t zb_backup_list(zb_backup_list_t *list);

/**
 * @brief Get most recent backup info
 *
 * Returns metadata for the most recent backup file.
 *
 * @param[out] info Pointer to backup info structure
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no backups exist
 * @return ESP_ERR_INVALID_ARG if info is NULL
 */
esp_err_t zb_backup_get_latest(zb_backup_info_t *info);

/**
 * @brief Process MQTT backup request
 *
 * Main entry point for handling backup-related MQTT requests.
 * Supports the following topics:
 * - zigbee2mqtt/bridge/request/backup - Full backup
 * - zigbee2mqtt/bridge/request/backup/state - State only backup
 * - zigbee2mqtt/bridge/request/backup/devices - Devices only backup
 *
 * @param[in] topic MQTT topic
 * @param[in] payload JSON payload
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if invalid request
 */
esp_err_t zb_backup_process_mqtt_backup(const char *topic, const char *payload, size_t len);

/**
 * @brief Process MQTT restore request
 *
 * Handles restore request from MQTT.
 * Topic: zigbee2mqtt/bridge/request/restore
 *
 * @param[in] payload JSON payload with optional filename
 * @param[in] len Payload length
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if backup not found
 */
esp_err_t zb_backup_process_mqtt_restore(const char *payload, size_t len);

/**
 * @brief Process MQTT backup list request
 *
 * Handles backup list request from MQTT.
 * Topic: zigbee2mqtt/bridge/request/backup/list
 *
 * @param[in] payload JSON payload (ignored)
 * @param[in] len Payload length
 * @return ESP_OK on success
 */
esp_err_t zb_backup_process_mqtt_list(const char *payload, size_t len);

/**
 * @brief Publish backup response via MQTT
 *
 * Publishes the result of a backup operation to MQTT.
 * Topic: zigbee2mqtt/bridge/response/backup
 *
 * @param[in] backup Backup data (for successful backup)
 * @param[in] status Operation status
 * @param[in] transaction_id Transaction ID (optional)
 * @return ESP_OK on success
 */
esp_err_t zb_backup_publish_response(const zb_backup_t *backup,
                                      zb_backup_status_t status,
                                      const char *transaction_id);

/**
 * @brief Publish restore response via MQTT
 *
 * Publishes the result of a restore operation to MQTT.
 * Topic: zigbee2mqtt/bridge/response/restore
 *
 * @param[in] result Restore result
 * @param[in] transaction_id Transaction ID (optional)
 * @return ESP_OK on success
 */
esp_err_t zb_backup_publish_restore_response(const zb_backup_restore_result_t *result,
                                              const char *transaction_id);

/**
 * @brief Publish backup list via MQTT
 *
 * Publishes the list of available backups to MQTT.
 * Topic: zigbee2mqtt/bridge/response/backup/list
 *
 * @param[in] list Backup list
 * @param[in] transaction_id Transaction ID (optional)
 * @return ESP_OK on success
 */
esp_err_t zb_backup_publish_list_response(const zb_backup_list_t *list,
                                           const char *transaction_id);

/**
 * @brief Convert backup to JSON
 *
 * Creates a cJSON representation of the backup data suitable for
 * MQTT publishing in Zigbee2MQTT format.
 *
 * @param[in] backup Backup data
 * @return cJSON object (caller must free with cJSON_Delete) or NULL on error
 */
struct cJSON* zb_backup_to_json(const zb_backup_t *backup);

/**
 * @brief Register backup event callback
 *
 * Registers a callback function for backup/restore events.
 *
 * @param[in] callback Event callback function
 * @return ESP_OK on success
 */
esp_err_t zb_backup_register_callback(zb_backup_event_cb_t callback);

/**
 * @brief Get backup status string
 *
 * Returns a human-readable string for a backup status code.
 *
 * @param[in] status Backup status code
 * @return Status string
 */
const char* zb_backup_status_str(zb_backup_status_t status);

/**
 * @brief Calculate CRC32 checksum
 *
 * Calculates CRC32 checksum for backup validation.
 *
 * @param[in] data Data to checksum
 * @param[in] len Data length
 * @return CRC32 checksum value
 */
uint32_t zb_backup_calc_checksum(const void *data, size_t len);

/**
 * @brief Self-test function for backup module
 *
 * Tests backup creation, save, load, and restore functionality.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_backup_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_BACKUP_H */
