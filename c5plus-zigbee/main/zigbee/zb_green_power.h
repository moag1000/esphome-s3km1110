/**
 * @file zb_green_power.h
 * @brief Zigbee Green Power (ZGP) API for ESP32-C5 Zigbee2MQTT Gateway
 *
 * This module provides Zigbee Green Power (ZGP) functionality for batteryless
 * and energy harvesting devices:
 * - Green Power Proxy (GPP) functionality
 * - Green Power Sink functionality
 * - GP Commissioning mode handling
 * - GP Translation Table (GP Command -> ZCL Action mapping)
 * - GP Proxy Table management (up to 10 GP devices)
 * - GP Security (shared key, individual key)
 * - NVS persistence for GP devices
 *
 * Supported GP Device Types:
 * - On/Off Switch (EnOcean PTM switches)
 * - Generic Switch (8-button remotes, Philips Hue Tap)
 * - Temperature/Humidity Sensors
 * - Door/Window Sensors
 *
 * MQTT Topics (Zigbee2MQTT compatible):
 * - zigbee2mqtt/bridge/request/greenpower/commission - Start GP commissioning
 * - zigbee2mqtt/bridge/request/greenpower/remove/{gp_id} - Remove GP device
 * - zigbee2mqtt/bridge/greenpower/devices - List of GP devices
 * - zigbee2mqtt/{friendly_name} - GP device state updates
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef ZB_GREEN_POWER_H
#define ZB_GREEN_POWER_H

#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Green Power Cluster Definitions
 * ============================================================================ */

/**
 * @brief Green Power Cluster ID
 */
#define ZB_GP_CLUSTER_ID                            0x0021

/**
 * @brief Green Power Endpoint (fixed by specification)
 */
#define ZB_GP_ENDPOINT                              242

/**
 * @brief Green Power Profile ID
 */
#define ZB_GP_PROFILE_ID                            0xA1E0

/* ============================================================================
 * GP Frame Types and Application IDs
 * ============================================================================ */

/**
 * @brief GP Frame Types
 */
typedef enum {
    ZB_GP_FRAME_TYPE_DATA = 0x00,           /**< Data frame */
    ZB_GP_FRAME_TYPE_MAINTENANCE = 0x01,    /**< Maintenance frame */
    ZB_GP_FRAME_TYPE_RESERVED = 0x02,       /**< Reserved */
    ZB_GP_FRAME_TYPE_RESERVED2 = 0x03       /**< Reserved */
} zb_gp_frame_type_t;

/**
 * @brief GP Application IDs
 */
typedef enum {
    ZB_GP_APP_ID_SRC_ID = 0x00,             /**< Source ID addressing (32-bit) */
    ZB_GP_APP_ID_LPED = 0x01,               /**< LPED (deprecated) */
    ZB_GP_APP_ID_IEEE = 0x02                /**< IEEE address (64-bit) */
} zb_gp_app_id_t;

/* ============================================================================
 * GP Command IDs
 * ============================================================================ */

/**
 * @brief GP Device Command IDs (from GP Device)
 */
#define ZB_GP_CMD_IDENTIFY                      0x00
#define ZB_GP_CMD_RECALL_SCENE_0                0x10
#define ZB_GP_CMD_RECALL_SCENE_1                0x11
#define ZB_GP_CMD_RECALL_SCENE_2                0x12
#define ZB_GP_CMD_RECALL_SCENE_3                0x13
#define ZB_GP_CMD_RECALL_SCENE_4                0x14
#define ZB_GP_CMD_RECALL_SCENE_5                0x15
#define ZB_GP_CMD_RECALL_SCENE_6                0x16
#define ZB_GP_CMD_RECALL_SCENE_7                0x17
#define ZB_GP_CMD_STORE_SCENE_0                 0x18
#define ZB_GP_CMD_STORE_SCENE_1                 0x19
#define ZB_GP_CMD_STORE_SCENE_2                 0x1A
#define ZB_GP_CMD_STORE_SCENE_3                 0x1B
#define ZB_GP_CMD_STORE_SCENE_4                 0x1C
#define ZB_GP_CMD_STORE_SCENE_5                 0x1D
#define ZB_GP_CMD_STORE_SCENE_6                 0x1E
#define ZB_GP_CMD_STORE_SCENE_7                 0x1F
#define ZB_GP_CMD_OFF                           0x20
#define ZB_GP_CMD_ON                            0x21
#define ZB_GP_CMD_TOGGLE                        0x22
#define ZB_GP_CMD_RELEASE                       0x23
#define ZB_GP_CMD_MOVE_UP                       0x30
#define ZB_GP_CMD_MOVE_DOWN                     0x31
#define ZB_GP_CMD_STEP_UP                       0x32
#define ZB_GP_CMD_STEP_DOWN                     0x33
#define ZB_GP_CMD_LEVEL_CONTROL_STOP            0x34
#define ZB_GP_CMD_MOVE_UP_WITH_ON_OFF           0x35
#define ZB_GP_CMD_MOVE_DOWN_WITH_ON_OFF         0x36
#define ZB_GP_CMD_STEP_UP_WITH_ON_OFF           0x37
#define ZB_GP_CMD_STEP_DOWN_WITH_ON_OFF         0x38
#define ZB_GP_CMD_MOVE_HUE_STOP                 0x40
#define ZB_GP_CMD_MOVE_HUE_UP                   0x41
#define ZB_GP_CMD_MOVE_HUE_DOWN                 0x42
#define ZB_GP_CMD_STEP_HUE_UP                   0x43
#define ZB_GP_CMD_STEP_HUE_DOWN                 0x44
#define ZB_GP_CMD_MOVE_SATURATION_STOP          0x45
#define ZB_GP_CMD_MOVE_SATURATION_UP            0x46
#define ZB_GP_CMD_MOVE_SATURATION_DOWN          0x47
#define ZB_GP_CMD_STEP_SATURATION_UP            0x48
#define ZB_GP_CMD_STEP_SATURATION_DOWN          0x49
#define ZB_GP_CMD_MOVE_COLOR                    0x4A
#define ZB_GP_CMD_STEP_COLOR                    0x4B
#define ZB_GP_CMD_LOCK_DOOR                     0x50
#define ZB_GP_CMD_UNLOCK_DOOR                   0x51
#define ZB_GP_CMD_PRESS_1_OF_1                  0x60
#define ZB_GP_CMD_RELEASE_1_OF_1                0x61
#define ZB_GP_CMD_PRESS_1_OF_2                  0x62
#define ZB_GP_CMD_RELEASE_1_OF_2                0x63
#define ZB_GP_CMD_PRESS_2_OF_2                  0x64
#define ZB_GP_CMD_RELEASE_2_OF_2                0x65
#define ZB_GP_CMD_SHORT_PRESS_1_OF_1            0x66
#define ZB_GP_CMD_SHORT_PRESS_1_OF_2            0x67
#define ZB_GP_CMD_SHORT_PRESS_2_OF_2            0x68
#define ZB_GP_CMD_8BIT_VECTOR_PRESS             0x69
#define ZB_GP_CMD_8BIT_VECTOR_RELEASE           0x6A
#define ZB_GP_CMD_ATTRIBUTE_REPORTING           0xA0
#define ZB_GP_CMD_MFR_SPECIFIC_ATTR_REPORTING   0xA1
#define ZB_GP_CMD_MULTI_CLUSTER_REPORTING       0xA2
#define ZB_GP_CMD_MFR_MULTI_CLUSTER_REPORTING   0xA3
#define ZB_GP_CMD_REQUEST_ATTRIBUTES            0xA4
#define ZB_GP_CMD_READ_ATTRIBUTES_RESPONSE      0xA5
#define ZB_GP_CMD_ZCL_TUNNELING                 0xA6
#define ZB_GP_CMD_COMPACT_ATTRIBUTE_REPORTING   0xA8
#define ZB_GP_CMD_ANY                           0xAF
#define ZB_GP_CMD_MFR_DEFINED_B0_BF             0xB0  /**< 0xB0-0xBF: manufacturer defined */
#define ZB_GP_CMD_COMMISSIONING                 0xE0
#define ZB_GP_CMD_DECOMMISSIONING               0xE1
#define ZB_GP_CMD_SUCCESS                       0xE2
#define ZB_GP_CMD_CHANNEL_REQUEST               0xE3
#define ZB_GP_CMD_APPLICATION_DESCRIPTION       0xE4
#define ZB_GP_CMD_COMMISSIONING_REPLY           0xF0
#define ZB_GP_CMD_WRITE_ATTRIBUTES              0xF1
#define ZB_GP_CMD_READ_ATTRIBUTES               0xF2
#define ZB_GP_CMD_CHANNEL_CONFIGURATION         0xF3

/**
 * @brief GP Cluster Command IDs (Proxy/Sink commands)
 */
#define ZB_GP_CLUSTER_CMD_NOTIFICATION              0x00
#define ZB_GP_CLUSTER_CMD_PAIRING_SEARCH            0x01
#define ZB_GP_CLUSTER_CMD_TUNNELING_STOP            0x03
#define ZB_GP_CLUSTER_CMD_COMMISSIONING_NOTIFICATION 0x04
#define ZB_GP_CLUSTER_CMD_SINK_COMMISSIONING_MODE   0x05
#define ZB_GP_CLUSTER_CMD_TRANSLATION_TABLE_UPDATE  0x07
#define ZB_GP_CLUSTER_CMD_TRANSLATION_TABLE_REQUEST 0x08
#define ZB_GP_CLUSTER_CMD_PAIRING_CONFIGURATION     0x09
#define ZB_GP_CLUSTER_CMD_SINK_TABLE_REQUEST        0x0A
#define ZB_GP_CLUSTER_CMD_PROXY_TABLE_RESPONSE      0x0B
#define ZB_GP_CLUSTER_CMD_RESPONSE                  0x01  /**< Server to client */
#define ZB_GP_CLUSTER_CMD_PAIRING                   0x01  /**< Server to client */
#define ZB_GP_CLUSTER_CMD_PROXY_COMMISSIONING_MODE  0x02  /**< Server to client */
#define ZB_GP_CLUSTER_CMD_TRANSLATION_TABLE_RESPONSE 0x08 /**< Server to client */
#define ZB_GP_CLUSTER_CMD_SINK_TABLE_RESPONSE       0x0A  /**< Server to client */
#define ZB_GP_CLUSTER_CMD_PROXY_TABLE_REQUEST       0x0B  /**< Server to client */

/* ============================================================================
 * GP Device Types
 * ============================================================================ */

/**
 * @brief GP Device Type IDs
 */
typedef enum {
    ZB_GP_DEVICE_TYPE_SIMPLE_GENERIC_1STATE_SWITCH = 0x00,  /**< Simple generic 1-state switch */
    ZB_GP_DEVICE_TYPE_SIMPLE_GENERIC_2STATE_SWITCH = 0x01,  /**< Simple generic 2-state switch */
    ZB_GP_DEVICE_TYPE_ON_OFF_SWITCH = 0x02,                 /**< On/Off switch */
    ZB_GP_DEVICE_TYPE_LEVEL_CONTROL_SWITCH = 0x03,          /**< Level control switch */
    ZB_GP_DEVICE_TYPE_SIMPLE_SENSOR = 0x04,                 /**< Simple sensor */
    ZB_GP_DEVICE_TYPE_ADVANCED_GENERIC_1STATE_SWITCH = 0x05, /**< Advanced generic 1-state switch */
    ZB_GP_DEVICE_TYPE_ADVANCED_GENERIC_2STATE_SWITCH = 0x06, /**< Advanced generic 2-state switch */
    ZB_GP_DEVICE_TYPE_GENERIC_SWITCH = 0x07,                /**< Generic switch (up to 8 buttons) */
    ZB_GP_DEVICE_TYPE_COLOR_DIMMER_SWITCH = 0x10,           /**< Color dimmer switch */
    ZB_GP_DEVICE_TYPE_LIGHT_SENSOR = 0x11,                  /**< Light sensor */
    ZB_GP_DEVICE_TYPE_OCCUPANCY_SENSOR = 0x12,              /**< Occupancy sensor */
    ZB_GP_DEVICE_TYPE_DOOR_SENSOR = 0x20,                   /**< Door/window sensor */
    ZB_GP_DEVICE_TYPE_TEMPERATURE_SENSOR = 0x30,            /**< Temperature sensor */
    ZB_GP_DEVICE_TYPE_PRESSURE_SENSOR = 0x31,               /**< Pressure sensor */
    ZB_GP_DEVICE_TYPE_FLOW_SENSOR = 0x32,                   /**< Flow sensor */
    ZB_GP_DEVICE_TYPE_ENVIRONMENT_SENSOR = 0x33,            /**< Environment sensor (multi) */
    ZB_GP_DEVICE_TYPE_MFR_SPECIFIC = 0xFE,                  /**< Manufacturer specific */
    ZB_GP_DEVICE_TYPE_UNDEFINED = 0xFF                      /**< Undefined device type */
} zb_gp_device_type_t;

/* ============================================================================
 * GP Security Definitions
 * ============================================================================ */

/**
 * @brief GP Security Levels
 */
typedef enum {
    ZB_GP_SECURITY_LEVEL_NO_SECURITY = 0x00,          /**< No security */
    ZB_GP_SECURITY_LEVEL_1LSB_COUNTER_MIC = 0x01,     /**< 1-byte frame counter + 2-byte MIC */
    ZB_GP_SECURITY_LEVEL_FULL_COUNTER_MIC = 0x02,     /**< 4-byte counter + 4-byte MIC */
    ZB_GP_SECURITY_LEVEL_ENCRYPTION = 0x03            /**< Encryption + 4-byte counter + 4-byte MIC */
} zb_gp_security_level_t;

/**
 * @brief GP Security Key Types
 */
typedef enum {
    ZB_GP_KEY_TYPE_NONE = 0x00,                       /**< No key */
    ZB_GP_KEY_TYPE_ZB_NWK_KEY = 0x01,                 /**< Zigbee network key */
    ZB_GP_KEY_TYPE_GP_GROUP_KEY = 0x02,               /**< GP group key */
    ZB_GP_KEY_TYPE_NWK_KEY_DERIVED_GP_GROUP = 0x03,   /**< Network-key derived GP group key */
    ZB_GP_KEY_TYPE_OUT_OF_BOX_GPD_KEY = 0x04,         /**< Out-of-the-box GPD key */
    ZB_GP_KEY_TYPE_DERIVED_INDIVIDUAL_GPD = 0x07      /**< Derived individual GPD key */
} zb_gp_key_type_t;

/**
 * @brief GP Security Key Size
 */
#define ZB_GP_SECURITY_KEY_SIZE                     16

/**
 * @brief GP Default TC Link Key (ZigBee Alliance 09)
 * Used for deriving GP keys in some configurations
 */
#define ZB_GP_DEFAULT_TC_LINK_KEY  { 0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, \
                                     0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39 }

/**
 * @brief GP Default Shared Key (ZigBee Green Power Link Key)
 */
#define ZB_GP_DEFAULT_SHARED_KEY   { 0x4C, 0x69, 0x6E, 0x6B, 0x4B, 0x65, 0x79, 0x47, \
                                     0x50, 0x44, 0x47, 0x50, 0x44, 0x47, 0x50, 0x44 }

/* ============================================================================
 * GP Configuration Limits
 * ============================================================================ */

/**
 * @brief Maximum number of GP devices in proxy table
 */
#ifndef CONFIG_ZB_GP_MAX_DEVICES
#define ZB_GP_MAX_DEVICES                           10
#else
#define ZB_GP_MAX_DEVICES                           CONFIG_ZB_GP_MAX_DEVICES
#endif

/**
 * @brief Maximum translation table entries per device
 */
#define ZB_GP_MAX_TRANSLATIONS_PER_DEVICE           8

/**
 * @brief Maximum total translation table entries
 */
#define ZB_GP_MAX_TRANSLATION_ENTRIES               (ZB_GP_MAX_DEVICES * ZB_GP_MAX_TRANSLATIONS_PER_DEVICE)

/**
 * @brief GP Commissioning timeout (seconds)
 */
#define ZB_GP_COMMISSIONING_TIMEOUT_SEC             180

/**
 * @brief GP Device friendly name maximum length
 */
#define ZB_GP_DEVICE_NAME_LEN                       32

/**
 * @brief GP Frame maximum payload size
 */
#define ZB_GP_MAX_PAYLOAD_SIZE                      64

/* ============================================================================
 * GP Data Structures
 * ============================================================================ */

/**
 * @brief GP Source ID (32-bit unique identifier)
 */
typedef uint32_t zb_gp_src_id_t;

/**
 * @brief GP Address structure (supports both Source ID and IEEE addressing)
 */
typedef struct {
    zb_gp_app_id_t app_id;              /**< Application ID (addressing mode) */
    union {
        zb_gp_src_id_t src_id;          /**< Source ID (32-bit, for app_id = 0) */
        struct {
            uint8_t ieee_addr[8];       /**< IEEE address (for app_id = 2) */
            uint8_t endpoint;           /**< GPD endpoint */
        } ieee;
    };
} zb_gp_addr_t;

/**
 * @brief GP Security Frame Counter
 */
typedef uint32_t zb_gp_frame_counter_t;

/**
 * @brief GP Security Key
 */
typedef struct {
    uint8_t key[ZB_GP_SECURITY_KEY_SIZE];  /**< Security key bytes */
} zb_gp_security_key_t;

/**
 * @brief GP Device Options Bitmap
 */
typedef struct {
    uint8_t mac_seq_num_cap : 1;        /**< MAC sequence number capability */
    uint8_t rx_on_cap : 1;              /**< RxOnWhenIdle capability */
    uint8_t application_info : 1;       /**< Application information present */
    uint8_t pan_id_request : 1;         /**< PAN ID request (commissioning) */
    uint8_t gp_security_key_request : 1; /**< GP security key request */
    uint8_t fixed_location : 1;         /**< Fixed location */
    uint8_t extended_options : 1;       /**< Extended options present */
    uint8_t reserved : 1;
} zb_gp_device_options_t;

/**
 * @brief GP Device structure (stored in proxy table)
 */
typedef struct {
    bool in_use;                                /**< Entry in use flag */
    zb_gp_addr_t gp_addr;                       /**< GP device address */
    zb_gp_device_type_t device_type;            /**< GP device type */
    char friendly_name[ZB_GP_DEVICE_NAME_LEN];  /**< User-friendly name */
    zb_gp_device_options_t options;             /**< Device options/capabilities */

    /* Security */
    zb_gp_security_level_t security_level;      /**< Security level */
    zb_gp_key_type_t key_type;                  /**< Security key type */
    zb_gp_security_key_t security_key;          /**< Security key (if individual) */
    zb_gp_frame_counter_t frame_counter;        /**< Last valid frame counter */
    bool security_key_valid;                    /**< Security key is valid */

    /* Commissioning info */
    uint8_t manufacturer_id[2];                 /**< Manufacturer ID (optional) */
    uint8_t model_id[2];                        /**< Model ID (optional) */
    uint8_t num_gp_cmds;                        /**< Number of GP commands */
    uint8_t gp_cmd_list[16];                    /**< List of GP commands */

    /* State */
    bool commissioned;                          /**< Device is commissioned */
    time_t last_seen;                           /**< Last activity timestamp */
    uint8_t link_quality;                       /**< Last received LQI */
    int8_t rssi;                                /**< Last received RSSI */
    uint32_t rx_count;                          /**< Received frame count */
} zb_gp_device_t;

/**
 * @brief GP Frame structure (received GP data frame)
 */
typedef struct {
    zb_gp_frame_type_t frame_type;              /**< Frame type */
    zb_gp_addr_t gp_addr;                       /**< Source GP address */
    bool auto_commissioning;                    /**< Auto-commissioning flag */
    bool nwk_frame_control_ext;                 /**< Network frame control extension */
    zb_gp_frame_counter_t frame_counter;        /**< Security frame counter */
    uint8_t gp_cmd_id;                          /**< GP command ID */
    uint8_t payload[ZB_GP_MAX_PAYLOAD_SIZE];    /**< Command payload */
    uint8_t payload_len;                        /**< Payload length */
    uint8_t mic[4];                             /**< Message Integrity Code */
    int8_t rssi;                                /**< Received RSSI */
    uint8_t lqi;                                /**< Link Quality Indicator */
} zb_gp_frame_t;

/**
 * @brief GP Translation Entry (maps GP command to ZCL action)
 */
typedef struct {
    bool in_use;                                /**< Entry in use flag */
    zb_gp_addr_t gp_addr;                       /**< GP device address */
    uint8_t gp_cmd_id;                          /**< GP command ID */
    uint8_t gp_endpoint;                        /**< GP endpoint (for IEEE) */

    /* Target ZCL action */
    uint8_t zcl_endpoint;                       /**< Target ZCL endpoint */
    uint16_t zcl_cluster_id;                    /**< Target cluster ID */
    uint8_t zcl_cmd_id;                         /**< ZCL command ID */
    uint8_t zcl_payload[16];                    /**< ZCL command payload */
    uint8_t zcl_payload_len;                    /**< Payload length */

    /* Additional options */
    uint16_t additional_info;                   /**< Additional info block */
} zb_gp_translation_entry_t;

/**
 * @brief GP Commissioning Frame structure
 */
typedef struct {
    zb_gp_addr_t gp_addr;                       /**< GP device address */
    zb_gp_device_type_t device_type;            /**< Device type */
    zb_gp_device_options_t options;             /**< Device options */
    bool extended_options;                      /**< Extended options present */
    zb_gp_security_level_t security_level;      /**< Requested security level */
    zb_gp_key_type_t key_type;                  /**< Key type */
    bool has_key;                               /**< Key is present in frame */
    zb_gp_security_key_t key;                   /**< Provided key */
    uint32_t key_mic;                           /**< Key MIC (if encrypted) */
    zb_gp_frame_counter_t frame_counter;        /**< Initial frame counter */
    uint8_t application_info;                   /**< Application info byte */
    uint16_t manufacturer_id;                   /**< Manufacturer ID */
    uint16_t model_id;                          /**< Model ID */
    uint8_t num_gp_cmds;                        /**< Number of GP commands */
    uint8_t gp_cmd_list[16];                    /**< GP command list */
    uint8_t num_server_clusters;                /**< Number of server clusters */
    uint16_t server_clusters[8];                /**< Server cluster list */
    uint8_t num_client_clusters;                /**< Number of client clusters */
    uint16_t client_clusters[8];                /**< Client cluster list */
} zb_gp_commissioning_frame_t;

/**
 * @brief GP Commissioning State
 */
typedef enum {
    ZB_GP_COMMISSIONING_STATE_IDLE = 0,         /**< Not in commissioning mode */
    ZB_GP_COMMISSIONING_STATE_ACTIVE,           /**< Commissioning mode active */
    ZB_GP_COMMISSIONING_STATE_PENDING,          /**< Waiting for confirmation */
    ZB_GP_COMMISSIONING_STATE_SUCCESS,          /**< Commissioning successful */
    ZB_GP_COMMISSIONING_STATE_FAILED            /**< Commissioning failed */
} zb_gp_commissioning_state_t;

/**
 * @brief GP Commissioning Configuration
 */
typedef struct {
    bool involve_tc;                            /**< Involve Trust Center */
    bool involve_proxies;                       /**< Involve all proxies */
    bool channel_present;                       /**< Channel configuration present */
    uint8_t channel;                            /**< Channel for GP device */
    bool exit_on_pairing_success;               /**< Exit on first pairing success */
    bool exit_on_first_pairing_fail;            /**< Exit on first pairing failure */
    bool remove;                                /**< Remove mode (decommission) */
    uint16_t window_sec;                        /**< Commissioning window duration */
} zb_gp_commissioning_config_t;

/**
 * @brief GP Statistics structure
 */
typedef struct {
    uint32_t frames_received;                   /**< Total GP frames received */
    uint32_t frames_processed;                  /**< Frames successfully processed */
    uint32_t frames_dropped_security;           /**< Frames dropped (security) */
    uint32_t frames_dropped_unknown;            /**< Frames from unknown devices */
    uint32_t frames_dropped_replay;             /**< Frames dropped (replay attack) */
    uint32_t commissioning_attempts;            /**< Commissioning attempts */
    uint32_t commissioning_success;             /**< Successful commissionings */
    uint32_t translations_triggered;            /**< Translation table lookups */
} zb_gp_stats_t;

/* ============================================================================
 * GP Callback Types
 * ============================================================================ */

/**
 * @brief GP Frame received callback type
 *
 * Called when a GP data frame is received and processed.
 *
 * @param[in] device    The GP device that sent the frame
 * @param[in] frame     The received GP frame
 */
typedef void (*zb_gp_frame_callback_t)(const zb_gp_device_t *device,
                                        const zb_gp_frame_t *frame);

/**
 * @brief GP Commissioning event callback type
 *
 * Called when a GP commissioning event occurs.
 *
 * @param[in] state     Current commissioning state
 * @param[in] frame     Commissioning frame (NULL if not applicable)
 * @param[in] device    Resulting device (NULL if not commissioned)
 */
typedef void (*zb_gp_commissioning_callback_t)(zb_gp_commissioning_state_t state,
                                                const zb_gp_commissioning_frame_t *frame,
                                                const zb_gp_device_t *device);

/**
 * @brief GP Device list changed callback type
 *
 * Called when a GP device is added or removed.
 *
 * @param[in] device    The affected device
 * @param[in] added     true if device was added, false if removed
 */
typedef void (*zb_gp_device_callback_t)(const zb_gp_device_t *device, bool added);

/* ============================================================================
 * GP Module Initialization and Control
 * ============================================================================ */

/**
 * @brief Initialize the Green Power module
 *
 * Initializes GP proxy table, translation table, security, and NVS storage.
 * Must be called after Zigbee stack initialization.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if memory allocation fails
 * @return ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t zb_gp_init(void);

/**
 * @brief Deinitialize the Green Power module
 *
 * Stops commissioning, saves state to NVS, and frees resources.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_gp_deinit(void);

/**
 * @brief Check if GP module is initialized
 *
 * @return true if initialized
 * @return false if not initialized
 */
bool zb_gp_is_initialized(void);

/**
 * @brief Get GP mutex for thread-safe operations
 *
 * @return Mutex handle or NULL if not initialized
 */
SemaphoreHandle_t zb_gp_get_mutex(void);

/* ============================================================================
 * GP Commissioning Mode
 * ============================================================================ */

/**
 * @brief Enable GP commissioning mode
 *
 * Opens the GP commissioning window to accept new GP devices.
 * The window closes automatically after the configured timeout.
 *
 * @param[in] config    Commissioning configuration (NULL for defaults)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not initialized
 * @return ESP_ERR_INVALID_ARG if config is invalid
 */
esp_err_t zb_gp_enable_commissioning(const zb_gp_commissioning_config_t *config);

/**
 * @brief Disable GP commissioning mode
 *
 * Closes the commissioning window immediately.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not in commissioning mode
 */
esp_err_t zb_gp_disable_commissioning(void);

/**
 * @brief Check if commissioning mode is active
 *
 * @return true if commissioning mode is active
 * @return false otherwise
 */
bool zb_gp_is_commissioning_active(void);

/**
 * @brief Get current commissioning state
 *
 * @return Current commissioning state
 */
zb_gp_commissioning_state_t zb_gp_get_commissioning_state(void);

/**
 * @brief Get remaining commissioning time
 *
 * @return Remaining seconds in commissioning window, 0 if not active
 */
uint16_t zb_gp_get_commissioning_remaining_time(void);

/* ============================================================================
 * GP Device Management
 * ============================================================================ */

/**
 * @brief Add a GP device to the proxy table
 *
 * Manually adds a GP device. Normally devices are added through commissioning.
 *
 * @param[in] device    Device structure to add (gp_addr required)
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if proxy table is full
 * @return ESP_ERR_INVALID_ARG if device is NULL or invalid
 */
esp_err_t zb_gp_add_device(const zb_gp_device_t *device);

/**
 * @brief Remove a GP device from the proxy table
 *
 * Removes the device and its translation table entries.
 *
 * @param[in] gp_addr   GP device address to remove
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_gp_remove_device(const zb_gp_addr_t *gp_addr);

/**
 * @brief Remove a GP device by Source ID
 *
 * Convenience function for Source ID addressed devices.
 *
 * @param[in] src_id    Source ID to remove
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_gp_remove_device_by_src_id(zb_gp_src_id_t src_id);

/**
 * @brief Get a GP device by address
 *
 * Returns a pointer to the device in the proxy table.
 * The pointer is valid until the device is removed.
 *
 * @param[in] gp_addr   GP device address
 * @return Pointer to device or NULL if not found
 */
const zb_gp_device_t* zb_gp_get_device(const zb_gp_addr_t *gp_addr);

/**
 * @brief Get a GP device by Source ID
 *
 * Convenience function for Source ID addressed devices.
 *
 * @param[in] src_id    Source ID to look up
 * @return Pointer to device or NULL if not found
 */
const zb_gp_device_t* zb_gp_get_device_by_src_id(zb_gp_src_id_t src_id);

/**
 * @brief Get all GP devices
 *
 * Copies all commissioned GP devices to the provided array.
 *
 * @param[out] devices      Destination array
 * @param[in]  max_count    Maximum devices to copy
 * @return Number of devices copied
 */
size_t zb_gp_get_devices(zb_gp_device_t *devices, size_t max_count);

/**
 * @brief Get number of commissioned GP devices
 *
 * @return Number of devices in proxy table
 */
size_t zb_gp_get_device_count(void);

/**
 * @brief Set GP device friendly name
 *
 * @param[in] gp_addr   GP device address
 * @param[in] name      Friendly name (max ZB_GP_DEVICE_NAME_LEN-1 chars)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_INVALID_ARG if name is NULL or too long
 */
esp_err_t zb_gp_set_device_name(const zb_gp_addr_t *gp_addr, const char *name);

/* ============================================================================
 * GP Translation Table
 * ============================================================================ */

/**
 * @brief Add a translation table entry
 *
 * Maps a GP command from a device to a ZCL action.
 *
 * @param[in] entry     Translation entry to add
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if translation table is full
 * @return ESP_ERR_INVALID_ARG if entry is NULL or invalid
 */
esp_err_t zb_gp_add_translation(const zb_gp_translation_entry_t *entry);

/**
 * @brief Remove a translation table entry
 *
 * @param[in] gp_addr   GP device address
 * @param[in] gp_cmd_id GP command ID
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if entry not found
 */
esp_err_t zb_gp_remove_translation(const zb_gp_addr_t *gp_addr, uint8_t gp_cmd_id);

/**
 * @brief Remove all translations for a device
 *
 * @param[in] gp_addr   GP device address
 * @return ESP_OK on success
 */
esp_err_t zb_gp_remove_all_translations(const zb_gp_addr_t *gp_addr);

/**
 * @brief Get translations for a device
 *
 * @param[in]  gp_addr      GP device address
 * @param[out] entries      Destination array
 * @param[in]  max_count    Maximum entries to return
 * @return Number of entries copied
 */
size_t zb_gp_get_translations(const zb_gp_addr_t *gp_addr,
                               zb_gp_translation_entry_t *entries,
                               size_t max_count);

/* ============================================================================
 * GP Frame Handling
 * ============================================================================ */

/**
 * @brief Handle incoming GP notification
 *
 * Called by the Zigbee stack when a GP Notification is received.
 * Processes the GP frame and triggers translations or callbacks.
 *
 * @param[in] frame     GP frame to process
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if frame is NULL
 * @return ESP_ERR_NOT_FOUND if device not in proxy table
 */
esp_err_t zb_gp_handle_notification(const zb_gp_frame_t *frame);

/**
 * @brief Handle incoming GP commissioning notification
 *
 * Called when a GP commissioning frame is received.
 *
 * @param[in] frame     Commissioning frame to process
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not in commissioning mode
 */
esp_err_t zb_gp_handle_commissioning(const zb_gp_commissioning_frame_t *frame);

/**
 * @brief Parse raw GP frame data
 *
 * Parses a raw GP frame into the structured format.
 *
 * @param[in]  data         Raw frame data
 * @param[in]  data_len     Frame data length
 * @param[out] frame        Output parsed frame
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if parameters are invalid
 * @return ESP_ERR_INVALID_SIZE if frame is too short
 */
esp_err_t zb_gp_parse_frame(const uint8_t *data, size_t data_len, zb_gp_frame_t *frame);

/* ============================================================================
 * GP Security
 * ============================================================================ */

/**
 * @brief Set GP shared key
 *
 * Sets the shared GP security key used for devices without individual keys.
 *
 * @param[in] key       Security key (16 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if key is NULL
 */
esp_err_t zb_gp_set_shared_key(const uint8_t key[ZB_GP_SECURITY_KEY_SIZE]);

/**
 * @brief Get GP shared key
 *
 * @param[out] key      Output buffer for key (16 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if key is NULL
 */
esp_err_t zb_gp_get_shared_key(uint8_t key[ZB_GP_SECURITY_KEY_SIZE]);

/**
 * @brief Set device individual key
 *
 * Sets an individual security key for a specific GP device.
 *
 * @param[in] gp_addr   GP device address
 * @param[in] key       Security key (16 bytes)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 * @return ESP_ERR_INVALID_ARG if parameters invalid
 */
esp_err_t zb_gp_set_device_key(const zb_gp_addr_t *gp_addr,
                                const uint8_t key[ZB_GP_SECURITY_KEY_SIZE]);

/**
 * @brief Verify GP frame security
 *
 * Verifies the MIC of a secured GP frame.
 *
 * @param[in] device    GP device (for key and frame counter)
 * @param[in] frame     GP frame to verify
 * @return ESP_OK if verification passes
 * @return ESP_ERR_INVALID_CRC if MIC is invalid
 * @return ESP_ERR_INVALID_STATE if replay attack detected
 */
esp_err_t zb_gp_verify_security(const zb_gp_device_t *device,
                                 const zb_gp_frame_t *frame);

/* ============================================================================
 * GP Callbacks
 * ============================================================================ */

/**
 * @brief Register GP frame callback
 *
 * Called when a valid GP frame is received from a commissioned device.
 *
 * @param[in] callback  Callback function
 * @return ESP_OK on success
 */
esp_err_t zb_gp_register_frame_callback(zb_gp_frame_callback_t callback);

/**
 * @brief Register GP commissioning callback
 *
 * Called during commissioning events (start, device found, complete).
 *
 * @param[in] callback  Callback function
 * @return ESP_OK on success
 */
esp_err_t zb_gp_register_commissioning_callback(zb_gp_commissioning_callback_t callback);

/**
 * @brief Register GP device change callback
 *
 * Called when a device is added or removed from the proxy table.
 *
 * @param[in] callback  Callback function
 * @return ESP_OK on success
 */
esp_err_t zb_gp_register_device_callback(zb_gp_device_callback_t callback);

/* ============================================================================
 * GP Statistics and Diagnostics
 * ============================================================================ */

/**
 * @brief Get GP statistics
 *
 * @param[out] stats    Output statistics structure
 * @return ESP_OK on success
 */
esp_err_t zb_gp_get_stats(zb_gp_stats_t *stats);

/**
 * @brief Reset GP statistics
 *
 * @return ESP_OK on success
 */
esp_err_t zb_gp_reset_stats(void);

/**
 * @brief Get GP device type name
 *
 * Returns a human-readable name for a GP device type.
 *
 * @param[in] device_type   Device type ID
 * @return Device type name string
 */
const char* zb_gp_device_type_to_string(zb_gp_device_type_t device_type);

/**
 * @brief Get GP command name
 *
 * Returns a human-readable name for a GP command.
 *
 * @param[in] cmd_id    GP command ID
 * @return Command name string
 */
const char* zb_gp_cmd_to_string(uint8_t cmd_id);

/* ============================================================================
 * GP NVS Persistence
 * ============================================================================ */

/**
 * @brief Save GP state to NVS
 *
 * Saves proxy table, translation table, and configuration to NVS.
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_gp_save_to_nvs(void);

/**
 * @brief Load GP state from NVS
 *
 * Loads previously saved GP state from NVS.
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if no saved state exists
 * @return ESP_FAIL on NVS error
 */
esp_err_t zb_gp_load_from_nvs(void);

/**
 * @brief Clear GP state from NVS
 *
 * Removes all saved GP state from NVS.
 *
 * @return ESP_OK on success
 */
esp_err_t zb_gp_clear_nvs(void);

/* ============================================================================
 * GP MQTT Integration
 * ============================================================================ */

/**
 * @brief Publish GP devices list to MQTT
 *
 * Publishes the list of commissioned GP devices to
 * zigbee2mqtt/bridge/greenpower/devices
 *
 * @return ESP_OK on success
 */
esp_err_t zb_gp_publish_devices(void);

/**
 * @brief Publish GP device state to MQTT
 *
 * Publishes the current state of a GP device based on the last
 * received command.
 *
 * @param[in] device    GP device
 * @param[in] frame     Last received frame
 * @return ESP_OK on success
 */
esp_err_t zb_gp_publish_device_state(const zb_gp_device_t *device,
                                      const zb_gp_frame_t *frame);

/**
 * @brief Handle MQTT commissioning request
 *
 * Processes zigbee2mqtt/bridge/request/greenpower/commission
 *
 * @param[in] payload   JSON payload (optional parameters)
 * @return ESP_OK on success
 */
esp_err_t zb_gp_mqtt_handle_commission_request(const char *payload);

/**
 * @brief Handle MQTT remove request
 *
 * Processes zigbee2mqtt/bridge/request/greenpower/remove/{gp_id}
 *
 * @param[in] gp_id_str Source ID string (hex format)
 * @return ESP_OK on success
 * @return ESP_ERR_NOT_FOUND if device not found
 */
esp_err_t zb_gp_mqtt_handle_remove_request(const char *gp_id_str);

/* ============================================================================
 * GP Self-Test
 * ============================================================================ */

/**
 * @brief Self-test function for GP module
 *
 * Tests GP initialization, device management, and translation table.
 *
 * @return ESP_OK if all tests pass
 * @return ESP_FAIL if any test fails
 */
esp_err_t zb_gp_self_test(void);

#ifdef __cplusplus
}
#endif

#endif /* ZB_GREEN_POWER_H */
