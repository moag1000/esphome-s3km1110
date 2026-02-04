/**
 * @file json_helpers.h
 * @brief JSON creation helper macros with automatic NULL checking
 *
 * Provides macros to reduce boilerplate code when creating cJSON objects
 * and arrays with proper error handling.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */
#ifndef JSON_HELPERS_H
#define JSON_HELPERS_H

#include "cJSON.h"
#include "esp_log.h"

/**
 * @brief Create a cJSON object with automatic NULL check
 *
 * Usage:
 *   JSON_OBJECT_CREATE_OR_RETURN(json, ESP_ERR_NO_MEM);
 *   // json is now guaranteed non-NULL
 *
 * @param var Variable name for the created object
 * @param error_return Value to return on failure
 */
#define JSON_OBJECT_CREATE_OR_RETURN(var, error_return) \
    cJSON *var = cJSON_CreateObject(); \
    if ((var) == NULL) { \
        ESP_LOGE(TAG, "Failed to create JSON object"); \
        return (error_return); \
    }

/**
 * @brief Create a cJSON array with automatic NULL check
 *
 * Usage:
 *   JSON_ARRAY_CREATE_OR_RETURN(arr, ESP_ERR_NO_MEM);
 *
 * @param var Variable name for the created array
 * @param error_return Value to return on failure
 */
#define JSON_ARRAY_CREATE_OR_RETURN(var, error_return) \
    cJSON *var = cJSON_CreateArray(); \
    if ((var) == NULL) { \
        ESP_LOGE(TAG, "Failed to create JSON array"); \
        return (error_return); \
    }

/**
 * @brief Create a cJSON object with cleanup on failure
 *
 * Usage:
 *   JSON_OBJECT_CREATE_OR_CLEANUP(json, parent, cleanup_label);
 *
 * @param var Variable name for the created object
 * @param cleanup_obj Object to delete on failure (can be NULL)
 * @param cleanup_label Label to goto on failure
 */
#define JSON_OBJECT_CREATE_OR_CLEANUP(var, cleanup_obj, cleanup_label) \
    cJSON *var = cJSON_CreateObject(); \
    if ((var) == NULL) { \
        ESP_LOGE(TAG, "Failed to create JSON object"); \
        if ((cleanup_obj) != NULL) cJSON_Delete(cleanup_obj); \
        goto cleanup_label; \
    }

/**
 * @brief Create a cJSON array with cleanup on failure
 *
 * Usage:
 *   JSON_ARRAY_CREATE_OR_CLEANUP(arr, parent, cleanup_label);
 *
 * @param var Variable name for the created array
 * @param cleanup_obj Object to delete on failure (can be NULL)
 * @param cleanup_label Label to goto on failure
 */
#define JSON_ARRAY_CREATE_OR_CLEANUP(var, cleanup_obj, cleanup_label) \
    cJSON *var = cJSON_CreateArray(); \
    if ((var) == NULL) { \
        ESP_LOGE(TAG, "Failed to create JSON array"); \
        if ((cleanup_obj) != NULL) cJSON_Delete(cleanup_obj); \
        goto cleanup_label; \
    }

/**
 * @brief Serialize JSON and check result
 *
 * Usage:
 *   JSON_TO_STRING_OR_RETURN(json_str, json_obj, ESP_FAIL);
 *
 * @param str_var Variable name for the string
 * @param json_obj JSON object to serialize (will be deleted)
 * @param error_return Value to return on failure
 */
#define JSON_TO_STRING_OR_RETURN(str_var, json_obj, error_return) \
    char *str_var = cJSON_PrintUnformatted(json_obj); \
    cJSON_Delete(json_obj); \
    if ((str_var) == NULL) { \
        ESP_LOGE(TAG, "Failed to serialize JSON"); \
        return (error_return); \
    }

#endif /* JSON_HELPERS_H */
