/**
 * @file version.c
 * @brief Version Management Implementation
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#include "version.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "VERSION";

/* Static buffer for full version string */
static char s_version_full[128];
static bool s_version_formatted = false;

/**
 * @brief Get full version string
 */
const char* version_get_string(void)
{
    if (!s_version_formatted) {
        snprintf(s_version_full, sizeof(s_version_full),
                "v%s (%s %s) [%s]",
                FIRMWARE_VERSION, BUILD_DATE, BUILD_TIME, GIT_COMMIT);
        s_version_formatted = true;
    }
    return s_version_full;
}

/**
 * @brief Get version number only
 */
const char* version_get_number(void)
{
    return FIRMWARE_VERSION;
}

/**
 * @brief Get build date string
 */
const char* version_get_build_date(void)
{
    return BUILD_DATE;
}

/**
 * @brief Get build time string
 */
const char* version_get_build_time(void)
{
    return BUILD_TIME;
}

/**
 * @brief Get git commit hash
 */
const char* version_get_git_commit(void)
{
    return GIT_COMMIT;
}

/**
 * @brief Get project name
 */
const char* version_get_project_name(void)
{
    return PROJECT_NAME;
}

/**
 * @brief Print version information to console
 */
void version_print(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  %s", PROJECT_NAME);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "Build Date: %s", BUILD_DATE);
    ESP_LOGI(TAG, "Build Time: %s", BUILD_TIME);
    ESP_LOGI(TAG, "Git Commit: %s", GIT_COMMIT);
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Get version as JSON
 */
int version_get_json(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return -1;
    }

    int written = snprintf(buffer, buffer_size,
        "{"
        "\"project\":\"%s\","
        "\"version\":\"%s\","
        "\"build_date\":\"%s\","
        "\"build_time\":\"%s\","
        "\"git_commit\":\"%s\""
        "}",
        PROJECT_NAME, FIRMWARE_VERSION, BUILD_DATE, BUILD_TIME, GIT_COMMIT
    );

    if (written < 0 || (size_t)written >= buffer_size) {
        return -1;
    }

    return 0;
}

/**
 * @brief Parse version string into major.minor.patch
 */
static void parse_version(const char *version_str, int *major, int *minor, int *patch)
{
    *major = 0;
    *minor = 0;
    *patch = 0;

    if (version_str == NULL) {
        return;
    }

    sscanf(version_str, "%d.%d.%d", major, minor, patch);
}

/**
 * @brief Compare version strings
 */
int version_compare(const char *other_version)
{
    if (other_version == NULL) {
        return 1; /* Current version is newer than nothing */
    }

    int curr_major, curr_minor, curr_patch;
    int other_major, other_minor, other_patch;

    parse_version(FIRMWARE_VERSION, &curr_major, &curr_minor, &curr_patch);
    parse_version(other_version, &other_major, &other_minor, &other_patch);

    /* Compare major version */
    if (curr_major != other_major) {
        return curr_major - other_major;
    }

    /* Compare minor version */
    if (curr_minor != other_minor) {
        return curr_minor - other_minor;
    }

    /* Compare patch version */
    return curr_patch - other_patch;
}
