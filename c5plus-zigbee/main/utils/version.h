/**
 * @file version.h
 * @brief Version Management for Firmware
 *
 * Provides version information for the gateway firmware including
 * version string, build date/time, and git commit hash.
 *
 * @copyright Copyright (c) 2026
 * @license Apache License 2.0
 */

#ifndef VERSION_H
#define VERSION_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Firmware version string (semantic versioning)
 */
#define FIRMWARE_VERSION "1.0.0"

/**
 * @brief Build date (set by compiler)
 */
#define BUILD_DATE __DATE__

/**
 * @brief Build time (set by compiler)
 */
#define BUILD_TIME __TIME__

/**
 * @brief Git commit hash (to be filled by build script)
 *
 * Can be set via build flag: -DGIT_COMMIT=\"abc123\"
 */
#ifndef GIT_COMMIT
#define GIT_COMMIT "unknown"
#endif

/**
 * @brief Project name
 */
#define PROJECT_NAME "ESP32-C5 Zigbee UART Coordinator"

/**
 * @brief Get full version string
 *
 * Returns formatted version string including version, build date, and commit.
 * Format: "v1.0.0 (2026-01-23 12:34:56) [abc123]"
 *
 * @return Pointer to static version string
 */
const char* version_get_string(void);

/**
 * @brief Get version number only
 *
 * Returns just the semantic version number (e.g., "1.0.0")
 *
 * @return Pointer to version number string
 */
const char* version_get_number(void);

/**
 * @brief Get build date string
 *
 * @return Pointer to build date string
 */
const char* version_get_build_date(void);

/**
 * @brief Get build time string
 *
 * @return Pointer to build time string
 */
const char* version_get_build_time(void);

/**
 * @brief Get git commit hash
 *
 * @return Pointer to git commit hash string
 */
const char* version_get_git_commit(void);

/**
 * @brief Get project name
 *
 * @return Pointer to project name string
 */
const char* version_get_project_name(void);

/**
 * @brief Print version information to console
 *
 * Outputs formatted version information via ESP_LOGI
 */
void version_print(void);

/**
 * @brief Get version as JSON
 *
 * Generates JSON representation of version info.
 *
 * @param[out] buffer Buffer to store JSON string
 * @param[in] buffer_size Size of buffer
 * @return 0 on success, -1 if buffer too small
 */
int version_get_json(char *buffer, size_t buffer_size);

/**
 * @brief Compare version strings
 *
 * Compares current version with another version string.
 *
 * @param[in] other_version Version string to compare (e.g., "1.0.1")
 * @return  0 if versions are equal
 *         <0 if current version is older
 *         >0 if current version is newer
 */
int version_compare(const char *other_version);

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H */
