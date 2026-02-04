/**
 * @file wifi_manager.h
 * @brief WiFi Manager for ESP32-C5+ Zigbee Coordinator
 *
 * Handles WiFi STA connection with DHCP and mDNS registration.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi in STA mode
 *
 * Connects to configured WiFi network with DHCP.
 * Blocks until connected or timeout.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Start mDNS service
 *
 * Registers device with hostname from Kconfig.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_start_mdns(void);

/**
 * @brief Check if WiFi is connected
 *
 * @return true if connected, false otherwise
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Get current IP address as string
 *
 * @param buf Buffer to store IP string
 * @param buf_len Buffer length (min 16 bytes)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t wifi_manager_get_ip(char *buf, size_t buf_len);

/**
 * @brief Get WiFi RSSI
 *
 * @return RSSI value in dBm, or 0 if not connected
 */
int8_t wifi_manager_get_rssi(void);

#ifdef __cplusplus
}
#endif
