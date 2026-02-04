/**
 * @file wifi_manager.c
 * @brief WiFi Manager for ESP32-C5+ Zigbee Coordinator
 */

#include "wifi_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "mdns.h"
#include "lwip/inet.h"

static const char *TAG = "WIFI_MGR";

/* Event group bits */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* Configuration from Kconfig */
#define WIFI_SSID           CONFIG_WIFI_SSID
#define WIFI_PASSWORD       CONFIG_WIFI_PASSWORD
#define WIFI_MAX_RETRY      CONFIG_WIFI_MAX_RETRY
#define MDNS_HOSTNAME       CONFIG_MDNS_HOSTNAME

/* 5GHz preference: fallback to 2.4GHz after this many failures */
#define WIFI_5GHZ_RETRY_LIMIT  3

/* State */
static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_sta_netif = NULL;
static int s_retry_num = 0;
static bool s_connected = false;
static bool s_5ghz_mode = true;
static esp_ip4_addr_t s_ip_addr = {0};

/* ============================================================================
 * Event Handlers
 * ============================================================================ */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi STA started, connecting...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *event =
                    (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(TAG, "Disconnected from AP (reason: %d)", event->reason);
                s_connected = false;

                if (s_retry_num < WIFI_MAX_RETRY) {
                    s_retry_num++;

                    /* After 3 failures on 5GHz, fall back to AUTO (allows 2.4GHz) */
                    if (s_5ghz_mode && s_retry_num == WIFI_5GHZ_RETRY_LIMIT) {
                        ESP_LOGW(TAG, "5GHz failed %d times, falling back to AUTO mode", WIFI_5GHZ_RETRY_LIMIT);
                        ESP_LOGW(TAG, "WARNING: 2.4GHz WiFi may interfere with Zigbee!");
                        esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
                        s_5ghz_mode = false;
                    }

                    ESP_LOGI(TAG, "Retry %d/%d (%s)...", s_retry_num, WIFI_MAX_RETRY,
                             s_5ghz_mode ? "5GHz" : "AUTO");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(TAG, "Failed to connect after %d attempts", WIFI_MAX_RETRY);
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            }

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to AP, waiting for IP...");
                s_retry_num = 0;
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_ip_addr = event->ip_info.ip;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        s_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

esp_err_t wifi_manager_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing WiFi (SSID: %s)", WIFI_SSID);

    /* Create event group */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize TCP/IP stack */
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create default event loop (may already exist) */
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create default WiFi STA netif */
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create STA netif");
        return ESP_FAIL;
    }

    /* Initialize WiFi with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register event handlers */
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WIFI_EVENT handler");
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP_EVENT handler");
        return ret;
    }

    /* Configure WiFi */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* STRONG 5GHz preference to avoid interference with Zigbee (2.4GHz) */
    ESP_LOGI(TAG, "Setting WiFi band: 5GHz ONLY (fallback to AUTO after %d failures)", WIFI_5GHZ_RETRY_LIMIT);
    ret = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set 5GHz-only mode: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "WARNING: 2.4GHz WiFi may interfere with Zigbee!");
        s_5ghz_mode = false;
    }

    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start WiFi */
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait for connection or failure */
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t wifi_manager_start_mdns(void)
{
    esp_err_t ret;

    if (!s_connected) {
        ESP_LOGW(TAG, "Cannot start mDNS: WiFi not connected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting mDNS (hostname: %s)", MDNS_HOSTNAME);

    ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mdns_hostname_set(MDNS_HOSTNAME);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mdns_hostname_set failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mdns_instance_name_set("ESP32-C5+ Zigbee Coordinator");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mdns_instance_name_set failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Announce Zigbee coordinator service */
    ret = mdns_service_add("Zigbee Coordinator", "_zigbee", "_tcp", 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mdns_service_add failed (non-critical): %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "mDNS started: %s.local", MDNS_HOSTNAME);
    return ESP_OK;
}

bool wifi_manager_is_connected(void)
{
    return s_connected;
}

esp_err_t wifi_manager_get_ip(char *buf, size_t buf_len)
{
    if (!s_connected || buf == NULL || buf_len < 16) {
        return ESP_ERR_INVALID_STATE;
    }

    snprintf(buf, buf_len, IPSTR, IP2STR(&s_ip_addr));
    return ESP_OK;
}

int8_t wifi_manager_get_rssi(void)
{
    if (!s_connected) {
        return 0;
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}
