# ============================================================================
# ESP32-C5+ Zigbee Coordinator - Main Component
# ============================================================================
# Chip-specific code only. The Zigbee stack is in the shared esp_zb_coordinator
# component located in ../components/esp_zb_coordinator.
#
# This component contains:
# - main.c: Application entry point
# - wifi/wifi_manager.c: WiFi STA + mDNS
# - led/led_controller.c: C5-specific LED control
# ============================================================================

set(SOURCES
    "main.c"

    # C5+-specific WiFi manager
    "wifi/wifi_manager.c"

    # C5-specific LED controller
    "led/led_controller.c"
)

# Note: uart_bridge.c is now in the shared esp_zb_coordinator component

set(INCLUDE_DIRS
    "."
    "uart"
    "wifi"
    "led"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES
        esp_zb_coordinator
        nvs_flash
        driver
        esp_timer
        esp_event
        esp_wifi
        esp_netif
        mdns
        led_strip
        json
    PRIV_REQUIRES
        spi_flash
)
