# ============================================================================
# ESP32-H2 Zigbee Coordinator - Main Component
# ============================================================================
# Chip-specific code only. The Zigbee stack is in the shared esp_zb_coordinator
# component located in ../components/esp_zb_coordinator.
#
# This component contains:
# - main.c: Application entry point
# - uart/uart_bridge.c: H2-specific UART hardware layer
# - led/led_controller.c: H2-specific LED control (stub - H2-Zero has no LED)
# ============================================================================

set(SOURCES
    "main.c"

    # H2-specific LED controller (stub for H2-Zero which has no LED)
    "led/led_controller.c"
)

# Note: uart_bridge.c is now in the shared esp_zb_coordinator component

set(INCLUDE_DIRS
    "."
    "uart"
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
        json
    PRIV_REQUIRES
        spi_flash
)
