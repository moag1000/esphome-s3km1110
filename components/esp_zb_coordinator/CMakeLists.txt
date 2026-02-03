# ============================================================================
# ESP Zigbee Coordinator - Shared Component
# ============================================================================
# Provides the Zigbee coordinator stack for ESP32-C5 and ESP32-H2.
# Platform-specific code is selected automatically based on CONFIG_IDF_TARGET.
#
# Usage in project CMakeLists.txt:
#   set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../components")
#
# ============================================================================

# Auto-detect target chip and select HAL implementation
# Use IDF_TARGET (available at CMake time) instead of CONFIG_IDF_TARGET_*
if("${IDF_TARGET}" STREQUAL "esp32c5")
    set(ZB_HAL_SRC "platform/esp32c5/zb_hal_c5.c")
    set(ZB_TARGET_NAME "ESP32-C5")
elseif("${IDF_TARGET}" STREQUAL "esp32h2")
    set(ZB_HAL_SRC "platform/esp32h2/zb_hal_h2.c")
    set(ZB_TARGET_NAME "ESP32-H2")
else()
    message(FATAL_ERROR "esp_zb_coordinator: Unsupported target '${IDF_TARGET}'. "
                        "Supported: esp32c5, esp32h2")
endif()

message(STATUS "esp_zb_coordinator: Building for ${ZB_TARGET_NAME}")

# ============================================================================
# Core Sources
# ============================================================================
set(ZB_CORE_SOURCES
    "src/core/device_state.c"
    "src/core/command_handler.c"
    "src/core/zb_hal_led.c"
)

# ============================================================================
# OTA Module
# ============================================================================
set(ZB_OTA_SOURCES
    "src/ota/uart_ota.c"
)

# ============================================================================
# Zigbee Stack Sources
# ============================================================================
set(ZB_ZIGBEE_SOURCES
    "src/zigbee/zb_coordinator.c"
    "src/zigbee/zb_network.c"
    "src/zigbee/zb_callbacks.c"
    "src/zigbee/zb_device_handler.c"
    "src/zigbee/zb_interview.c"
    "src/zigbee/zb_reporting.c"
    "src/zigbee/zb_availability.c"
    "src/zigbee/zb_zcl_helpers.c"
    "src/zigbee/zb_binding.c"
    "src/zigbee/zb_groups.c"
    "src/zigbee/zb_scenes.c"
    "src/zigbee/zb_alarms.c"
    "src/zigbee/zb_diagnostics.c"
    "src/zigbee/zb_install_codes.c"
    "src/zigbee/zb_cmd_retry.c"
    "src/zigbee/zb_tuya.c"
    "src/zigbee/zb_green_power.c"
    "src/zigbee/zb_touchlink.c"
    "src/zigbee/zb_topology.c"
    "src/zigbee/zb_time_server.c"
    "src/zigbee/zb_poll_control.c"
    "src/zigbee/zb_demand_response.c"
    "src/zigbee/zb_hvac_dehumid.c"
    "src/zigbee/zb_multi_pan.c"
    "src/zigbee/zb_backup.c"
    "src/zigbee/zb_ota.c"
    "src/zigbee/zb_cluster_closures.c"
    "src/zigbee/zb_cluster_electrical.c"
    "src/zigbee/zb_cluster_hvac.c"
    "src/zigbee/zb_cluster_measurement.c"
    "src/zigbee/zb_cluster_security.c"
    "src/zigbee/zb_router.c"
)

# ============================================================================
# Device Driver Sources
# ============================================================================
set(ZB_DRIVER_SOURCES
    # Tuya drivers
    "src/zigbee/tuya/tuya_driver_registry.c"
    "src/zigbee/tuya/tuya_fingerbot.c"
    # Xiaomi/Aqara drivers
    "src/zigbee/xiaomi/xiaomi_tlv.c"
    "src/zigbee/xiaomi/aqara_vibration.c"
)

# ============================================================================
# UART Layer (protocol + bridge)
# ============================================================================
set(ZB_UART_SOURCES
    "src/uart/uart_protocol.c"
    "src/uart/uart_bridge.c"
)

# ============================================================================
# Utility Sources
# ============================================================================
set(ZB_UTIL_SOURCES
    "src/utils/json_utils.c"
    "src/utils/version.c"
)

# ============================================================================
# Combine all sources
# ============================================================================
set(ZB_ALL_SOURCES
    ${ZB_CORE_SOURCES}
    ${ZB_ZIGBEE_SOURCES}
    ${ZB_DRIVER_SOURCES}
    ${ZB_UART_SOURCES}
    ${ZB_UTIL_SOURCES}
    ${ZB_OTA_SOURCES}
    ${ZB_HAL_SRC}
)

# ============================================================================
# Include directories
# ============================================================================
set(ZB_INCLUDE_DIRS
    "include"
    "src"
    "src/core"
    "src/zigbee"
    "src/zigbee/tuya"
    "src/zigbee/xiaomi"
    "src/uart"
    "src/utils"
    "src/ota"
)

# ============================================================================
# Register component
# ============================================================================
idf_component_register(
    SRCS ${ZB_ALL_SOURCES}
    INCLUDE_DIRS ${ZB_INCLUDE_DIRS}
    REQUIRES
        nvs_flash
        driver
        esp_timer
        esp_event
        esp-zigbee-lib
        json
    PRIV_REQUIRES
        spi_flash
        mbedtls
        spiffs
        app_update
        bootloader_support
)

# ============================================================================
# H2 Memory Optimizations
# ============================================================================
if("${IDF_TARGET}" STREQUAL "esp32h2")
    message(STATUS "esp_zb_coordinator: Applying H2 memory optimizations")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        ZB_MEMORY_CONSTRAINED=1
    )
    # Additional size optimizations for H2
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Os
    )
endif()

# ============================================================================
# Version information
# ============================================================================
# Pass build info to the version module
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    COMPONENT_BUILD_DATE="${BUILD_TIMESTAMP}"
    COMPONENT_TARGET="${ZB_TARGET_NAME}"
)
