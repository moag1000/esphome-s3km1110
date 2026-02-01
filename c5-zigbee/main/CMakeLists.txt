set(SOURCES
    "main.c"

    # UART Bridge (replaces MQTT bridge)
    "uart/uart_bridge.c"
    "uart/uart_protocol.c"

    # Core
    "core/device_state.c"
    "core/command_handler.c"

    # Zigbee coordinator
    "zigbee/zb_coordinator.c"
    "zigbee/zb_network.c"
    "zigbee/zb_callbacks.c"
    "zigbee/zb_device_handler.c"
    "zigbee/zb_interview.c"
    "zigbee/zb_reporting.c"
    "zigbee/zb_availability.c"
    "zigbee/zb_zcl_helpers.c"
    "zigbee/zb_binding.c"
    "zigbee/zb_groups.c"
    "zigbee/zb_scenes.c"
    "zigbee/zb_alarms.c"
    "zigbee/zb_diagnostics.c"
    "zigbee/zb_install_codes.c"
    "zigbee/zb_cmd_retry.c"
    "zigbee/zb_tuya.c"
    "zigbee/zb_green_power.c"
    "zigbee/zb_touchlink.c"
    "zigbee/zb_topology.c"
    "zigbee/zb_time_server.c"
    "zigbee/zb_poll_control.c"
    "zigbee/zb_demand_response.c"
    "zigbee/zb_hvac_dehumid.c"
    "zigbee/zb_multi_pan.c"
    "zigbee/zb_backup.c"
    "zigbee/zb_ota.c"
    "zigbee/zb_cluster_closures.c"
    "zigbee/zb_cluster_electrical.c"
    "zigbee/zb_cluster_hvac.c"
    "zigbee/zb_cluster_measurement.c"
    "zigbee/zb_cluster_security.c"

    # Tuya
    "zigbee/tuya/tuya_driver_registry.c"
    "zigbee/tuya/tuya_fingerbot.c"

    # LED
    "led/led_controller.c"

    # Utils
    "utils/json_utils.c"
    "utils/version.c"
)

set(INCLUDE_DIRS
    "."
    "uart"
    "core"
    "zigbee"
    "zigbee/tuya"
    "led"
    "utils"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES
        nvs_flash
        driver
        esp_timer
        esp_event
        esp-zigbee-lib
        led_strip
        json
    PRIV_REQUIRES
        spi_flash
        mbedtls
        spiffs
)
