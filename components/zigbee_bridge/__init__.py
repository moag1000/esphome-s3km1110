import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart", "md5"]

zigbee_bridge_ns = cg.esphome_ns.namespace("zigbee_bridge")
ZigbeeBridge = zigbee_bridge_ns.class_(
    "ZigbeeBridge", cg.Component, uart.UARTDevice
)

# Existing config keys
CONF_COORDINATOR_STATUS = "coordinator_status_id"
CONF_LAST_EVENT = "last_event_id"
CONF_NETWORK_INFO = "network_info_id"
CONF_DEVICE_COUNT = "device_count_id"
CONF_ZIGBEE_CHANNEL = "zigbee_channel_id"
CONF_COORDINATOR_READY = "coordinator_ready_id"
CONF_PERMIT_JOIN = "permit_join_id"

# New OTA config keys
CONF_RESET_PIN = "reset_pin"
CONF_BOOT_PIN = "boot_pin"
CONF_OTA_PROGRESS = "ota_progress_id"
CONF_OTA_STATUS = "ota_status_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZigbeeBridge),
            # Existing sensors
            cv.Optional(CONF_COORDINATOR_STATUS): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_LAST_EVENT): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_NETWORK_INFO): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_DEVICE_COUNT): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_ZIGBEE_CHANNEL): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_COORDINATOR_READY): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_PERMIT_JOIN): cv.use_id(cg.EntityBase),
            # OTA sensors (optional)
            cv.Optional(CONF_OTA_PROGRESS): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_OTA_STATUS): cv.use_id(cg.EntityBase),
            # Hardware recovery pins (optional)
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_BOOT_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


    # Existing sensor bindings
    if CONF_COORDINATOR_STATUS in config:
        sens = await cg.get_variable(config[CONF_COORDINATOR_STATUS])
        cg.add(var.set_coordinator_status(sens))
    if CONF_LAST_EVENT in config:
        sens = await cg.get_variable(config[CONF_LAST_EVENT])
        cg.add(var.set_last_event(sens))
    if CONF_NETWORK_INFO in config:
        sens = await cg.get_variable(config[CONF_NETWORK_INFO])
        cg.add(var.set_network_info(sens))
    if CONF_DEVICE_COUNT in config:
        sens = await cg.get_variable(config[CONF_DEVICE_COUNT])
        cg.add(var.set_device_count(sens))
    if CONF_ZIGBEE_CHANNEL in config:
        sens = await cg.get_variable(config[CONF_ZIGBEE_CHANNEL])
        cg.add(var.set_zigbee_channel(sens))
    if CONF_COORDINATOR_READY in config:
        sens = await cg.get_variable(config[CONF_COORDINATOR_READY])
        cg.add(var.set_coordinator_ready(sens))
    if CONF_PERMIT_JOIN in config:
        sens = await cg.get_variable(config[CONF_PERMIT_JOIN])
        cg.add(var.set_permit_join_active(sens))

    # OTA sensor bindings (optional)
    if CONF_OTA_PROGRESS in config:
        sens = await cg.get_variable(config[CONF_OTA_PROGRESS])
        cg.add(var.set_ota_progress(sens))
    if CONF_OTA_STATUS in config:
        sens = await cg.get_variable(config[CONF_OTA_STATUS])
        cg.add(var.set_ota_status(sens))

    # Hardware recovery pin bindings (optional)
    if CONF_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(pin))
    if CONF_BOOT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_BOOT_PIN])
        cg.add(var.set_boot_pin(pin))
