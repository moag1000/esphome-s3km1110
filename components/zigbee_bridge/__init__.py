import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

zigbee_bridge_ns = cg.esphome_ns.namespace("zigbee_bridge")
ZigbeeBridge = zigbee_bridge_ns.class_(
    "ZigbeeBridge", cg.Component, uart.UARTDevice
)

CONF_COORDINATOR_STATUS = "coordinator_status_id"
CONF_LAST_EVENT = "last_event_id"
CONF_NETWORK_INFO = "network_info_id"
CONF_DEVICE_COUNT = "device_count_id"
CONF_ZIGBEE_CHANNEL = "zigbee_channel_id"
CONF_COORDINATOR_READY = "coordinator_ready_id"
CONF_PERMIT_JOIN = "permit_join_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZigbeeBridge),
            cv.Optional(CONF_COORDINATOR_STATUS): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_LAST_EVENT): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_NETWORK_INFO): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_DEVICE_COUNT): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_ZIGBEE_CHANNEL): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_COORDINATOR_READY): cv.use_id(cg.EntityBase),
            cv.Optional(CONF_PERMIT_JOIN): cv.use_id(cg.EntityBase),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

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
