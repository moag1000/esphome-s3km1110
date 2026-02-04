import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart", "md5"]

zigbee_bridge_ns = cg.esphome_ns.namespace("zigbee_bridge")
ZigbeeBridge = zigbee_bridge_ns.class_(
    "ZigbeeBridge", cg.Component, uart.UARTDevice
)

# Actions
FactoryResetAction = zigbee_bridge_ns.class_("FactoryResetAction", automation.Action)
RebootAction = zigbee_bridge_ns.class_("RebootAction", automation.Action)
PermitJoinAction = zigbee_bridge_ns.class_("PermitJoinAction", automation.Action)
SetBridgeModeUartAction = zigbee_bridge_ns.class_("SetBridgeModeUartAction", automation.Action)
SetBridgeModeMqttAction = zigbee_bridge_ns.class_("SetBridgeModeMqttAction", automation.Action)

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

# MQTT Mode config keys (for WiFi-capable coordinators like C5+)
CONF_MQTT_MODE = "mqtt_mode"
CONF_MQTT_BROKER = "broker"
CONF_MQTT_PORT = "port"
CONF_MQTT_USERNAME = "username"
CONF_MQTT_PASSWORD = "password"
CONF_MQTT_PREFIX = "topic_prefix"
# Note: mqtt_mode_switch and c5_wifi_capable are now defined via platforms
# (switch.py and binary_sensor.py)

# MQTT Mode sub-schema (optional, for WiFi-capable coordinators)
MQTT_MODE_SCHEMA = cv.Schema({
    cv.Required(CONF_MQTT_BROKER): cv.string,
    cv.Optional(CONF_MQTT_PORT, default=1883): cv.port,
    cv.Optional(CONF_MQTT_USERNAME, default=""): cv.string,
    cv.Optional(CONF_MQTT_PASSWORD, default=""): cv.string,
    cv.Optional(CONF_MQTT_PREFIX, default="zigbee2mqtt"): cv.string,
})

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
            # MQTT Mode (optional, for WiFi-capable coordinators like C5+)
            # Switch and binary sensor are defined via platforms (switch.py, binary_sensor.py)
            cv.Optional(CONF_MQTT_MODE): MQTT_MODE_SCHEMA,
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

    # MQTT Mode configuration (optional, for WiFi-capable coordinators)
    # Switch and binary sensor are linked via platforms (switch.py, binary_sensor.py)
    if CONF_MQTT_MODE in config:
        mqtt_conf = config[CONF_MQTT_MODE]
        cg.add(var.set_mqtt_broker(mqtt_conf[CONF_MQTT_BROKER]))
        cg.add(var.set_mqtt_port(mqtt_conf[CONF_MQTT_PORT]))
        cg.add(var.set_mqtt_username(mqtt_conf[CONF_MQTT_USERNAME]))
        cg.add(var.set_mqtt_password(mqtt_conf[CONF_MQTT_PASSWORD]))
        cg.add(var.set_mqtt_prefix(mqtt_conf[CONF_MQTT_PREFIX]))


# ============================================================================
# Action Schemas (for use in automations and Home Assistant services)
# ============================================================================

ZIGBEE_BRIDGE_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(ZigbeeBridge),
})

@automation.register_action(
    "zigbee_bridge.factory_reset",
    FactoryResetAction,
    ZIGBEE_BRIDGE_ACTION_SCHEMA,
)
async def factory_reset_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "zigbee_bridge.reboot",
    RebootAction,
    ZIGBEE_BRIDGE_ACTION_SCHEMA,
)
async def reboot_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


PERMIT_JOIN_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(ZigbeeBridge),
    cv.Optional("duration", default=180): cv.templatable(cv.uint16_t),
})

@automation.register_action(
    "zigbee_bridge.permit_join",
    PermitJoinAction,
    PERMIT_JOIN_ACTION_SCHEMA,
)
async def permit_join_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config["duration"], args, cg.uint16)
    cg.add(var.set_duration(template_))
    return var


# ============================================================================
# Bridge Mode Actions (for WiFi-capable coordinators like C5+)
# ============================================================================

@automation.register_action(
    "zigbee_bridge.set_mode_uart",
    SetBridgeModeUartAction,
    ZIGBEE_BRIDGE_ACTION_SCHEMA,
)
async def set_mode_uart_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@automation.register_action(
    "zigbee_bridge.set_mode_mqtt",
    SetBridgeModeMqttAction,
    ZIGBEE_BRIDGE_ACTION_SCHEMA,
)
async def set_mode_mqtt_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
