import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG
from . import zigbee_bridge_ns, ZigbeeBridge

DEPENDENCIES = ["zigbee_bridge"]

# Define MqttModeSwitch class
MqttModeSwitch = zigbee_bridge_ns.class_("MqttModeSwitch", switch.Switch)

CONF_ZIGBEE_BRIDGE_ID = "zigbee_bridge_id"

CONFIG_SCHEMA = switch.switch_schema(
    MqttModeSwitch,
    entity_category=ENTITY_CATEGORY_CONFIG,
    icon="mdi:server-network",
).extend(
    {
        cv.Required(CONF_ZIGBEE_BRIDGE_ID): cv.use_id(ZigbeeBridge),
    }
)


async def to_code(config):
    var = await switch.new_switch(config)
    parent = await cg.get_variable(config[CONF_ZIGBEE_BRIDGE_ID])
    cg.add(parent.set_mqtt_mode_switch(var))
