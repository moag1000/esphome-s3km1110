import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC, DEVICE_CLASS_CONNECTIVITY
from . import zigbee_bridge_ns, ZigbeeBridge

DEPENDENCIES = ["zigbee_bridge"]

CONF_ZIGBEE_BRIDGE_ID = "zigbee_bridge_id"
CONF_C5_WIFI_CAPABLE = "c5_wifi_capable"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ZIGBEE_BRIDGE_ID): cv.use_id(ZigbeeBridge),
        cv.Optional(CONF_C5_WIFI_CAPABLE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:wifi",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ZIGBEE_BRIDGE_ID])

    if CONF_C5_WIFI_CAPABLE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_C5_WIFI_CAPABLE])
        cg.add(parent.set_c5_wifi_capable_sensor(sens))
