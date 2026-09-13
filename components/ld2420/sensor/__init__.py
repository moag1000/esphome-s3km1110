import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MOVING_DISTANCE,
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_CENTIMETER,
)

from .. import CONF_LD2420_ID, LD2420Component, ld2420_ns

TOTAL_GATES = 16

LD2420Sensor = ld2420_ns.class_("LD2420Sensor", sensor.Sensor, cg.Component)

CONF_GATE_ENERGY = "gate_energy"

# Per-gate energy, as gate_0 ... gate_15. This is the number each gate's
# threshold is compared against, so it is the only direct way to see why the
# radar decided what it decided — a gate sitting just under its threshold
# explains a miss, one riding above it explains a false trigger.

CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(LD2420Sensor),
            cv.GenerateID(CONF_LD2420_ID): cv.use_id(LD2420Component),
            cv.Optional(CONF_MOVING_DISTANCE): sensor.sensor_schema(
                device_class=DEVICE_CLASS_DISTANCE, unit_of_measurement=UNIT_CENTIMETER
            ),
            cv.Optional(CONF_GATE_ENERGY): cv.Schema(
                {
                    cv.Optional(f"gate_{gate}"): sensor.sensor_schema(
                        accuracy_decimals=0,
                        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                        icon="mdi:chart-histogram",
                    )
                    for gate in range(TOTAL_GATES)
                }
            ),
        }
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    if CONF_MOVING_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_MOVING_DISTANCE])
        cg.add(var.set_distance_sensor(sens))
    if gate_energy_config := config.get(CONF_GATE_ENERGY):
        for gate in range(TOTAL_GATES):
            if gate_config := gate_energy_config.get(f"gate_{gate}"):
                sens = await sensor.new_sensor(gate_config)
                cg.add(var.set_energy_sensor(gate, sens))
    ld2420 = await cg.get_variable(config[CONF_LD2420_ID])
    cg.add(ld2420.register_listener(var))
