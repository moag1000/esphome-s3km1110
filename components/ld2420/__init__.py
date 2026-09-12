import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@descipher"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

ld2420_ns = cg.esphome_ns.namespace("ld2420")
LD2420Component = ld2420_ns.class_("LD2420Component", cg.Component, uart.UARTDevice)

CONF_LD2420_ID = "ld2420_id"

# Pin numbers for the UART recovery cycle. It tears the port down and rebuilds
# it, so it has to know which pins to rebuild it on; there is no public getter
# on the UART component. Defaults match the wiring this component shipped with.
CONF_RECOVERY_TX_PIN = "recovery_tx_pin"
CONF_RECOVERY_RX_PIN = "recovery_rx_pin"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2420Component),
            cv.Optional(CONF_RECOVERY_TX_PIN, default=17): cv.int_range(min=0, max=48),
            cv.Optional(CONF_RECOVERY_RX_PIN, default=18): cv.int_range(min=0, max=48),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2420_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_recovery_pins(config[CONF_RECOVERY_TX_PIN], config[CONF_RECOVERY_RX_PIN]))
