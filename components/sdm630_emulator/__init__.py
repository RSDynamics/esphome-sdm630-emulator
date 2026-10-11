import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_PORT

CODEOWNERS = []
DEPENDENCIES = ["network"]
AUTO_LOAD = ["socket"]

sdm630_ns = cg.esphome_ns.namespace("sdm630_emulator")
SDM630Emulator = sdm630_ns.class_("SDM630Emulator", cg.Component)

CONF_UNIT_ID = "unit_id"
CONF_MAX_AGE = "max_age"
CONF_NOMINAL_VOLTAGE = "nominal_voltage"

PHASES = (1, 2, 3)
KEYS = (
    ("voltage_l{}", "set_voltage"),
    ("power_delivered_l{}", "set_power_delivered"),
    ("power_returned_l{}", "set_power_returned"),
)


def _schema():
    schema = {
        cv.GenerateID(): cv.declare_id(SDM630Emulator),
        cv.Optional(CONF_PORT, default=502): cv.port,
        cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=247),
        cv.Optional(CONF_MAX_AGE, default="10s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_NOMINAL_VOLTAGE, default=230.0): cv.float_range(min=100.0, max=300.0),
        # L1 import/export power are required: L1 import is also the freshness signal.
        cv.Required("power_delivered_l1"): cv.use_id(sensor.Sensor),
        cv.Required("power_returned_l1"): cv.use_id(sensor.Sensor),
    }
    for phase in PHASES:
        schema[cv.Optional(f"voltage_l{phase}")] = cv.use_id(sensor.Sensor)
        if phase > 1:
            schema[cv.Optional(f"power_delivered_l{phase}")] = cv.use_id(sensor.Sensor)
            schema[cv.Optional(f"power_returned_l{phase}")] = cv.use_id(sensor.Sensor)
    return cv.Schema(schema).extend(cv.COMPONENT_SCHEMA)


CONFIG_SCHEMA = _schema()


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(var.set_max_age(config[CONF_MAX_AGE]))
    cg.add(var.set_nominal_voltage(config[CONF_NOMINAL_VOLTAGE]))

    for phase in PHASES:
        for key_fmt, setter in KEYS:
            key = key_fmt.format(phase)
            if key in config:
                sens = await cg.get_variable(config[key])
                cg.add(getattr(var, setter)(phase - 1, sens))
