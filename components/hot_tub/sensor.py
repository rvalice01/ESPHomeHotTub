import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, ICON_THERMOMETER

hot_tub_ns = cg.esphome_ns.namespace("hot_tub")
HotTub = hot_tub_ns.class_("HotTub", cg.Component)

CONF_TEMPERATURE = "temperature"
CONF_PRESSURE_SWITCH_STATE = "pressure_switch_state"
CONF_PUMP_1_STATE = "pump_1_state"
CONF_PUMP_2_STATE = "pump_2_state"
CONF_HEATING_ACTIVE = "heating_active"
CONF_ERROR_MESSAGES = "error_messages"

CONFIG_SCHEMA = cv.Schema(
    {
        # Key point: GenerateID makes it optional in YAML but always present in config.
        cv.GenerateID(): cv.declare_id(HotTub),

        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement="°F",
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_PRESSURE_SWITCH_STATE): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_PUMP_1_STATE): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_PUMP_2_STATE): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_HEATING_ACTIVE): sensor.sensor_schema(accuracy_decimals=0),
        cv.Optional(CONF_ERROR_MESSAGES): sensor.sensor_schema(accuracy_decimals=0),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TEMPERATURE in config:
        s = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(s))

    if CONF_PRESSURE_SWITCH_STATE in config:
        s = await sensor.new_sensor(config[CONF_PRESSURE_SWITCH_STATE])
        cg.add(var.set_pressure_switch_sensor(s))

    if CONF_PUMP_1_STATE in config:
        s = await sensor.new_sensor(config[CONF_PUMP_1_STATE])
        cg.add(var.set_pump_1_sensor(s))

    if CONF_PUMP_2_STATE in config:
        s = await sensor.new_sensor(config[CONF_PUMP_2_STATE])
        cg.add(var.set_pump_2_sensor(s))

    if CONF_HEATING_ACTIVE in config:
        s = await sensor.new_sensor(config[CONF_HEATING_ACTIVE])
        cg.add(var.set_heating_active_sensor(s))

    if CONF_ERROR_MESSAGES in config:
        s = await sensor.new_sensor(config[CONF_ERROR_MESSAGES])
        cg.add(var.set_error_messages_sensor(s))