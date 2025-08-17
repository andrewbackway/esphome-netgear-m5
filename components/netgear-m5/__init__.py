import esphome.codegen as cg
import esphome.config_validation as cv

import esphome.components.sensor as sensor
import esphome.components.text_sensor as text_sensor

from esphome.const import CONF_ID

AUTO_LOAD = ["sensor", "text_sensor"]
CODEOWNERS = ["@you"]

# C++ namespace/class (unchanged)
netgear_m5_ns = cg.esphome_ns.namespace("netgear_m5")
NetgearM5Component = netgear_m5_ns.class_("NetgearM5Component", cg.Component)

# Config keys
CONF_HOST = "host"
CONF_POLL_INTERVAL = "poll_interval"
CONF_SENSORS = "sensors"
CONF_TEXT_SENSORS = "text_sensors"
CONF_PATH = "path"

# Use new schema builders (SENSOR_SCHEMA is deprecated)
SENSOR_SCHEMA = sensor.sensor_schema().extend(
    {cv.Required(CONF_PATH): cv.string}
)

TEXT_SENSOR_SCHEMA = text_sensor.text_sensor_schema().extend(
    {cv.Required(CONF_PATH): cv.string}
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NetgearM5Component),
        cv.Required(CONF_HOST): cv.string,
        cv.Optional(CONF_POLL_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(SENSOR_SCHEMA),
        cv.Optional(CONF_TEXT_SENSORS, default=[]): cv.ensure_list(TEXT_SENSOR_SCHEMA),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))

    # Numeric sensors
    for s_conf in config[CONF_SENSORS]:
        s = await sensor.new_sensor(s_conf)
        cg.add(var.bind_numeric_sensor(s_conf[CONF_PATH], s))

    # Text sensors
    for ts_conf in config[CONF_TEXT_SENSORS]:
        ts = await text_sensor.new_text_sensor(ts_conf)
        cg.add(var.bind_text_sensor(ts_conf[CONF_PATH], ts))
