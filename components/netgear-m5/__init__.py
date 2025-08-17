import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, UNIT_EMPTY, ICON_EMPTY

AUTO_LOAD = ["sensor", "text_sensor"]
CODEOWNERS = ["@you"]

netgear_m5_ns = cg.esphome_ns.namespace("netgear_m5")
NetgearM5Component = netgear_m5_ns.class_("NetgearM5Component", cg.Component)

CONF_HOST = "host"
CONF_POLL_INTERVAL = "poll_interval"
CONF_SENSORS = "sensors"
CONF_TEXT_SENSORS = "text_sensors"
CONF_PATH = "path"
CONF_UNIT = "unit"
CONF_ICON = "icon"

SENSOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NAME): cv.string,
        cv.Required(CONF_PATH): cv.string,
        cv.Optional(CONF_UNIT, default=UNIT_EMPTY): cv.string,
        cv.Optional(CONF_ICON, default=ICON_EMPTY): cv.icon,
    }
)

TEXT_SENSOR_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NAME): cv.string,
        cv.Required(CONF_PATH): cv.string,
        cv.Optional(CONF_ICON, default=ICON_EMPTY): cv.icon,
    }
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
    for s in config[CONF_SENSORS]:
        sens = await cg.new_sensor(s[CONF_NAME], s[CONF_UNIT], s[CONF_ICON])
        cg.add(var.bind_numeric_sensor(s[CONF_PATH], sens))

    # Text sensors
    for ts in config[CONF_TEXT_SENSORS]:
        sens = await cg.new_text_sensor(ts[CONF_NAME], ts[CONF_ICON])
        cg.add(var.bind_text_sensor(ts[CONF_PATH], sens))
