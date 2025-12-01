import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

MULTI_CONF = True

netgear_m5_ns = cg.esphome_ns.namespace("netgear_m5")
NetgearM5Component = netgear_m5_ns.class_("NetgearM5Component", cg.Component)

# Exported constants for use by platform sensors
CONF_NETGEAR_M5_ID = "netgear_m5_id"
CONF_HOST = "host"
CONF_PASSWORD = "password"
CONF_POLL_INTERVAL = "poll_interval"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NetgearM5Component),
        cv.Required(CONF_HOST): cv.string,
        cv.Required(CONF_PASSWORD): cv.string,
        cv.Optional(CONF_POLL_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))