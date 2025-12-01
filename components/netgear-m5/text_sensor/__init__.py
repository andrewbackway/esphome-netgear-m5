import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from .. import netgear_m5_ns, NetgearM5Component, CONF_NETGEAR_M5_ID

DEPENDENCIES = ["netgear-m5"]

CONF_PATH = "path"

NetgearM5TextSensor = netgear_m5_ns.class_(
    "NetgearM5TextSensor", text_sensor.TextSensor, cg.Component
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(NetgearM5TextSensor)
    .extend(
        {
            cv.GenerateID(CONF_NETGEAR_M5_ID): cv.use_id(NetgearM5Component),
            cv.Required(CONF_PATH): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_NETGEAR_M5_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_path(config[CONF_PATH]))
