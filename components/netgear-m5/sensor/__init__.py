import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from .. import netgear_m5_ns, NetgearM5Component, CONF_NETGEAR_M5_ID

DEPENDENCIES = ["netgear-m5"]

CONF_PATH = "path"

NetgearM5Sensor = netgear_m5_ns.class_(
    "NetgearM5Sensor", sensor.Sensor, cg.Component
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(NetgearM5Sensor)
    .extend(
        {
            cv.GenerateID(CONF_NETGEAR_M5_ID): cv.use_id(NetgearM5Component),
            cv.Required(CONF_PATH): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    
    parent = await cg.get_variable(config[CONF_NETGEAR_M5_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_path(config[CONF_PATH]))
    
    await cg.register_component(var, config)
