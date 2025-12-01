import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from .. import netgear_m5_ns, NetgearM5Component, CONF_NETGEAR_M5_ID

DEPENDENCIES = ["netgear-m5"]

CONF_PATH = "path"
CONF_ON_VALUE = "on_value"
CONF_OFF_VALUE = "off_value"

NetgearM5BinarySensor = netgear_m5_ns.class_(
    "NetgearM5BinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(NetgearM5BinarySensor)
    .extend(
        {
            cv.GenerateID(CONF_NETGEAR_M5_ID): cv.use_id(NetgearM5Component),
            cv.Required(CONF_PATH): cv.string,
            cv.Optional(CONF_ON_VALUE, default="true"): cv.string,
            cv.Optional(CONF_OFF_VALUE, default="false"): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    
    parent = await cg.get_variable(config[CONF_NETGEAR_M5_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_path(config[CONF_PATH]))
    cg.add(var.set_on_value(config[CONF_ON_VALUE]))
    cg.add(var.set_off_value(config[CONF_OFF_VALUE]))
    
    await cg.register_component(var, config)
