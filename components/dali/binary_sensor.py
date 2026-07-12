"""DALI bus status binary sensor (configured under the dali: component)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

CONF_BUS_STATUS = "bus_status"


def bus_status_schema(dali_ns):
    dali_bus_status_binary_sensor = dali_ns.class_(
        "DaliBusStatusBinarySensor", binary_sensor.BinarySensor, cg.Component
    )
    return binary_sensor.binary_sensor_schema(dali_bus_status_binary_sensor).extend(
        {
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
        }
    )


async def register_bus_status(dali_bus, config):
    bs_conf = config[CONF_BUS_STATUS]
    bs = cg.new_Pvariable(bs_conf[CONF_ID])
    await cg.register_component(bs, bs_conf)
    await binary_sensor.register_binary_sensor(bs, bs_conf)
    cg.add(bs.set_parent(dali_bus))
    cg.add(dali_bus.set_bus_status_sensor(bs))
    cg.add(bs.set_update_interval(bs_conf[CONF_UPDATE_INTERVAL]))
