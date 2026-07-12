"""DALI bus diagnostics text sensor (configured under the dali: component)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

CONF_DIAG = "diag"


def diag_schema(dali_ns):
    dali_diag_text_sensor = dali_ns.class_(
        "DaliDiagTextSensor", text_sensor.TextSensor, cg.Component
    )
    return text_sensor.text_sensor_schema(dali_diag_text_sensor).extend(
        {
            cv.Optional(CONF_UPDATE_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
        }
    )


async def register_diag(dali_bus, config):
    diag_conf = config[CONF_DIAG]
    diag = cg.new_Pvariable(diag_conf[CONF_ID])
    await cg.register_component(diag, diag_conf)
    await text_sensor.register_text_sensor(diag, diag_conf)
    cg.add(diag.set_parent(dali_bus))
    cg.add(dali_bus.set_diag_sensor(diag))
    cg.add(diag.set_update_interval(diag_conf[CONF_UPDATE_INTERVAL]))
