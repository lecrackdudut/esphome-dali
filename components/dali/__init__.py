from typing import OrderedDict
from esphome import pins
from esphome.const import CONF_ID, CONF_RX_PIN, CONF_TX_PIN, CONF_DISCOVERY
from esphome.core import CORE

import esphome.codegen as cg
import esphome.config_validation as cv

from . import binary_sensor as dali_binary_sensor

AUTO_LOAD = ["light", "output", "binary_sensor"]

CONF_DALI_BUS = 'dali_bus'
CONF_INITIALIZE_ADDRESSES = 'initialize_addresses'
CONF_MAX_DISCOVERED_LIGHTS = 'max_discovered_lights'
CONF_DEBUG_TX_RX = 'debug_tx_rx'

# DALI short addresses are 0..63; discovery may create one LightState per device.
DEFAULT_MAX_DISCOVERED_LIGHTS = 64

dali_ns = cg.esphome_ns.namespace('dali')
dali_lib_ns = cg.global_ns
DaliBusComponent = dali_ns.class_('DaliBusComponent', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DaliBusComponent),
    cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_DISCOVERY): cv.All(cv.requires_component("light"), cv.boolean),
    cv.Optional(CONF_INITIALIZE_ADDRESSES): cv.boolean,
    cv.Optional(CONF_MAX_DISCOVERED_LIGHTS, default=DEFAULT_MAX_DISCOVERED_LIGHTS): cv.int_range(
        min=1, max=DEFAULT_MAX_DISCOVERED_LIGHTS
    ),
    cv.Optional(CONF_DEBUG_TX_RX, default=False): cv.boolean,
    cv.Optional(dali_binary_sensor.CONF_BUS_STATUS): dali_binary_sensor.bus_status_schema(dali_ns),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config: OrderedDict):
    var = cg.new_Pvariable(config[CONF_ID])
    bus = await cg.register_component(var, config)

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))
    
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    if config.get(CONF_DEBUG_TX_RX, False):
        cg.add(var.set_debug_tx_rx(True))

    if config.get(CONF_DISCOVERY, False):
        cg.add(var.do_device_discovery())

        # When discovery is enabled but no light components are defined
        # in the YAML, we need to make it look like we have a light
        # defined so it will compile in support. Without this, USE_LIGHT
        # will not be defined.
        #
        # This can be done by registering this bus component as a light,
        # making the core think there is at least one light defined.
        CORE.register_platform_component("light", bus)

        # Runtime discovery calls App.register_light() / register_component_()
        # for each device. ESPHome sizes lights_ and components_ StaticVectors at
        # compile time from platform_counts / component_ids — extra push_back()
        # calls are silently dropped when the vector is full.
        max_discovered = config[CONF_MAX_DISCOVERED_LIGHTS]
        CORE.platform_counts["light"] += max_discovered - 1

        base_component_count = len(CORE.component_ids)
        for i in range(max_discovered):
            CORE.component_ids.add(f"__dali_dynamic_reserve_{i}")

        # Override ESPHOME_COMPONENT_COUNT emitted earlier by esphome core.
        cg.add_define("ESPHOME_COMPONENT_COUNT", base_component_count + max_discovered)

    if config.get(CONF_INITIALIZE_ADDRESSES, False):
        cg.add(var.do_initialize_addresses())

    if dali_binary_sensor.CONF_BUS_STATUS in config:
        await dali_binary_sensor.register_bus_status(var, config)
