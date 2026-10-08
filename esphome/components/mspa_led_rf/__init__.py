"""ESP32/CC1101 RX and manual TX for the MSpa Denver LED remote."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import event, text_sensor
from esphome.const import CONF_ID

AUTO_LOAD = ["event", "text_sensor"]
DEPENDENCIES = ["esp32"]

CONF_RX_FREQUENCY = "rx_frequency"
CONF_TX_FREQUENCY = "tx_frequency"
CONF_REMOTE_EVENT = "remote_event"
CONF_LAST_BUTTON = "last_button"
CONF_LAST_TX_BUTTON = "last_tx_button"

BUTTON_NAMES = [
    "power", "mode_plus", "speed_minus", "demo", "speed_plus",
    "color_plus", "mode_minus", "bright_plus", "color_minus",
    "bright_minus", "white", "red", "green", "blue", "yellow",
    "cyan", "pink",
]

ns = cg.esphome_ns.namespace("mspa_led_rf")
MspaLedRfComponent = ns.class_("MspaLedRfComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MspaLedRfComponent),
    cv.Optional(CONF_RX_FREQUENCY, default=433.973): cv.float_range(min=430.0, max=440.0),
    cv.Optional(CONF_TX_FREQUENCY, default=434.0254): cv.float_range(min=430.0, max=440.0),
    cv.Required(CONF_REMOTE_EVENT): event.event_schema(device_class="button"),
    cv.Required(CONF_LAST_BUTTON): text_sensor.text_sensor_schema(),
    cv.Required(CONF_LAST_TX_BUTTON): text_sensor.text_sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_library("SPI", None)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_rx_frequency(config[CONF_RX_FREQUENCY]))
    cg.add(var.set_tx_frequency(config[CONF_TX_FREQUENCY]))
    remote_event = await event.new_event(
        config[CONF_REMOTE_EVENT], event_types=BUTTON_NAMES
    )
    cg.add(var.set_remote_event(remote_event))
    last_button = await text_sensor.new_text_sensor(config[CONF_LAST_BUTTON])
    cg.add(var.set_last_button(last_button))
    last_tx_button = await text_sensor.new_text_sensor(config[CONF_LAST_TX_BUTTON])
    cg.add(var.set_last_tx_button(last_tx_button))
