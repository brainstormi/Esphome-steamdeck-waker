"""Steam Deck waker: BLE HID keyboard on Bluedroid that coexists with bluetooth_proxy.

Port of the idea in DxKoster/Steam-Deck-Waker (NimBLE) to ESPHome's own
Bluedroid stack, so the same ESP32 can also run esp32_ble_tracker +
bluetooth_proxy. The HID GATT service comes from ESP-IDF's esp_hid
(ble_hidd.c); this component owns the GATT server callback (no
esp32_ble_server may be used together with it) and does its own advertising,
including the high-duty *directed* advertising "knock" that wakes a
suspended Steam Deck. It also exposes the HID System Control keys
(Sleep / Power) so the host can be suspended from Home Assistant.
"""

import esphome.codegen as cg
from esphome.components import esp32_ble
from esphome.components.esp32 import include_builtin_idf_component
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME
from esphome.core import CORE
import esphome.final_validate as fv

DEPENDENCIES = ["esp32_ble"]

CONF_DIRECT_ATTEMPTS = "direct_attempts"
CONF_PAUSE_SCANNER = "pause_scanner"

deck_waker_ns = cg.esphome_ns.namespace("deck_waker")
DeckWaker = deck_waker_ns.class_("DeckWaker", cg.Component)


def _final_validate(config):
    if "esp32_ble_server" in fv.full_config.get():
        raise cv.Invalid(
            "deck_waker owns the GATT server and cannot be combined with esp32_ble_server"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DeckWaker),
            cv.GenerateID(esp32_ble.CONF_BLE_ID): cv.use_id(esp32_ble.ESP32BLE),
            cv.Optional(CONF_NAME, default="Steam Deck Waker"): cv.All(
                cv.string, cv.Length(max=20)
            ),
            cv.Optional(CONF_DIRECT_ATTEMPTS, default=8): cv.int_range(min=1, max=20),
            # Pause esp32_ble_tracker (bluetooth_proxy) scanning while knocking
            cv.Optional(CONF_PAUSE_SCANNER, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    esp32_ble.consume_connection_slots(1, "deck_waker"),
)

FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[esp32_ble.CONF_BLE_ID])
    cg.add(var.set_parent(parent))
    esp32_ble.register_gap_event_handler(parent, var)

    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_direct_attempts(config[CONF_DIRECT_ATTEMPTS]))
    cg.add(var.set_pause_scanner(config[CONF_PAUSE_SCANNER]))
    if "esp32_ble_tracker" in CORE.config:
        cg.add_define("DECK_WAKER_PAUSE_SCAN")

    # ESP-IDF's HID device library (ble_hidd.c is built for Bluedroid)
    include_builtin_idf_component("esp_hid")
