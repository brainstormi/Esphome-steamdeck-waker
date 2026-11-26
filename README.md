# ESPHome Steam Deck BLE Waker (+ Bluetooth proxy)

Wake **and suspend** a Steam Deck OLED from Home Assistant with an ESP32 that
pretends to be a Bluetooth LE keyboard — **while the same ESP32 keeps working as
a regular ESPHome `bluetooth_proxy`** (for Bermuda, PadSpan, Private BLE Device,
etc.).

No scripts, services or settings changes are needed on the Steam Deck beyond
pairing the "keyboard" and enabling *"Allow this device to wake Steam Deck"* in
the Steam Bluetooth settings.

> [!IMPORTANT]
> **Only tested on a Steam Deck OLED.** See
> [Steam Deck LCD](#steam-deck-lcd) for why it is not expected to work on the
> LCD model.

## Credits

This project is a rewrite inspired by
**[DxKoster/Steam-Deck-Waker](https://github.com/DxKoster/Steam-Deck-Waker)**,
which discovered and documented the key ideas:

- a suspended Steam Deck keeps Bluetooth alive and can be woken by a bonded
  BLE keyboard;
- the wake signal is a **high-duty directed advertising "knock"**
  (`ADV_DIRECT_IND`) aimed at the bonded host;
- the per-device BlueZ `WakeAllowed` flag must be set.

Thank you to its author. The code here is new (different Bluetooth stack and
architecture, see below), but none of it would exist without that project.

## Differences from the original project

| | Steam-Deck-Waker (original) | This project |
|---|---|---|
| Bluetooth stack | NimBLE (patched `ble_keyboard` component) | **Bluedroid**, ESPHome's own `esp32_ble` stack |
| Works with `bluetooth_proxy` / `esp32_ble_tracker` on the same ESP32 | No — NimBLE and Bluedroid cannot coexist | **Yes** |
| HID implementation | Custom NimBLE HID | ESP-IDF `esp_hid` (`ble_hidd.c`), GATT server owned by this component |
| Wake | Directed knock + key presses (space ×2) | Directed knock only — **no key presses** (the connection itself wakes the Deck) |
| Advertising while idle | Undirected advertising so the Deck can reconnect | **Silent once paired** (undirected advertising wakes a `WakeAllowed` Deck by itself, see below) |
| Deck-side service to keep `WakeAllowed=true` | Required (installer + systemd user service) | **Not needed in our tests** (see [WakeAllowed](#wakeallowed-and-reboots)) |
| Suspend the Deck | — | **Yes**: HID *System Sleep* key (Linux `KEY_SLEEP`) |
| Radio contention | — | Pauses the proxy scanner and gives Bluetooth coexistence priority over Wi-Fi only while knocking |
| Retries | Home Assistant script | Built in, configurable from the device page (**Wake attempts**) |

## Features

- **Wake Steam Deck** button: knocks with high-duty directed advertising until
  the Deck connects (1–20 attempts of ~1.28 s each). Sends no keys.
- **Suspend Steam Deck** button: sends the HID *System Sleep* usage. If the
  Deck is awake but not connected, it knocks first, then sends the key.
- **Wake attempts** number (config): number of knocks per request, stored in flash.
- **Pairing mode (2 min)** and **Forget BLE pairings** buttons.
- **Steam Deck connected** binary sensor and **Paired host** text sensor.
- Everything else is a normal ESPHome Bluetooth proxy.

## Requirements and tested setup

| Item | Tested with |
|---|---|
| Console | Steam Deck **OLED**, SteamOS stable (November 2026) |
| ESP32 | Classic ESP32 (ESP32-WROOM-32, chip rev 1.0, 4 MB flash) |
| ESPHome | 2026.11.0 (ESP-IDF 5.5.6) |
| Framework | `esp-idf` (required; Arduino is not supported) |

Other ESP32 variants with BLE (S3, C3, C6) should work in principle because the
component only uses ESP-IDF APIs, but they have **not been tested**.

## Installation

1. Copy [`example/steamdeck-waker.yaml`](example/steamdeck-waker.yaml) into
   your ESPHome configuration, replace `YOUR_GITHUB_USER` in
   `external_components`, and provide the secrets from
   [`example/secrets.yaml.example`](example/secrets.yaml.example).
2. Flash the ESP32 (USB the first time, OTA afterwards) and adopt it in Home
   Assistant.
3. **Pair the Steam Deck** (the Deck must be awake). A fresh ESP32 with no
   bonds advertises automatically; otherwise press **Pairing mode (2 min)**.
   On the Deck: *Settings → Bluetooth* → pair **"Steam Deck Waker"**.
4. In the Deck's Bluetooth settings open the device and enable
   **"Allow this device to wake Steam Deck"**.
5. Suspend the Deck, wait a minute, press **Wake Steam Deck**.

> [!NOTE]
> If you update the firmware in a way that changes the HID report map (for
> example from an older version without the *Suspend* report), the Deck keeps
> using the cached descriptor. **Forget the device on the Deck, press
> *Forget BLE pairings*, then pair again.**

## Configuration

```yaml
deck_waker:
  id: waker
  name: "Steam Deck Waker"   # advertised name while pairing (max 20 chars)
  direct_attempts: 8         # knocks per request (1..20)
  pause_scanner: true        # pause esp32_ble_tracker while knocking
```

C++ methods available from lambdas: `wake()`, `sleep_host()`, `power_key()`,
`send_key(uint8_t hid_usage)`, `start_pairing_mode(seconds)`, `clear_bonds()`,
`set_direct_attempts(n)`, `is_connected()`, `bonded_peer_str()`.

`esp32_ble` must allow one extra connection for the HID link
(`max_connections` = proxy `connection_slots` + 1). The component uses
`io_capability: none` with `auth_req_mode: sc_bond` (Just Works, bonded).

## How it works

1. **One Bluetooth stack.** ESPHome's `esp32_ble` (Bluedroid) runs the proxy's
   scanner/GATT client. This component registers the GATT *server* callback
   itself and builds the HID service with ESP-IDF's `esp_hid` library
   (keyboard report + System Control report). Because of that it cannot be
   combined with `esp32_ble_server` (validation will fail).
2. **Silent when paired.** We observed that a suspended Deck with
   `WakeAllowed` enabled also wakes up on ordinary *undirected* connectable
   advertising from the bonded keyboard. So after pairing the ESP32 does not
   advertise at all, and the Deck can stay asleep. Undirected advertising is
   used only while there is no bond or during pairing mode.
3. **Wake = knock.** On request it sends high-duty directed advertising
   (`ADV_TYPE_DIRECT_IND_HIGH`, max 1.28 s per burst) to the bonded host's
   identity address, repeating up to *Wake attempts* times. The Deck's
   controller accepts the connection and the host resumes. No key press is
   needed.
4. **Radio priority while knocking.** The ESP32 shares one 2.4 GHz radio
   between Wi-Fi, BLE scanning and advertising. While knocking, the component
   pauses the proxy scanner and sets the coexistence preference to Bluetooth.
   Both are restored as soon as the Deck connects (typically within 0.1–9 s)
   or the attempts run out. In our tests knocks were unreliable before this
   change and reliable after it.
5. **Suspend.** Once connected (knocking first if necessary) it sends the HID
   Generic Desktop *System Sleep* usage, which Linux reports as `KEY_SLEEP`;
   SteamOS suspends and drops the link.

## WakeAllowed and reboots

The original project found that SteamOS resets BlueZ's per-device
`WakeAllowed` flag on reboot and ships a Deck-side service to keep it set. In
our tests the Steam UI toggle was enough, and `bluetoothctl info <ESP32 BT MAC>`
reported `WakeAllowed: yes`.

**Persistence across Steam Deck reboots has not been verified yet.** If wake
stops working after a reboot or SteamOS update, check
`bluetoothctl info <ESP32 BT MAC> | grep WakeAllowed`; if it says `no`, use the
[Deck-side fix from the original project](https://github.com/DxKoster/Steam-Deck-Waker/tree/main/deck-side).

## Steam Deck LCD

Not tested, and **not expected to work**:

- The OLED model's wireless hardware keeps Bluetooth powered during suspend,
  which is what makes Bluetooth wake possible. On the LCD model, wake-on-
  Bluetooth was only added later by Valve as an **experimental** feature.
- Valve's SteamOS 3.7 release notes listed as a known issue that **Bluetooth
  LE devices are not compatible with Bluetooth wake on LCD models**. This waker
  is a BLE device.
- Valve has disabled and re-enabled the LCD feature several times because of
  spurious wake-ups, so its state depends on the SteamOS version.

If you own an LCD Deck and try it, reports are welcome.

## Troubleshooting

- **Deck wakes up on its own right after suspending**: something is
  advertising connectably — make sure pairing mode is not active and you run a
  version of this component that stays silent once bonded.
- **Knocks sent but the Deck never connects**: check `WakeAllowed` (above),
  increase *Wake attempts*, move the ESP32 closer to the Deck.
- **Suspend does nothing**: re-pair (see the note in Installation) so the Deck
  reads the current HID descriptor.
- **`Paired host` shows the wrong device**: the component uses the most recent
  bond. Press *Forget BLE pairings* and pair only the Deck.

## Limitations

- One host (the most recently bonded one).
- *Forget BLE pairings* removes **all** Bluedroid bonds on the ESP32.
- Cannot be combined with `esp32_ble_server` (it owns the GATT server).
- A fully powered-off Deck cannot be woken over Bluetooth; keep it in sleep.

## License

MIT — see [LICENSE](LICENSE).
Not affiliated with Valve. "Steam Deck" and "SteamOS" are trademarks of Valve
Corporation.
