# ESPHome S3KM1110 mmWave Presence Sensor

ESPHome firmware for the **Waveshare S3KM1110** 24GHz mmWave presence sensor on an **ESP32-S3-DevKitC-1 N16R8** (16MB Flash, 8MB Octal PSRAM).

## Features

- **mmWave Presence Detection** via patched LD2420 component (protocol-compatible with S3KM1110)
- **Bluetooth Proxy** for Home Assistant BLE integrations
- **Zigbee Bridge** to ESP32-C5 Coordinator via UART (JSON Lines protocol)
- **GPIO Presence Output** (hardware pin, no UART parsing needed)
- **Web Server** for local status/control
- **Full Gate Configuration** (16 gates, move/still thresholds)

## Hardware

| Component | Model |
|-----------|-------|
| MCU | ESP32-S3-DevKitC-1 N16R8 |
| Sensor | Waveshare HMMD (S3KM1110, 24GHz) |
| Zigbee | ESP32-C5 Coordinator (optional) |

### Wiring (S3 left pin header)

```
3V3    -> 3V3 (Pin 1)
GND    -> GND (Pin 2)
GPIO17 -> RX  (Pin 4)  <- ESP TX to Sensor RX
GPIO18 -> TX  (Pin 3)  <- ESP RX from Sensor TX
GPIO8  -> OT2 (Pin 5)  <- Presence GPIO output (optional)
```

### Zigbee Bridge Wiring (S3 right header -> C5 left header)

```
GPIO15 (S3 TX) -> IO6 (C5 RX)  Orange
GPIO16 (S3 RX) <- IO7 (C5 TX)  White
3V3            -> 3V3           Purple
GND            -> GND           Brown
```

> **Note:** GPIO35-37 are NOT available (used by Octal PSRAM on N16R8).

## Patched LD2420 Component

The stock ESPHome LD2420 component fails to initialize the S3KM1110 after software reboots (OTA, API restart, watchdog). The sensor becomes unresponsive because GPIO17 (UART TX) drops LOW during the ESP32 bootloader phase, which the sensor interprets as a UART break condition and enters an unrecoverable state.

### Fixes applied

1. **`gpio_hold_en` on shutdown** — Before any software restart, GPIO17 is detached from the UART peripheral, driven HIGH, and latched via `gpio_hold_en()`. The RTC hold keeps the pin HIGH through the entire reboot sequence (bootloader + app init), preventing the UART break.

2. **`gpio_hold_dis` on setup** — At the start of LD2420 `setup()`, the hold is released so the UART driver can take control of the pin.

3. **UART RX buffer flush** — Drains any buffered sensor data before attempting command mode, preventing streaming data from corrupting command framing.

4. **`get_firmware_int` crash fix** — Prevents `stoi` crash when the sensor returns non-numeric version strings.

5. **Retry logic** — If initial config mode fails, retries with full UART driver reset.

> **Hardware reset button:** The `gpio_hold_en` fix only covers software restarts. For hardware reset (EN pin), add an external **10kOhm pull-up resistor** from GPIO17 to 3V3.

## Installation

### Via ESPHome Dashboard (recommended)

Add this as an external component source in your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/moag1000/esphome-s3km1110
      ref: main
    components: [ld2420, zigbee_bridge]
```

### Local development

Clone and compile directly:

```bash
git clone https://github.com/moag1000/esphome-s3km1110.git
cd esphome-s3km1110
esphome run esp32-s3-mmwave.yaml
```

## Configuration

Copy `esp32-s3-mmwave.yaml` and adjust:

- `substitutions` — device name
- `api.encryption.key` — generate a new key
- `ota.password` — set your OTA password
- `wifi` — your network credentials
- `mqtt` — broker address (if using Zigbee bridge)

## License

MIT
