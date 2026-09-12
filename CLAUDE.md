# CLAUDE.md - ESPHome S3 + Zigbee Coordinator Project

## Project Overview

Multi-chip Zigbee coordinator system with ESP32-S3 (ESPHome) as host and ESP32-C5/H2 as Zigbee radio.

## GitHub Repository

- **URL:** `https://github.com/moag1000/esphome-s3km1110`
- **Branch:** `main`

### External Components Workflow

ESPHome lädt die Components von GitHub. Um Änderungen zu deployen:

```bash
# 1. Änderungen committen
git add components/zigbee_bridge/*.cpp components/zigbee_bridge/*.h components/zigbee_bridge/*.py
git commit -m "fix: Beschreibung der Änderung"

# 2. Auf GitHub pushen
git push

# 3. Im ESPHome Dashboard neu kompilieren
# Der Server holt automatisch die neueste Version von GitHub
```

**WICHTIG:** Die `external_components` in der YAML zeigen auf GitHub:
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/moag1000/esphome-s3km1110
      ref: main
    components: [ld2420, zigbee_bridge]
```

## Important Endpoints

### ESPHome Server
- **Dashboard:** `http://esphome.wildtierpark.local:6052`
- **Upload YAML via API: gone.** `POST /edit?configuration=…` answers
  `405 Method Not Allowed` on the current Device Builder, as do `PUT`/`PATCH`
  on the same path and `/api/edit`, `/save`, `/api/files/save`,
  `/api/configuration`. Those paths fall through to a catch-all that serves
  the web app on GET and nothing else. Verified 2026-09-12.
- **Compile/Flash:** Via Dashboard Web-UI (WebSocket-based, not REST)

### Build and deploy from this machine (verified 2026-09-12)

The local CLI is the working path, and it is ahead of the server — CLI
2026.8.2 against the server's 2026.8.0:

```bash
esphome config  esp32-s3-mmwave.yaml            # validate
esphome compile esp32-s3-mmwave.yaml            # build
esphome upload  esp32-s3-mmwave.yaml --device 192.168.2.28
```

**The server's copy of the YAML is then stale.** Building locally does not
update it, and there is no REST route to push it. Pressing INSTALL in the
dashboard afterwards rebuilds the *old* config and silently reverts the
deployment. Either paste the file into the dashboard editor after changing
it, or treat the CLI as the only way this device gets flashed.

Changes under `components/` still have to be committed and pushed before a
build picks them up, and the `ref:` in the YAML has to name the new commit —
`external_components` is pinned to a SHA, not to `main`.

### Devices
| Device | Address | Config File |
|--------|---------|-------------|
| mmWave Presence Sensor | `mmwave-presence.wildtierpark.local` | `esp32-s3-mmwave.yaml` |

## Hardware Setup

### UART Connections (S3 ↔ Zigbee Coordinator)

> **Unresolved: this table and the YAML disagree on which pin is TX.**
> `esp32-s3-mmwave.yaml` configures `uart_zigbee` as `tx_pin: GPIO15,
> rx_pin: GPIO16` — the opposite of the table below. Only one can match the
> wiring. It has not shown up as a fault because no coordinator has been
> attached to the S3 since the swap, so nothing has exercised this link.
> Check against the physical wiring before wiring a coordinator back up, and
> delete whichever line is wrong.

```
ESP32-S3 (ESPHome)          ESP32-C5/H2 (Coordinator)
GPIO16 (TX) ───────────────► RX (C5:GPIO6, H2:GPIO23)
GPIO15 (RX) ◄─────────────── TX (C5:GPIO7, H2:GPIO24)
GND ────────────────────────── GND
```

### Zigbee Configuration
- **Channel:** 11 (best Aqara device compatibility)
- **Don't use Channel 20** - Aqara DJT11LM can't join on it

## Serial Ports (macOS)
- **ESP32-C5:** `/dev/tty.usbmodem*` (check with `ls /dev/tty.usbmodem*`)
- **ESP32-H2:** Similar USB-serial pattern

## Build Commands

### C5 Zigbee Coordinator (UART only)
```bash
cd c5-zigbee
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

### C5+ Zigbee Coordinator (WiFi + UART)
```bash
cd c5plus-zigbee
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

**C5+ Features:**
- WiFi 6 connectivity (same network as S3)
- mDNS: `c5plus-zigbee.local`
- WiFi/Zigbee coexistence enabled
- PSRAM optimized for network buffers

**WiFi Config (sdkconfig.defaults or menuconfig):**
- SSID: FRITZ!Box 7590 DQ
- DHCP enabled (no static IP)

### H2 Zigbee Coordinator
```bash
cd h2-zigbee
idf.py set-target esp32h2
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

### ESPHome S3 (via Server)
1. Upload YAML via API (see above)
2. Open Dashboard → Click INSTALL → Wirelessly

## LED Status Codes

### Zigbee Coordinator (H2/C5)
| Status | Color | Effect |
|--------|-------|--------|
| Boot | Blue | Solid |
| Normal | Green | Solid |
| Pairing | Orange | Blink 1Hz |
| Error | Red | Solid |
| OTA | Purple | Solid |

### ESP32-S3 mmWave
| Status | Color | Effect |
|--------|-------|--------|
| Boot | Blue | Pulse (3s) |
| Normal | Green 20% | Solid |
| Presence | Green | Pulse |
| Permit Join | Orange | Blink |
| Zigbee Error | Red | Solid |

## MQTT Commands (via Home Assistant)

Available services after ESPHome integration:
- `zigbee_bridge.factory_reset` - Reset Zigbee network and clear NVS
- `zigbee_bridge.reboot` - Reboot coordinator
- `zigbee_bridge.permit_join` - Enable pairing mode
