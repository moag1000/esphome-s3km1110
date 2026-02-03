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
- **Upload YAML via API:**
  ```bash
  curl -X POST "http://esphome.wildtierpark.local:6052/edit?configuration=<name>.yaml" \
    -H "Content-Type: text/plain" \
    --data-binary @<local-file.yaml>
  ```
- **Compile/Flash:** Via Dashboard Web-UI (WebSocket-basiert, nicht REST)

### Devices
| Device | Address | Config File |
|--------|---------|-------------|
| mmWave Presence Sensor | `mmwave-presence.wildtierpark.local` | `esp32-s3-mmwave.yaml` |

## Hardware Setup

### UART Connections (S3 ↔ Zigbee Coordinator)
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

### C5 Zigbee Coordinator
```bash
cd c5-zigbee
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

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
