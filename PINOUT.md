# Pinout & Wiring Documentation

## Boards

| Role | Board | Module |
|------|-------|--------|
| Main MCU | SparkleIoT XH-S3E N16R8 | ESP32-S3 (16MB Flash, 8MB Octal PSRAM) |
| Zigbee Coordinator | Waveshare ESP32-C5-WiFi6-KIT-NXRX 16R8 | ESP32-C5 (16MB Flash, 8MB PSRAM) |
| Zigbee Coordinator (Alt) | ESP32-H2-Zero | ESP32-H2 (4MB Flash, no PSRAM) |
| mmWave Sensor | Waveshare HMMD | S3KM1110 (24GHz) |

---

## ESP32-S3 — SparkleIoT XH-S3E N16R8

Top view, USB ports at top:

```
                    ┌──────────┐
                    │ USB  USB │
                    │          │
          GND  [ ] │          │ [ ]  14
          GND  [ ] │          │ [ ]  5Vin
           19  [ ] │          │ [ ]  GND
           20  [ ] │          │ [ ]  13
           21  [ ] │          │ [ ]  12
           47  [ ] │          │ [ ]  11
           48  [ ] │          │ [ ]  10
           45  [ ] │          │ [ ]  9
            0  [ ] │          │ [ ]  3
           35  [ ] │          │ [ ]  46
           36  [ ] │          │ [ ]  8
           37  [ ] │          │ [ ]  18
           38  [ ] │          │ [ ]  17
           39  [ ] │          │ [ ]  16
           40  [ ] │          │ [ ]  15
           41  [ ] │          │ [ ]  7
           42  [ ] │          │ [ ]  6
            2  [ ] │          │ [ ]  5
            1  [ ] │          │ [ ]  4
           RX  [ ] │          │ [ ]  3V3
           TX  [ ] │          │ [ ]  3V3
          GND  [ ] │          │ [ ]  RST
                    │  ░░░░░░  │
                    │  MODULE  │
                    │  ░░░░░░  │
                    └──────────┘
                      Antenna
```

> **Note:** GPIO35, GPIO36, GPIO37 are **NOT usable** — directly connected to Octal PSRAM on N16R8 modules.

---

## ESP32-C5 — Waveshare ESP32-C5-WiFi6-KIT-NXRX 16R8

Top view, USB port at top:

```
                    ┌──────────┐
                    │   USB    │
                    │          │
         VBAT  [ ] │          │ [ ]  VBAT
          GND  [ ] │          │ [ ]  GND
           5V  [ ] │          │ [ ]  IO13
         IO25  [ ] │          │ [ ]  IO14
         IO26  [ ] │          │ [ ]  GND
         IO10  [ ] │          │ [ ]  IO28
          IO9  [ ] │          │ [ ]  NC
          IO8  [ ] │          │ [ ]  IO5
          IO7  [ ] │          │ [ ]  IO4
          IO6  [ ] │          │ [ ]  IO27
          IO1  [ ] │          │ [ ]  NC/IO15
          IO0  [ ] │          │ [ ]  IO23
          IO3  [ ] │          │ [ ]  IO24
          IO2  [ ] │          │ [ ]  RXD
          RST  [ ] │          │ [ ]  TXD
          3V3  [ ] │          │ [ ]  GND
                    │  ░░░░░░  │
                    │  MODULE  │
                    │  ░░░░░░  │
                    └──────────┘
                      Antenna
```

---

## mmWave Sensor — Waveshare HMMD (S3KM1110, 24GHz)

Top view, connector (J2) at bottom:

```
        ┌──────────────────────┐
        │  ┌──┐          ┌──┐ │
        │  │//│    R14    │//│ │
        │  │//│    C15    │//│ │
        │  └──┘          └──┘ │
        │         ┌────┐       │
        │    U3   │ U1 │   U2 │
        │         └────┘       │
        │  ┌──┐          ┌──┐ │
        │  │//│          │//│ │
        │  └──┘          └──┘ │
        │  HMMD-mmWave-Sensor  │
        │                      │
        │  (o) (o) (o) (o) (o) │
        └──┼───┼───┼───┼───┼──┘
          3V3 GND  TX  RX  OT2
          (1) (2) (3) (4) (5)
                   J2
```

The four gold patches are the 24GHz radar antennas. U1 is the S3KM1110 radar SoC.

---

## Wiring: mmWave Sensor to ESP32-S3

Connected to the **right pin header** of the S3 board.

| S3 Pin | Function | Sensor Pin (J2) | Description |
|--------|----------|------------------|-------------|
| 3V3 | Power | 3V3 (1) | Sensor power supply |
| GND | Ground | GND (2) | Common ground |
| GPIO17 | UART TX | RX (4) | ESP transmits to sensor |
| GPIO18 | UART RX | TX (3) | ESP receives from sensor |
| GPIO8 | GPIO In | OT2 (5) | Presence output (optional) |

> **Important:** Add an external **10k pull-up resistor** from GPIO17 to 3V3 to prevent UART break on hardware reset. See [Patched LD2420 Component](README.md#patched-ld2420-component).

---

## Wiring: Zigbee Bridge (ESP32-S3 to ESP32-C5)

UART connection between the **right header of the S3** and the **left header of the C5**.

| S3 Pin | Direction | C5 Pin | Cable Color | Description |
|--------|-----------|--------|-------------|-------------|
| GPIO15 | TX -> RX | IO6 | Orange | S3 transmits to C5 |
| GPIO16 | RX <- TX | IO7 | White | S3 receives from C5 |
| 3V3 | -> | 3V3 | Purple | Power from S3 to C5 |
| GND | -> | GND | Brown | Common ground |

### Wiring diagram

```
  SparkleIoT XH-S3E              Waveshare ESP32-C5
  (right header)                  (left header)
  ┌────────────┐                  ┌────────────┐
  │            │                  │            │
  │    GPIO15  ├──── Orange ─────►│  IO6       │
  │    (TX)    │                  │  (RX)      │
  │            │                  │            │
  │    GPIO16  │◄──── White ──────┤  IO7       │
  │    (RX)    │                  │  (TX)      │
  │            │                  │            │
  │    3V3     ├──── Purple ─────►│  3V3       │
  │            │                  │            │
  │    GND     ├──── Brown ──────►│  GND       │
  │            │                  │            │
  └────────────┘                  └────────────┘
```

---

## Wiring: Zigbee Bridge (ESP32-S3 to ESP32-H2)

Alternative coordinator using ESP32-H2-Zero module.

| S3 Pin | Direction | H2 Pin | Cable Color | Description |
|--------|-----------|--------|-------------|-------------|
| GPIO15 | TX -> RX | GPIO23 | Orange | S3 transmits to H2 |
| GPIO16 | RX <- TX | GPIO24 | White | S3 receives from H2 |
| 3V3 | -> | 3V3 | Purple | Power from S3 to H2 |
| GND | -> | GND | Brown | Common ground |

### Wiring diagram (H2)

```
  SparkleIoT XH-S3E              ESP32-H2-Zero
  (right header)
  ┌────────────┐                  ┌────────────┐
  │            │                  │            │
  │    GPIO15  ├──── Orange ─────►│  GPIO23    │
  │    (TX)    │                  │  (RX)      │
  │            │                  │            │
  │    GPIO16  │◄──── White ──────┤  GPIO24    │
  │    (RX)    │                  │  (TX)      │
  │            │                  │            │
  │    3V3     ├──── Purple ─────►│  3V3       │
  │            │                  │            │
  │    GND     ├──── Brown ──────►│  GND       │
  │            │                  │            │
  └────────────┘                  └────────────┘
```

---

## Optional: OTA Recovery Pins

For UART-OTA firmware updates with hardware recovery capability, connect RST and BOOT pins.
**These are optional** — software OTA works without them.

| S3 Pin | Direction | C5 Pin | H2 Pin | Description |
|--------|-----------|--------|--------|-------------|
| GPIO4 | -> | RST/EN | RST/EN | Reset coordinator |
| GPIO5 | -> | IO9 | GPIO9 | Boot mode (hold LOW during reset) |

### Recovery sequence

1. S3 pulls GPIO5 (BOOT) LOW
2. S3 pulses GPIO4 (RST) LOW for 100ms
3. Coordinator enters bootloader mode
4. S3 can flash via esptool/UART

---

## Pin Summary Table

| Signal | ESP32-S3 | ESP32-C5 | ESP32-H2 | Kconfig |
|--------|----------|----------|----------|---------|
| **UART Bridge** | | | | |
| S3 TX → Coord RX | GPIO15 | IO6 | GPIO23 | `ZB_UART_RX_PIN` |
| S3 RX ← Coord TX | GPIO16 | IO7 | GPIO24 | `ZB_UART_TX_PIN` |
| **mmWave Sensor** | | | | |
| S3 TX → Sensor RX | GPIO17 | - | - | - |
| S3 RX ← Sensor TX | GPIO18 | - | - | - |
| Presence Output | GPIO8 | - | - | - |
| **Status LED** | | | | |
| WS2812 (onboard) | - | IO8 | *(none)* | `ZB_LED_GPIO` |
| **OTA Recovery** *(optional)* | | | | |
| Reset | GPIO4 | EN | EN | - |
| Boot Mode | GPIO5 | IO9 | GPIO9 | - |

---

## Kconfig Pin Configuration

Coordinator UART pins can be changed in `sdkconfig.defaults` or via `idf.py menuconfig`:

```
# C5 defaults
CONFIG_ZB_UART_TX_PIN=7
CONFIG_ZB_UART_RX_PIN=6
CONFIG_ZB_LED_GPIO=8

# H2 defaults
CONFIG_ZB_UART_TX_PIN=24
CONFIG_ZB_UART_RX_PIN=23
CONFIG_ZB_LED_GPIO=-1   # No LED
```

---

## UART Configuration

| Parameter | Value |
|-----------|-------|
| Baudrate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Protocol | JSON (newline-terminated) |
