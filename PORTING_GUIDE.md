# Zigbee Coordinator Porting Guide

## Multi-Chip Architecture

This project supports multiple ESP32 chips using a shared Zigbee coordinator component.

```
esphome-s3km1110/
├── components/
│   └── esp_zb_coordinator/      # Shared Zigbee stack (~100 files)
│       ├── include/zb_hal.h     # HAL interface
│       ├── platform/            # Chip-specific implementations
│       │   ├── esp32c5/
│       │   └── esp32h2/
│       └── src/                 # Zigbee stack source
│
├── c5-zigbee/                   # C5 project
│   └── main/
│       ├── main.c               # C5 entry point
│       ├── uart/uart_bridge.c   # C5 UART hardware
│       └── led/led_controller.c # C5 LED (WS2812)
│
└── h2-zigbee/                   # H2 project
    └── main/
        ├── main.c               # H2 entry point
        ├── uart/uart_bridge.c   # H2 UART hardware
        └── led/led_controller.c # H2 LED (stub/none)
```

## Chip Comparison

| Feature | ESP32-C5 | ESP32-H2 |
|---------|----------|----------|
| CPU | 240 MHz dual-core | 96 MHz single-core |
| RAM | 512 KB | 320 KB |
| PSRAM | Optional (8MB) | Not available |
| WiFi | WiFi 6 | None |
| Zigbee | Yes | Yes |
| **UART TX** | GPIO 7 | GPIO 24 |
| **UART RX** | GPIO 6 | GPIO 23 |
| **LED GPIO** | GPIO 8 (WS2812) | None (-1) |
| Max devices | 64 | 32 |
| Stack size | 6 KB | 4 KB |

## Building

### ESP32-C5

```bash
cd c5-zigbee
idf.py set-target esp32c5
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### ESP32-H2

```bash
cd h2-zigbee
idf.py set-target esp32h2
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Adding a New Chip (ESP32-XX)

### Step 1: Create HAL Implementation

Create `components/esp_zb_coordinator/platform/esp32xx/zb_hal_xx.c`:

```c
#include "zb_hal.h"
#include "esp_mac.h"
#include "esp_log.h"

const char* zb_hal_get_chip_name(void) { return "ESP32-XX"; }
zb_hal_chip_t zb_hal_get_chip_type(void) { return ZB_HAL_CHIP_ESP32_XX; }
int zb_hal_get_uart_tx_pin(void) { return YOUR_TX_GPIO; }
int zb_hal_get_uart_rx_pin(void) { return YOUR_RX_GPIO; }
int zb_hal_get_led_gpio(void) { return YOUR_LED_GPIO; }  // -1 if none
size_t zb_hal_get_zb_task_stack_size(void) { return YOUR_STACK_SIZE; }
// ... implement all HAL functions
```

### Step 2: Add to zb_hal.h Enum

```c
typedef enum {
    ZB_HAL_CHIP_UNKNOWN = 0,
    ZB_HAL_CHIP_ESP32_C5,
    ZB_HAL_CHIP_ESP32_H2,
    ZB_HAL_CHIP_ESP32_XX,  // Add new chip
} zb_hal_chip_t;
```

### Step 3: Update CMakeLists.txt

```cmake
elseif(CONFIG_IDF_TARGET_ESP32XX)
    set(ZB_HAL_SRC "platform/esp32xx/zb_hal_xx.c")
    set(ZB_TARGET_NAME "ESP32-XX")
```

### Step 4: Update Kconfig

Add chip-specific defaults:

```kconfig
config ZB_MAX_DEVICES
    int "Maximum Zigbee devices"
    default 64 if IDF_TARGET_ESP32C5
    default 32 if IDF_TARGET_ESP32H2
    default 48 if IDF_TARGET_ESP32XX  # Add your default
```

### Step 5: Create Project

```bash
mkdir -p xx-zigbee/main/{uart,led}
# Copy structure from c5-zigbee or h2-zigbee
# Adapt sdkconfig.defaults for your chip
```

## Memory Optimization (H2)

The H2 build uses memory-constrained settings:

- **Smaller stacks**: 4KB instead of 6KB
- **Fewer devices**: 32 instead of 64
- **Disabled features**: OTA, Touchlink, Diagnostics
- **Size optimization**: `-Os` compiler flag
- **Reduced logging**: INFO level only

## Testing Checklist

After porting to a new chip:

1. [ ] Build compiles without errors
2. [ ] Flash and boot successfully
3. [ ] UART communication with S3 works
4. [ ] Zigbee network forms
5. [ ] Device pairing works
6. [ ] Tuya devices work (Fingerbot)
7. [ ] Xiaomi devices work (Vibration sensor)
8. [ ] LED status (if available) works
9. [ ] NVS persistence works
10. [ ] Stress test with 30+ devices

## Troubleshooting

### Build Error: Unsupported target

Add your chip to `CMakeLists.txt` in the component.

### Runtime: Stack overflow

Increase stack sizes in HAL or Kconfig.

### Runtime: Out of memory

Reduce `ZB_MAX_DEVICES` or disable optional features.

### UART not working

Verify GPIO pins in HAL match your hardware.
