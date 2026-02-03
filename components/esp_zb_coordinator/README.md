# ESP Zigbee Coordinator Component

Shared Zigbee coordinator stack for ESP32-C5 and ESP32-H2.

## Overview

This component provides a complete Zigbee 3.0 coordinator implementation that can be used with multiple ESP32 chips. Platform-specific code is abstracted through a Hardware Abstraction Layer (HAL).

## Supported Chips

| Chip | RAM | PSRAM | Max Devices | Features |
|------|-----|-------|-------------|----------|
| ESP32-C5 | 512 KB | Yes (optional) | 64 | Full features, OTA, Touchlink |
| ESP32-H2 | 320 KB | No | 32 | Memory-optimized, essential features |

## Directory Structure

```
esp_zb_coordinator/
├── CMakeLists.txt          # Build configuration
├── Kconfig                 # Memory tuning options
├── include/
│   └── zb_hal.h            # Hardware Abstraction Layer interface
├── platform/
│   ├── esp32c5/
│   │   └── zb_hal_c5.c     # C5-specific HAL implementation
│   └── esp32h2/
│       └── zb_hal_h2.c     # H2-specific HAL implementation
└── src/
    ├── core/               # Device state, command handling
    ├── zigbee/             # Zigbee stack implementation
    │   ├── tuya/           # Tuya device drivers
    │   └── xiaomi/         # Xiaomi/Aqara device drivers
    ├── uart/               # UART protocol layer
    └── utils/              # JSON utilities, version info
```

## Usage

### In your project CMakeLists.txt:

```cmake
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../components"
)
```

### In your main CMakeLists.txt:

```cmake
idf_component_register(
    ...
    REQUIRES
        esp_zb_coordinator
        ...
)
```

### In your code:

```c
#include "zb_coordinator.h"
#include "zb_device_handler.h"
#include "zb_hal.h"

void app_main(void)
{
    // Log HAL configuration
    zb_hal_log_config();

    // Initialize coordinator
    zb_coordinator_init();
    zb_coordinator_start();
}
```

## HAL Functions

The HAL provides chip-agnostic access to:

- **Chip identification**: `zb_hal_get_chip_name()`, `zb_hal_get_chip_type()`
- **MAC address**: `zb_hal_read_mac()`
- **GPIO pins**: `zb_hal_get_uart_tx_pin()`, `zb_hal_get_uart_rx_pin()`, `zb_hal_get_led_gpio()`
- **Memory config**: `zb_hal_get_zb_task_stack_size()`, `zb_hal_has_psram()`
- **Network limits**: `zb_hal_get_max_network_devices()`, `zb_hal_get_max_children()`
- **Feature flags**: `zb_hal_feature_ota_enabled()`, `zb_hal_feature_touchlink_enabled()`

## Kconfig Options

Configure via `menuconfig`:

- `ZB_MAX_DEVICES`: Maximum Zigbee devices (8-128)
- `ZB_COORDINATOR_STACK_SIZE`: Coordinator task stack size
- `ZB_ENABLE_OTA`: Enable OTA support
- `ZB_ENABLE_TOUCHLINK`: Enable Touchlink commissioning
- `ZB_ENABLE_GREEN_POWER`: Enable Green Power support

## Adding a New Chip

1. Create `platform/esp32xx/zb_hal_xx.c`
2. Implement all functions from `zb_hal.h`
3. Add chip detection in `CMakeLists.txt`:

```cmake
elseif(CONFIG_IDF_TARGET_ESP32XX)
    set(ZB_HAL_SRC "platform/esp32xx/zb_hal_xx.c")
```

4. Add Kconfig defaults for the new chip

## License

Apache License 2.0
