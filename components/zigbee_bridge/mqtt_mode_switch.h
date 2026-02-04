#pragma once

#include "esphome/components/switch/switch.h"

namespace esphome {
namespace zigbee_bridge {

// Forward declaration
class ZigbeeBridge;

/**
 * MQTT Mode Switch for WiFi-capable Zigbee coordinators (C5+).
 *
 * When turned ON: Switches coordinator to MQTT mode (direct connection to broker)
 * When turned OFF: Switches coordinator back to UART mode (S3 bridges to MQTT)
 *
 * This switch is only effective when the connected coordinator has WiFi capability.
 */
class MqttModeSwitch : public switch_::Switch {
 public:
  void set_parent(ZigbeeBridge *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  ZigbeeBridge *parent_{nullptr};
};

}  // namespace zigbee_bridge
}  // namespace esphome
