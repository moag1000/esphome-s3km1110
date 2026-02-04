#include "mqtt_mode_switch.h"
#include "zigbee_bridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zigbee_bridge {

static const char *const TAG = "zigbee_bridge.mqtt_switch";

void MqttModeSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent ZigbeeBridge not set");
    return;
  }

  // Check if coordinator has WiFi capability
  if (!this->parent_->is_c5_wifi_capable()) {
    ESP_LOGW(TAG, "Coordinator does not have WiFi capability - cannot switch to MQTT mode");
    // Revert switch to OFF state
    this->publish_state(false);
    return;
  }

  if (state) {
    // Switch to MQTT mode
    ESP_LOGI(TAG, "Switching coordinator to MQTT mode...");
    this->parent_->send_bridge_mode_mqtt();
  } else {
    // Switch to UART mode
    ESP_LOGI(TAG, "Switching coordinator to UART mode...");
    this->parent_->send_bridge_mode_uart();
  }

  // Note: We don't call publish_state() here because the actual state
  // will be confirmed via the response from the coordinator.
  // The ZigbeeBridge::handle_response_() method will update the switch state.
}

}  // namespace zigbee_bridge
}  // namespace esphome
