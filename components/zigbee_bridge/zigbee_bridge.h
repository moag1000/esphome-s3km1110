#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif
#include <string>
#include <vector>
#include <map>

namespace esphome {
namespace zigbee_bridge {

class ZigbeeBridge : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Send command to C5
  void send_permit_join(uint16_t duration);
  void send_network_info();
  void send_device_list();
  void send_raw(const std::string &json_line);

  // Setters for codegen
  void set_coordinator_status(text_sensor::TextSensor *s) { coordinator_status_ = s; }
  void set_last_event(text_sensor::TextSensor *s) { last_event_ = s; }
  void set_network_info(text_sensor::TextSensor *s) { network_info_ = s; }
  void set_device_count(sensor::Sensor *s) { device_count_ = s; }
  void set_zigbee_channel(sensor::Sensor *s) { zigbee_channel_ = s; }
  void set_coordinator_ready(binary_sensor::BinarySensor *s) { coordinator_ready_ = s; }
  void set_permit_join_active(binary_sensor::BinarySensor *s) { permit_join_active_ = s; }

  // Text sensors for HA
  text_sensor::TextSensor *coordinator_status_{nullptr};
  text_sensor::TextSensor *last_event_{nullptr};
  text_sensor::TextSensor *network_info_{nullptr};

  // Sensors
  sensor::Sensor *device_count_{nullptr};
  sensor::Sensor *zigbee_channel_{nullptr};

  // Binary sensor
  binary_sensor::BinarySensor *coordinator_ready_{nullptr};
  binary_sensor::BinarySensor *permit_join_active_{nullptr};

 protected:
  void process_line_(const std::string &line);
  void handle_ready_(const std::string &line);
  void handle_device_joined_(const std::string &line);
  void handle_device_left_(const std::string &line);
  void handle_interview_done_(const std::string &line);
  void handle_attribute_(const std::string &line);
  void handle_response_(const std::string &line);
  void handle_permit_join_(const std::string &line);
  void handle_network_info_(const std::string &line);
  void handle_device_list_(const std::string &line);
  void handle_tuya_dp_(const std::string &line);

  // Command helpers
  void send_tuya_set_(const std::string &ieee, int dp, const std::string &dp_type, int value);
  void send_tuya_set_str_(const std::string &ieee, int dp, const std::string &value);
  void handle_device_command_(const std::string &ieee, const std::string &payload);
  bool is_fingerbot_(const std::string &model);

  // MQTT helpers
  void mqtt_publish_(const std::string &topic, const std::string &payload);
  void mqtt_publish_bridge_discovery_();
  void mqtt_publish_discovery_(const std::string &ieee, const std::string &model,
                                const std::string &manufacturer, const std::string &name);
  void mqtt_publish_device_state_(const std::string &ieee);

  // Simple JSON helpers (no external lib needed)
  std::string json_get_string_(const std::string &json, const std::string &key);
  int json_get_int_(const std::string &json, const std::string &key, int default_val = 0);
  double json_get_double_(const std::string &json, const std::string &key, double default_val = 0.0);

  // Per-device info cache
  struct DeviceInfo {
    std::string friendly_name;
    std::string model;
    std::string manufacturer;
    std::string state;       // "ON" / "OFF"
    int battery{-1};
    int linkquality{0};
    // Tuya fingerbot state
    std::string mode{"push"};   // "push" / "switch" / "program"
    int down_movement{50};      // 51-100
    int up_movement{0};         // 0-49
    int sustain_time{0};        // 0-255 seconds
    bool reverse{false};
    bool touch_control{true};
    bool program_enable{false};
    bool repeat_forever{false};
    std::string program;
  };
  std::map<std::string, DeviceInfo> devices_;

  std::string rx_buffer_;
  uint32_t cmd_id_{0};
  bool c5_ready_{false};
  bool device_list_requested_{false};
  bool permit_join_sent_{false};
  uint32_t last_rx_time_{0};
  int device_count_val_{0};
};

}  // namespace zigbee_bridge
}  // namespace esphome
