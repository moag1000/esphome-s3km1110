#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
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

// OTA State Machine
enum OtaState {
  OTA_IDLE,           // No OTA in progress
  OTA_STARTING,       // Sent ota_start, waiting for ota_ready
  OTA_SENDING,        // Sending chunks
  OTA_WAITING_ACK,    // Waiting for chunk acknowledgement
  OTA_FINALIZING,     // Sent ota_end, waiting for ota_complete
  OTA_COMPLETE,       // OTA successful, target rebooting
  OTA_ERROR           // OTA failed
};

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

  // OTA Setters (optional)
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_boot_pin(GPIOPin *pin) { boot_pin_ = pin; }
  void set_ota_progress(sensor::Sensor *s) { ota_progress_ = s; }
  void set_ota_status(text_sensor::TextSensor *s) { ota_status_ = s; }

  // OTA API - called from Home Assistant services
  void start_ota_from_buffer(const uint8_t *data, size_t size);
  void start_ota_from_url(const std::string &url);
  void abort_ota();
  bool is_ota_in_progress() const { return ota_state_ != OTA_IDLE; }
  float get_ota_progress() const;
  OtaState get_ota_state() const { return ota_state_; }

  // Hardware Recovery API (requires reset_pin and boot_pin configured)
  void trigger_reset();
  void trigger_recovery_mode();
  bool has_recovery_pins() const { return reset_pin_ != nullptr && boot_pin_ != nullptr; }

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
  bool is_vibration_(const std::string &model);

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
    // Aqara vibration sensor state
    bool has_vibration_data{false};  // True after first real event data received
    bool vibration{false};
    std::string action;
    double angle_x{0};
    double angle_y{0};
    double angle_z{0};
    int strength{0};
    int voltage{0};
    std::string sensitivity{"medium"};
  };
  std::map<std::string, DeviceInfo> devices_;

  std::string rx_buffer_;
  uint32_t cmd_id_{0};
  bool c5_ready_{false};
  bool device_list_requested_{false};
  bool permit_join_sent_{false};
  uint32_t last_rx_time_{0};
  int device_count_val_{0};

  // ============================================================================
  // OTA State and Members
  // ============================================================================

  // OTA Sensors (optional)
  sensor::Sensor *ota_progress_{nullptr};
  text_sensor::TextSensor *ota_status_{nullptr};

  // Hardware Recovery Pins (optional)
  GPIOPin *reset_pin_{nullptr};
  GPIOPin *boot_pin_{nullptr};

  // OTA State Machine
  OtaState ota_state_{OTA_IDLE};
  std::vector<uint8_t> ota_buffer_;
  size_t ota_total_size_{0};
  size_t ota_sent_size_{0};
  uint32_t ota_chunk_seq_{0};
  uint32_t ota_chunk_size_{4096};  // Updated by ota_ready response
  std::string ota_md5_;
  uint32_t ota_last_activity_{0};
  uint32_t ota_timeout_ms_{30000};

  // OTA Internal Methods
  void ota_send_start_();
  void ota_send_next_chunk_();
  void ota_send_end_();
  void ota_handle_response_(const std::string &type, const std::string &json);
  void ota_set_state_(OtaState state);
  void ota_set_status_(const std::string &status);
  void ota_update_progress_();
  void ota_abort_(const std::string &reason);
  void ota_complete_();
  void ota_check_timeout_();

  // Hardware Recovery Internal
  void pulse_reset_();
  void enter_bootloader_();
  void exit_bootloader_();

  // Base64 helper
  std::string base64_encode_(const uint8_t *data, size_t len);
  std::string md5_hash_(const uint8_t *data, size_t len);
};

}  // namespace zigbee_bridge
}  // namespace esphome
