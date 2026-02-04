#include "zigbee_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif
#ifdef USE_ESP_IDF
#include "esp_http_client.h"
#endif
#include "esphome/components/md5/md5.h"

namespace esphome {
namespace zigbee_bridge {

static const char *const TAG = "zigbee_bridge";

void ZigbeeBridge::setup() {
  ESP_LOGI(TAG, "Zigbee Bridge initializing...");
  this->rx_buffer_.reserve(512);

  // Initialize sensors with default values
  if (this->coordinator_ready_ != nullptr)
    this->coordinator_ready_->publish_state(false);
  if (this->permit_join_active_ != nullptr)
    this->permit_join_active_->publish_state(false);
  if (this->coordinator_status_ != nullptr)
    this->coordinator_status_->publish_state("waiting");
  if (this->device_count_ != nullptr)
    this->device_count_->publish_state(0);

  // Initialize OTA sensors
  if (this->ota_progress_ != nullptr)
    this->ota_progress_->publish_state(0);
  if (this->ota_status_ != nullptr)
    this->ota_status_->publish_state("idle");

  // Initialize optional hardware recovery pins
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);  // Not in reset
    ESP_LOGI(TAG, "Reset pin configured for hardware recovery");
  }
  if (this->boot_pin_ != nullptr) {
    this->boot_pin_->setup();
    this->boot_pin_->digital_write(true);  // Normal boot mode
    ESP_LOGI(TAG, "Boot pin configured for hardware recovery");
  }

#ifdef USE_MQTT
  // Subscribe to incoming commands from HA via MQTT
  auto *mqtt = mqtt::global_mqtt_client;
  if (mqtt != nullptr) {
    mqtt->subscribe("zigbee_bridge/bridge/command", [this](const std::string &topic, const std::string &payload) {
      ESP_LOGI(TAG, "MQTT cmd: %s", payload.c_str());
      this->send_raw(payload);
    }, 0);
    ESP_LOGI(TAG, "Subscribed to zigbee_bridge/bridge/command");
  }
#endif
}

void ZigbeeBridge::set_enabled(bool enabled) {
  if (this->enabled_ == enabled)
    return;

  this->enabled_ = enabled;
  ESP_LOGI(TAG, "Coordinator communication %s", enabled ? "ENABLED" : "DISABLED");

  if (this->coordinator_status_ != nullptr) {
    if (enabled) {
      this->coordinator_status_->publish_state(this->c5_ready_ ? "ready" : "waiting");
    } else {
      this->coordinator_status_->publish_state("disabled");
    }
  }

  if (this->coordinator_ready_ != nullptr && !enabled) {
    this->coordinator_ready_->publish_state(false);
  }
}

void ZigbeeBridge::loop() {
  // Skip processing if disabled
  if (!this->enabled_) {
    // Still drain UART buffer to prevent overflow
    while (this->available()) {
      uint8_t c;
      this->read_byte(&c);
    }
    return;
  }

  // On first loop after boot, request network info from C5 to sync state
  if (!this->c5_ready_ && this->cmd_id_ == 0 && millis() > 5000) {
    ESP_LOGI(TAG, "Requesting C5 network info to sync state...");
    this->send_network_info();
  }

  // Once C5 is ready, open network for known devices to rejoin
  if (this->c5_ready_ && !this->permit_join_sent_) {
    this->permit_join_sent_ = true;
    ESP_LOGI(TAG, "Sending auto permit_join (180s) for device rejoin...");
    this->send_permit_join(180);
  }

  // Request device list once MQTT is connected (devices need MQTT to publish)
  if (this->c5_ready_ && !this->device_list_requested_) {
#ifdef USE_MQTT
    auto *mqtt_client = mqtt::global_mqtt_client;
    if (mqtt_client != nullptr && mqtt_client->is_connected()) {
      ESP_LOGI(TAG, "MQTT connected, requesting device list from C5...");
      this->send_device_list();
      this->device_list_requested_ = true;
    }
#else
    this->device_list_requested_ = true;
#endif
  }

  // Check OTA timeout
  this->ota_check_timeout_();

  while (this->available()) {
    uint8_t c;
    this->read_byte(&c);
    this->last_rx_time_ = millis();

    if (c == '\n') {
      if (!this->rx_buffer_.empty()) {
        // Remove trailing \r if present
        if (this->rx_buffer_.back() == '\r')
          this->rx_buffer_.pop_back();
        if (!this->rx_buffer_.empty()) {
          this->process_line_(this->rx_buffer_);
        }
        this->rx_buffer_.clear();
      }
    } else if (c >= 0x20 && this->rx_buffer_.size() < 2048) {
      this->rx_buffer_ += (char) c;
    }
  }
}

void ZigbeeBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "Zigbee Bridge:");
  ESP_LOGCONFIG(TAG, "  C5 Ready: %s", this->c5_ready_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Devices: %d", this->device_count_val_);
}

// --- Command sending ---

void ZigbeeBridge::send_permit_join(uint16_t duration) {
  uint32_t id = ++this->cmd_id_;
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"cmd\":\"permit_join\",\"duration\":%u,\"id\":%lu}\n",
           duration, (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: permit_join duration=%u id=%lu", duration, (unsigned long) id);
}

void ZigbeeBridge::send_network_info() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"network_info\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: network_info id=%lu", (unsigned long) id);
}

void ZigbeeBridge::send_device_list() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"device_list\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: device_list id=%lu", (unsigned long) id);
}

void ZigbeeBridge::send_raw(const std::string &json_line) {
  this->write_str(json_line.c_str());
  if (json_line.empty() || json_line.back() != '\n')
    this->write_byte('\n');
  ESP_LOGD(TAG, "TX raw: %s", json_line.c_str());
}

void ZigbeeBridge::send_factory_reset() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"factory_reset\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGW(TAG, "TX: factory_reset id=%lu - Coordinator will restart!", (unsigned long) id);
  // Mark as not ready since coordinator will restart
  this->c5_ready_ = false;
  if (this->coordinator_ready_)
    this->coordinator_ready_->publish_state(false);
  if (this->coordinator_status_)
    this->coordinator_status_->publish_state("Factory Reset...");
}

void ZigbeeBridge::send_reboot() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"reboot\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: reboot id=%lu - Coordinator will restart", (unsigned long) id);
  // Mark as not ready since coordinator will restart
  this->c5_ready_ = false;
  if (this->coordinator_ready_)
    this->coordinator_ready_->publish_state(false);
  if (this->coordinator_status_)
    this->coordinator_status_->publish_state("Rebooting...");
}

void ZigbeeBridge::send_sleep() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"sleep\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: sleep id=%lu - Coordinator entering sleep mode", (unsigned long) id);

  // Also disable local processing
  this->enabled_ = false;

  if (this->coordinator_status_)
    this->coordinator_status_->publish_state("sleeping");
}

void ZigbeeBridge::send_wake() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"wake\",\"id\":%lu}\n",
           (unsigned long) id);
  this->write_str(buf);
  ESP_LOGI(TAG, "TX: wake id=%lu - Coordinator resuming operation", (unsigned long) id);

  // Re-enable local processing
  this->enabled_ = true;

  if (this->coordinator_status_)
    this->coordinator_status_->publish_state(this->c5_ready_ ? "ready" : "waiting");
}

// --- Line processing ---

void ZigbeeBridge::process_line_(const std::string &line) {
  // Find the start of JSON (skip any garbage before '{')
  size_t json_start = line.find('{');
  if (json_start == std::string::npos) {
    // No JSON found, skip silently
    return;
  }

  std::string json_line = line.substr(json_start);
  ESP_LOGD(TAG, "RX: %s", json_line.c_str());

  // Extract "type" field
  std::string type = this->json_get_string_(json_line, "type");
  if (type.empty()) {
    ESP_LOGW(TAG, "No 'type' field in: %s", json_line.c_str());
    return;
  }

  if (type == "ready") {
    this->handle_ready_(json_line);
  } else if (type == "device_joined") {
    this->handle_device_joined_(json_line);
  } else if (type == "device_left") {
    this->handle_device_left_(json_line);
  } else if (type == "interview_done") {
    this->handle_interview_done_(json_line);
  } else if (type == "attribute") {
    this->handle_attribute_(json_line);
  } else if (type == "response") {
    this->handle_response_(json_line);
  } else if (type == "permit_join") {
    this->handle_permit_join_(json_line);
  } else if (type == "network_info") {
    this->handle_network_info_(json_line);
  } else if (type == "device_list") {
    this->handle_device_list_(json_line);
  } else if (type == "device_state") {
    // Full device state from C5 — extract LQI and driver state
    std::string ieee = this->json_get_string_(json_line, "ieee");
    if (!ieee.empty()) {
      auto it = this->devices_.find(ieee);
      if (it != this->devices_.end()) {
        int lqi = this->json_get_int_(json_line, "link_quality", 0);
        if (lqi > 0) it->second.linkquality = lqi;

        // Extract driver state fields from nested "state" object
        if (this->is_vibration_(it->second.model)) {
          // Find the "state":{...} sub-object and parse fields from it
          size_t state_pos = json_line.find("\"state\":{");
          if (state_pos != std::string::npos) {
            // Extract the state sub-object
            size_t obj_start = json_line.find('{', state_pos + 7);
            if (obj_start != std::string::npos) {
              int depth = 1;
              size_t obj_end = obj_start + 1;
              while (obj_end < json_line.size() && depth > 0) {
                if (json_line[obj_end] == '{') depth++;
                else if (json_line[obj_end] == '}') depth--;
                obj_end++;
              }
              std::string state_json = json_line.substr(obj_start, obj_end - obj_start);

              auto &dev = it->second;
              // Parse vibration fields from state sub-object
              // Only update event data if the JSON contains "vibration" key (C5 omits it if no event yet)
              if (state_json.find("\"vibration\":") != std::string::npos) {
                dev.has_vibration_data = true;
                std::string action = this->json_get_string_(state_json, "action");
                if (!action.empty()) dev.action = action;
                dev.vibration = (state_json.find("\"vibration\":true") != std::string::npos);
                dev.angle_x = this->json_get_double_(state_json, "angle_x", dev.angle_x);
                dev.angle_y = this->json_get_double_(state_json, "angle_y", dev.angle_y);
                dev.angle_z = this->json_get_double_(state_json, "angle_z", dev.angle_z);
                dev.strength = this->json_get_int_(state_json, "strength", dev.strength);
              }
              // Battery and voltage come from 0xFF01 TLV — always update if present
              int bat = this->json_get_int_(state_json, "battery", -1);
              if (bat >= 0) dev.battery = bat;
              int volt = this->json_get_int_(state_json, "voltage", 0);
              if (volt > 0) dev.voltage = volt;
              std::string sens = this->json_get_string_(state_json, "sensitivity");
              if (!sens.empty()) dev.sensitivity = sens;
            }
          }
        }

        this->mqtt_publish_device_state_(ieee);

        // Device sent state — it's online
        auto *mqtt = mqtt::global_mqtt_client;
        if (mqtt != nullptr && mqtt->is_connected()) {
          std::string avail_topic = "zigbee_bridge/" + it->second.friendly_name + "/availability";
          mqtt->publish(avail_topic, std::string("online"), 0, true);
        }
      }
      this->mqtt_publish_("devices/" + ieee + "/state", json_line);
    }
  } else if (type == "tuya_dp") {
    this->handle_tuya_dp_(json_line);
  } else if (type == "availability") {
    // Device online/offline change from C5 availability tracker
    std::string ieee = this->json_get_string_(json_line, "ieee");
    if (!ieee.empty()) {
      bool online = (json_line.find("\"online\":true") != std::string::npos);
      auto it = this->devices_.find(ieee);
      if (it != this->devices_.end()) {
        std::string avail_topic = "zigbee_bridge/" + it->second.friendly_name + "/availability";
        auto *mqtt = mqtt::global_mqtt_client;
        if (mqtt != nullptr && mqtt->is_connected()) {
          mqtt->publish(avail_topic, std::string(online ? "online" : "offline"), 0, true);
        }
      }
      this->mqtt_publish_("devices/" + ieee + "/availability", online ? "online" : "offline");
      ESP_LOGI(TAG, "Device %s availability: %s", ieee.c_str(), online ? "online" : "offline");
    }
  } else if (type == "poll_check_in") {
    // Sleepy device checked in — update availability
    std::string ieee = this->json_get_string_(json_line, "ieee");
    if (!ieee.empty()) {
      this->mqtt_publish_("devices/" + ieee + "/availability", "online");
      // Also publish on friendly-name topic (used by HA discovery)
      auto it = this->devices_.find(ieee);
      if (it != this->devices_.end()) {
        auto *mqtt = mqtt::global_mqtt_client;
        if (mqtt != nullptr && mqtt->is_connected()) {
          std::string avail_topic = "zigbee_bridge/" + it->second.friendly_name + "/availability";
          mqtt->publish(avail_topic, std::string("online"), 0, true);
        }
      }
      ESP_LOGI(TAG, "Poll check-in from %s", ieee.c_str());
    }
  } else if (type == "device_announce") {
    // Device announced itself — it's online
    std::string ieee = this->json_get_string_(json_line, "ieee");
    if (!ieee.empty()) {
      this->mqtt_publish_("devices/" + ieee + "/availability", "online");
      // Also publish on friendly-name topic (used by HA discovery)
      auto it = this->devices_.find(ieee);
      if (it != this->devices_.end()) {
        auto *mqtt = mqtt::global_mqtt_client;
        if (mqtt != nullptr && mqtt->is_connected()) {
          std::string avail_topic = "zigbee_bridge/" + it->second.friendly_name + "/availability";
          mqtt->publish(avail_topic, std::string("online"), 0, true);
        }
      }
      this->mqtt_publish_("bridge/event/device_announce", json_line);
    }
  } else if (type == "device_unavailable") {
    // Device didn't respond to MAC/APS — mark offline
    std::string ieee = this->json_get_string_(json_line, "ieee");
    if (!ieee.empty()) {
      ESP_LOGW(TAG, "Device unavailable: %s", ieee.c_str());
      // Also publish on friendly-name topic (used by HA discovery)
      auto it = this->devices_.find(ieee);
      if (it != this->devices_.end()) {
        auto *mqtt = mqtt::global_mqtt_client;
        if (mqtt != nullptr && mqtt->is_connected()) {
          std::string avail_topic = "zigbee_bridge/" + it->second.friendly_name + "/availability";
          mqtt->publish(avail_topic, std::string("offline"), 0, true);
        }
      }
      this->mqtt_publish_("bridge/event/device_unavailable", json_line);
    }
  } else if (type == "command_failed") {
    // ZCL command failed
    ESP_LOGW(TAG, "Command failed: %s", json_line.c_str());
    this->mqtt_publish_("bridge/event/command_failed", json_line);
  } else if (type == "alarm") {
    // Alarm notification from device
    std::string ieee = this->json_get_string_(json_line, "ieee");
    ESP_LOGW(TAG, "ALARM from %s: %s", ieee.c_str(), json_line.c_str());
    this->mqtt_publish_("bridge/event/alarm", json_line);
    if (!ieee.empty()) {
      this->mqtt_publish_("devices/" + ieee + "/alarm", json_line);
    }
  } else if (type == "nlme_status") {
    ESP_LOGW(TAG, "NLME status: %s", json_line.c_str());
    this->mqtt_publish_("bridge/event/nlme_status", json_line);
  } else if (type == "network_started") {
    ESP_LOGI(TAG, "Network started on C5");
    this->mqtt_publish_("bridge/event/network_started", json_line);
  } else if (type == "ota_ready" || type == "ota_ack" ||
             type == "ota_complete" || type == "ota_error") {
    // OTA responses from C5/H2
    this->ota_handle_response_(type, json_line);
  } else {
    ESP_LOGD(TAG, "Unknown type: %s", type.c_str());
  }
}

void ZigbeeBridge::handle_ready_(const std::string &line) {
  // Network info is nested: {"type":"ready","version":"1.0","network":{"pan_id":"0x1A62","channel":11}}
  // Our simple parser works on flat JSON, so search for channel and pan_id anywhere in the string
  int channel = this->json_get_int_(line, "channel");
  int pan_id = this->json_get_int_(line, "pan_id");

  if (this->c5_ready_) {
    // C5 sent ready again — likely rebooted. Log but don't re-publish all states.
    ESP_LOGW(TAG, "C5 sent READY again (reboot?) PAN=0x%04X CH=%d", pan_id, channel);
    return;
  }

  this->c5_ready_ = true;
  ESP_LOGI(TAG, "C5 Coordinator READY (PAN=0x%04X, CH=%d)", pan_id, channel);

  if (this->coordinator_ready_ != nullptr)
    this->coordinator_ready_->publish_state(true);
  if (this->coordinator_status_ != nullptr)
    this->coordinator_status_->publish_state("ready");
  if (this->zigbee_channel_ != nullptr)
    this->zigbee_channel_->publish_state(channel);
  if (this->network_info_ != nullptr) {
    char buf[64];
    snprintf(buf, sizeof(buf), "PAN=0x%04X CH=%d", pan_id, channel);
    this->network_info_->publish_state(buf);
  }

  // Publish bridge status to MQTT
  char mqtt_buf[128];
  snprintf(mqtt_buf, sizeof(mqtt_buf),
           "{\"state\":\"online\",\"pan_id\":\"0x%04X\",\"channel\":%d}",
           pan_id, channel);
  this->mqtt_publish_("bridge/state", std::string(mqtt_buf));
}

void ZigbeeBridge::handle_device_joined_(const std::string &line) {
  std::string ieee = this->json_get_string_(line, "ieee");
  int short_addr = this->json_get_int_(line, "short_addr");
  this->device_count_val_++;

  ESP_LOGI(TAG, "Device JOINED: %s (0x%04X)", ieee.c_str(), short_addr);

  if (this->device_count_ != nullptr)
    this->device_count_->publish_state(this->device_count_val_);

  // Publish to MQTT
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"ieee\":\"%s\",\"short_addr\":\"0x%04X\",\"event\":\"joined\"}",
           ieee.c_str(), short_addr);
  this->mqtt_publish_("bridge/event/device_joined", std::string(payload));
  this->mqtt_publish_("devices/" + ieee + "/availability", "online");
}

void ZigbeeBridge::handle_device_left_(const std::string &line) {
  std::string ieee = this->json_get_string_(line, "ieee");
  if (this->device_count_val_ > 0)
    this->device_count_val_--;

  ESP_LOGI(TAG, "Device LEFT: %s", ieee.c_str());

  if (this->device_count_ != nullptr)
    this->device_count_->publish_state(this->device_count_val_);

  // Publish to MQTT
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"ieee\":\"%s\",\"event\":\"left\"}", ieee.c_str());
  this->mqtt_publish_("bridge/event/device_left", std::string(payload));
  this->mqtt_publish_("devices/" + ieee + "/availability", "offline");
}

void ZigbeeBridge::handle_interview_done_(const std::string &line) {
  std::string ieee = this->json_get_string_(line, "ieee");
  std::string model = this->json_get_string_(line, "model");
  std::string mfg = this->json_get_string_(line, "manufacturer");

  ESP_LOGI(TAG, "Interview DONE: %s model=%s mfg=%s", ieee.c_str(), model.c_str(), mfg.c_str());

  // Publish device info to MQTT
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"ieee\":\"%s\",\"model\":\"%s\",\"manufacturer\":\"%s\",\"interviewed\":true}",
           ieee.c_str(), model.c_str(), mfg.c_str());
  this->mqtt_publish_("devices/" + ieee + "/info", std::string(payload));
  this->mqtt_publish_("bridge/event/interview_done", std::string(payload));

  // Publish HA MQTT Discovery for newly interviewed device
  this->mqtt_publish_discovery_(ieee, model, mfg, ieee);
}

void ZigbeeBridge::handle_attribute_(const std::string &line) {
  std::string ieee = this->json_get_string_(line, "ieee");
  int ep = this->json_get_int_(line, "ep");
  int cluster = this->json_get_int_(line, "cluster");
  int attr = this->json_get_int_(line, "attr");
  double value = this->json_get_double_(line, "value");

  ESP_LOGD(TAG, "Attribute: %s ep=%d cluster=0x%04X attr=0x%04X val=%.2f",
           ieee.c_str(), ep, cluster, attr, value);

  // Publish raw attribute to MQTT — forward entire JSON line for maximum flexibility
  this->mqtt_publish_("devices/" + ieee + "/attribute", line);
}

void ZigbeeBridge::handle_response_(const std::string &line) {
  int id = this->json_get_int_(line, "id");
  std::string status = this->json_get_string_(line, "status");

  ESP_LOGI(TAG, "Response: id=%d status=%s", id, status.c_str());
}

void ZigbeeBridge::handle_permit_join_(const std::string &line) {
  int duration = this->json_get_int_(line, "duration");
  bool enabled = duration > 0;

  ESP_LOGI(TAG, "Permit join: %s (duration=%d)", enabled ? "OPEN" : "CLOSED", duration);

  if (this->permit_join_active_ != nullptr)
    this->permit_join_active_->publish_state(enabled);
}

void ZigbeeBridge::handle_network_info_(const std::string &line) {
  int pan_id = this->json_get_int_(line, "pan_id");
  int channel = this->json_get_int_(line, "channel");
  int count = this->json_get_int_(line, "device_count");

  ESP_LOGI(TAG, "Network: PAN=0x%04X CH=%d devices=%d", pan_id, channel, count);

  // Mark C5 as ready if not already (network_info response means C5 is running)
  if (!this->c5_ready_) {
    this->c5_ready_ = true;
    if (this->coordinator_ready_ != nullptr)
      this->coordinator_ready_->publish_state(true);
    if (this->coordinator_status_ != nullptr)
      this->coordinator_status_->publish_state("ready");

    // Publish bridge status to MQTT
    char mqtt_buf[128];
    snprintf(mqtt_buf, sizeof(mqtt_buf),
             "{\"state\":\"online\",\"pan_id\":\"0x%04X\",\"channel\":%d}",
             pan_id, channel);
    this->mqtt_publish_("bridge/state", std::string(mqtt_buf));
  }

  this->device_count_val_ = count;
  if (this->device_count_ != nullptr)
    this->device_count_->publish_state(count);
  if (this->zigbee_channel_ != nullptr)
    this->zigbee_channel_->publish_state(channel);
  if (this->network_info_ != nullptr) {
    char buf[64];
    snprintf(buf, sizeof(buf), "PAN=0x%04X CH=%d dev=%d", pan_id, channel, count);
    this->network_info_->publish_state(buf);
  }
}

void ZigbeeBridge::handle_device_list_(const std::string &line) {
  // C5 sends: {"type":"device_list","devices":[{...},{...}]}
  // Parse each device object from the array and publish to MQTT
  ESP_LOGI(TAG, "Device list received (%d bytes)", (int) line.size());

  int count = 0;
  size_t pos = 0;

  // Find "devices":[ and iterate through objects
  size_t arr_start = line.find("\"devices\":[");
  if (arr_start == std::string::npos) {
    ESP_LOGW(TAG, "No devices array in device_list");
    return;
  }
  pos = arr_start + 11;  // skip past "devices":[

  while (pos < line.size()) {
    // Find next device object
    size_t obj_start = line.find('{', pos);
    if (obj_start == std::string::npos)
      break;

    // Find matching closing brace (simple — no nested objects expected)
    size_t obj_end = line.find('}', obj_start);
    if (obj_end == std::string::npos)
      break;

    std::string dev_json = line.substr(obj_start, obj_end - obj_start + 1);
    std::string ieee = this->json_get_string_(dev_json, "ieee");

    if (!ieee.empty()) {
      std::string model = this->json_get_string_(dev_json, "model");
      std::string mfg = this->json_get_string_(dev_json, "manufacturer");
      std::string name = this->json_get_string_(dev_json, "friendly_name");
      std::string short_addr = this->json_get_string_(dev_json, "short_addr");

      ESP_LOGI(TAG, "  Device: %s model=%s mfg=%s name=%s",
               ieee.c_str(), model.c_str(), mfg.c_str(), name.c_str());

      // Publish device info to MQTT (retained)
      char payload[384];
      snprintf(payload, sizeof(payload),
               "{\"ieee\":\"%s\",\"short_addr\":\"%s\",\"model\":\"%s\","
               "\"manufacturer\":\"%s\",\"friendly_name\":\"%s\"}",
               ieee.c_str(), short_addr.c_str(), model.c_str(),
               mfg.c_str(), name.c_str());
      this->mqtt_publish_("devices/" + ieee + "/info", std::string(payload));

      // Check online status
      bool online = (dev_json.find("\"online\":true") != std::string::npos);
      this->mqtt_publish_("devices/" + ieee + "/availability",
                          online ? "online" : "offline");

      // Publish HA MQTT Discovery config for this device
      this->mqtt_publish_discovery_(ieee, model, mfg, name);
      count++;
    }

    pos = obj_end + 1;
  }

  ESP_LOGI(TAG, "Published %d devices to MQTT", count);
  this->device_count_val_ = count;
  if (this->device_count_ != nullptr)
    this->device_count_->publish_state(count);
}

// --- MQTT publish helper ---

void ZigbeeBridge::mqtt_publish_(const std::string &topic, const std::string &payload) {
#ifdef USE_MQTT
  auto *mqtt = mqtt::global_mqtt_client;
  if (mqtt == nullptr || !mqtt->is_connected()) {
    return;
  }
  // Full topic: zigbee_bridge/<topic>  (topic_prefix from YAML config is applied by ESPHome)
  std::string full_topic = "zigbee_bridge/" + topic;
  // Retain bridge/state and device availability, fire-and-forget for events
  bool retain = (topic.find("bridge/state") != std::string::npos ||
                 topic.find("/availability") != std::string::npos ||
                 topic.find("/info") != std::string::npos);
  mqtt->publish(full_topic, payload, 0, retain);
  ESP_LOGV(TAG, "MQTT: %s = %s", full_topic.c_str(), payload.c_str());
#endif
}

// --- Aggregated device state publishing (Z2M-compatible) ---

void ZigbeeBridge::mqtt_publish_device_state_(const std::string &ieee) {
#ifdef USE_MQTT
  auto it = this->devices_.find(ieee);
  if (it == this->devices_.end()) return;

  auto &dev = it->second;
  std::string topic = "zigbee_bridge/" + dev.friendly_name;

  // Build aggregated state JSON (Z2M-compatible)
  std::string json = "{\"state\":\"" + dev.state + "\"";
  if (dev.battery >= 0) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", dev.battery);
    json += ",\"battery\":";
    json += tmp;
  }
  if (dev.linkquality > 0) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", dev.linkquality);
    json += ",\"linkquality\":";
    json += tmp;
  }
  // Aqara vibration sensor fields
  if (this->is_vibration_(dev.model)) {
    // Only include event data if we've received at least one real event
    if (dev.has_vibration_data) {
      json += std::string(",\"vibration\":") + (dev.vibration ? "true" : "false");
      json += ",\"action\":\"" + dev.action + "\"";
      char tmp[32];
      snprintf(tmp, sizeof(tmp), "%.1f", dev.angle_x);
      json += ",\"angle_x\":"; json += tmp;
      snprintf(tmp, sizeof(tmp), "%.1f", dev.angle_y);
      json += ",\"angle_y\":"; json += tmp;
      snprintf(tmp, sizeof(tmp), "%.1f", dev.angle_z);
      json += ",\"angle_z\":"; json += tmp;
      snprintf(tmp, sizeof(tmp), "%d", dev.strength);
      json += ",\"strength\":"; json += tmp;
    }
    if (dev.voltage > 0) {
      char tmp[32];
      snprintf(tmp, sizeof(tmp), "%d", dev.voltage);
      json += ",\"voltage\":"; json += tmp;
    }
    json += ",\"sensitivity\":\"" + dev.sensitivity + "\"";
  }
  // Tuya fingerbot fields
  if (this->is_fingerbot_(dev.model)) {
    json += ",\"mode\":\"" + dev.mode + "\"";
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", dev.down_movement);
    json += ",\"down_movement\":";
    json += tmp;
    snprintf(tmp, sizeof(tmp), "%d", dev.up_movement);
    json += ",\"up_movement\":";
    json += tmp;
    snprintf(tmp, sizeof(tmp), "%d", dev.sustain_time);
    json += ",\"sustain_time\":";
    json += tmp;
    json += std::string(",\"reverse\":\"") + (dev.reverse ? "ON" : "OFF") + "\"";
    json += std::string(",\"touch_control\":\"") + (dev.touch_control ? "ON" : "OFF") + "\"";
    json += std::string(",\"program_enable\":\"") + (dev.program_enable ? "ON" : "OFF") + "\"";
    json += std::string(",\"repeat_forever\":\"") + (dev.repeat_forever ? "ON" : "OFF") + "\"";
    json += ",\"program\":\"" + dev.program + "\"";
  }
  json += "}";

  auto *mqtt = mqtt::global_mqtt_client;
  if (mqtt != nullptr && mqtt->is_connected()) {
    mqtt->publish(topic, json, 0, true);
  }
#endif
}

// --- HA MQTT Discovery (Z2M-compatible format) ---

void ZigbeeBridge::mqtt_publish_bridge_discovery_() {
#ifdef USE_MQTT
  auto *mqtt_client = mqtt::global_mqtt_client;
  if (mqtt_client == nullptr || !mqtt_client->is_connected())
    return;

  char bridge_config[384];
  snprintf(bridge_config, sizeof(bridge_config),
           "{\"name\":\"Bridge State\","
           "\"unique_id\":\"zigbee_bridge_state\","
           "\"state_topic\":\"zigbee_bridge/bridge/state\","
           "\"value_template\":\"{{ value_json.state }}\","
           "\"entity_category\":\"diagnostic\","
           "\"device\":{\"identifiers\":[\"zigbee_bridge\"],\"name\":\"ESP32 Zigbee Bridge\","
           "\"manufacturer\":\"Espressif\",\"model\":\"ESP32-S3 + ESP32-C5\",\"sw_version\":\"1.0.0\"}}");

  mqtt_client->publish("homeassistant/sensor/zigbee_bridge/state/config",
                        std::string(bridge_config), 0, true);
  ESP_LOGI(TAG, "Published bridge device discovery");
#endif
}

void ZigbeeBridge::mqtt_publish_discovery_(const std::string &ieee,
                                            const std::string &model,
                                            const std::string &manufacturer,
                                            const std::string &name) {
#ifdef USE_MQTT
  auto *mqtt_client = mqtt::global_mqtt_client;
  if (mqtt_client == nullptr || !mqtt_client->is_connected())
    return;

  // Ensure bridge device is registered first
  this->mqtt_publish_bridge_discovery_();

  // Clear old discovery topics (from previous format with node_id_suffix instead of /node_id/suffix)
  {
    std::string old_node = "0x" + ieee;
    std::string empty;
    // Old format: homeassistant/<type>/<node_id>_<entity>/config
    mqtt_client->publish("homeassistant/switch/" + old_node + "_switch/config", empty, 0, true);
    mqtt_client->publish("homeassistant/sensor/" + old_node + "_battery/config", empty, 0, true);
    mqtt_client->publish("homeassistant/sensor/" + old_node + "_linkquality/config", empty, 0, true);
    ESP_LOGD(TAG, "Cleared old discovery topics for %s", old_node.c_str());
  }

  // Cache device info
  auto &dev = this->devices_[ieee];
  dev.friendly_name = name.empty() ? ("Device_" + ieee.substr(ieee.size() > 4 ? ieee.size() - 4 : 0)) : name;
  dev.model = model;
  dev.manufacturer = manufacturer;
  if (dev.state.empty()) dev.state = "OFF";

  std::string node_id = "0x" + ieee;
  std::string friendly = dev.friendly_name;
  std::string state_topic = "zigbee_bridge/" + friendly;
  std::string avail_topic = "zigbee_bridge/" + friendly + "/availability";
  std::string cmd_topic = "zigbee_bridge/" + friendly + "/set";

  // Device block (reused in all entity configs)
  std::string device_block = "\"device\":{\"name\":\"" + friendly + "\","
    "\"identifiers\":[\"zigbee_bridge_" + node_id + "\"],"
    "\"model\":\"" + model + "\",\"manufacturer\":\"" + manufacturer + "\","
    "\"sw_version\":\"1.0.0\",\"via_device\":\"zigbee_bridge\"}";

  // Availability block (plain string "online"/"offline")
  std::string avail_block = "\"availability_topic\":\"" + avail_topic + "\"";

  bool fingerbot = this->is_fingerbot_(model);
  bool vibration = this->is_vibration_(model);

  ESP_LOGW(TAG, "Discovery: ieee=%s model='%s' vibration=%d fingerbot=%d",
           ieee.c_str(), model.c_str(), (int)vibration, (int)fingerbot);

  // ============= Switch entity (main ON/OFF — skip for vibration sensor) =============
  if (!vibration)
  {
    std::string cfg = "{\"unique_id\":\"" + node_id + "_switch\","
      "\"name\":\"Switch\","
      "\"state_topic\":\"" + state_topic + "\","
      + avail_block + ","
      "\"command_topic\":\"" + cmd_topic + "\","
      "\"value_template\":\"{{ value_json.state }}\","
      "\"payload_on\":\"{\\\"state\\\":\\\"ON\\\"}\","
      "\"payload_off\":\"{\\\"state\\\":\\\"OFF\\\"}\","
      "\"state_on\":\"ON\",\"state_off\":\"OFF\","
      + device_block + "}";
    mqtt_client->publish("homeassistant/switch/" + node_id + "/switch/config", cfg, 0, true);
  }

  // ============= Battery sensor =============
  {
    std::string cfg = "{\"unique_id\":\"" + node_id + "_battery\","
      "\"name\":\"Battery\","
      "\"device_class\":\"battery\","
      "\"state_class\":\"measurement\","
      "\"unit_of_measurement\":\"%\","
      "\"icon\":\"mdi:battery\","
      "\"entity_category\":\"diagnostic\","
      "\"state_topic\":\"" + state_topic + "\","
      + avail_block + ","
      "\"value_template\":\"{{ value_json.battery | default(0) }}\","
      + device_block + "}";
    mqtt_client->publish("homeassistant/sensor/" + node_id + "/battery/config", cfg, 0, true);
  }

  // ============= Link quality sensor =============
  {
    std::string cfg = "{\"unique_id\":\"" + node_id + "_linkquality\","
      "\"name\":\"Linkquality\","
      "\"icon\":\"mdi:signal\","
      "\"state_class\":\"measurement\","
      "\"unit_of_measurement\":\"lqi\","
      "\"entity_category\":\"diagnostic\","
      "\"state_topic\":\"" + state_topic + "\","
      + avail_block + ","
      "\"value_template\":\"{{ value_json.linkquality }}\","
      + device_block + "}";
    mqtt_client->publish("homeassistant/sensor/" + node_id + "/linkquality/config", cfg, 0, true);
  }

  // ============= Fingerbot-specific entities =============
  if (fingerbot) {
    // --- Mode select (push/switch/program) ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_mode\","
        "\"name\":\"Mode\","
        "\"icon\":\"mdi:gesture-tap-button\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.mode }}\","
        "\"command_template\":\"{\\\"mode\\\": \\\"{{ value }}\\\"}\","
        "\"options\":[\"push\",\"switch\",\"program\"],"
        + device_block + "}";
      mqtt_client->publish("homeassistant/select/" + node_id + "/mode/config", cfg, 0, true);
    }

    // --- Switch: Reverse ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_reverse\","
        "\"name\":\"Reverse\","
        "\"icon\":\"mdi:swap-vertical\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.reverse }}\","
        "\"payload_on\":\"{\\\"reverse\\\":\\\"ON\\\"}\","
        "\"payload_off\":\"{\\\"reverse\\\":\\\"OFF\\\"}\","
        "\"state_on\":\"ON\",\"state_off\":\"OFF\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/switch/" + node_id + "/reverse/config", cfg, 0, true);
    }

    // --- Switch: Touch Control ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_touch_control\","
        "\"name\":\"Touch control\","
        "\"icon\":\"mdi:gesture-tap\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.touch_control }}\","
        "\"payload_on\":\"{\\\"touch_control\\\":\\\"ON\\\"}\","
        "\"payload_off\":\"{\\\"touch_control\\\":\\\"OFF\\\"}\","
        "\"state_on\":\"ON\",\"state_off\":\"OFF\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/switch/" + node_id + "/touch_control/config", cfg, 0, true);
    }

    // --- Switch: Program Enable ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_program_enable\","
        "\"name\":\"Program enable\","
        "\"icon\":\"mdi:playlist-check\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.program_enable }}\","
        "\"payload_on\":\"{\\\"program_enable\\\":\\\"ON\\\"}\","
        "\"payload_off\":\"{\\\"program_enable\\\":\\\"OFF\\\"}\","
        "\"state_on\":\"ON\",\"state_off\":\"OFF\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/switch/" + node_id + "/program_enable/config", cfg, 0, true);
    }

    // --- Switch: Repeat Forever ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_repeat_forever\","
        "\"name\":\"Repeat forever\","
        "\"icon\":\"mdi:repeat\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.repeat_forever }}\","
        "\"payload_on\":\"{\\\"repeat_forever\\\":\\\"ON\\\"}\","
        "\"payload_off\":\"{\\\"repeat_forever\\\":\\\"OFF\\\"}\","
        "\"state_on\":\"ON\",\"state_off\":\"OFF\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/switch/" + node_id + "/repeat_forever/config", cfg, 0, true);
    }

    // --- Number: Down Movement (51-100%) ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_down_movement\","
        "\"name\":\"Down movement\","
        "\"icon\":\"mdi:arrow-down-bold\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.down_movement }}\","
        "\"command_template\":\"{\\\"down_movement\\\": {{ value }}}\","
        "\"min\":51,\"max\":100,\"step\":1,"
        "\"unit_of_measurement\":\"%\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/number/" + node_id + "/down_movement/config", cfg, 0, true);
    }

    // --- Number: Up Movement (0-49%) ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_up_movement\","
        "\"name\":\"Up movement\","
        "\"icon\":\"mdi:arrow-up-bold\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.up_movement }}\","
        "\"command_template\":\"{\\\"up_movement\\\": {{ value }}}\","
        "\"min\":0,\"max\":49,\"step\":1,"
        "\"unit_of_measurement\":\"%\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/number/" + node_id + "/up_movement/config", cfg, 0, true);
    }

    // --- Number: Sustain Time (0-255s) ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_sustain_time\","
        "\"name\":\"Sustain time\","
        "\"icon\":\"mdi:timer-outline\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.sustain_time }}\","
        "\"command_template\":\"{\\\"sustain_time\\\": {{ value }}}\","
        "\"min\":0,\"max\":255,\"step\":1,"
        "\"unit_of_measurement\":\"s\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/number/" + node_id + "/sustain_time/config", cfg, 0, true);
    }

    // --- Button: Push ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_push\","
        "\"name\":\"Push\","
        "\"icon\":\"mdi:gesture-tap\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"payload_press\":\"{\\\"push\\\": \\\"\\\"}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/button/" + node_id + "/push/config", cfg, 0, true);
    }

    // --- Button: On ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_btn_on\","
        "\"name\":\"On\","
        "\"icon\":\"mdi:power-on\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"payload_press\":\"{\\\"state\\\": \\\"ON\\\"}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/button/" + node_id + "/on/config", cfg, 0, true);
    }

    // --- Button: Off ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_btn_off\","
        "\"name\":\"Off\","
        "\"icon\":\"mdi:power-off\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"payload_press\":\"{\\\"state\\\": \\\"OFF\\\"}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/button/" + node_id + "/off/config", cfg, 0, true);
    }

    // --- Button: Run Program ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_run_program\","
        "\"name\":\"Run program\","
        "\"icon\":\"mdi:play\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"payload_press\":\"{\\\"run_program\\\": \\\"\\\"}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/button/" + node_id + "/run_program/config", cfg, 0, true);
    }

    // --- Text: Program ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_program\","
        "\"name\":\"Program\","
        "\"icon\":\"mdi:playlist-edit\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.program }}\","
        "\"command_template\":\"{\\\"program\\\": \\\"{{ value }}\\\"}\","
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/text/" + node_id + "/program/config", cfg, 0, true);
    }
  }

  // ============= Aqara vibration sensor entities =============
  if (vibration) {
    // --- Binary Sensor: Vibration ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_vibration\","
        "\"name\":\"Vibration\","
        "\"device_class\":\"vibration\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ 'ON' if value_json.vibration else 'OFF' }}\","
        "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
        "\"off_delay\":65,"
        + device_block + "}";
      mqtt_client->publish("homeassistant/binary_sensor/" + node_id + "/vibration/config", cfg, 0, true);
    }

    // --- Sensor: Action ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_action\","
        "\"name\":\"Action\","
        "\"icon\":\"mdi:gesture-double-tap\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.action }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/action/config", cfg, 0, true);
    }

    // --- Sensor: Angle X ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_angle_x\","
        "\"name\":\"Angle X\","
        "\"icon\":\"mdi:angle-acute\","
        "\"state_class\":\"measurement\","
        "\"unit_of_measurement\":\"\\u00b0\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.angle_x | default(0) }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/angle_x/config", cfg, 0, true);
    }

    // --- Sensor: Angle Y ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_angle_y\","
        "\"name\":\"Angle Y\","
        "\"icon\":\"mdi:angle-acute\","
        "\"state_class\":\"measurement\","
        "\"unit_of_measurement\":\"\\u00b0\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.angle_y | default(0) }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/angle_y/config", cfg, 0, true);
    }

    // --- Sensor: Angle Z ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_angle_z\","
        "\"name\":\"Angle Z\","
        "\"icon\":\"mdi:angle-acute\","
        "\"state_class\":\"measurement\","
        "\"unit_of_measurement\":\"\\u00b0\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.angle_z | default(0) }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/angle_z/config", cfg, 0, true);
    }

    // --- Sensor: Strength ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_strength\","
        "\"name\":\"Strength\","
        "\"icon\":\"mdi:flash\","
        "\"state_class\":\"measurement\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.strength | default(0) }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/strength/config", cfg, 0, true);
    }

    // --- Sensor: Voltage ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_voltage\","
        "\"name\":\"Voltage\","
        "\"device_class\":\"voltage\","
        "\"state_class\":\"measurement\","
        "\"unit_of_measurement\":\"mV\","
        "\"icon\":\"mdi:flash-triangle\","
        "\"entity_category\":\"diagnostic\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"value_template\":\"{{ value_json.voltage | default(0) }}\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/sensor/" + node_id + "/voltage/config", cfg, 0, true);
    }

    // --- Select: Sensitivity ---
    {
      std::string cfg = "{\"unique_id\":\"" + node_id + "_sensitivity\","
        "\"name\":\"Sensitivity\","
        "\"icon\":\"mdi:tune-vertical\","
        "\"state_topic\":\"" + state_topic + "\","
        + avail_block + ","
        "\"command_topic\":\"" + cmd_topic + "\","
        "\"value_template\":\"{{ value_json.sensitivity }}\","
        "\"command_template\":\"{\\\"sensitivity\\\": \\\"{{ value }}\\\"}\","
        "\"options\":[\"high\",\"medium\",\"low\"],"
        "\"entity_category\":\"config\","
        + device_block + "}";
      mqtt_client->publish("homeassistant/select/" + node_id + "/sensitivity/config", cfg, 0, true);
    }
  }

  // Subscribe to unified command topic for this device
  mqtt_client->subscribe(cmd_topic, [this, ieee](const std::string &topic, const std::string &payload) {
    ESP_LOGI(TAG, "Device cmd: %s = %s", topic.c_str(), payload.c_str());
    this->handle_device_command_(ieee, payload);
  }, 0);
  ESP_LOGI(TAG, "Subscribed to %s", cmd_topic.c_str());

  // Publish initial availability (plain string)
  mqtt_client->publish(avail_topic, std::string("online"), 0, true);

  // Publish initial state
  this->mqtt_publish_device_state_(ieee);

  const char *type_str = fingerbot ? "fingerbot (15 entities)" :
                          vibration ? "vibration (11 entities)" : "generic (3 entities)";
  ESP_LOGI(TAG, "Published discovery for %s (%s) - %s", friendly.c_str(), model.c_str(), type_str);
#endif
}

// --- Fingerbot model detection ---

bool ZigbeeBridge::is_fingerbot_(const std::string &model) {
  return (model.find("TS0001") != std::string::npos);
}

bool ZigbeeBridge::is_vibration_(const std::string &model) {
  return (model.find("lumi.vibration") != std::string::npos);
}

// --- Tuya DP report handler (C5 → S3) ---

void ZigbeeBridge::handle_tuya_dp_(const std::string &line) {
  std::string ieee = this->json_get_string_(line, "ieee");
  if (ieee.empty()) return;

  int dp = this->json_get_int_(line, "dp");
  int value = this->json_get_int_(line, "value");
  std::string str_value = this->json_get_string_(line, "str_value");

  auto it = this->devices_.find(ieee);
  if (it == this->devices_.end()) {
    ESP_LOGW(TAG, "Tuya DP from unknown device %s", ieee.c_str());
    return;
  }
  auto &dev = it->second;

  ESP_LOGI(TAG, "Tuya DP: %s dp=%d value=%d", ieee.c_str(), dp, value);

  // Map DPs to device state (_TZ3210 fingerbot variant)
  switch (dp) {
    case 1:   // switch (push trigger)
      dev.state = value ? "ON" : "OFF";
      break;
    case 101: // mode (enum: 0=push, 1=switch, 2=program)
      if (value == 0) dev.mode = "push";
      else if (value == 1) dev.mode = "switch";
      else if (value == 2) dev.mode = "program";
      break;
    case 102: // down_movement (51-100)
      dev.down_movement = value;
      break;
    case 103: // sustain_time (0-255 seconds)
      dev.sustain_time = value;
      break;
    case 104: // reverse (enum 0-1)
      dev.reverse = (value != 0);
      break;
    case 105: // battery (0-100)
      dev.battery = value;
      break;
    case 106: // up_movement (0-49)
      dev.up_movement = value;
      break;
    case 107: // touch_control (bool)
      dev.touch_control = (value != 0);
      break;
    case 109: // program (raw/string)
      if (!str_value.empty()) dev.program = str_value;
      break;
    case 117: // repeat_forever (bool)
      dev.repeat_forever = (value != 0);
      break;
    case 121: // program_enable (bool)
      dev.program_enable = (value != 0);
      break;
    default:
      ESP_LOGD(TAG, "Unknown Tuya DP %d from %s", dp, ieee.c_str());
      break;
  }

  // Republish aggregated state
  this->mqtt_publish_device_state_(ieee);
}

// --- Send Tuya DP set command to C5 ---

void ZigbeeBridge::send_tuya_set_(const std::string &ieee, int dp, const std::string &dp_type, int value) {
  uint32_t id = ++this->cmd_id_;
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"cmd\":\"tuya_set\",\"ieee\":\"%s\",\"dp\":%d,\"dp_type\":\"%s\",\"value\":%d,\"id\":%lu}",
           ieee.c_str(), dp, dp_type.c_str(), value, (unsigned long) id);
  this->write_str(buf);
  this->write_byte('\n');
  ESP_LOGI(TAG, "TX: tuya_set ieee=%s dp=%d type=%s val=%d", ieee.c_str(), dp, dp_type.c_str(), value);
}

void ZigbeeBridge::send_tuya_set_str_(const std::string &ieee, int dp, const std::string &value) {
  uint32_t id = ++this->cmd_id_;
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"cmd\":\"tuya_set\",\"ieee\":\"%s\",\"dp\":%d,\"dp_type\":\"string\",\"str_value\":\"%s\",\"id\":%lu}",
           ieee.c_str(), dp, value.c_str(), (unsigned long) id);
  this->write_str(buf);
  this->write_byte('\n');
  ESP_LOGI(TAG, "TX: tuya_set ieee=%s dp=%d type=string val=%s", ieee.c_str(), dp, value.c_str());
}

// --- Unified MQTT command handler for a device ---

void ZigbeeBridge::handle_device_command_(const std::string &ieee, const std::string &payload) {
  auto it = this->devices_.find(ieee);
  if (it == this->devices_.end()) return;
  auto &dev = it->second;

  // State ON/OFF (main switch — uses ZCL on/off cluster 0x0006)
  std::string state_val = this->json_get_string_(payload, "state");
  if (!state_val.empty()) {
    int value = (state_val == "ON") ? 1 : 0;
    char cmd_buf[128];
    snprintf(cmd_buf, sizeof(cmd_buf),
             "{\"cmd\":\"set\",\"ieee\":\"%s\",\"ep\":1,\"cluster\":6,\"attr\":0,\"value\":%d}",
             ieee.c_str(), value);
    this->send_raw(std::string(cmd_buf));
    dev.state = state_val;
    this->mqtt_publish_device_state_(ieee);
    return;  // state command handled
  }

  // Push button (triggers on/off cycle)
  if (payload.find("\"push\"") != std::string::npos) {
    char cmd_buf[128];
    snprintf(cmd_buf, sizeof(cmd_buf),
             "{\"cmd\":\"set\",\"ieee\":\"%s\",\"ep\":1,\"cluster\":6,\"attr\":0,\"value\":1}",
             ieee.c_str());
    this->send_raw(std::string(cmd_buf));
    return;
  }

  // Run program
  if (payload.find("\"run_program\"") != std::string::npos) {
    // Enable program and trigger
    this->send_tuya_set_(ieee, 121, "bool", 1);  // program_enable
    this->send_tuya_set_(ieee, 1, "bool", 1);    // trigger
    return;
  }

  // Mode select (push/switch/program)
  std::string mode_val = this->json_get_string_(payload, "mode");
  if (!mode_val.empty()) {
    int mode_int = 0;
    if (mode_val == "switch") mode_int = 1;
    else if (mode_val == "program") mode_int = 2;
    this->send_tuya_set_(ieee, 101, "enum", mode_int);
    dev.mode = mode_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Reverse
  std::string reverse_val = this->json_get_string_(payload, "reverse");
  if (!reverse_val.empty()) {
    bool on = (reverse_val == "ON");
    this->send_tuya_set_(ieee, 104, "enum", on ? 1 : 0);
    dev.reverse = on;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Touch control
  std::string touch_val = this->json_get_string_(payload, "touch_control");
  if (!touch_val.empty()) {
    bool on = (touch_val == "ON");
    this->send_tuya_set_(ieee, 107, "bool", on ? 1 : 0);
    dev.touch_control = on;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Program enable
  std::string pe_val = this->json_get_string_(payload, "program_enable");
  if (!pe_val.empty()) {
    bool on = (pe_val == "ON");
    this->send_tuya_set_(ieee, 121, "bool", on ? 1 : 0);
    dev.program_enable = on;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Repeat forever
  std::string rf_val = this->json_get_string_(payload, "repeat_forever");
  if (!rf_val.empty()) {
    bool on = (rf_val == "ON");
    this->send_tuya_set_(ieee, 117, "bool", on ? 1 : 0);
    dev.repeat_forever = on;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Down movement (number)
  int dm_val = this->json_get_int_(payload, "down_movement", -1);
  if (dm_val >= 51 && dm_val <= 100) {
    this->send_tuya_set_(ieee, 102, "value", dm_val);
    dev.down_movement = dm_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Up movement (number)
  int um_val = this->json_get_int_(payload, "up_movement", -1);
  if (um_val >= 0 && um_val <= 49) {
    this->send_tuya_set_(ieee, 106, "value", um_val);
    dev.up_movement = um_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Sustain time (number)
  int st_val = this->json_get_int_(payload, "sustain_time", -1);
  if (st_val >= 0 && st_val <= 255) {
    this->send_tuya_set_(ieee, 103, "value", st_val);
    dev.sustain_time = st_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Program (text)
  std::string prog_val = this->json_get_string_(payload, "program");
  if (!prog_val.empty()) {
    this->send_tuya_set_str_(ieee, 109, prog_val);
    dev.program = prog_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  // Sensitivity (Aqara vibration sensor)
  std::string sens_val = this->json_get_string_(payload, "sensitivity");
  if (!sens_val.empty()) {
    // Forward sensitivity command as a generic ZCL command to C5
    // C5 driver will handle the manufacturer-specific write
    uint32_t id = ++this->cmd_id_;
    char cmd_buf[192];
    snprintf(cmd_buf, sizeof(cmd_buf),
             "{\"cmd\":\"device_cmd\",\"ieee\":\"%s\",\"payload\":\"{\\\"sensitivity\\\": \\\"%s\\\"}\",\"id\":%lu}",
             ieee.c_str(), sens_val.c_str(), (unsigned long) id);
    this->write_str(cmd_buf);
    this->write_byte('\n');
    dev.sensitivity = sens_val;
    this->mqtt_publish_device_state_(ieee);
    return;
  }

  ESP_LOGW(TAG, "Unhandled command payload: %s", payload.c_str());
}

// --- Simple JSON helpers (no dependency on ArduinoJson) ---

std::string ZigbeeBridge::json_get_string_(const std::string &json, const std::string &key) {
  std::string search = "\"" + key + "\":\"";
  size_t pos = json.find(search);
  if (pos == std::string::npos)
    return "";
  pos += search.size();
  size_t end = json.find('"', pos);
  if (end == std::string::npos)
    return "";
  return json.substr(pos, end - pos);
}

int ZigbeeBridge::json_get_int_(const std::string &json, const std::string &key, int default_val) {
  // Try "key":123 pattern
  std::string search = "\"" + key + "\":";
  size_t pos = json.find(search);
  if (pos == std::string::npos)
    return default_val;
  pos += search.size();
  // Skip whitespace
  while (pos < json.size() && json[pos] == ' ')
    pos++;
  if (pos >= json.size())
    return default_val;
  // Handle hex "0x1A62" or plain number
  if (pos + 1 < json.size() && json[pos] == '"') {
    // String value like "0x1A62"
    pos++;
    std::string val;
    while (pos < json.size() && json[pos] != '"')
      val += json[pos++];
    if (val.size() > 2 && val[0] == '0' && (val[1] == 'x' || val[1] == 'X'))
      return (int) strtol(val.c_str(), nullptr, 16);
    return atoi(val.c_str());
  }
  // Numeric value
  return atoi(json.c_str() + pos);
}

double ZigbeeBridge::json_get_double_(const std::string &json, const std::string &key, double default_val) {
  std::string search = "\"" + key + "\":";
  size_t pos = json.find(search);
  if (pos == std::string::npos)
    return default_val;
  pos += search.size();
  while (pos < json.size() && json[pos] == ' ')
    pos++;
  if (pos >= json.size())
    return default_val;
  return atof(json.c_str() + pos);
}

// ============================================================================
// OTA Implementation
// ============================================================================

void ZigbeeBridge::start_ota_from_buffer(const uint8_t *data, size_t size) {
  if (this->ota_state_ != OTA_IDLE) {
    ESP_LOGW(TAG, "OTA already in progress, aborting previous");
    this->ota_abort_("New OTA started");
  }

  if (size == 0 || data == nullptr) {
    ESP_LOGE(TAG, "Invalid OTA data");
    return;
  }

  ESP_LOGI(TAG, "Starting OTA update: %zu bytes", size);

  // Store firmware in buffer
  this->ota_buffer_.assign(data, data + size);
  this->ota_total_size_ = size;
  this->ota_sent_size_ = 0;
  this->ota_chunk_seq_ = 0;

  // Calculate MD5
  this->ota_md5_ = this->md5_hash_(data, size);
  ESP_LOGI(TAG, "Firmware MD5: %s", this->ota_md5_.c_str());

  this->ota_set_state_(OTA_STARTING);
  this->ota_send_start_();
}

void ZigbeeBridge::start_ota_from_url(const std::string &url) {
  if (this->ota_state_ != OTA_IDLE) {
    ESP_LOGW(TAG, "OTA already in progress");
    return;
  }

  ESP_LOGI(TAG, "Downloading firmware from: %s", url.c_str());

  // Use ESP-IDF HTTP client
  esp_http_client_config_t config = {};
  config.url = url.c_str();
  config.timeout_ms = 60000;
  config.buffer_size = 4096;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGE(TAG, "Failed to init HTTP client");
    return;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    return;
  }

  int content_length = esp_http_client_fetch_headers(client);
  if (content_length <= 0) {
    ESP_LOGE(TAG, "Invalid content length: %d", content_length);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return;
  }

  ESP_LOGI(TAG, "Firmware size: %d bytes", content_length);

  // Allocate buffer for firmware
  std::vector<uint8_t> buffer(content_length);
  int total_read = 0;

  while (total_read < content_length) {
    int read_len = esp_http_client_read(client,
                                        reinterpret_cast<char*>(buffer.data() + total_read),
                                        content_length - total_read);
    if (read_len <= 0) {
      ESP_LOGE(TAG, "Error reading firmware at %d/%d", total_read, content_length);
      break;
    }
    total_read += read_len;
    ESP_LOGD(TAG, "Downloaded %d/%d bytes", total_read, content_length);
  }

  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (total_read != content_length) {
    ESP_LOGE(TAG, "Incomplete download: %d/%d bytes", total_read, content_length);
    return;
  }

  ESP_LOGI(TAG, "Download complete, starting OTA transfer");
  this->start_ota_from_buffer(buffer.data(), buffer.size());
}

void ZigbeeBridge::abort_ota() {
  if (this->ota_state_ == OTA_IDLE) {
    return;
  }
  this->ota_abort_("User aborted");
}

float ZigbeeBridge::get_ota_progress() const {
  if (this->ota_total_size_ == 0) return 0.0f;
  return (float)this->ota_sent_size_ / (float)this->ota_total_size_ * 100.0f;
}

void ZigbeeBridge::ota_send_start_() {
  uint32_t id = ++this->cmd_id_;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"cmd\":\"ota_start\",\"size\":%zu,\"md5\":\"%s\",\"id\":%u}",
           this->ota_total_size_, this->ota_md5_.c_str(), id);
  this->write_str(buf);
  this->write('\n');
  this->ota_last_activity_ = millis();
  ESP_LOGI(TAG, "Sent ota_start command");
}

void ZigbeeBridge::ota_send_next_chunk_() {
  if (this->ota_state_ != OTA_SENDING) return;

  size_t remaining = this->ota_total_size_ - this->ota_sent_size_;
  size_t chunk_len = std::min(remaining, (size_t)this->ota_chunk_size_);

  // Base64 encode the chunk
  std::string b64 = this->base64_encode_(
      &this->ota_buffer_[this->ota_sent_size_], chunk_len);

  // Build and send command
  uint32_t id = ++this->cmd_id_;
  std::string cmd = "{\"cmd\":\"ota_data\",\"seq\":" +
                    std::to_string(this->ota_chunk_seq_) +
                    ",\"data\":\"" + b64 +
                    "\",\"id\":" + std::to_string(id) + "}";

  this->write_str(cmd.c_str());
  this->write('\n');

  this->ota_set_state_(OTA_WAITING_ACK);
  this->ota_last_activity_ = millis();

  ESP_LOGD(TAG, "Sent chunk %u (%zu bytes, b64 len %zu)",
           this->ota_chunk_seq_, chunk_len, b64.size());
}

void ZigbeeBridge::ota_send_end_() {
  uint32_t id = ++this->cmd_id_;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cmd\":\"ota_end\",\"id\":%u}", id);
  this->write_str(buf);
  this->write('\n');
  this->ota_set_state_(OTA_FINALIZING);
  this->ota_last_activity_ = millis();
  ESP_LOGI(TAG, "Sent ota_end command");
}

void ZigbeeBridge::ota_handle_response_(const std::string &type, const std::string &json) {
  this->ota_last_activity_ = millis();

  if (type == "ota_ready") {
    if (this->ota_state_ != OTA_STARTING) {
      ESP_LOGW(TAG, "Unexpected ota_ready in state %d", this->ota_state_);
      return;
    }
    this->ota_chunk_size_ = this->json_get_int_(json, "chunk_size", 4096);
    std::string partition = this->json_get_string_(json, "partition");
    ESP_LOGI(TAG, "Target ready for OTA: chunk_size=%u, partition=%s",
             this->ota_chunk_size_, partition.c_str());

    this->ota_set_state_(OTA_SENDING);
    this->ota_set_status_("sending");
    this->ota_send_next_chunk_();
  }
  else if (type == "ota_ack") {
    if (this->ota_state_ != OTA_WAITING_ACK) {
      ESP_LOGW(TAG, "Unexpected ota_ack in state %d", this->ota_state_);
      return;
    }

    uint32_t seq = this->json_get_int_(json, "seq", 0xFFFFFFFF);
    if (seq != this->ota_chunk_seq_) {
      ESP_LOGW(TAG, "Sequence mismatch: got %u, expected %u", seq, this->ota_chunk_seq_);
      // Could retry here, for now abort
      this->ota_abort_("Sequence mismatch");
      return;
    }

    // Advance counters
    size_t chunk_len = std::min(this->ota_total_size_ - this->ota_sent_size_,
                                (size_t)this->ota_chunk_size_);
    this->ota_sent_size_ += chunk_len;
    this->ota_chunk_seq_++;

    // Update progress
    this->ota_update_progress_();

    int progress = this->json_get_int_(json, "progress", -1);
    ESP_LOGI(TAG, "OTA progress: %d%% (%zu/%zu bytes)",
             progress, this->ota_sent_size_, this->ota_total_size_);

    // Check if all chunks sent
    if (this->ota_sent_size_ >= this->ota_total_size_) {
      this->ota_send_end_();
    } else {
      this->ota_set_state_(OTA_SENDING);
      this->ota_send_next_chunk_();
    }
  }
  else if (type == "ota_complete") {
    bool md5_ok = this->json_get_int_(json, "md5_ok", 0) != 0;
    int reboot_in = this->json_get_int_(json, "rebooting_in", 3);

    if (md5_ok) {
      ESP_LOGI(TAG, "OTA complete! Target rebooting in %d seconds", reboot_in);
      this->ota_complete_();
    } else {
      this->ota_abort_("MD5 verification failed on target");
    }
  }
  else if (type == "ota_error") {
    std::string error = this->json_get_string_(json, "error");
    std::string message = this->json_get_string_(json, "message");
    this->ota_abort_(error + ": " + message);
  }
}

void ZigbeeBridge::ota_set_state_(OtaState state) {
  this->ota_state_ = state;
}

void ZigbeeBridge::ota_set_status_(const std::string &status) {
  if (this->ota_status_ != nullptr) {
    this->ota_status_->publish_state(status);
  }
}

void ZigbeeBridge::ota_update_progress_() {
  if (this->ota_progress_ != nullptr) {
    float progress = this->get_ota_progress();
    this->ota_progress_->publish_state(progress);
  }
}

void ZigbeeBridge::ota_abort_(const std::string &reason) {
  ESP_LOGE(TAG, "OTA aborted: %s", reason.c_str());

  // Send abort command to target (in case it's waiting)
  if (this->ota_state_ != OTA_IDLE && this->ota_state_ != OTA_ERROR) {
    uint32_t id = ++this->cmd_id_;
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"cmd\":\"ota_abort\",\"id\":%u}", id);
    this->write_str(buf);
    this->write('\n');
  }

  this->ota_set_state_(OTA_ERROR);
  this->ota_set_status_("error: " + reason);

  // Clear buffer
  this->ota_buffer_.clear();

  // Reset to idle after a moment
  this->ota_set_state_(OTA_IDLE);
}

void ZigbeeBridge::ota_complete_() {
  this->ota_set_state_(OTA_COMPLETE);
  this->ota_set_status_("complete - rebooting");
  this->ota_update_progress_();

  // Clear buffer
  this->ota_buffer_.clear();

  // Target will reboot, expect ready message again
  this->c5_ready_ = false;

  // Reset to idle
  this->ota_set_state_(OTA_IDLE);
  this->ota_set_status_("idle");
}

void ZigbeeBridge::ota_check_timeout_() {
  if (this->ota_state_ == OTA_IDLE || this->ota_state_ == OTA_COMPLETE ||
      this->ota_state_ == OTA_ERROR) {
    return;
  }

  uint32_t now = millis();
  if (now - this->ota_last_activity_ > this->ota_timeout_ms_) {
    this->ota_abort_("Timeout waiting for response");
  }
}

// ============================================================================
// Hardware Recovery Implementation
// ============================================================================

void ZigbeeBridge::trigger_reset() {
  if (this->reset_pin_ == nullptr) {
    ESP_LOGW(TAG, "Reset pin not configured");
    return;
  }
  ESP_LOGI(TAG, "Triggering target reset...");
  this->pulse_reset_();

  // Target will reboot
  this->c5_ready_ = false;
}

void ZigbeeBridge::trigger_recovery_mode() {
  if (!this->has_recovery_pins()) {
    ESP_LOGW(TAG, "Recovery mode requires both reset_pin and boot_pin configured");
    return;
  }

  ESP_LOGI(TAG, "Entering bootloader/recovery mode...");
  this->enter_bootloader_();
}

void ZigbeeBridge::pulse_reset_() {
  if (this->reset_pin_ == nullptr) return;
  this->reset_pin_->digital_write(false);
  delay(50);
  this->reset_pin_->digital_write(true);
}

void ZigbeeBridge::enter_bootloader_() {
  if (this->boot_pin_ == nullptr || this->reset_pin_ == nullptr) return;

  // Set BOOT pin low (strapping for bootloader mode)
  this->boot_pin_->digital_write(false);
  delay(10);

  // Pulse reset
  this->pulse_reset_();

  delay(100);

  // C5/H2 is now in ROM bootloader mode
  // BOOT pin stays low until explicit exit_bootloader_()
  ESP_LOGI(TAG, "Target is now in ROM bootloader mode");
}

void ZigbeeBridge::exit_bootloader_() {
  if (this->boot_pin_ != nullptr) {
    this->boot_pin_->digital_write(true);  // Normal boot mode
  }
  delay(10);
  this->pulse_reset_();
  ESP_LOGI(TAG, "Exited bootloader mode, target rebooting normally");
}

// ============================================================================
// Helper Functions
// ============================================================================

std::string ZigbeeBridge::base64_encode_(const uint8_t *data, size_t len) {
  static const char b64_chars[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string result;
  result.reserve(((len + 2) / 3) * 4);

  for (size_t i = 0; i < len; i += 3) {
    uint32_t n = ((uint32_t)data[i]) << 16;
    if (i + 1 < len) n |= ((uint32_t)data[i + 1]) << 8;
    if (i + 2 < len) n |= data[i + 2];

    result += b64_chars[(n >> 18) & 0x3F];
    result += b64_chars[(n >> 12) & 0x3F];
    result += (i + 1 < len) ? b64_chars[(n >> 6) & 0x3F] : '=';
    result += (i + 2 < len) ? b64_chars[n & 0x3F] : '=';
  }

  return result;
}

std::string ZigbeeBridge::md5_hash_(const uint8_t *data, size_t len) {
  md5::MD5Digest digest;
  digest.init();
  digest.add(data, len);
  digest.calculate();
  char hex[33];
  digest.get_hex(hex);
  return std::string(hex);
}

}  // namespace zigbee_bridge
}  // namespace esphome
