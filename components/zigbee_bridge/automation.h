#pragma once

#include "zigbee_bridge.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace zigbee_bridge {

template<typename... Ts>
class FactoryResetAction : public Action<Ts...> {
 public:
  explicit FactoryResetAction(ZigbeeBridge *parent) : parent_(parent) {}

  void play(Ts... x) override { this->parent_->send_factory_reset(); }

 protected:
  ZigbeeBridge *parent_;
};

template<typename... Ts>
class RebootAction : public Action<Ts...> {
 public:
  explicit RebootAction(ZigbeeBridge *parent) : parent_(parent) {}

  void play(Ts... x) override { this->parent_->send_reboot(); }

 protected:
  ZigbeeBridge *parent_;
};

template<typename... Ts>
class PermitJoinAction : public Action<Ts...> {
 public:
  explicit PermitJoinAction(ZigbeeBridge *parent) : parent_(parent) {}

  void set_duration(uint16_t duration) { this->duration_ = duration; }
  void set_duration(std::function<uint16_t(Ts...)> func) { this->duration_func_ = func; }

  void play(Ts... x) override {
    uint16_t duration = this->duration_;
    if (this->duration_func_.has_value()) {
      duration = (*this->duration_func_)(x...);
    }
    this->parent_->send_permit_join(duration);
  }

 protected:
  ZigbeeBridge *parent_;
  uint16_t duration_{180};
  optional<std::function<uint16_t(Ts...)>> duration_func_{};
};

}  // namespace zigbee_bridge
}  // namespace esphome
