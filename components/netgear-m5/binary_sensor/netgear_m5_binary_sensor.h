#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "../netgear-m5.h"

namespace esphome {
namespace netgear_m5 {

class NetgearM5BinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
  void set_parent(NetgearM5Component *parent) { parent_ = parent; }
  void set_path(const std::string &path) { path_ = path; }
  void set_on_value(const std::string &on_value) { on_value_ = on_value; }
  void set_off_value(const std::string &off_value) { off_value_ = off_value; }
  
  void setup() override {
    parent_->register_binary_sensor(this);
  }
  
  void dump_config() override;
  
  const std::string &get_path() const { return path_; }
  const std::string &get_on_value() const { return on_value_; }
  const std::string &get_off_value() const { return off_value_; }

 protected:
  NetgearM5Component *parent_{nullptr};
  std::string path_;
  std::string on_value_{"true"};
  std::string off_value_{"false"};
};

}  // namespace netgear_m5
}  // namespace esphome
