#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "../netgear-m5.h"

namespace esphome {
namespace netgear_m5 {

class NetgearM5Sensor : public sensor::Sensor, public Component {
 public:
  void set_parent(NetgearM5Component *parent) { parent_ = parent; }
  void set_path(const std::string &path) { path_ = path; }
  
  void setup() override {
    parent_->register_sensor(this);
  }
  
  void dump_config() override;
  
  const std::string &get_path() const { return path_; }

 protected:
  NetgearM5Component *parent_{nullptr};
  std::string path_;
};

}  // namespace netgear_m5
}  // namespace esphome
