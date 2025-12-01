#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace netgear_m5 {

// Forward declaration
class NetgearM5Component;

class NetgearM5TextSensor : public text_sensor::TextSensor, public Component {
 public:
  void set_parent(NetgearM5Component *parent) { parent_ = parent; }
  void set_path(const std::string &path) { path_ = path; }
  
  void setup() override;
  
  void dump_config() override;
  
  const std::string &get_path() const { return path_; }

 protected:
  NetgearM5Component *parent_{nullptr};
  std::string path_;
};

}  // namespace netgear_m5
}  // namespace esphome
