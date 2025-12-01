#include "netgear_m5_binary_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5.binary_sensor";

void NetgearM5BinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "Netgear M5 Binary Sensor", this);
  ESP_LOGCONFIG(TAG, "  JSON Path: %s", this->path_.c_str());
  ESP_LOGCONFIG(TAG, "  On Value: %s", this->on_value_.c_str());
  ESP_LOGCONFIG(TAG, "  Off Value: %s", this->off_value_.c_str());
}

}  // namespace netgear_m5
}  // namespace esphome
