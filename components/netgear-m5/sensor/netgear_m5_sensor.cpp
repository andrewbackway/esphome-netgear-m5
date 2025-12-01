#include "netgear_m5_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5.sensor";

void NetgearM5Sensor::dump_config() {
  LOG_SENSOR("", "Netgear M5 Sensor", this);
  ESP_LOGCONFIG(TAG, "  JSON Path: %s", this->path_.c_str());
}

}  // namespace netgear_m5
}  // namespace esphome
