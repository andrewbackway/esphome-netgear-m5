#include "netgear_m5_text_sensor.h"
#include "../netgear-m5.h"
#include "esphome/core/log.h"

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5.text_sensor";

void NetgearM5TextSensor::dump_config() {
  LOG_TEXT_SENSOR("", "Netgear M5 Text Sensor", this);
  ESP_LOGCONFIG(TAG, "  JSON Path: %s", this->path_.c_str());
}

}  // namespace netgear_m5
}  // namespace esphome
