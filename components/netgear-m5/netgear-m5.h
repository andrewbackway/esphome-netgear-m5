#pragma once

#include "esphome.h"
#include "esphome/core/log.h"

#include <string>
#include <vector>

extern "C" {
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <lwip/sockets.h>
  #include <lwip/netdb.h>
}

namespace esphome {
namespace netgear_m5 {

class NetgearM5Component : public Component {
 public:
  void set_host(const std::string &host) { host_ = host; }
  void set_poll_interval(uint32_t ms) { poll_interval_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void bind_numeric_sensor(const std::string &json_path, sensor::Sensor *s);
  void bind_text_sensor(const std::string &json_path, text_sensor::TextSensor *s);

 protected:
  static void task_trampoline_(void *param);
  void task_loop_();
  bool fetch_once_(std::string &body);
  static bool extract_http_body_(const std::string &raw, std::string &body_out);
  static std::string dotted_lookup_(const std::string &path, const ::ArduinoJson::JsonVariantConst &root);
  void publish_pending_();

  std::string host_;
  uint32_t poll_interval_ms_{30000};

  TaskHandle_t task_handle_{nullptr};
  volatile bool has_new_payload_{false};
  std::string last_payload_;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  struct NumBinding { std::string path; sensor::Sensor *sensor{nullptr}; };
  struct TextBinding { std::string path; text_sensor::TextSensor *sensor{nullptr}; };
  std::vector<NumBinding> num_bindings_;
  std::vector<TextBinding> text_bindings_;
};

}  // namespace netgear_m5
}  // namespace esphome
