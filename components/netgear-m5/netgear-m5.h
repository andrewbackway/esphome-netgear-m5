#pragma once

#include <map>
#include <string>
#include <vector>

#include <ArduinoJson.h>
#include "esp_http_client.h"
#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}

namespace esphome {
namespace netgear_m5 {

class NetgearM5Component : public Component {
 public:
  void set_host(const std::string &host) { host_ = host; }
  void set_password(const std::string &password) { password_ = password; }
  void set_poll_interval(uint32_t ms) { poll_interval_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  void bind_numeric_sensor(const std::string &json_path, sensor::Sensor *s);
  void bind_text_sensor(const std::string &json_path,
                        text_sensor::TextSensor *s);
  void bind_binary_sensor(const std::string &json_path,
                          binary_sensor::BinarySensor *s,
                          const std::string &on_value,
                          const std::string &off_value);

  // Access cookie
  const std::string &cookie() const { return cookie_; }

 protected:
  // Signal strength calculation helpers
  int clamp01(int v, int lo, int hi);
  int bars_from_rsrp(float rsrp_dbm);
  int quality_adjust_from_rsrq(float rsrq_db);
  int quality_adjust_from_sinr(float sinr_db);
  int bars_from_rssi(float rssi_dbm);
  int calc_mobile_bars(bool has_rsrp, float rsrp_dbm, bool has_rsrq,
                       float rsrq_db, bool has_sinr, float sinr_db,
                       bool has_rssi, float rssi_dbm);

  // Dynamic buffer for JSON responses - allocated only during fetch
  // This avoids permanently consuming 32KB of RAM
  static constexpr size_t STREAM_BUF_SIZE = 32 * 1024;  // 32KB for full JSON
  char* stream_buf_{nullptr};  // Dynamically allocated during fetch
  size_t stream_len_{0};

  // JSON filter document - built once at setup, used for all parses
  // This filters the ~28KB JSON down to only the fields we need
  JsonDocument json_filter_;

  int last_status_code_ = 0;
  std::map<std::string, std::string> last_headers_;
  bool logged_in_ = false;

  // Task management
  static void task_trampoline_(void *param);
  void task_loop_();

  // JSON filter building
  void build_json_filter_();
  void add_path_to_filter_(const std::string &path);

  // Memory-efficient fetch and parse with filtering
  bool fetch_and_parse_();

  // Helper to extract signal strength values from state map
  float extract_signal_value_(const std::string &key);

  // JSON path lookup helper
  static std::string dotted_lookup_(
      const std::string &path, const ::ArduinoJson::JsonVariantConst &root);

  // Sensor value publishing
  void publish_pending_();

  // HTTP request for small responses (login, cookies)
  esp_err_t _request(const std::string &url, esp_http_client_method_t method,
                     const std::string &body, const std::string &content_type,
                     std::string &response);

  // Memory-efficient streaming JSON request with filtering
  esp_err_t stream_json_request_(const std::string &url,
                                  esp_http_client_method_t method,
                                  const std::string &body,
                                  const std::string &content_type,
                                  JsonDocument &filter);

  // Event handlers for HTTP client
  static esp_err_t _event_handler(esp_http_client_event_t *evt);
  static esp_err_t _stream_event_handler(esp_http_client_event_t *evt);

  std::string sec_token_;
  std::string host_;
  std::string password_;
  std::string cookie_;
  uint32_t poll_interval_ms_{30000};

  TaskHandle_t task_handle_{nullptr};
  volatile bool has_new_state_{false};
  std::map<std::string, std::string> state_;  // Stores parsed JSON values
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  struct NumBinding {
    std::string path;
    sensor::Sensor *sensor{nullptr};
  };
  struct TextBinding {
    std::string path;
    text_sensor::TextSensor *sensor{nullptr};
  };
  struct BinBinding {
    std::string path;
    binary_sensor::BinarySensor *sensor{nullptr};
    std::string on_value{"true"};
    std::string off_value{"false"};
  };

  std::vector<NumBinding> num_bindings_;
  std::vector<TextBinding> text_bindings_;
  std::vector<BinBinding> bin_bindings_;

  // Context for small HTTP requests (login, cookies)
  struct RequestContext {
    NetgearM5Component *instance;
    std::string *response;
  };

  // Context for streaming JSON requests with filtering
  struct StreamContext {
    NetgearM5Component *instance;
    JsonDocument *filter;
    bool parse_complete;
    bool parse_error;
  };
};

}  // namespace netgear_m5
}  // namespace esphome