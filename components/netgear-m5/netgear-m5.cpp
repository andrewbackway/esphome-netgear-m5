#include "netgear-m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>
#include <cstdlib>

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5";

void NetgearM5Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Netgear M5 component...");
  xTaskCreate(task_trampoline_, "netgear_m5_task", 8192, this, 1, &this->task_handle_);
}

void NetgearM5Component::loop() {
  // No periodic tasks in main loop; handled by task_loop_
}

void NetgearM5Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Netgear M5 Component:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Poll Interval: %u ms", this->poll_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Numeric Sensors: %u", this->num_bindings_.size());
  ESP_LOGCONFIG(TAG, "  Text Sensors: %u", this->text_bindings_.size());
  ESP_LOGCONFIG(TAG, "  Binary Sensors: %u", this->bin_bindings_.size());
}

void NetgearM5Component::task_trampoline_(void *param) {
  NetgearM5Component *component = static_cast<NetgearM5Component *>(param);
  component->task_loop_();
  vTaskDelete(nullptr);
}

bool NetgearM5Component::fetch_once_(std::string &body) {
  // Placeholder: Implement HTTP request to Netgear M5
  // Example: Use lwip/sockets to fetch data from host_
  ESP_LOGD(TAG, "Fetching data from %s", this->host_.c_str());
  // Replace with actual HTTP client logic
  // For compilation, return false (indicating failure)
  return false;
}

bool NetgearM5Component::extract_http_body_(const std::string &raw, std::string &body_out) {
  // Placeholder: Extract JSON body from HTTP response
  ESP_LOGD(TAG, "Extracting HTTP body, raw size: %u bytes", raw.size());
  body_out = raw; // Simplified for compilation
  return true;
}

void NetgearM5Component::task_loop_() {
  while (true) {
    std::string body;
    if (fetch_once_(body)) {
      std::string json_body;
      if (extract_http_body_(body, json_body)) {
        taskENTER_CRITICAL(&this->mux_);
        this->last_payload_ = json_body;
        this->has_new_payload_ = true;
        taskEXIT_CRITICAL(&this->mux_);
        ESP_LOGD(TAG, "Fetched payload, size: %u bytes", json_body.size());
      } else {
        ESP_LOGW(TAG, "Failed to extract HTTP body");
      }
    } else {
      ESP_LOGW(TAG, "Failed to fetch data from %s", this->host_.c_str());
    }
    vTaskDelay(this->poll_interval_ms_ / portTICK_PERIOD_MS);
  }
}

void NetgearM5Component::publish_pending_() {
  std::string payload;
  taskENTER_CRITICAL(&this->mux_);
  payload.swap(this->last_payload_);
  this->has_new_payload_ = false;
  taskEXIT_CRITICAL(&this->mux_);
  if (payload.empty()) {
    ESP_LOGD(TAG, "No payload to process");
    return;
  }

  ESP_LOGD(TAG, "Free heap before parsing: %u bytes", ESP.getFreeHeap());
  ArduinoJson::JsonDocument doc;
  ArduinoJson::DeserializationError err = ArduinoJson::deserializeJson(doc, payload);
  if (err) {
    ESP_LOGW(TAG, "JSON parse failed in publish_pending_: %s (payload size: %u bytes)", err.c_str(), payload.size());
    return;
  }
  ESP_LOGD(TAG, "Free heap after parsing: %u bytes", ESP.getFreeHeap());
  auto root = doc.as<ArduinoJson::JsonVariantConst>();

  // Numeric sensors
  for (auto &b : this->num_bindings_) {
    if (!b.sensor)
      continue;
    std::string v = dotted_lookup_(b.path, root);
    ESP_LOGD(TAG, "Numeric sensor path %s: value %s", b.path.c_str(), v.c_str());
    if (!v.empty()) {
      char *endptr;
      double value = strtod(v.c_str(), &endptr);
      if (endptr == v.c_str() || *endptr != '\0') {
        ESP_LOGW(TAG, "Invalid numeric value at %s: %s", b.path.c_str(), v.c_str());
        continue;
      }
      b.sensor->publish_state(value);
    }
  }

  // Text sensors
  for (auto &b : this->text_bindings_) {
    if (!b.sensor)
      continue;
    std::string v = dotted_lookup_(b.path, root);
    ESP_LOGD(TAG, "Text sensor path %s: value %s", b.path.c_str(), v.c_str());
    if (!v.empty())
      b.sensor->publish_state(v);
  }

  // Binary sensors
  for (auto &b : this->bin_bindings_) {
    if (!b.sensor)
      continue;
    std::string v = dotted_lookup_(b.path, root);
    ESP_LOGD(TAG, "Binary sensor path %s: value %s (on: %s, off: %s)", b.path.c_str(), v.c_str(), b.on_value.c_str(), b.off_value.c_str());
    if (v == b.on_value) {
      b.sensor->publish_state(true);
    } else if (v == b.off_value) {
      b.sensor->publish_state(false);
    }
  }
}

std::string NetgearM5Component::dotted_lookup_(const std::string &path, const ::ArduinoJson::JsonVariantConst &root) {
  ::ArduinoJson::JsonVariantConst cur = root;
  size_t i = 0;

  while (i < path.size()) {
    size_t dot = path.find('.', i);
    std::string token = path.substr(i, dot == std::string::npos ? std::string::npos : dot - i);

    size_t lb = token.find('[');
    if (lb != std::string::npos && token.back() == ']') {
      std::string key = token.substr(0, lb);
      int index = atoi(token.substr(lb + 1, token.size() - lb - 2).c_str());

      if (!key.empty()) {
        auto next = cur[key.c_str()];
        if (next.isNull())
          return {};
        cur = next;
      }
      if (!cur.is<ArduinoJson::JsonArrayConst>())
        return {};
      auto arr = cur.as<ArduinoJson::JsonArrayConst>();
      if (index < 0 || static_cast<size_t>(index) >= arr.size())
        return {};
      cur = arr[index];
    } else {
      auto next = cur[token.c_str()];
      if (next.isNull())
        return {};
      cur = next;
    }

    if (dot == std::string::npos)
      break;
    i = dot + 1;
  }

  if (cur.is<const char *>())
    return cur.as<const char *>();
  if (cur.is<int>())
    return std::to_string(cur.as<int>());
  if (cur.is<long>())
    return std::to_string(cur.as<long>());
  if (cur.is<double>())
    return std::to_string(cur.as<double>());
  if (cur.is<bool>())
    return cur.as<bool>() ? "true" : "false";

  std::string out;
  serializeJson(cur, out);
  return out;
}

void NetgearM5Component::bind_numeric_sensor(const std::string &json_path, sensor::Sensor *s) {
  this->num_bindings_.push_back({json_path, s});
}

void NetgearM5Component::bind_text_sensor(const std::string &json_path, text_sensor::TextSensor *s) {
  this->text_bindings_.push_back({json_path, s});
}

void NetgearM5Component::bind_binary_sensor(const std::string &json_path,
                                           binary_sensor::BinarySensor *s,
                                           const std::string &on_value,
                                           const std::string &off_value) {
  this->bin_bindings_.push_back({json_path, s, on_value, off_value});
}

}  // namespace netgear_m5
}  // namespace esphome