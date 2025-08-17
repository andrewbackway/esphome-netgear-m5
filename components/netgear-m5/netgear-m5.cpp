#include "netgear-m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>  

#include "esphome/components/http_request/http_request.h"


namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5";

void NetgearM5Component::setup() {
  ESP_LOGD(TAG, "Setting up Netgear M5 component");

  xTaskCreatePinnedToCore(
      &NetgearM5Component::task_trampoline_,
      "netgear_m5_task",
      8192,
      this,
      4,
      &this->task_handle_,
      1);
}

void NetgearM5Component::loop() {
  if (this->has_new_payload_) {
    this->publish_pending_();
  }
}

void NetgearM5Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Netgear M5 Component:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", (unsigned) this->poll_interval_ms_);
}

void NetgearM5Component::task_trampoline_(void *param) {
  static_cast<NetgearM5Component *>(param)->task_loop_();
}

void NetgearM5Component::task_loop_() {
  const TickType_t delay_ticks = pdMS_TO_TICKS(this->poll_interval_ms_);
  for (;;) {
    if (!network::is_connected()) {
      ESP_LOGW(TAG, "Network not connected, skipping fetch");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    std::string url = "http://" + this->host_ + "/api/model.json?internalapi=1";

    // Use the perform method to get an HttpContainer.
    std::shared_ptr<http_request::HttpContainer> container = this->http_client_->perform(url, "GET", "", {}, {});

    // Check if the request was successful and we have a valid container.
    if (container && container->get_status_code() == 200) {
      std::string body = container->get_response_as_string();
      if (!body.empty()) {
        ESP_LOGD(TAG, "Received response body: %s", body.c_str());

        // Process the response body.
        taskENTER_CRITICAL(&this->mux_);
        this->last_payload_ = body;
        this->has_new_payload_ = true;
        taskEXIT_CRITICAL(&this->mux_);
      } else {
        ESP_LOGW(TAG, "Received empty response body");
      }
    } else {
      ESP_LOGW(TAG, "HTTP request failed, status code: %d", container ? container->get_status_code() : -1);
    }
    
    vTaskDelay(delay_ticks);
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

  ESP_LOGD(TAG, "Free heap before parsing: %u bytes", esp_get_free_heap_size());
  ArduinoJson::JsonDocument doc;
  ArduinoJson::DeserializationError err = ArduinoJson::deserializeJson(doc, payload);
  if (err) {
    ESP_LOGW(TAG, "JSON parse failed in publish_pending_: %s (payload size: %u bytes)", err.c_str(), payload.size());
    return;
  }
  ESP_LOGD(TAG, "Free heap after parsing: %u bytes", esp_get_free_heap_size());
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