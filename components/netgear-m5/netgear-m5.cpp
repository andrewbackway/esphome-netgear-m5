#include "netgear-m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>  

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5";

void NetgearM5Component::setup() {
  ESP_LOGD(TAG, "Setting up Netgear M5 component");
  // Wait for network connectivity
  for (int i = 0; i < 30; i++) {
    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = nullptr;
    // Test connectivity with Google's DNS
    int err = getaddrinfo("8.8.8.8", "53", &hints, &res);
    if (err == 0 && res != nullptr) {
      freeaddrinfo(res);
      ESP_LOGD(TAG, "Network is connected");
      break;
    }
    ESP_LOGD(TAG, "Waiting for network connection (%d/30)...", i + 1);
    delay(1000);
  }
  xTaskCreatePinnedToCore(
      &NetgearM5Component::task_trampoline_,
      "netgear_m5_task",
      12288, // Increased stack size
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
      vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before retrying
      continue;
    }
    std::string raw;
    if (this->fetch_once_(raw)) {
      std::string body;
      if (extract_http_body_(raw, body)) {
        taskENTER_CRITICAL(&this->mux_);
        this->last_payload_ = std::move(body);
        this->has_new_payload_ = true;
        taskEXIT_CRITICAL(&this->mux_);
      }
    }
    vTaskDelay(delay_ticks);
  }
}

bool NetgearM5Component::fetch_once_(std::string &body) {
  if (this->host_.empty()) {
    ESP_LOGW(TAG, "Host is empty, cannot fetch data");
    return false;
  }

  const char *port = "80";
  const char *path = "/api/model.json?internalapi=1";

  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo *res = nullptr;
  ESP_LOGD(TAG, "Resolving host: %s", this->host_.c_str());
  int err = getaddrinfo(this->host_.c_str(), port, &hints, &res);
  if (err != 0 || res == nullptr) {
    // Fallback for gai_strerror if not available
    #ifdef gai_strerror
    ESP_LOGW(TAG, "DNS resolution failed for %s: %s", this->host_.c_str(), gai_strerror(err));
    #else
    ESP_LOGW(TAG, "DNS resolution failed for %s: error code %d", this->host_.c_str(), err);
    #endif
    if (res) freeaddrinfo(res);
    return false;
  }

  int sock = -1;
  for (struct addrinfo *p = res; p != nullptr; p = p->ai_next) {
    sock = lwip_socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (sock < 0) {
      ESP_LOGW(TAG, "Failed to create socket: %d", errno);
      continue;
    }

    struct timeval tv{.tv_sec = 5, .tv_usec = 0};
    if (lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
      ESP_LOGW(TAG, "Failed to set receive timeout: %d", errno);
      lwip_close(sock);
      sock = -1;
      continue;
    }
    if (lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
      ESP_LOGW(TAG, "Failed to set send timeout: %d", errno);
      lwip_close(sock);
      sock = -1;
      continue;
    }

    ESP_LOGD(TAG, "Connecting to %s:%s", this->host_.c_str(), port);
    if (lwip_connect(sock, p->ai_addr, p->ai_addrlen) == 0) {
      break;
    }
    ESP_LOGW(TAG, "Connection failed: %d", errno);
    lwip_close(sock);
    sock = -1;
  }
  freeaddrinfo(res);
  if (sock < 0) {
    ESP_LOGW(TAG, "Failed to connect to %s:%s", this->host_.c_str(), port);
    return false;
  }

  std::string req = "GET " + std::string(path) + " HTTP/1.1\r\n" +
                    "Host: " + this->host_ + "\r\n" +
                    "User-Agent: ESPHome-NetgearM5/1.0\r\n" +
                    "Connection: close\r\n" +
                    "Accept: application/json\r\n" +
                    "\r\n";
  ESP_LOGD(TAG, "Sending HTTP request to %s", this->host_.c_str());
  int sent = lwip_send(sock, req.data(), req.size(), 0);
  if (sent < 0) {
    ESP_LOGW(TAG, "Failed to send request: %d", errno);
    lwip_close(sock);
    return false;
  }

  char buf[1024];
  std::string rx;
  rx.reserve(4096); // Reserve space for typical response size
  for (;;) {
    int n = lwip_recv(sock, buf, sizeof(buf) - 1, 0);
    if (n < 0) {
      ESP_LOGW(TAG, "Receive failed: %d", errno);
      break;
    }
    if (n == 0) {
      break; // Connection closed
    }
    buf[n] = '\0'; // Null-terminate for safety
    rx.append(buf, n);
  }
  ESP_LOGD(TAG, "Received %u bytes from %s", rx.size(), this->host_.c_str());
  lwip_close(sock);

  body.swap(rx);
  return true;
}

bool NetgearM5Component::extract_http_body_(const std::string &raw, std::string &body_out) {
  auto pos = raw.find("\r\n\r\n");
  if (pos == std::string::npos) return false;
  body_out.assign(raw.begin() + pos + 4, raw.end());
  return true;
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