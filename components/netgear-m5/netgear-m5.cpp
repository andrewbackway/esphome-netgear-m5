#include "netgear_m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>  

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5";

void NetgearM5Component::setup() {
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
  if (this->host_.empty()) return false;

  const char *port = "80";
  const char *path = "/api/model.json?internalapi=1";

  struct addrinfo hints = {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo *res = nullptr;
  int err = getaddrinfo(this->host_.c_str(), port, &hints, &res);
  if (err != 0 || res == nullptr) return false;

  int sock = -1;
  for (struct addrinfo *p = res; p != nullptr; p = p->ai_next) {
    sock = lwip_socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (sock < 0) continue;

    struct timeval tv{.tv_sec=5, .tv_usec=0};
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (lwip_connect(sock, p->ai_addr, p->ai_addrlen) == 0) break;
    lwip_close(sock);
    sock = -1;
  }
  freeaddrinfo(res);
  if (sock < 0) return false;

  std::string req = "GET " + std::string(path) + " HTTP/1.1\r\n" + 
                    "Host: " + this->host_ + "\r\n" + 
                    "User-Agent: ESPHome-NetgearM5/1.0\r\n" +
                    "Connection: close\r\n" + 
                    "Accept: application/json\r\n" + 
                    "\r\n";
  lwip_send(sock, req.data(), req.size(), 0);

  char buf[1024];
  std::string rx;
  for (;;) {
    int n = lwip_recv(sock, buf, sizeof(buf), 0);
    if (n <= 0) break;
    rx.append(buf, buf + n);
  }
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
  if (payload.empty()) return;

  ArduinoJson::JsonDocument doc;
  if (deserializeJson(doc, payload)) return;
  auto root = doc.as<ArduinoJson::JsonVariantConst>();

  // Push numeric bindings
  for (auto &b : this->num_bindings_) {
    if (!b.sensor) continue;
    std::string v = dotted_lookup_(b.path, root);
    if (!v.empty()) b.sensor->publish_state(strtod(v.c_str(), nullptr));
  }
  // Push text bindings
  for (auto &b : this->text_bindings_) {
    if (!b.sensor) continue;
    std::string v = dotted_lookup_(b.path, root);
    if (!v.empty()) b.sensor->publish_state(v);
  }
}

// Supports dotted paths + array indexes (e.g., "wwan.bandRegion[0].name")
std::string NetgearM5Component::dotted_lookup_(const std::string &path, const ::ArduinoJson::JsonVariantConst &root) {
  ::ArduinoJson::JsonVariantConst cur = root;
  size_t i = 0;

  while (i < path.size()) {
    size_t dot = path.find('.', i);
    std::string token = path.substr(i, dot == std::string::npos ? std::string::npos : dot - i);

    // Handle array notation: key[index]
    size_t lb = token.find('[');
    if (lb != std::string::npos && token.back() == ']') {
      std::string key = token.substr(0, lb);
      int index = atoi(token.substr(lb + 1, token.size() - lb - 2).c_str());

      if (!key.empty()) {
        if (!cur.containsKey(key)) return {};
        cur = cur[key];
      }

      if (!cur.is<JsonArrayConst>()) return {};
      auto arr = cur.as<JsonArrayConst>();
      if (index < 0 || index >= arr.size()) return {};
      cur = arr[index];
    } else {
      if (!cur.containsKey(token)) return {};
      cur = cur[token];
    }

    if (dot == std::string::npos) break;
    i = dot + 1;
  }

  if (cur.is<const char*>()) return cur.as<const char*>();
  if (cur.is<int>()) return std::to_string(cur.as<int>());
  if (cur.is<double>()) return std::to_string(cur.as<double>());
  if (cur.is<bool>()) return cur.as<bool>() ? "true" : "false";

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

}  // namespace netgear_m5
}  // namespace esphome
