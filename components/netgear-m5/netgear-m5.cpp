#include "netgear-m5.h"

#include <ArduinoJson.h>

#include <cstring>
#include <string>

namespace esphome {
namespace netgear_m5 {

static const char *const TAG = "netgear_m5";

void NetgearM5Component::setup() {
  ESP_LOGD(TAG, "Setting up Netgear M5 component");

  xTaskCreatePinnedToCore(&NetgearM5Component::task_trampoline_,
                          "netgear_m5_task", 8192, this, 4, &this->task_handle_,
                          1);
}

void NetgearM5Component::loop() {
  if (this->has_new_state_) {
    this->publish_pending_();
  }
}

void NetgearM5Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Netgear M5 Component:");
  ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms",
                (unsigned)this->poll_interval_ms_);
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
    std::string body;
    if (this->fetch_once_(body)) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
        ESP_LOGW(TAG, "JSON parse failed: %s (payload size: %u bytes)",
                 err.c_str(), body.size());
        vTaskDelay(delay_ticks);
        continue;
      }
      if (!doc.is<JsonObject>()) {
        ESP_LOGW(TAG, "Parsed JSON is not an object");
        vTaskDelay(delay_ticks);
        continue;
      }

      taskENTER_CRITICAL(&this->mux_);
      this->state_.clear();
      auto root = doc.as<ArduinoJson::JsonObjectConst>();
      this->sec_token_ = dotted_lookup_("session.secToken", root);

      // Store all bound values
      for (const auto &b : this->num_bindings_) {
        this->state_[b.path] = dotted_lookup_(b.path, root);
      }
      for (const auto &b : this->text_bindings_) {
        this->state_[b.path] = dotted_lookup_(b.path, root);
      }
      for (const auto &b : this->bin_bindings_) {
        this->state_[b.path] = dotted_lookup_(b.path, root);
      }
      this->has_new_state_ = true;
      taskEXIT_CRITICAL(&this->mux_);
    }
    vTaskDelay(delay_ticks);
  }
}

bool NetgearM5Component::fetch_once_(std::string &body) {
  ESP_LOGD(TAG, "Fetching data from Netgear M5");
  if (cookies_.empty()) {
    esp_err_t first_err = this->_request(
        "http://" + this->host_ + "/sess_cd_tmp?op=%2F&oq=", HTTP_METHOD_GET,
        "", "", body);
    if (first_err != ESP_OK || cookies_.empty()) {
      ESP_LOGE(TAG, "Unable to obtain first cookie");
      return false;
    }
  }

  if (!this->logged_in_) {
    ESP_LOGD(TAG, "Extracting login token");
    this->_request("http://" + this->host_ + "/api/model.json?internalapi=1",
                   HTTP_METHOD_GET, "", "", body);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      ESP_LOGW(TAG, "JSON parse failed: %s", err.c_str());
      return false;
    }
    if (doc.is<JsonObject>()) {
      this->sec_token_ =
          dotted_lookup_("session.secToken", doc.as<JsonObjectConst>());
    }

    if (this->sec_token_.empty()) {
      ESP_LOGE(TAG, "Failed to extract session token");
      return true;
    }

    std::string login_body = "session.password=" + this->password_ +
                             "&token=" + this->sec_token_ +
                             "&ok_redirect=%2Findex.html&" +
                             "err_redirect=%2Findex.html%3Floginfailed";

    std::string login_response;
    esp_err_t login_err = this->_request(
        "http://" + this->host_ + "/Forms/config", HTTP_METHOD_POST, login_body,
        "application/x-www-form-urlencoded", login_response);

    if (login_err != ESP_OK) {
      ESP_LOGE(TAG, "Login failed");
      return false;
    }

    this->logged_in_ = true;
    ESP_LOGD(TAG, "Login OK, response size=%d", login_response.size());
  }

  return this->_request(
             "http://" + this->host_ + "/api/model.json?internalapi=1",
             HTTP_METHOD_GET, "", "", body) == ESP_OK;
}

esp_err_t NetgearM5Component::_request(const std::string &url,
                                       esp_http_client_method_t method,
                                       const std::string &body,
                                       const std::string &content_type,
                                       std::string &response) {
  esp_err_t err = ESP_FAIL;
  std::string current_url = url;

  while (true) {
    ESP_LOGD(TAG, "Preparing HTTP request: %s", current_url.c_str());

    esp_http_client_config_t config = {};
    config.url = current_url.c_str();
    config.event_handler = _event_handler;
    RequestContext ctx{this, &response};
    config.user_data = &ctx;
    config.disable_auto_redirect = true;
    config.timeout_ms = 120000;
    config.buffer_size = 4096;
    config.buffer_size_tx = 4096;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
      ESP_LOGE(TAG, "Failed to init HTTP client");
      return ESP_FAIL;
    }

    esp_http_client_set_method(client, method);

    if (!cookies_.empty()) {
      std::string cookie_header;
      for (const auto &c : cookies_) {
        if (!cookie_header.empty()) cookie_header += "; ";
        cookie_header += c;
      }
      esp_http_client_set_header(client, "Cookie", cookie_header.c_str());
    }

    if (!body.empty()) {
      esp_http_client_set_post_field(client, body.c_str(), body.size());
      if (!content_type.empty()) {
        esp_http_client_set_header(client, "Content-Type",
                                   content_type.c_str());
      }
    }

    ESP_LOGD(TAG, "Sending HTTP request: %s", current_url.c_str());
    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
      int status_code = esp_http_client_get_status_code(client);
      this->last_status_code_ = status_code;
      ESP_LOGD(TAG, "HTTP Status = %d", this->last_status_code_);

      auto setCookieValue = this->last_headers_.find("Set-Cookie");
      if (setCookieValue != this->last_headers_.end()) {
        ESP_LOGI(TAG, "Set-Cookie: %s", setCookieValue->second.c_str());
        this->cookies_.clear();
        this->cookies_.push_back(setCookieValue->second);
      }

      if (status_code == 302) {
        auto locationValue = this->last_headers_.find("Location");
        if (locationValue != this->last_headers_.end() &&
            locationValue->second != current_url) {
          ESP_LOGI(TAG, "Redirect Location Found: %s",
                   locationValue->second.c_str());
          current_url = locationValue->second;

          size_t pos = current_url.find("index.html");
          if (pos != std::string::npos) {
            ESP_LOGI(TAG, "Cancelling redirection to index.html");
            current_url.clear();
          }
        }
      } else {
        current_url.clear();
      }
    } else {
      ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
      current_url.clear();
    }

    esp_http_client_cleanup(client);
    if (current_url.empty()) break;
  }
  return err;
}

esp_err_t NetgearM5Component::_event_handler(esp_http_client_event_t *evt) {
  auto *ctx = static_cast<RequestContext *>(evt->user_data);
  if (!ctx) return ESP_FAIL;

  NetgearM5Component *self = ctx->instance;
  std::string *resp = ctx->response;

  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_HEADERS_SENT:
      ESP_LOGD(TAG, "HTTP_EVENT_HEADERS_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER");
      if (evt->header_key && evt->header_value) {
        ESP_LOGD(TAG, "Header: %s: %s", evt->header_key, evt->header_value);
        self->last_headers_[evt->header_key] = evt->header_value;
      }
      break;
    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
      if (evt->user_data) {
        resp->clear();
        self->last_headers_.clear();
      }
      break;
    case HTTP_EVENT_ON_DATA:
      if (evt->data && evt->data_len > 0) {
        resp->append((const char *)evt->data, evt->data_len);
      }
      break;
    case HTTP_EVENT_REDIRECT:
      ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
      break;
    default:
      ESP_LOGD(TAG, "HTTP event: %d", evt->event_id);
      break;
  }
  return ESP_OK;
}

void NetgearM5Component::publish_pending_() {
  taskENTER_CRITICAL(&this->mux_);
  auto state = this->state_;
  this->has_new_state_ = false;
  taskEXIT_CRITICAL(&this->mux_);

  // Numeric sensors
  for (auto &b : this->num_bindings_) {
    if (!b.sensor) continue;
    auto it = state.find(b.path);
    if (it != state.end() && !it->second.empty()) {
      ESP_LOGD(TAG, "Numeric sensor path %s: value %s", b.path.c_str(),
               it->second.c_str());
      char *endptr;
      errno = 0;
      double value = strtod(it->second.c_str(), &endptr);
      if (endptr == it->second.c_str() || *endptr != '\0' || errno == ERANGE) {
        ESP_LOGW(TAG, "Invalid numeric value at %s: %s", b.path.c_str(),
                 it->second.c_str());
        continue;
      }
      b.sensor->publish_state(value);
    }
  }

  // Text sensors
  for (auto &b : this->text_bindings_) {
    if (!b.sensor) continue;
    auto it = state.find(b.path);
    if (it != state.end() && !it->second.empty()) {
      ESP_LOGD(TAG, "Text sensor path %s: value %s", b.path.c_str(),
               it->second.c_str());
      b.sensor->publish_state(it->second);
    }
  }

  // Binary sensors
  for (auto &b : this->bin_bindings_) {
    if (!b.sensor) continue;
    auto it = state.find(b.path);
    if (it != state.end()) {
      ESP_LOGD(TAG, "Binary sensor path %s: value %s (on: %s, off: %s)",
               b.path.c_str(), it->second.c_str(), b.on_value.c_str(),
               b.off_value.c_str());
      if (it->second == b.on_value) {
        b.sensor->publish_state(true);
      } else if (it->second == b.off_value) {
        b.sensor->publish_state(false);
      }
    }
  }
}

std::string NetgearM5Component::dotted_lookup_(
    const std::string &path, const ::ArduinoJson::JsonVariantConst &root) {
  ESP_LOGD(TAG, "Looking up path: %s", path.c_str());

  ::ArduinoJson::JsonVariantConst cur = root;
  size_t i = 0;

  while (i < path.size()) {
    size_t dot = path.find('.', i);
    std::string token =
        path.substr(i, dot == std::string::npos ? std::string::npos : dot - i);

    size_t lb = token.find('[');
    if (lb != std::string::npos && token.back() == ']') {
      std::string key = token.substr(0, lb);
      std::string index_str = token.substr(lb + 1, token.size() - lb - 2);
      int index = atoi(index_str.c_str());
      ESP_LOGD(TAG, "Array access: key=%s, index=%d", key.c_str(), index);

      if (!key.empty()) {
        auto next = cur[key.c_str()];
        if (next.isNull()) {
          ESP_LOGW(TAG, "Key not found: %s", key.c_str());
          return {};
        }
        cur = next;
      }
      if (!cur.is<ArduinoJson::JsonArrayConst>()) {
        ESP_LOGW(TAG, "Not an array at key: %s", key.c_str());
        return {};
      }
      auto arr = cur.as<ArduinoJson::JsonArrayConst>();
      if (index < 0 || static_cast<size_t>(index) >= arr.size()) {
        ESP_LOGW(TAG, "Invalid array index: %d (array size: %u)", index,
                 arr.size());
        return {};
      }
      cur = arr[index];
    } else {
      auto next = cur[token.c_str()];
      if (next.isNull()) {
        ESP_LOGW(TAG, "Key not found: %s", token.c_str());
        return {};
      }
      cur = next;
    }

    if (dot == std::string::npos) break;
    i = dot + 1;
  }

  if (cur.isNull()) {
    ESP_LOGW(TAG, "Final value is null for path: %s", path.c_str());
    return {};
  }

  if (cur.is<const char *>()) {
    std::string result = cur.as<const char *>();
    ESP_LOGD(TAG, "Found string value: %s", result.c_str());
    return result;
  }
  if (cur.is<int>()) {
    std::string result = std::to_string(cur.as<int>());
    ESP_LOGD(TAG, "Found int value: %s", result.c_str());
    return result;
  }
  if (cur.is<long>()) {
    std::string result = std::to_string(cur.as<long>());
    ESP_LOGD(TAG, "Found long value: %s", result.c_str());
    return result;
  }
  if (cur.is<double>()) {
    std::string result = std::to_string(cur.as<double>());
    ESP_LOGD(TAG, "Found double value: %s", result.c_str());
    return result;
  }
  if (cur.is<bool>()) {
    std::string result = cur.as<bool>() ? "true" : "false";
    ESP_LOGD(TAG, "Found bool value: %s", result.c_str());
    return result;
  }

  std::string out;
  serializeJson(cur, out);
  ESP_LOGD(TAG, "Found complex value: %s", out.c_str());
  return out;
}

void NetgearM5Component::bind_numeric_sensor(const std::string &json_path,
                                             sensor::Sensor *s) {
  this->num_bindings_.push_back({json_path, s});
}

void NetgearM5Component::bind_text_sensor(const std::string &json_path,
                                          text_sensor::TextSensor *s) {
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