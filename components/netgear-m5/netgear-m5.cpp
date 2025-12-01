#include "netgear-m5.h"
#include "sensor/netgear_m5_sensor.h"
#include "text_sensor/netgear_m5_text_sensor.h"
#include "binary_sensor/netgear_m5_binary_sensor.h"

#include <ArduinoJson.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace esphome {
namespace netgear_m5 {

static const char* const TAG = "netgear_m5";
static constexpr size_t MAX_HTTP_BODY = 4 * 1024;  // 4 KB cap for login/cookie responses

// Memory-efficient streaming buffer - smaller chunks for HTTP data
static constexpr size_t STREAM_CHUNK_SIZE = 1024;

void NetgearM5Component::setup() {
  ESP_LOGD(TAG, "Setting up Netgear M5 component");

  // Reserve capacity for sensors to reduce heap fragmentation
  this->sensors_.reserve(10);
  this->text_sensors_.reserve(10);
  this->binary_sensors_.reserve(5);

  // Build base JSON filter structure so sensors can add their paths during registration
  this->build_json_filter_();

  // Note: Background task will be started in loop() after sensors have had a chance to register
}

void NetgearM5Component::loop() {
  // Start background task on first loop iteration (after all sensors have set up)
  if (!this->task_started_) {
    ESP_LOGD(TAG, "Starting background task (registered %u sensors, %u text sensors, %u binary sensors)",
             (unsigned)this->sensors_.size(), (unsigned)this->text_sensors_.size(),
             (unsigned)this->binary_sensors_.size());
    
    xTaskCreatePinnedToCore(&NetgearM5Component::task_trampoline_,
                            "netgear_m5_task", 6144, this, 4, &this->task_handle_,
                            1);
    this->task_started_ = true;
  }
  
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

void NetgearM5Component::task_trampoline_(void* param) {
  static_cast<NetgearM5Component*>(param)->task_loop_();
}

void NetgearM5Component::task_loop_() {
  const TickType_t delay_ticks = pdMS_TO_TICKS(this->poll_interval_ms_);
  for (;;) {
    if (!network::is_connected()) {
      ESP_LOGW(TAG, "Network not connected, skipping fetch");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGD(TAG, "Starting fetch cycle (free heap: %u bytes)",
             esp_get_free_heap_size());

    if (this->fetch_and_parse_()) {
      this->has_new_state_ = true;
    }

    vTaskDelay(delay_ticks);
  }
}

void NetgearM5Component::build_json_filter_() {
  // Build a filter document that includes only the paths we need
  // This dramatically reduces memory usage when parsing the ~28KB JSON response
  // by filtering during deserialization (not after)

  // Always need session token for login
  this->json_filter_["session"]["secToken"] = true;

  // Signal strength fields for bars calculation
  this->json_filter_["wwan"]["signalStrength"]["rsrp"] = true;
  this->json_filter_["wwan"]["signalStrength"]["rsrq"] = true;
  this->json_filter_["wwan"]["signalStrength"]["sinr"] = true;
  this->json_filter_["wwan"]["signalStrength"]["rssi"] = true;

  // Do NOT add user-configured binding paths here; those are added incrementally in bind_* methods
  ESP_LOGD(TAG, "Built JSON filter base");
}

void NetgearM5Component::add_path_to_filter_(const std::string& path) {
  // Parse dotted path like "power.battChargeLevel" and add to filter
  // This creates the nested structure: filter["power"]["battChargeLevel"] = true

  ESP_LOGD(TAG, "Adding path to JSON filter: %s", path.c_str());
  JsonVariant current = this->json_filter_.as<JsonVariant>();
  size_t start = 0;

  while (start < path.size()) {
    size_t dot = path.find('.', start);
    size_t end = (dot == std::string::npos) ? path.size() : dot;

    std::string key = path.substr(start, end - start);

    // Handle array notation like "list[0]"
    size_t bracket = key.find('[');
    if (bracket != std::string::npos) {
      std::string array_key = key.substr(0, bracket);
      // For arrays, we just need to mark the first element as needing parsing
      // ArduinoJson will apply the filter to all array elements
      current = current[array_key.c_str()];
      current = current[0];
    } else {
      current = current[key.c_str()];
    }

    if (dot == std::string::npos) {
      // Last segment - mark as true to include this field
      current.set(true);
      break;
    }
    start = dot + 1;
  }
}

float NetgearM5Component::extract_signal_value_(const std::string& key) {
  // Helper to extract signal strength values from state map
  auto it = this->state_.find(key);
  if (it != this->state_.end() && !it->second.empty()) {
    return static_cast<float>(atof(it->second.c_str()));
  }
  return std::numeric_limits<float>::quiet_NaN();
}

bool NetgearM5Component::fetch_and_parse_() {
  // Memory-efficient fetch and parse using ArduinoJson filtering
  // Dynamically allocate buffer only during fetch to avoid permanent RAM usage

  // Allocate buffer at start of fetch
  if (this->stream_buf_ == nullptr) {
    this->stream_buf_ = static_cast<char*>(malloc(STREAM_BUF_SIZE));
    if (this->stream_buf_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate stream buffer (free heap: %u bytes)",
               esp_get_free_heap_size());
      return false;
    }
    ESP_LOGD(TAG, "Allocated %u byte stream buffer", (unsigned)STREAM_BUF_SIZE);
  }

  bool success = false;

  if (cookie_.empty()) {
    std::string unused;
    esp_err_t first_err = this->_request(
        "http://" + this->host_ + "/sess_cd_tmp?op=%2F&oq=", HTTP_METHOD_GET,
        "", "", unused);
    if (first_err != ESP_OK || cookie_.empty()) {
      ESP_LOGE(TAG, "Unable to obtain first cookie");
      goto cleanup;
    }
  }

  {
    const std::string model_url =
        "http://" + this->host_ + "/api/model.json?internalapi=1";

    // For login, we need to fetch without filter first to get the token
    if (!this->logged_in_) {
      ESP_LOGD(TAG, "Performing login sequence");

      // Fetch and parse with minimal filter for login token only
      JsonDocument login_filter;
      login_filter["session"]["secToken"] = true;

      this->stream_len_ = 0;
      esp_err_t err = this->stream_json_request_(model_url, HTTP_METHOD_GET, "",
                                                  "", login_filter);
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to fetch model.json for login");
        goto cleanup;
      }

      if (this->sec_token_.empty()) {
        ESP_LOGE(TAG, "Failed to extract session token");
        goto cleanup;
      }

      // Perform login
      std::string login_body = "session.password=" + this->password_ +
                               "&token=" + this->sec_token_ +
                               "&ok_redirect=%2Findex.html&" +
                               "err_redirect=%2Findex.html%3Floginfailed";

      std::string unused;
      esp_err_t login_err = this->_request(
          "http://" + this->host_ + "/Forms/config", HTTP_METHOD_POST, login_body,
          "application/x-www-form-urlencoded", unused);

      if (login_err != ESP_OK) {
        ESP_LOGE(TAG, "Login failed");
        goto cleanup;
      }

      this->logged_in_ = true;
      ESP_LOGD(TAG, "Login OK");
    }

    // Main data fetch with full filter
    this->stream_len_ = 0;
    this->state_.clear();

    esp_err_t err = this->stream_json_request_(model_url, HTTP_METHOD_GET, "", "",
                                                this->json_filter_);

    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to fetch model.json");
      goto cleanup;
    }

    // Calculate bars from signal strength values using helper function
    float rsrp_dbm = extract_signal_value_("wwan.signalStrength.rsrp");
    float rsrq_db = extract_signal_value_("wwan.signalStrength.rsrq");
    float sinr_db = extract_signal_value_("wwan.signalStrength.sinr");
    float rssi_dbm = extract_signal_value_("wwan.signalStrength.rssi");

    bool has_rsrp = !std::isnan(rsrp_dbm);
    bool has_rsrq = !std::isnan(rsrq_db);
    bool has_sinr = !std::isnan(sinr_db);
    bool has_rssi = !std::isnan(rssi_dbm);

    int bars = this->calc_mobile_bars(has_rsrp, rsrp_dbm, has_rsrq, rsrq_db,
                                      has_sinr, sinr_db, has_rssi, rssi_dbm);
    this->state_["wwan.signalStrength.bars"] = std::to_string(bars);

    success = true;
  }

cleanup:
  // Free buffer after fetch to release memory
  if (this->stream_buf_ != nullptr) {
    free(this->stream_buf_);
    this->stream_buf_ = nullptr;
    ESP_LOGD(TAG, "Freed stream buffer (free heap: %u bytes)",
             esp_get_free_heap_size());
  }

  return success;
}

// REMOVED: fetch_once_ - replaced by fetch_and_parse_() with filtering

esp_err_t NetgearM5Component::stream_json_request_(
    const std::string& url, esp_http_client_method_t method,
    const std::string& body, const std::string& content_type,
    JsonDocument& filter) {
  // Memory-efficient JSON streaming with filtering
  // Instead of buffering the entire 28KB response, we use ArduinoJson's
  // DeserializationOption::Filter to parse only needed fields during streaming

  esp_err_t err = ESP_FAIL;
  std::string current_url = url;

  while (true) {
    ESP_LOGD(TAG, "Preparing streaming JSON request: %s", current_url.c_str());

    // Set up streaming context
    StreamContext stream_ctx;
    stream_ctx.instance = this;
    stream_ctx.filter = &filter;
    stream_ctx.parse_complete = false;
    stream_ctx.parse_error = false;

    esp_http_client_config_t config = {};
    config.url = current_url.c_str();
    config.event_handler = _stream_event_handler;
    config.user_data = &stream_ctx;
    config.disable_auto_redirect = true;
    config.timeout_ms = 120000;
    // Smaller buffer sizes for memory efficiency
    config.buffer_size = 512;
    config.buffer_size_tx = 512;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
      ESP_LOGE(TAG, "Failed to init HTTP client");
      return ESP_FAIL;
    }

    esp_http_client_set_method(client, method);

    if (!cookie_.empty()) {
      esp_http_client_set_header(client, "Cookie", cookie_.c_str());
    }

    if (!body.empty()) {
      esp_http_client_set_post_field(client, body.c_str(), body.size());
      if (!content_type.empty()) {
        esp_http_client_set_header(client, "Content-Type",
                                   content_type.c_str());
      }
    }

    ESP_LOGD(TAG, "Sending streaming request: %s", current_url.c_str());
    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
      int status_code = esp_http_client_get_status_code(client);
      this->last_status_code_ = status_code;
      ESP_LOGD(TAG, "HTTP Status = %d", this->last_status_code_);

      auto setCookieValue = this->last_headers_.find("Set-Cookie");
      if (setCookieValue != this->last_headers_.end()) {
        ESP_LOGI(TAG, "Set-Cookie: %s", setCookieValue->second.c_str());
        this->cookie_ = setCookieValue->second;
      }

      if (status_code == 302) {
        auto locationValue = this->last_headers_.find("Location");
        if (locationValue != this->last_headers_.end() &&
            locationValue->second != current_url) {
          ESP_LOGI(TAG, "Redirect Location Found: %s",
                   locationValue->second.c_str());

          size_t pos = locationValue->second.find("index.html");
          if (pos != std::string::npos) {
            ESP_LOGI(TAG, "Cancelling redirection to index.html");
            current_url.clear();
          } else {
            current_url = locationValue->second;
          }
        }
      } else {
        current_url.clear();
      }

      // Check if parsing was successful
      if (stream_ctx.parse_error) {
        ESP_LOGW(TAG, "JSON parsing failed during streaming");
        err = ESP_FAIL;
      }
    } else {
      ESP_LOGE(TAG, "Streaming request failed: %s", esp_err_to_name(err));
      current_url.clear();
    }

    esp_http_client_cleanup(client);
    if (current_url.empty()) break;
  }
  return err;
}

esp_err_t NetgearM5Component::_stream_event_handler(esp_http_client_event_t* evt) {
  auto* ctx = static_cast<StreamContext*>(evt->user_data);
  if (!ctx) return ESP_FAIL;

  NetgearM5Component* self = ctx->instance;

  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(TAG, "HTTP_EVENT_ERROR (stream)");
      ctx->parse_error = true;
      break;

    case HTTP_EVENT_ON_HEADER:
      if (evt->header_key && evt->header_value) {
        if (strcmp(evt->header_key, "Set-Cookie") == 0 ||
            strcmp(evt->header_key, "Location") == 0) {
          self->last_headers_[evt->header_key] = evt->header_value;
        }
      }
      break;

    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED (stream)");
      self->last_headers_.clear();
      // Reset stream buffer for new response
      self->stream_len_ = 0;
      break;

    case HTTP_EVENT_ON_DATA:
      if (evt->data && evt->data_len > 0) {
        // Accumulate data in small streaming buffer
        size_t remaining = STREAM_BUF_SIZE - self->stream_len_;
        size_t to_copy = (evt->data_len < remaining) ? evt->data_len : remaining;

        if (to_copy > 0) {
          memcpy(self->stream_buf_ + self->stream_len_, evt->data, to_copy);
          self->stream_len_ += to_copy;
        }
      }
      break;

    case HTTP_EVENT_ON_FINISH:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH (stream), received %u bytes",
               (unsigned)self->stream_len_);
      // Parse the accumulated JSON with filtering
      if (self->stream_len_ > 0 && ctx->filter) {
        // Note: deserializeJson uses the explicit length, so null-termination
        // is optional. We add it when possible for debugging convenience.
        if (self->stream_len_ < STREAM_BUF_SIZE) {
          self->stream_buf_[self->stream_len_] = '\0';
        }

        JsonDocument doc;
        DeserializationError err = deserializeJson(
            doc, self->stream_buf_, self->stream_len_,
            DeserializationOption::Filter(*ctx->filter));

        if (err) {
          ESP_LOGW(TAG, "Filtered JSON parse failed: %s", err.c_str());
          ctx->parse_error = true;
        } else if (doc.is<JsonObject>()) {
          // Extract values from filtered document
          auto root = doc.as<ArduinoJson::JsonObjectConst>();

          // Extract session token
          auto sec_token = dotted_lookup_("session.secToken", root);
          if (!sec_token.empty()) {
            self->sec_token_ = sec_token;
          }

          // Extract all sensor values using platform-based sensor registration
          for (const auto* sensor : self->sensors_) {
            std::string val = dotted_lookup_(sensor->get_path(), root);
            if (!val.empty()) {
              self->state_[sensor->get_path()] = val;
            }
          }
          for (const auto* sensor : self->text_sensors_) {
            std::string val = dotted_lookup_(sensor->get_path(), root);
            if (!val.empty()) {
              self->state_[sensor->get_path()] = val;
            }
          }
          for (const auto* sensor : self->binary_sensors_) {
            std::string val = dotted_lookup_(sensor->get_path(), root);
            if (!val.empty()) {
              self->state_[sensor->get_path()] = val;
            }
          }

          // Extract signal strength values for bars calculation
          self->state_["wwan.signalStrength.rsrp"] =
              dotted_lookup_("wwan.signalStrength.rsrp", root);
          self->state_["wwan.signalStrength.rsrq"] =
              dotted_lookup_("wwan.signalStrength.rsrq", root);
          self->state_["wwan.signalStrength.sinr"] =
              dotted_lookup_("wwan.signalStrength.sinr", root);
          self->state_["wwan.signalStrength.rssi"] =
              dotted_lookup_("wwan.signalStrength.rssi", root);

          ctx->parse_complete = true;
          ESP_LOGD(TAG, "Filtered JSON parsed successfully");
        } else {
          ESP_LOGW(TAG, "Parsed JSON is not an object");
          ctx->parse_error = true;
        }
      }
      break;

    default:
      break;
  }
  return ESP_OK;
}

esp_err_t NetgearM5Component::_request(const std::string& url,
                                       esp_http_client_method_t method,
                                       const std::string& body,
                                       const std::string& content_type,
                                       std::string& response) {
  esp_err_t err = ESP_FAIL;
  std::string current_url = url;

  // Clear response but don't pre-reserve - these are small responses (login/cookie)
  response.clear();

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

    if (!cookie_.empty()) {
      esp_http_client_set_header(client, "Cookie", cookie_.c_str());
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
        this->cookie_ = setCookieValue->second;
      }

      if (status_code == 302) {
        auto locationValue = this->last_headers_.find("Location");
        if (locationValue != this->last_headers_.end() &&
            locationValue->second != current_url) {
          ESP_LOGI(TAG, "Redirect Location Found: %s",
                   locationValue->second.c_str());
          
          // Only cancel index.html redirect for GET requests
          // POST to /Forms/config needs to complete the redirect to finalize login
          size_t pos = locationValue->second.find("index.html");
          if (pos != std::string::npos && method == HTTP_METHOD_GET) {
            ESP_LOGI(TAG, "Cancelling GET redirection to index.html");
            current_url.clear();
          } else {
            current_url = locationValue->second;
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

// REMOVED: request_into_buffer_ - replaced by stream_json_request_ with filtering

esp_err_t NetgearM5Component::_event_handler(esp_http_client_event_t* evt) {
  auto* ctx = static_cast<RequestContext*>(evt->user_data);
  if (!ctx) return ESP_FAIL;

  NetgearM5Component* self = ctx->instance;
  std::string* resp = ctx->response;

  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_HEADERS_SENT:
      ESP_LOGD(TAG, "HTTP_EVENT_HEADERS_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      if (evt->header_key && evt->header_value) {
        // Only store headers we actually use to avoid heap fragmentation
        if (strcmp(evt->header_key, "Set-Cookie") == 0 ||
            strcmp(evt->header_key, "Location") == 0) {
          self->last_headers_[evt->header_key] = evt->header_value;
        }
      }
      break;
    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
      self->last_headers_.clear();
      if (resp) {
        resp->clear();
      }
      break;
    case HTTP_EVENT_ON_DATA:
      // Only collect response body for small responses (login/cookie)
      if (evt->data && evt->data_len > 0 && resp) {
        size_t remaining = 0;
        if (resp->size() < MAX_HTTP_BODY)
          remaining = MAX_HTTP_BODY - resp->size();

        if (remaining == 0) {
          ESP_LOGW(TAG, "Response body reached MAX_HTTP_BODY (%d)",
                   (int)MAX_HTTP_BODY);
          break;
        }

        size_t to_copy = evt->data_len;
        if (to_copy > remaining) to_copy = remaining;

        // Check heap before append to prevent OOM crash
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 8192) {
          ESP_LOGW(TAG, "Low heap (%u bytes), skipping response body",
                   free_heap);
          break;
        }

        resp->append(static_cast<const char*>(evt->data), to_copy);
      }
      break;
    case HTTP_EVENT_REDIRECT:
      ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
      break;
    default:
      break;
  }
  return ESP_OK;
}

void NetgearM5Component::publish_pending_() {
  taskENTER_CRITICAL(&this->mux_);
  auto state = this->state_;
  this->has_new_state_ = false;
  this->state_.clear();  // Free memory after copying, sensors maintain their own state
  taskEXIT_CRITICAL(&this->mux_);

  // Numeric sensors
  for (auto* sensor : this->sensors_) {
    if (!sensor) continue;
    const std::string& path = sensor->get_path();
    auto it = state.find(path);
    if (it != state.end() && !it->second.empty()) {
      ESP_LOGD(TAG, "Numeric sensor path %s: value %s", path.c_str(),
               it->second.c_str());
      char* endptr;
      errno = 0;
      double value = strtod(it->second.c_str(), &endptr);
      if (endptr == it->second.c_str() || *endptr != '\0' || errno == ERANGE) {
        ESP_LOGW(TAG, "Invalid numeric value at %s: %s", path.c_str(),
                 it->second.c_str());
        continue;
      }
      sensor->publish_state(value);
    }
  }

  // Text sensors
  for (auto* sensor : this->text_sensors_) {
    if (!sensor) continue;
    const std::string& path = sensor->get_path();
    auto it = state.find(path);
    if (it != state.end() && !it->second.empty()) {
      ESP_LOGD(TAG, "Text sensor path %s: value %s", path.c_str(),
               it->second.c_str());
      sensor->publish_state(it->second);
    }
  }

  // Binary sensors
  for (auto* sensor : this->binary_sensors_) {
    if (!sensor) continue;
    const std::string& path = sensor->get_path();
    auto it = state.find(path);
    if (it != state.end()) {
      ESP_LOGD(TAG, "Binary sensor path %s: value %s (on: %s, off: %s)",
               path.c_str(), it->second.c_str(), sensor->get_on_value().c_str(),
               sensor->get_off_value().c_str());
      if (it->second == sensor->get_on_value()) {
        sensor->publish_state(true);
      } else if (it->second == sensor->get_off_value()) {
        sensor->publish_state(false);
      }
    }
  }
}

std::string NetgearM5Component::dotted_lookup_(
    const std::string& path, const ::ArduinoJson::JsonVariantConst& root) {
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
                 (unsigned)arr.size());
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

  if (cur.is<const char*>()) {
    std::string result = cur.as<const char*>();
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

void NetgearM5Component::register_sensor(NetgearM5Sensor* sensor) {
  ESP_LOGD(TAG, "Registering numeric sensor for path: %s", sensor->get_path().c_str());
  this->sensors_.push_back(sensor);
  add_path_to_filter_(sensor->get_path());
}

void NetgearM5Component::register_text_sensor(NetgearM5TextSensor* sensor) {
  ESP_LOGD(TAG, "Registering text sensor for path: %s", sensor->get_path().c_str());
  this->text_sensors_.push_back(sensor);
  add_path_to_filter_(sensor->get_path());
}

void NetgearM5Component::register_binary_sensor(NetgearM5BinarySensor* sensor) {
  ESP_LOGD(TAG, "Registering binary sensor for path: %s (on: %s, off: %s)",
           sensor->get_path().c_str(), sensor->get_on_value().c_str(), 
           sensor->get_off_value().c_str());
  this->binary_sensors_.push_back(sensor);
  add_path_to_filter_(sensor->get_path());
}

int NetgearM5Component::clamp01(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int NetgearM5Component::bars_from_rsrp(float rsrp_dbm) {
  if (rsrp_dbm >= -85) return 5;
  if (rsrp_dbm >= -90) return 4;
  if (rsrp_dbm >= -100) return 3;
  if (rsrp_dbm >= -110) return 2;
  if (rsrp_dbm >= -120) return 1;
  return 0;
}

int NetgearM5Component::quality_adjust_from_rsrq(float rsrq_db) {
  if (rsrq_db > -10) return 0;
  if (rsrq_db >= -14) return -1;
  return -2;
}

int NetgearM5Component::quality_adjust_from_sinr(float sinr_db) {
  if (sinr_db >= 20) return +1;
  if (sinr_db >= 13) return 0;
  if (sinr_db >= 0) return -1;
  return -2;
}

int NetgearM5Component::bars_from_rssi(float rssi_dbm) {
  if (rssi_dbm >= -65) return 5;
  if (rssi_dbm >= -75) return 4;
  if (rssi_dbm >= -85) return 3;
  if (rssi_dbm >= -95) return 2;
  if (rssi_dbm >= -105) return 1;
  return 0;
}

int NetgearM5Component::calc_mobile_bars(bool has_rsrp, float rsrp_dbm,
                                         bool has_rsrq, float rsrq_db,
                                         bool has_sinr, float sinr_db,
                                         bool has_rssi, float rssi_dbm) {
  int bars = -1;

  if (has_rsrp) {
    bars = bars_from_rsrp(rsrp_dbm);
    if (has_rsrq)
      bars += quality_adjust_from_rsrq(rsrq_db);
    else if (has_sinr)
      bars += quality_adjust_from_sinr(sinr_db);
  } else if (has_rssi) {
    bars = bars_from_rssi(rssi_dbm);
  } else {
    bars = 0;  // unknown
  }

  return clamp01(bars, 0, 5);
}

}  // namespace netgear_m5
}  // namespace esphome
