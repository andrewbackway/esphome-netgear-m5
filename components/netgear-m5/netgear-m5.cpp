#include "netgear-m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>

namespace esphome
{
    namespace netgear_m5
    {

        static const char *const TAG = "netgear_m5";

        void NetgearM5Component::setup()
        {
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

        void NetgearM5Component::loop()
        {
            if (this->has_new_payload_)
            {
                this->publish_pending_();
            }
        }

        void NetgearM5Component::dump_config()
        {
            ESP_LOGCONFIG(TAG, "Netgear M5 Component:");
            ESP_LOGCONFIG(TAG, "  Host: %s", this->host_.c_str());
            ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", (unsigned)this->poll_interval_ms_);
        }

        void NetgearM5Component::task_trampoline_(void *param)
        {
            static_cast<NetgearM5Component *>(param)->task_loop_();
        }

        void NetgearM5Component::task_loop_()
        {
            const TickType_t delay_ticks = pdMS_TO_TICKS(this->poll_interval_ms_);
            for (;;)
            {
                if (!network::is_connected())
                {
                    ESP_LOGW(TAG, "Network not connected, skipping fetch");
                    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before retrying
                    continue;
                }
                std::string body;
                if (this->fetch_once_(body))
                {
                    taskENTER_CRITICAL(&this->mux_);
                    this->last_payload_ = std::move(body);
                    this->has_new_payload_ = true;
                    taskEXIT_CRITICAL(&this->mux_);
                }
                vTaskDelay(delay_ticks);
            }
        }
        
        bool NetgearM5Component::fetch_once_(std::string &body) {
            
        }

        void NetgearM5Component::publish_pending_()
        {
            std::string payload;
            taskENTER_CRITICAL(&this->mux_);
            payload.swap(this->last_payload_);
            this->has_new_payload_ = false;
            taskEXIT_CRITICAL(&this->mux_);
            if (payload.empty())
            {
                ESP_LOGD(TAG, "No payload to process");
                return;
            }

             // Log full response
            ESP_LOGD(TAG, "Full JSON Payload (%u bytes):", payload.size());
            std::string log_safe_rx = payload;
            for (char &c : log_safe_rx)
            {
                if (c == '\r')
                    c = ' ';
                else if (c == '\n')
                    c = ' ';
                else if (c < 32 || c >= 127)
                    c = ' ';
            }
            const size_t chunk_size = 64;
            for (size_t i = 0; i < log_safe_rx.size(); i += chunk_size)
            {
                std::string chunk = log_safe_rx.substr(i, chunk_size);
                ESP_LOGD(TAG, "Response chunk %s", chunk.c_str());
            }



            ESP_LOGD(TAG, "Free heap before parsing: %u bytes", esp_get_free_heap_size());
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, payload);
            if (err)
            {
                ESP_LOGW(TAG, "JSON parse failed in publish_pending_: %s (payload size: %u bytes)", err.c_str(), payload.size());
                return;
            }
            ESP_LOGD(TAG, "Free heap after parsing: %u bytes", esp_get_free_heap_size());

           
            // Check if doc is an object
            if (!doc.is<JsonObject>())
            {
                ESP_LOGW(TAG, "Parsed JSON is not an object");
                return;
            }

            // Use the entire document as root
            auto root = doc.as<ArduinoJson::JsonObjectConst>();

            // Log all top-level keys for debugging
            std::string keys;
            for (JsonPairConst kv : root)
            {
                keys += kv.key().c_str();
                keys += ", ";
            }
            if (!keys.empty())
            {
                keys.erase(keys.size() - 2); // Remove trailing ", "
                ESP_LOGD(TAG, "Top-level JSON keys: %s", keys.c_str());
            }
            else
            {
                ESP_LOGW(TAG, "No top-level JSON keys found");
            }

            // Numeric sensors
            for (auto &b : this->num_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);
                ESP_LOGD(TAG, "Numeric sensor path %s: value %s", b.path.c_str(), v.c_str());
                if (!v.empty())
                {
                    char *endptr;
                    errno = 0;
                    double value = strtod(v.c_str(), &endptr);
                    if (endptr == v.c_str() || *endptr != '\0' || errno == ERANGE)
                    {
                        ESP_LOGW(TAG, "Invalid numeric value at %s: %s", b.path.c_str(), v.c_str());
                        continue;
                    }
                    b.sensor->publish_state(value);
                }
            }

            // Text sensors
            for (auto &b : this->text_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);
                ESP_LOGD(TAG, "Text sensor path %s: value %s", b.path.c_str(), v.c_str());
                if (!v.empty())
                    b.sensor->publish_state(v);
            }

            // Binary sensors
            for (auto &b : this->bin_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);
                ESP_LOGD(TAG, "Binary sensor path %s: value %s (on: %s, off: %s)", b.path.c_str(), v.c_str(), b.on_value.c_str(), b.off_value.c_str());
                if (v == b.on_value)
                {
                    b.sensor->publish_state(true);
                }
                else if (v == b.off_value)
                {
                    b.sensor->publish_state(false);
                }
            }
        }

        std::string NetgearM5Component::dotted_lookup_(const std::string &path, const ::ArduinoJson::JsonVariantConst &root)
        {
            ESP_LOGD(TAG, "Looking up path: %s", path.c_str());

            ::ArduinoJson::JsonVariantConst cur = root;
            size_t i = 0;

            // Log the root JSON for debugging
            std::string root_json;
            serializeJson(root, root_json);
            ESP_LOGD(TAG, "Root JSON: %s", root_json.c_str());

            while (i < path.size())
            {
                size_t dot = path.find('.', i);
                std::string token = path.substr(i, dot == std::string::npos ? std::string::npos : dot - i);
                ESP_LOGD(TAG, "Processing token: %s", token.c_str());

                size_t lb = token.find('[');
                if (lb != std::string::npos && token.back() == ']')
                {
                    std::string key = token.substr(0, lb);
                    std::string index_str = token.substr(lb + 1, token.size() - lb - 2);
                    int index = atoi(index_str.c_str());
                    ESP_LOGD(TAG, "Array access: key=%s, index=%d", key.c_str(), index);

                    if (!key.empty())
                    {
                        auto next = cur[key.c_str()];
                        if (next.isNull())
                        {
                            ESP_LOGW(TAG, "Key not found: %s", key.c_str());
                            return {};
                        }
                        cur = next;
                    }
                    if (!cur.is<ArduinoJson::JsonArrayConst>())
                    {
                        ESP_LOGW(TAG, "Not an array at key: %s", key.c_str());
                        return {};
                    }
                    auto arr = cur.as<ArduinoJson::JsonArrayConst>();
                    if (index < 0 || static_cast<size_t>(index) >= arr.size())
                    {
                        ESP_LOGW(TAG, "Invalid array index: %d (array size: %u)", index, arr.size());
                        return {};
                    }
                    cur = arr[index];
                }
                else
                {
                    auto next = cur[token.c_str()];
                    if (next.isNull())
                    {
                        ESP_LOGW(TAG, "Key not found: %s", token.c_str());
                        return {};
                    }
                    cur = next;
                }

                if (dot == std::string::npos)
                    break;
                i = dot + 1;
            }

            if (cur.isNull())
            {
                ESP_LOGW(TAG, "Final value is null for path: %s", path.c_str());
                return {};
            }

            if (cur.is<const char *>())
            {
                std::string result = cur.as<const char *>();
                ESP_LOGD(TAG, "Found string value: %s", result.c_str());
                return result;
            }
            if (cur.is<int>())
            {
                std::string result = std::to_string(cur.as<int>());
                ESP_LOGD(TAG, "Found int value: %s", result.c_str());
                return result;
            }
            if (cur.is<long>())
            {
                std::string result = std::to_string(cur.as<long>());
                ESP_LOGD(TAG, "Found long value: %s", result.c_str());
                return result;
            }
            if (cur.is<double>())
            {
                std::string result = std::to_string(cur.as<double>());
                ESP_LOGD(TAG, "Found double value: %s", result.c_str());
                return result;
            }
            if (cur.is<bool>())
            {
                std::string result = cur.as<bool>() ? "true" : "false";
                ESP_LOGD(TAG, "Found bool value: %s", result.c_str());
                return result;
            }

            std::string out;
            serializeJson(cur, out);
            ESP_LOGD(TAG, "Found complex value: %s", out.c_str());
            return out;
        }

        void NetgearM5Component::bind_numeric_sensor(const std::string &json_path, sensor::Sensor *s)
        {
            this->num_bindings_.push_back({json_path, s});
        }

        void NetgearM5Component::bind_text_sensor(const std::string &json_path, text_sensor::TextSensor *s)
        {
            this->text_bindings_.push_back({json_path, s});
        }

        void NetgearM5Component::bind_binary_sensor(const std::string &json_path,
                                                    binary_sensor::BinarySensor *s,
                                                    const std::string &on_value,
                                                    const std::string &off_value)
        {
            this->bin_bindings_.push_back({json_path, s, on_value, off_value});
        }

    } // namespace netgear_m5
} // namespace esphome