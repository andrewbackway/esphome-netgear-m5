#include "netgear-m5.h"

#include <cstring>
#include <string>
#include <ArduinoJson.h>
#include <cstdlib>

namespace esphome
{
    namespace netgear_m5
    {

        // ... [unchanged setup, loop, task, fetch_once_, extract_http_body_ etc.] ...

        void NetgearM5Component::publish_pending_()
        {
            std::string payload;
            taskENTER_CRITICAL(&this->mux_);
            payload.swap(this->last_payload_);
            this->has_new_payload_ = false;
            taskEXIT_CRITICAL(&this->mux_);
            if (payload.empty())
                return;

            ArduinoJson::JsonDocument doc(131072);
            auto err = deserializeJson(doc, payload);
            if (err)
            {
                ESP_LOGW(TAG, "JSON parse failed in publish_pending_: %s", err.c_str());
                return;
            }
            auto root = doc.as<ArduinoJson::JsonVariantConst>();

            // Numeric
            for (auto &b : this->num_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);
                if (!v.empty())
                    b.sensor->publish_state(strtod(v.c_str(), nullptr));
            }

            // Text
            for (auto &b : this->text_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);
                if (!v.empty())
                    b.sensor->publish_state(v);
            }

            // Binary
            for (auto &b : this->bin_bindings_)
            {
                if (!b.sensor)
                    continue;
                std::string v = dotted_lookup_(b.path, root);

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
            ::ArduinoJson::JsonVariantConst cur = root;
            size_t i = 0;

            while (i < path.size())
            {
                size_t dot = path.find('.', i);
                std::string token = path.substr(i, dot == std::string::npos ? std::string::npos : dot - i);

                size_t lb = token.find('[');
                if (lb != std::string::npos && token.back() == ']')
                {
                    std::string key = token.substr(0, lb);
                    int index = atoi(token.substr(lb + 1, token.size() - lb - 2).c_str());

                    if (!key.empty())
                    {
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
                }
                else
                {
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
