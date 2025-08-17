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
                std::string raw;
                if (this->fetch_once_(raw))
                {
                    std::string body;
                    if (extract_http_body_(raw, body))
                    {
                        taskENTER_CRITICAL(&this->mux_);
                        this->last_payload_ = std::move(body);
                        this->has_new_payload_ = true;
                        taskEXIT_CRITICAL(&this->mux_);
                    }
                }
                vTaskDelay(delay_ticks);
            }
        }

        bool fetch_once_(std::string &body)
        {
            ESP_LOGD(TAG, "Fetching data from Netgear M5");

            if (this->host_.empty())
            {
                ESP_LOGW(TAG, "Host is empty, cannot fetch data");
                return false;
            }

            const char *port = "80";
            std::string current_path = "/api/model.json?internalapi=1";
            int max_redirects = 3;
            int redirect_count = 0;

            while (redirect_count < max_redirects)
            {
                struct addrinfo hints = {};
                hints.ai_family = AF_INET;
                hints.ai_socktype = SOCK_STREAM;

                struct addrinfo *res = nullptr;
                int err = getaddrinfo(this->host_.c_str(), port, &hints, &res);
                if (err != 0 || res == nullptr)
                {
                    ESP_LOGW(TAG, "DNS resolution failed for %s", this->host_.c_str());
                    if (res)
                        freeaddrinfo(res);
                    return false;
                }

                int sock = -1;
                for (struct addrinfo *p = res; p != nullptr; p = p->ai_next)
                {
                    sock = lwip_socket(p->ai_family, p->ai_socktype, p->ai_protocol);
                    if (sock < 0)
                        continue;

                    struct timeval tv{.tv_sec = 10, .tv_usec = 0};
                    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                    lwip_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

                    if (lwip_connect(sock, p->ai_addr, p->ai_addrlen) == 0)
                        break;
                    lwip_close(sock);
                    sock = -1;
                }
                freeaddrinfo(res);
                if (sock < 0)
                {
                    ESP_LOGW(TAG, "Failed to connect to %s:%s", this->host_.c_str(), port);
                    return false;
                }

                // Build request
                std::string req = "GET " + current_path + " HTTP/1.1\r\n" +
                                  "Host: " + this->host_ + "\r\n" +
                                  "User-Agent: ESPHome-NetgearM5/1.0\r\n" +
                                  "Connection: close\r\n" +
                                  "Accept: application/json\r\n";
                if (!this->cookie_.empty())
                {
                    req += "Cookie: " + this->cookie_ + "\r\n";
                }
                req += "\r\n";

                if (lwip_send(sock, req.data(), req.size(), 0) < 0)
                {
                    ESP_LOGW(TAG, "Failed to send request: %d", errno);
                    lwip_close(sock);
                    return false;
                }

                // Read response
                char buf[1024];
                std::string rx;
                rx.reserve(8192);

                while (true)
                {
                    int n = lwip_recv(sock, buf, sizeof(buf) - 1, 0);
                    if (n <= 0)
                        break;
                    buf[n] = '\0';
                    rx.append(buf, n);
                }
                lwip_close(sock);

                // Log full response, escaping newlines
                ESP_LOGD(TAG, "Full HTTP response (%u bytes):", rx.size());
                std::string log_safe_rx = rx;
                for (char &c : log_safe_rx)
                {
                    if (c == '\r')
                        c = '\\';
                    else if (c == '\n')
                        c = 'n';
                    else if (c < 32 || c >= 127)
                        c = '?';
                }
                const size_t chunk_size = 256;
                for (size_t i = 0; i < log_safe_rx.size(); i += chunk_size)
                {
                    std::string chunk = log_safe_rx.substr(i, chunk_size);
                    ESP_LOGD(TAG, "Response chunk [%u-%u]: %s", i, i + chunk.size() - 1, chunk.c_str());
                }

                // Find header/body split
                auto header_end = rx.find("\r\n\r\n");
                if (header_end == std::string::npos)
                {
                    ESP_LOGW(TAG, "Malformed HTTP response, no header-body separator");
                    return false;
                }

                std::string headers = rx.substr(0, header_end);
                std::string body_part = rx.substr(header_end + 4);

                // Check for Set-Cookie header
                auto cookie_pos = headers.find("Set-Cookie:");
                if (cookie_pos != std::string::npos)
                {
                    size_t start = cookie_pos + 11;
                    size_t end = headers.find("\r\n", start);
                    if (end == std::string::npos)
                        end = headers.size();
                    this->cookie_ = headers.substr(start, end - start);
                    // Trim whitespace
                    this->cookie_.erase(0, this->cookie_.find_first_not_of(" \t"));
                    this->cookie_.erase(this->cookie_.find_last_not_of(" \t") + 1);
                    ESP_LOGD(TAG, "Stored cookie: %s", this->cookie_.c_str());
                }

                // Check for redirect
                if (headers.find("HTTP/1.1 302") != std::string::npos ||
                    headers.find("HTTP/1.0 302") != std::string::npos)
                {
                    auto loc_pos = headers.find("Location:");
                    if (loc_pos != std::string::npos)
                    {
                        size_t start = loc_pos + 9;
                        size_t end = headers.find("\r\n", start);
                        if (end == std::string::npos)
                            end = headers.size();
                        std::string location = headers.substr(start, end - start);
                        // Trim whitespace
                        location.erase(0, location.find_first_not_of(" \t"));
                        location.erase(location.find_last_not_of(" \t") + 1);
                        ESP_LOGD(TAG, "Redirect detected to: %s", location.c_str());

                        // Update path for redirect
                        if (location.find("http://") == 0)
                        {
                            size_t path_start = location.find("/", 7);
                            if (path_start == std::string::npos)
                            {
                                current_path = "/";
                            }
                            else
                            {
                                current_path = location.substr(path_start);
                            }
                        }
                        else
                        {
                            current_path = location;
                        }

                        redirect_count++;
                        ESP_LOGD(TAG, "Following redirect to %s", current_path.c_str());
                        continue;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Redirect response missing Location header");
                        return false;
                    }
                }

                // Look for Content-Length
                size_t content_length = 0;
                auto cl_pos = headers.find("Content-Length:");
                if (cl_pos != std::string::npos)
                {
                    //try
                    //{
                        content_length = std::stoul(headers.substr(cl_pos + 15));
                    //}
                    //catch (...)
                    //{
                        content_length = 0;
                    //}
                }

                if (content_length > 0)
                {
                    if (body_part.size() < content_length)
                    {
                        ESP_LOGW(TAG, "Body truncated: expected %u bytes, got %u bytes", (unsigned)content_length, (unsigned)body_part.size());
                    }
                    else
                    {
                        body = body_part.substr(0, content_length);
                    }
                }
                else
                {
                    body = body_part;
                }

                // Log body, escaping newlines
                ESP_LOGD(TAG, "Extracted HTTP body (%u bytes):", body.size());
                std::string log_safe_body = body;
                for (char &c : log_safe_body)
                {
                    if (c == '\r')
                        c = '\\';
                    else if (c == '\n')
                        c = 'n';
                    else if (c < 32 || c >= 127)
                        c = '?';
                }
                for (size_t i = 0; i < log_safe_body.size(); i += chunk_size)
                {
                    std::string chunk = log_safe_body.substr(i, chunk_size);
                    ESP_LOGD(TAG, "Body chunk [%u-%u]: %s", i, i + chunk.size() - 1, chunk.c_str());
                }

                return true;
            }

            ESP_LOGW(TAG, "Max redirects (%d) reached", max_redirects);
            return false;
        }

        bool NetgearM5Component::extract_http_body_(const std::string &raw, std::string &body_out)
        {
            auto pos = raw.find("\r\n\r\n");
            if (pos == std::string::npos)
                return false;
            body_out.assign(raw.begin() + pos + 4, raw.end());
            return true;
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

            ESP_LOGD(TAG, "Free heap before parsing: %u bytes", esp_get_free_heap_size());
            ArduinoJson::JsonDocument doc;
            ArduinoJson::DeserializationError err = ArduinoJson::deserializeJson(doc, payload);
            if (err)
            {
                ESP_LOGW(TAG, "JSON parse failed in publish_pending_: %s (payload size: %u bytes)", err.c_str(), payload.size());
                return;
            }
            ESP_LOGD(TAG, "Free heap after parsing: %u bytes", esp_get_free_heap_size());
            auto root = doc.as<ArduinoJson::JsonVariantConst>();

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
                    double value = strtod(v.c_str(), &endptr);
                    if (endptr == v.c_str() || *endptr != '\0')
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