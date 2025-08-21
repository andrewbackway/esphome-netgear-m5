# Netgear M5 ESPHome Component

This project provides an ESPHome component for integrating with Netgear M5 routers, enabling monitoring of various router metrics through sensors in an ESPHome environment. The component fetches data from the router's API, processes it, and exposes it via numeric, text, and binary sensors.

## Features

- **Customizable Sensors**: Supports numeric, text, and binary sensors to monitor various router parameters.
- **Configurable Polling**: Adjustable poll interval to control how frequently data is fetched from the router.
- **Secure Communication**: Handles authentication with the router using a provided password and manages session cookies.
- **JSON Path Navigation**: Uses a dotted notation to extract specific values from the router's JSON API responses.
- **ESPHome Integration**: Seamlessly integrates with ESPHome's sensor, text_sensor, and binary_sensor components.

## Requirements

- ESPHome (version compatible with the provided code)
- ArduinoJson library for JSON parsing
- ESP32 or compatible microcontroller with network connectivity
- Netgear M5 router with accessible API endpoint

## Configuration

Add the following to your ESPHome YAML configuration file:

```yaml

external_components:
 - source: github://andrewbackway/esphome-netgear-m5

netgear-m5:
  host: 10.0.1.1 # Replace with router IP
  password: Password # Replace with router admin password
  poll_interval: 10s

  sensors: 
  - name: Battery Level
    path: power.battChargeLevel
    unit_of_measurement: "%"
  - name: Device Temperature
    path: general.devTemperature
    unit_of_measurement: "°C"
  - name: RSSI
    path: wwan.signalStrength.rssi
    unit_of_measurement: "dBm"
  - name: RSRP
    path: wwan.signalStrength.rsrp
    unit_of_measurement: "dBm"
  - name: RSRQ
    path: wwan.signalStrength.rsrq
    unit_of_measurement: "dB"
  - name: WiFi Clients
    path: wifi.clientCount
  - name: Cell Tower ID
    path: wwanadv.cellId
  - name: Bars
    path: wwan.signalStrength.bars

  text_sensors: 
  - name: Device Name
    path: general.deviceName
  - name: Mobile Carrier
    path: wwan.registerNetworkDisplay 
  - name: Mobile Band
    path: wwan.connectionText    
  - name: Mobile Status
    path: wwan.connection
  - name: Offload WiFi SSID
    path: wifi.offload.connectionSsid
  - name: Offload Scan Progress
    path: wifi.offload.scanProgress

  binary_sensors:
    - name: Offloading via WiFi
      path: wifi.offload.status
      on_value: "On"
      off_value: "Off"
```

### Configuration Parameters

- `host` (Required): The IP address or hostname of the Netgear M5 router.
- `password` (Required): The admin password for the router.
- `poll_interval` (Optional): Time interval between API polls (default: 30 seconds).
- `sensors` (Optional): List of numeric sensors with their JSON paths and configurations.
- `text_sensors` (Optional): List of text sensors with their JSON paths.
- `binary_sensors` (Optional): List of binary sensors with JSON paths and on/off values.

## Usage

1. **Define Sensors**: Specify the JSON paths for the data you want to monitor. For example, `system.uptime` or `system.firmware_version`.
2. **Monitor Data**: Once configured and uploaded, the component will periodically fetch data from the router and publish it to the defined sensors.
3. **Home Assistant Integration**: If using Home Assistant, the sensors will automatically appear in your dashboard after ESPHome integration.

## Example JSON Path

The component uses a dotted notation to navigate the JSON response from the router's API. For example, if the API returns:

```json
{
  "system": {
    "uptime": 3600,
    "firmware_version": "1.0.0"
  }
}
```

You can access the uptime with the path `system.uptime` and the firmware version with `system.firmware_version`.

## Debugging

- **Logs**: Enable debug logging in ESPHome to see detailed information about HTTP requests, JSON parsing, and sensor updates.
- **Common Issues**:
  - Ensure the router's IP and password are correct.
  - Verify network connectivity between the ESP32 and the router.
  - Check the JSON paths match the router's API response structure.
