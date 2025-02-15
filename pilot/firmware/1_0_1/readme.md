# Firmware 1.0.1

This firmware is designed for an ESP32-based controller that interfaces with various sensors and relays through an MCP23017 I/O expander. It provides MQTT connectivity for remote monitoring and control of agricultural systems.

## Features

- WiFi connectivity with WifiManager for easy configuration
- Secure MQTT communication
- Support for 12 relay outputs via MCP23017
- ADC readings from 4 analog inputs
- Pulse counting from 2 digital inputs
- System monitoring (battery voltage, WiFi signal strength)
- OTA (Over-The-Air) firmware updates
- Watchdog timer for system reliability

## Hardware Requirements

- ESP32 microcontroller
- MCP23017 I/O expander
- 12 relay modules
- Battery monitoring circuit
- 4 analog inputs (0-10V)
- 2 pulse input channels
- 5V and 3.3V power control relays

### Pin Configuration

#### ESP32 Pins
- BATTERY_PIN: 32 (Battery voltage monitoring)
- RELAY_5V_PIN: 33 (5V power control)
- RELAY_3V3_PIN: 19 (3.3V power control)
- MCP_FEEDBACK_PIN: 14
- DEBUG_RX: 26
- DEBUG_TX: 27
- LED_BUILTIN: 2

#### ADC Pins
- ADC1: 36
- ADC2: 39
- ADC3: 34
- ADC4: 35

#### Pulse Input Pins
- PULSE1: 17
- PULSE2: 16

## MQTT Protocol

### Topics

The device subscribes to and publishes on the following topics:
- `{DEVICE_ID}/system` - System commands
- `{DEVICE_ID}/config` - Configuration messages
- `{DEVICE_ID}/readings` - Sensor readings
- `{DEVICE_ID}/status` - Status updates

### Message Formats

#### Relay Control (Config)
Format: `0x30+0+12+{state1}+{state2}+...+{state12}`
- Each state can be 0 (OFF) or 1 (ON)

#### System Readings
Format: `0x00+0+4+{firmware_version}+{battery_voltage}+{rssi}+{wifi_ssid}`

#### ADC Readings
Format: `0x20+0+4+{voltage1}+{voltage2}+{voltage3}+{voltage4}`
- Voltages are in the range 0-10V

#### Pulse Readings
Format: `0x10+0+2+{count1}+{count2}`
- Counts are 32-bit unsigned integers

## OTA Updates

The firmware supports OTA updates via HTTPS. Updates can be triggered by sending an MQTT message to the system topic:
```
{DEVICE_ID}/system: ota:https://raw.githubusercontent.com/path/to/firmware.bin
```

Security restrictions:
- Only accepts URLs from raw.githubusercontent.com
- URL length must be between 30 and 512 characters

## System Commands

Available system commands (via MQTT):
- `reset` - Triggers a system restart
- `ota:{url}` - Initiates OTA update from specified URL

## Development and Building

### Dependencies

Required Arduino libraries:
- WiFi
- WiFiClientSecure
- PubSubClient
- WiFiManager
- DNSServer
- WebServer
- Adafruit_MCP23X17
- esp_task_wdt

### Configuration

Before deploying, configure the following parameters:
```cpp
const char* mqtt_server = "mqtt.growa.ai";
const int mqtt_port = 8883;
const char* mqtt_user = "";
const char* mqtt_password = "";
#define DEVICE_ID "0001"
#define FIRMWARE_VERSION "P.1.0.1"
```

## Safety Features

- Watchdog timer (60-second timeout)
- Automatic reconnection to WiFi and MQTT
- Power control for sensors via dedicated relays
- Error reporting via MQTT status messages

## Maintenance

The device performs the following periodic tasks:
- System readings every 30 seconds
- MCP23017 status checks
- Watchdog timer reset every 30 seconds
- LED status indication via MCP23017 pins

## Troubleshooting

Common status messages:
- "Failed to connect to update server" - OTA update server unreachable
- "Invalid URL - must be raw.githubusercontent.com" - Incorrect OTA URL
- "OTA update failed" - Update process failed
- "Error initializing MCP23017" - I2C communication issue

For more detailed diagnostics, monitor the serial output at 115200 baud.
