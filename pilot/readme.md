# Growa Pilot

ESP32-based control system with MQTT support, multiple I/O management, and OTA updates.

## Main Features

- WiFi management through WiFiManager
- MQTT communication with CRC verification
- Configurable OTA support
- External serial debug (UART2)
- Hardware and software watchdog
- Configurable I/O management
- Anti-overflow pulse counting system

## Hardware Configuration

### ESP32 Pins
- GPIO32: Battery (ADC)
- GPIO33: Relay 5VDC
- GPIO19: Relay 3.3VDC
- GPIO26: Debug RX
- GPIO27: Debug TX
- GPIO17: PULSE_0 (with SN74HC14)
- GPIO16: PULSE_1 (with SN74HC14)
- GPIO36,39,34,35: ADC inputs

### MCP23017 (Address 0x20)
- GPB0: LED Blink
- GPB1-GPB6: Relays 1-6
- GPB7: ESP32 Feedback (GPIO14)
- GPA0: Not connected
- GPA1-GPA6: Relays 7-12
- GPA7: Watchdog

## MQTT Commands

### 1. Board Configuration

#### Timing Readings
```
Topic: IDDevice/config/board-port(timing)-setting
Payload: value
```
Sets the interval between readings in milliseconds

#### Polling Rate
```
Topic: IDDevice/config/board-port(polling)-setting
Payload: value
```
Sets the polling interval in milliseconds

#### Reset
```
Topic: IDDevice/config/board-port(reset)-setting
Payload: 1
```
Restarts the device

#### Restore
```
Topic: IDDevice/config/board-port(restore)-setting
Payload: 1
```
Restores default settings

### 2. I/O Configuration

#### PULSE Configuration
```
Topic: IDDevice/config/PULSE_0-port(0)-setting
Payload: type_enabled_interval
```
Example: `1_1_1000` (input type, enabled, 1000ms polling)

#### ADC Configuration
```
Topic: IDDevice/config/ADC_0-port(0)-setting
Payload: type_enabled_interval
```
Example: `1_1_5000` (input type, enabled, 5000ms polling)

#### MCP23017 Configuration
```
Topic: IDDevice/config/MCP23017-port(0x20)-setting
Payload: type_enabled_interval
```
Example: `2_1_1000` (output type, enabled, 1000ms polling)

### 3. OTA Configuration

#### OTA Enable
```
Topic: IDDevice/config/ota-port(enabled)-setting
Payload: 1/0
```
Enables (1) or disables (0) OTA updates

#### Version URL
```
Topic: IDDevice/config/ota-port(version_url)-setting
Payload: url
```
Sets the version file URL

#### Firmware URL
```
Topic: IDDevice/config/ota-port(firmware_url)-setting
Payload: url
```
Sets the firmware file URL

## Message Format

### Base Structure
```
IDDevice/type/identifier-port(X)-value(Y):CRC
```

### Configuration Parameters
- `type`: 1=input, 2=output
- `enabled`: 0=disabled, 1=enabled
- `interval`: milliseconds between readings (0=no polling)

## Readings

### Board Info
```
Topic: IDDevice/readings/board-port(0x00)-value
Payload: version_battery_rssi_uptime:CRC
```
- version: firmware version
- battery: battery voltage
- rssi: WiFi signal strength
- uptime: running time in seconds

### Pulse Counters
```
Topic: IDDevice/readings/PULSE_0-port(0)-value
Payload: count:CRC
```
Total number of counted pulses

### ADC
```
Topic: IDDevice/readings/ADC_0-port(0)-value
Payload: voltage:CRC
```
Read voltage (0-5V)

### MCP23017
```
Topic: IDDevice/readings/MCP23017-port(0x20)-value
Payload: state:CRC
```
Relay state in binary format

## Important Notes

1. All commands require configuration before use
2. CRC is calculated on the entire message before the colon
3. Hardware watchdog has a 30-second timeout
4. OTA updates are checked every 30 minutes when enabled
5. The system confirms each received configuration
6. In case of OTA error, the system will make a maximum of 3 attempts

## WiFi Reset

To reset the WiFi configuration, hold the BOOT button for 5 seconds. The device will restart and create a new access point named "GROWA PILOT".
