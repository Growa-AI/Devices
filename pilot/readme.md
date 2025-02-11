# Growa Pilot ESP32

## Description
ESP32-based control system with support for multiple I/O devices, MQTT communication, and hardware watchdog functionality.

## Hardware Configuration

### ESP32 Pins
- **Power Control**
  - GPIO32: Battery monitoring
  - GPIO33: 5V relay control
  - GPIO19: 3.3V relay control

- **Debug Interface**
  - GPIO26: Debug RX
  - GPIO27: Debug TX

- **Input Pins**
  - GPIO17: PULSE_0 (with SN74HC14)
  - GPIO16: PULSE_1 (with SN74HC14)
  - GPIO14: MCP23017 feedback monitor

- **ADC Inputs**
  - GPIO36: ADC input 0
  - GPIO39: ADC input 1
  - GPIO34: ADC input 2
  - GPIO35: ADC input 3

### MCP23017 (I2C Address: 0x20)
- **Output Configuration**
  - GPB0: LED Blink
  - GPB1-GPB6: Relays 1-6
  - GPB7: ESP32 Feedback
  - GPA0: Not connected
  - GPA1-GPA6: Relays 7-12
  - GPA7: Watchdog

## MQTT Protocol

### Topics
- `GrowaPilot/readings` - Device readings and status
- `GrowaPilot/config` - Device configuration
- `GrowaPilot/system` - System commands

### Message Examples

#### 1. Device Readings (GrowaPilot/readings)

Board Status:
```json
{
    "device": "BOARD",
    "type": "BOARD",
    "firmware": "1.0.5",
    "battery": "3.85",
    "rssi": "-65",
    "net": "WiFi(MyNetwork)"
}
```

ADC Reading:
```json
{
    "device": "ADC_36",
    "type": "ADC",
    "address": "36",
    "value": "3.45"
}
```

Pulse Counter:
```json
{
    "device": "PULSE_0",
    "type": "PULSE",
    "address": "17",
    "value": "1234"
}
```

MCP23017 State:
```json
{
    "device": "MCP23017",
    "type": "I2C",
    "address": "0x20",
    "value": "000100010000"
}
```

#### 2. Device Configuration (GrowaPilot/config)

MCP23017 Output Control:
```json
{
    "device": "MCP23017",
    "value": "000100010000"
}
```

#### 3. System Commands (GrowaPilot/system)

Reset Device:
```json
{
    "command": "reset"
}
```

Set Reading Interval:
```json
{
    "command": "timing",
    "value": 60000
}
```

Set Polling Interval:
```json
{
    "command": "polling",
    "value": 10000
}
```

## Device Types and Hardware

### ADC Configuration
- Fixed pins: 36, 39, 34, 35
- Auto-initialized at startup
- 12-bit resolution
- 0-3.3V range

### Pulse Counters
- Fixed pins: 17, 16
- Hardware debounce via SN74HC14
- Auto-initialized at startup
- 32-bit counters

### MCP23017
- Fixed I2C address: 0x20
- 12 relay outputs
- Hardware watchdog output
- Hardware feedback monitoring

## System Features

### Power Management
- Controlled 5V and 3.3V power rails
- Power-cycled during readings
- Battery voltage monitoring

### Watchdog System
- Hardware watchdog via MCP23017 GPA7
- Hardware feedback via MCP23017 GPB7 to ESP32 GPIO14
- Configurable polling interval

### Status Indication
- LED blink on MCP23017 GPB0
- LED active only when MCP23017 is responding
- Debug output on dedicated UART

### WiFi Configuration
- WiFiManager for initial setup
- Creates "GROWA PILOT" AP when unconfigured
- Persists WiFi credentials

### MQTT Communication
- QoS 2 for all messages
- Automatic reconnection
- Retained messages for device state

## Default Settings
- Reading Interval: 60 seconds
- Polling Interval: 10 seconds
- JSON message format
- All inputs enabled at startup

## Error Handling
- Hardware watchdog monitoring
- Communication status feedback
- Error reporting via MQTT
- Debug output on serial ports

## Dependencies
- WiFi.h
- PubSubClient.h
- Wire.h
- Adafruit_MCP23X17.h
- WiFiManager.h
- ArduinoJson.h
