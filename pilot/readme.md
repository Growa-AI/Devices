# Growa Pilot MQTT Protocol Documentation

## Description
Growa Pilot is an ESP32-based control system that manages multiple digital and analog I/O through MQTT communication. The system supports multiple devices, including:
- Pulse counters for flow measurements
- ADC inputs for analog readings
- MCP23017 I/O expanders for digital I/O control
- Network connectivity via WiFi, LoRaWAN, or GSM

## Hardware Configuration

### ESP32 Pins
- ADC inputs: GPIO36, GPIO39, GPIO34, GPIO35
- Pulse inputs with Schmitt trigger (SN74HC14): GPIO17, GPIO16
- Battery monitoring: GPIO32
- Internal power control: GPIO33 (5V), GPIO19 (3.3V)
- Debug serial: GPIO26 (RX), GPIO27 (TX)

### MCP23017 (Address 0x20)
- GPB0: LED Blink
- GPB1-GPB6: Relays 1-6
- GPB7: ESP32 Feedback
- GPA0: Not connected
- GPA1-GPA6: Relays 7-12
- GPA7: Watchdog

## MQTT Protocol Structure

### Topic Categories
- `GrowaPilot/readings` - Device readings and status information
- `GrowaPilot/config` - Device configuration settings
- `GrowaPilot/system` - System-level commands and control

## 1. Readings Messages (GrowaPilot/readings)

### Board Status
Reports system status including firmware version, battery level, and network connection.
```json
Topic: GrowaPilot/readings
Payload:
{
    "device": "BOARD",
    "address": "0x00",
    "firmware": "1.0.5",
    "battery": "3.85",
    "rssi": "-65",
    "net": "WiFi(MyNetwork)"  // or "LoRaWAN(DevEUI)" or "Sim(ID SIM)"
}
```

### Pulse Counter Reading
Reports pulse count from flow meters or similar devices.
```json
Topic: GrowaPilot/readings
Payload:
{
    "device": "PULSE_0",
    "port": "19",
    "address": "0x40",
    "value": "1234"
}
```

### ADC Reading
Reports voltage readings from analog inputs.
```json
Topic: GrowaPilot/readings
Payload:
{
    "device": "ADC_1",
    "port": "36",
    "address": "0x30",
    "value": "4.17"
}
```

### MCP23017 Status
Reports the state of digital I/O expander outputs.
```json
Topic: GrowaPilot/readings
Payload:
{
    "device": "MCP23017",
    "address": "0x20",
    "value": "010000000100"
}
```

### Error State
Reports device errors and failures.
```json
Topic: GrowaPilot/readings
Payload:
{
    "device": "ADC_0",
    "address": "0x30",
    "error": {
        "code": 1,
        "message": "Device not responding",
        "details": "Timeout after 3 attempts"
    }
}
```

## 2. Configuration Messages (GrowaPilot/config)

### Configure Pulse Counter
Sets up pulse counter parameters.
```json
Topic: GrowaPilot/config
Payload:
{
    "device": "PULSE_0",
    "port": "19",
    "address": "0x40",
    "type": 1,
    "enabled": true
}
```

### Configure ADC
Sets up analog input parameters.
```json
Topic: GrowaPilot/config
Payload:
{
    "device": "ADC_0",
    "port": "36",
    "address": "0x30",
    "type": 1,
    "enabled": true
}
```

### Configure MCP23017
Sets up digital outputs and their states.
```json
Topic: GrowaPilot/config
Payload:
{
    "device": "MCP23017",
    "address": "0x20",
    "type": 2,
    "enabled": true,
    "value": "010000000100"
}
```

## 3. System Messages (GrowaPilot/system)

### Reset Command
Restarts the device or resets to factory settings.
```json
Topic: GrowaPilot/system
Payload:
{
    "command": "reset"
}
```

### Firmware Update
Initiates OTA firmware update.
```json
Topic: GrowaPilot/system
Payload:
{
    "command": "update",
    "version": "1.0.6",
    "url": "https://ota.growa.ai/firmware.bin"
}
```

### Set Reading Timing
Sets the interval for sensor readings (in milliseconds).
```json
Topic: GrowaPilot/system
Payload:
{
    "command": "timing",
    "value": "60000"
}
```

### Set Polling Interval
Sets the watchdog polling interval (in milliseconds).
```json
Topic: GrowaPilot/system
Payload:
{
    "command": "polling",
    "value": "10000"
}
```

## Technical Specifications

### Device Types
- Type 1: Input devices (ADC, PULSE)
- Type 2: Output devices (MCP23017)

### Address Ranges
- Board: 0x00
- MCP23017: 0x20-0x27 (up to 8 devices)
- ADC: 0x30-0x37 (up to 8 channels)
- PULSE: 0x40-0x47 (up to 8 counters)

### MQTT Details
- QoS Level: 2 (Exactly once delivery)
- Clean Session: True
- Keep Alive: 60 seconds
- Will Topic: GrowaPilot/readings (Last Will and Testament)

## Important Notes
1. All values in payloads are strings for consistency
2. Device addresses must be within specified ranges
3. Port numbers correspond to physical GPIO pins
4. MCP23017 value is a 12-character binary string representing output states
5. All messages use QoS 2 for guaranteed delivery
6. Network connectivity is automatically managed and reported
7. Device requires initial configuration before operation
8. Watchdog ensures system reliability

## Error Codes
- 1: Device not responding
- 2: Communication error
- 3: Value out of range
- 4: Device not configured
- 5: Address conflict
