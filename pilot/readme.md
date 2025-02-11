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

## Command Classification

## 1. System Commands (Address: 0x00)

### System Configuration
```
Topic: IDDevice/system/[command]-value(Y):CRC
```

Available system commands:
| Command    | Description              | Value Format        | Example               |
|------------|-------------------------|--------------------|-----------------------|
| name       | Device name             | string            | "Pilot_123"           |
| network    | Network settings        | ssid_pass_mode    | "MyWifi_pass_0"      |
| mqtt       | MQTT configuration      | server_port_user_pass | "mqtt.growa.ai_1883_user_pass" |
| log        | Logging level           | level             | "debug"              |
| time       | Time synchronization    | timestamp         | "1634567890"         |
| backup     | Backup settings         | trigger           | "1"                  |
| restore    | Restore settings        | trigger           | "1"                  |
| reset      | Reset device            | type              | "factory"            |

Examples:

Set device name:
```
Topic: IDDevice/system/name-value
Payload: Pilot_Lab1:XXXX
```

Configure MQTT:
```
Topic: IDDevice/system/mqtt-value
Payload: mqtt.growa.ai_1883_mqttroot_k<V<k3:n73mQCF7:XXXX
```

Reset device:
```
Topic: IDDevice/system/reset-value
Payload: factory:XXXX
```

## 2. I/O Commands (Address: 0x20-0x47)

### Input Configuration
```
Topic: IDDevice/config/[device]-port(address:port)-setting(Y):CRC
```
[Previous input examples...]

### Output Configuration
```
Topic: IDDevice/config/[device]-port(address:port)-setting(Y):CRC
```
[Previous output examples...]

## 3. Firmware Commands (Address: 0x00)

### OTA Configuration
```
Topic: IDDevice/firmware/[command]-value(Y):CRC
```

Available firmware commands:
| Command    | Description              | Value Format        | Example               |
|------------|-------------------------|--------------------|-----------------------|
| check      | Check for updates       | trigger           | "1"                  |
| url        | Update server URL       | string            | "https://ota.growa.ai" |
| version    | Set version info        | string            | "1.0.5"              |
| update     | Start update            | trigger           | "1"                  |
| rollback   | Rollback firmware       | trigger           | "1"                  |

Examples:

Set OTA server:
```
Topic: IDDevice/firmware/url-value
Payload: https://ota.growa.ai:XXXX
```

Start update:
```
Topic: IDDevice/firmware/update-value
Payload: 1:XXXX
```

## 1. Device Address Registry

### System Addresses (0x00-0x0F)
| Address | Description              | Note                      |
|---------|-------------------------|---------------------------|
| 0x00    | Board                   | System control            |
| 0x01    | Reserved               | Future system expansion   |

### I/O Expander Addresses (0x20-0x27)
| Address | Description              | Note                      |
|---------|-------------------------|---------------------------|
| 0x20    | MCP23017 #1            | Default I/O expander      |
| 0x21    | MCP23017 #2            | Secondary I/O expander    |
| 0x22    | MCP23017 #3            | Available                 |
| ...     | ...                     | ...                       |
| 0x27    | MCP23017 #8            | Maximum address           |

### ADC Addresses (0x30-0x37)
| Address | Description              | Note                      |
|---------|-------------------------|---------------------------|
| 0x30    | ADC Module #1           | Primary ADC              |
| 0x31    | ADC Module #2           | Secondary ADC            |
| ...     | ...                     | ...                       |
| 0x37    | ADC Module #8           | Maximum address          |

### Pulse Counter Addresses (0x40-0x47)
| Address | Description              | Note                      |
|---------|-------------------------|---------------------------|
| 0x40    | Pulse Counter #1        | Primary counter          |
| 0x41    | Pulse Counter #2        | Secondary counter        |
| ...     | ...                     | ...                       |
| 0x47    | Pulse Counter #8        | Maximum address          |

## 2. Port Registry

## 1. Board Ports (0x00-0x0F)
| Port   | Description              | Value Format               | Example                |
|--------|-------------------------|---------------------------|------------------------|
| 0x00   | Board Info             | ver_bat_rssi_uptime      | 1.0_3.25_-72_3600     |
| 0x01   | Timing                 | milliseconds              | 60000                 |
| 0x02   | Polling                | milliseconds              | 1000                  |
| 0x03   | OTA Control            | enabled_version_firmware  | 1_url1_url2           |
| 0x04   | Reset                  | trigger                   | 1                     |
| 0x05   | Restore               | trigger                   | 1                     |

## 2. Digital Inputs (0x10-0x1F)
| Port   | Description              | Value Format               | Example                |
|--------|-------------------------|---------------------------|------------------------|
| 0x10   | PULSE_0                | counts                    | 1234                  |
| 0x11   | PULSE_1                | counts                    | 5678                  |

## 3. Analog Inputs (0x20-0x2F)
| Port   | Description              | Value Format               | Example                |
|--------|-------------------------|---------------------------|------------------------|
| 0x20   | ADC_0 (GPIO36)         | voltage                   | 3.45                  |
| 0x21   | ADC_1 (GPIO39)         | voltage                   | 2.78                  |
| 0x22   | ADC_2 (GPIO34)         | voltage                   | 1.92                  |
| 0x23   | ADC_3 (GPIO35)         | voltage                   | 4.12                  |

## 4. Digital Outputs (0x30-0x3F)
| Port   | Description              | Value Format               | Example                |
|--------|-------------------------|---------------------------|------------------------|
| 0x30   | MCP23017 Relays        | binary state              | 000100010000          |
| 0x31   | ESP32 GPIO33 (5V)      | binary state              | 1                     |
| 0x32   | ESP32 GPIO19 (3.3V)    | binary state              | 0                     |

## Readings Format and Examples

### 1. Board Readings (Address: 0x00)
```
Topic: IDDevice/readings/board-port(0x00:0x00)-value
Payload: 1.0_3.25_-72_3600:XXXX
```
Contains:
- Firmware version: 1.0
- Battery voltage: 3.25V
- WiFi RSSI: -72dB
- Uptime: 3600 seconds

### 2. Pulse Counter Readings (Address: 0x40-0x47)

Primary Pulse Counter (0x40):
```
Topic: IDDevice/readings/PULSE_0-port(0x40:0x00)-value
Payload: 1234:XXXX
```

Secondary Pulse Counter (0x41):
```
Topic: IDDevice/readings/PULSE_1-port(0x41:0x00)-value
Payload: 5678:XXXX
```

### 3. ADC Readings (Address: 0x30-0x37)

Primary ADC Module (0x30):
```
Topic: IDDevice/readings/ADC_0-port(0x30:0x00)-value
Payload: 3.45:XXXX

Topic: IDDevice/readings/ADC_1-port(0x30:0x01)-value
Payload: 2.78:XXXX
```

Secondary ADC Module (0x31):
```
Topic: IDDevice/readings/ADC_0-port(0x31:0x00)-value
Payload: 1.92:XXXX

Topic: IDDevice/readings/ADC_1-port(0x31:0x01)-value
Payload: 4.12:XXXX
```

### 4. MCP23017 Readings (Address: 0x20-0x27)

Primary MCP23017 (0x20):
```
Topic: IDDevice/readings/MCP23017-port(0x20:0x00)-value
Payload: 000100010000:XXXX
```

Secondary MCP23017 (0x21):
```
Topic: IDDevice/readings/MCP23017-port(0x21:0x00)-value
Payload: 000000110000:XXXX
```

### Reading Intervals

1. Default intervals by device type:
   - Board info: 60 seconds
   - Pulse counters: 1 second
   - ADC: 5 seconds
   - MCP23017: 1 second

2. Custom intervals can be set via configuration:
```
Topic: IDDevice/config/[device]-port(address:port)-setting
Payload: type_enabled_interval
```

### Error Readings

If a reading fails or device is not responding:
```
Topic: IDDevice/readings/[device]-port(address:port)-error
Payload: error_code:XXXX
```

Error codes:
- 0x01: Device not responding
- 0x02: Communication error
- 0x03: Value out of range
- 0x04: Device not configured
- 0x05: Address conflict

### Status Monitoring

Device status messages:
```
Topic: IDDevice/status/[device]-port(address:port)
Payload: status_code:XXXX
```

Status codes:
- 0x00: OK
- 0x01: Warning
- 0x02: Error
- 0x03: Critical

Example status message:
```
Topic: IDDevice/status/MCP23017-port(0x20:0x00)
Payload: 0x01_Low voltage detected:XXXX
```

### Configuration Message
```
IDDevice/config/[device]-port(address:port)-setting(Y):CRC
```
Where:
- address = Device address in hex (e.g., 0x20)
- port = Port number in hex (e.g., 0x01)
- Y = Configuration value

### Reading Message
```
IDDevice/readings/[device]-port(address:port)-value(Y):CRC
```

## Examples with Multiple Devices

### Configuring Multiple MCP23017
```
# Configure first MCP23017 (0x20)
Topic: IDDevice/config/MCP23017-port(0x20:0x01)-setting
Payload: 2_1_1000

# Configure second MCP23017 (0x21)
Topic: IDDevice/config/MCP23017-port(0x21:0x01)-setting
Payload: 2_1_1000
```

### Reading from Multiple ADCs
```
# Read from first ADC module (0x30)
Topic: IDDevice/readings/ADC-port(0x30:0x01)-value
Payload: 3.45:XXXX

# Read from second ADC module (0x31)
Topic: IDDevice/readings/ADC-port(0x31:0x01)-value
Payload: 2.78:XXXX
```

### Configuration with Address Inheritance
When address is omitted, device uses default address:
```
# These are equivalent for first MCP23017
IDDevice/config/MCP23017-port(0x20:0x01)-setting
IDDevice/config/MCP23017-port(0x01)-setting  # Uses default 0x20
```

### Configuration Message
```
IDDevice/config/[device]-port(X)-setting(Y):CRC
```
Where:
- X = Port number in hex (e.g., 0x20)
- Y = Configuration value

### Reading Message
```
IDDevice/readings/[device]-port(X)-value(Y):CRC
```
Where:
- X = Port number in hex
- Y = Current value

## Port Type Conventions

### Board Ports (0x00-0x0F)
- Reserved for system configuration
- No type/enabled settings required
- Direct value setting

### Input Ports (0x10-0x2F)
Configuration format: `type_enabled_interval`
- type = 1 (input)
- enabled = 0/1
- interval = polling interval in ms

Example:
```
IDDevice/config/PULSE_0-port(0x10)-setting(1_1_1000):CRC
```

### Output Ports (0x30-0x3F)
Configuration format: `type_enabled_interval`
- type = 2 (output)
- enabled = 0/1
- interval = update interval in ms

Example:
```
IDDevice/config/MCP23017-port(0x30)-setting(2_1_1000):CRC
```


### 1. Board Configuration

#### Timing Readings
```
Topic: IDDevice/config/board-port(timing)-setting
Payload: 60000
CRC calculated: XXXX
Complete message: IDDevice/config/board-port(timing)-setting(60000):XXXX
```
Sets readings every 60 seconds (60000ms)

Response from device:
```
Topic: IDDevice/config/board-port(timing)-confirm
Payload: 60000
```

#### Polling Rate
```
Topic: IDDevice/config/board-port(polling)-setting
Payload: 1000
Complete message: IDDevice/config/board-port(polling)-setting(1000):XXXX
```
Sets polling every 1 second (1000ms)

Response:
```
Topic: IDDevice/config/board-port(polling)-confirm
Payload: 1000
```

#### Reset Command
```
Topic: IDDevice/config/board-port(reset)-setting
Payload: 1
Complete message: IDDevice/config/board-port(reset)-setting(1):XXXX
```
Device will restart after receiving this command

#### Restore Command
```
Topic: IDDevice/config/board-port(restore)-setting
Payload: 1
Complete message: IDDevice/config/board-port(restore)-setting(1):XXXX
```
Restores default settings

### 2. I/O Configuration

#### PULSE Configuration Examples

Configure PULSE_0:
```
Topic: IDDevice/config/PULSE_0-port(0)-setting
Payload: 1_1_1000
Complete message: IDDevice/config/PULSE_0-port(0)-setting(1_1_1000):XXXX
```
- 1: input type
- 1: enabled
- 1000: read every 1000ms

Response:
```
Topic: IDDevice/config/PULSE_0-port(0)-confirm
Payload: 1_1_1000
```

Reading example:
```
Topic: IDDevice/readings/PULSE_0-port(0)-value
Payload: 1234:XXXX
```
Shows 1234 pulses counted

Disable PULSE_1:
```
Topic: IDDevice/config/PULSE_1-port(0)-setting
Payload: 1_0_0
Complete message: IDDevice/config/PULSE_1-port(0)-setting(1_0_0):XXXX
```

#### ADC Configuration Examples

Configure ADC_0:
```
Topic: IDDevice/config/ADC_0-port(0)-setting
Payload: 1_1_5000
Complete message: IDDevice/config/ADC_0-port(0)-setting(1_1_5000):XXXX
```
- 1: input type
- 1: enabled
- 5000: read every 5 seconds

Reading example:
```
Topic: IDDevice/readings/ADC_0-port(0)-value
Payload: 3.45:XXXX
```
Shows 3.45V on ADC0

Configure all ADC channels:
```
ADC_0: IDDevice/config/ADC_0-port(0)-setting(1_1_5000):XXXX
ADC_1: IDDevice/config/ADC_1-port(0)-setting(1_1_5000):XXXX
ADC_2: IDDevice/config/ADC_2-port(0)-setting(1_1_5000):XXXX
ADC_3: IDDevice/config/ADC_3-port(0)-setting(1_1_5000):XXXX
```

#### MCP23017 Configuration Examples

Enable MCP23017:
```
Topic: IDDevice/config/MCP23017-port(0x20)-setting
Payload: 2_1_1000
Complete message: IDDevice/config/MCP23017-port(0x20)-setting(2_1_1000):XXXX
```
- 2: output type
- 1: enabled
- 1000: update every 1000ms

Set relay states:
```
Topic: IDDevice/config/MCP23017-port(0x20)-setting
Payload: 000100010000
Complete message: IDDevice/config/MCP23017-port(0x20)-setting(000100010000):XXXX
```
Activates relays 1 and 5

Reading example:
```
Topic: IDDevice/readings/MCP23017-port(0x20)-value
Payload: 000100010000:XXXX
```
Shows current relay states

### 3. OTA Configuration Examples

Enable OTA:
```
Topic: IDDevice/config/ota-port(enabled)-setting
Payload: 1
Complete message: IDDevice/config/ota-port(enabled)-setting(1):XXXX
```

Set version URL:
```
Topic: IDDevice/config/ota-port(version_url)-setting
Payload: https://raw.githubusercontent.com/Growa-AI/OTA/main/firmware/version.txt
```

Set firmware URL:
```
Topic: IDDevice/config/ota-port(firmware_url)-setting
Payload: https://raw.githubusercontent.com/Growa-AI/OTA/main/firmware/firmware.ino.bin
```

OTA Status Messages:
```
Topic: IDDevice/status/ota
Possible payloads:
- "Starting OTA Update Process"
- "Downloading firmware version 1.0.5"
- "Update successful, restarting"
- "Update failed: connection error"
```

### 4. Board Info Reading Example
```
Topic: IDDevice/readings/board-port(0x00)-value
Payload: 1.0_3.25_-72_3600:XXXX
```
Indicates:
- Firmware version: 1.0
- Battery voltage: 3.25V
- WiFi RSSI: -72dB
- Uptime: 3600 seconds (1 hour)

## Message Format

### Base Structure
```
IDDevice/type/identifier-port(X)-value(Y):CRC
```

### Configuration Parameters
- `type`: 1=input, 2=output
- `enabled`: 0=disabled, 1=enabled
- `interval`: milliseconds between readings (0=no polling)

## Important Notes

1. All commands require configuration before use
2. CRC is calculated on the entire message before the colon
3. Hardware watchdog has a 30-second timeout
4. OTA updates are checked every 30 minutes when enabled
5. The system confirms each received configuration
6. In case of OTA error, the system will make a maximum of 3 attempts

## WiFi Reset

To reset the WiFi configuration, hold the BOOT button for 5 seconds. The device will restart and create a new access point named "GROWA PILOT".
