/**
 * Growa Pilot - ESP32 Control System
 * Version 1.0.5
 * Part 1: Includes and Global Definitions
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <driver/adc.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <map>

// Device identification
#define DEVICE_NAME "GrowaPilot"
#define FIRMWARE_VERSION "1.0.5"

// Watchdog configuration
#define WDT_TIMEOUT_SECONDS 30   // 30 seconds watchdog timeout

// MQTT Configuration
const char* mqtt_server = "mqtt.growa.ai";
const int mqtt_port = 1883;
const char* mqtt_user = "mqttroot";
const char* mqtt_password = "k<V<k3:n73mQCF7";

// GPIO Pin Definitions
#define BATTERY_PIN 32
#define RELAY_5V_PIN 33
#define RELAY_3V3_PIN 19
#define MCP_FEEDBACK_PIN 14
#define DEBUG_RX 26
#define DEBUG_TX 27

// Hardware definitions
const uint8_t MCP23017_ADDRESS = 0x20;
const uint8_t ADC_PINS[] = {36, 39, 34, 35};    // ADC input pins
const uint8_t PULSE_PINS[] = {17, 16};          // Pulse counter pins

// MCP23017 Pin Definitions
#define LED_BLINK_PIN 8
#define MCP_FEEDBACK_GPB7 7
#define MCP_WATCHDOG_GPA7 15

// Device types
enum DeviceType {
    TYPE_ADC,
    TYPE_PULSE,
    TYPE_I2C,
    TYPE_BOARD
};

// Device status structure
struct DeviceStatus {
    bool configured = false;
    bool enabled = false;
    bool initialized = false;
    DeviceType type = TYPE_BOARD;
    String value = "";
    uint32_t lastReading = 0;
};

// Global Objects
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Adafruit_MCP23X17 mcp;
HardwareSerial SerialExt(2);

// Device management - moved to global scope to resolve scoping issues
std::map<String, DeviceStatus> devices;

// Timing variables
uint32_t readingInterval = 60000;    // Default 60s
uint32_t pollingInterval = 10000;    // Default 10s
uint32_t lastReading = 0;
uint32_t lastWatchdogToggle = 0;
bool watchdogState = false;
bool mcpResponding = false;

// Pulse counter variables
volatile uint32_t pulseCount0 = 0;
volatile uint32_t pulseCount1 = 0;

// Forward declarations
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void connectMQTT();
void readSensors();
void handleMCPWatchdog();
void initializeDeviceMaps();

// Debug Functions
void debug_print(const char* message) {
    Serial.print(message);
    SerialExt.print(message);
}

void debug_println(const char* message) {
    Serial.println(message);
    SerialExt.println(message);
}

// Interrupt Handlers
void IRAM_ATTR pulse0_isr() {
    pulseCount0++;
}

void IRAM_ATTR pulse1_isr() {
    pulseCount1++;
}

/**
 * Initialize MCP23017
 * All pins configured as outputs
 */
bool initializeMCP23017(const String& deviceName) {
    if (!mcp.begin_I2C(MCP23017_ADDRESS)) {
        debug_println("MCP23017 initialization failed");
        return false;
    }
    
    pinMode(MCP_FEEDBACK_PIN, INPUT);

    // Configure all MCP pins as outputs
    for(uint8_t i = 0; i < 16; i++) {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }
    
    devices[deviceName].initialized = true;
    debug_println("MCP23017 initialized successfully");
    return true;
}

/**
 * Initialize ADC
 */
bool initializeADC(const String& deviceName) {
    int pin = deviceName.substring(4).toInt();
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    adc1_channel_t channel;
    switch(pin) {
        case 36: channel = ADC1_CHANNEL_0; break;
        case 39: channel = ADC1_CHANNEL_3; break;
        case 34: channel = ADC1_CHANNEL_6; break;
        case 35: channel = ADC1_CHANNEL_7; break;
        default: return false;
    }
    
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
    devices[deviceName].initialized = true;
    return true;
}

/**
 * Initialize Pulse Counter
 */
bool initializePulseCounter(const String& deviceName) {
    if(deviceName == "PULSE_0") {
        pinMode(PULSE_PINS[0], INPUT);
        attachInterrupt(digitalPinToInterrupt(PULSE_PINS[0]), pulse0_isr, FALLING);
    }
    else if(deviceName == "PULSE_1") {
        pinMode(PULSE_PINS[1], INPUT);
        attachInterrupt(digitalPinToInterrupt(PULSE_PINS[1]), pulse1_isr, FALLING);
    }
    else {
        return false;
    }
    
    devices[deviceName].initialized = true;
    return true;
}

/**
 * Initialize all devices
 */
void initializeDeviceMaps() {
    // Initialize Board
    DeviceStatus boardStatus;
    boardStatus.enabled = true;
    boardStatus.configured = true;
    boardStatus.initialized = true;
    boardStatus.type = TYPE_BOARD;
    devices["BOARD"] = boardStatus;
    
    // Initialize ADC channels
    for(uint8_t i = 0; i < sizeof(ADC_PINS); i++) {
        String deviceName = "ADC_" + String(ADC_PINS[i]);
        DeviceStatus adcStatus;
        adcStatus.enabled = true;
        adcStatus.configured = true;
        adcStatus.type = TYPE_ADC;
        devices[deviceName] = adcStatus;
        initializeADC(deviceName);
    }
    
    // Initialize Pulse counters
    for(uint8_t i = 0; i < sizeof(PULSE_PINS); i++) {
        String deviceName = "PULSE_" + String(i);
        DeviceStatus pulseStatus;
        pulseStatus.enabled = true;
        pulseStatus.configured = true;
        pulseStatus.type = TYPE_PULSE;
        devices[deviceName] = pulseStatus;
        initializePulseCounter(deviceName);
    }
    
    // Initialize MCP23017
    DeviceStatus mcpStatus;
    mcpStatus.enabled = true;
    mcpStatus.configured = true;
    mcpStatus.type = TYPE_I2C;
    devices["MCP23017"] = mcpStatus;
    initializeMCP23017("MCP23017");
}

// Separate battery voltage reading function
float readBatteryVoltage() {
    return analogRead(BATTERY_PIN) * (20.0 / 4095.0);  // Scale to 0-20V range
}

/**
 * Growa Pilot - ESP32 Control System
 * Version 1.0.5
 * Part 2: Core Functionality and Main Program Flow
 */

// This file should be included after part1 with all previous definitions

/**
 * Handle MCP23017 watchdog and feedback
 */
void handleMCPWatchdog() {
    if (millis() - lastWatchdogToggle >= pollingInterval) {
        watchdogState = !watchdogState;
        
        if(devices["MCP23017"].enabled) {
            mcp.digitalWrite(MCP_FEEDBACK_GPB7, watchdogState);
            mcp.digitalWrite(MCP_WATCHDOG_GPA7, watchdogState);
            
            delay(1);
            bool feedback = digitalRead(MCP_FEEDBACK_PIN);
            if(feedback != watchdogState) {
                mcpResponding = false;
                debug_println("MCP feedback mismatch");
            } else {
                mcpResponding = true;
            }
        }
        
        lastWatchdogToggle = millis();
    }
}

/**
 * Publish device reading
 */
void publishReading(const String& device) {
    if (!devices[device].enabled) return;
    
    JsonDocument doc;
    doc["device"] = device;
    doc["type"] = devices[device].type;
    
    switch(devices[device].type) {
        case TYPE_ADC: {
            int pin = device.substring(4).toInt();
            float voltage = analogRead(pin) * 3.3 / 4095.0;
            doc["address"] = String(pin);
            doc["value"] = String(voltage, 2);
            break;
        }
        case TYPE_PULSE: {
            int pulseIndex = device.substring(6).toInt();
            uint32_t count = (pulseIndex == 0) ? pulseCount0 : pulseCount1;
            doc["address"] = String(PULSE_PINS[pulseIndex]);
            doc["value"] = String(count);
            break;
        }
        case TYPE_I2C: {
            if (device == "MCP23017") {
                uint16_t state = mcp.readGPIOAB();
                char stateStr[13];
                for (int i = 0; i < 12; i++) {
                    stateStr[i] = (state & (1 << i)) ? '1' : '0';
                }
                stateStr[12] = '\0';
                doc["address"] = "0x20";
                doc["value"] = stateStr;
            }
            break;
        }
        case TYPE_BOARD: {
            float battery = readBatteryVoltage();
            doc["firmware"] = FIRMWARE_VERSION;
            doc["battery"] = String(battery, 2);
            doc["rssi"] = String(WiFi.RSSI());
            doc["net"] = "WiFi(" + WiFi.SSID() + ")";
            break;
        }
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    mqttClient.publish("GrowaPilot/readings", jsonString.c_str(), true);
}

/**
 * Read and publish all sensors
 */
void readSensors() {
    publishReading("BOARD");
    
    for(uint8_t pin : ADC_PINS) {
        publishReading("ADC_" + String(pin));
    }
    
    publishReading("PULSE_0");
    publishReading("PULSE_1");
    
    publishReading("MCP23017");
}

/**
 * Handle MQTT messages
 */
void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String topicStr = String(topic);
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        debug_println("JSON parsing failed");
        return;
    }
    
    if (topicStr.startsWith("GrowaPilot/system")) {
        String command = doc["command"];
        
        if (command == "reset") {
            ESP.restart();
        }
        else if (command == "timing") {
            readingInterval = doc["value"].as<uint32_t>();
        }
        else if (command == "polling") {
            pollingInterval = doc["value"].as<uint32_t>();
        }
    }
    else if (topicStr.startsWith("GrowaPilot/config")) {
        if (doc["device"] == "MCP23017" && doc.containsKey("value")) {
            String value = doc["value"];
            uint16_t outputs = 0;
            
            for (int i = 0; i < 12; i++) {
                if (value[i] == '1') {
                    outputs |= (1 << i);
                }
            }
            
            uint16_t currentState = mcp.readGPIOAB();
            outputs |= (currentState & 0xC000);
            
            mcp.writeGPIOAB(outputs);
            debug_println("Updated relay states");
        }
    }
}

/**
 * Connect to MQTT broker
 */
void connectMQTT() {
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 3) {  // Limit connection attempts
        debug_print("Connecting to MQTT...");
        if (mqttClient.connect(DEVICE_NAME, mqtt_user, mqtt_password)) {
            debug_println("connected");
            mqttClient.subscribe("GrowaPilot/config/#");
            mqttClient.subscribe("GrowaPilot/system/#");
            break;
        } else {
            debug_println("failed, retrying in 5s");
            delay(5000);
            attempts++;
        }
        esp_task_wdt_reset();  // Reset watchdog during connection attempts
    }
}

/**
 * Comprehensive Setup Function
 */
void setup() {
    // Initialize serial communications
    Serial.begin(115200);
    SerialExt.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX);
    
    debug_println("Growa Pilot starting...");
    
    // Configure ESP32 watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);                // Add current task to watchdog
    
    pinMode(RELAY_5V_PIN, OUTPUT);
    pinMode(RELAY_3V3_PIN, OUTPUT);
    digitalWrite(RELAY_5V_PIN, LOW);
    digitalWrite(RELAY_3V3_PIN, LOW);
    
    // WiFiManager configuration
    wifiManager.setConfigPortalTimeout(180);  // 3 minutes timeout for config portal
    wifiManager.autoConnect("GROWA PILOT");
    
    esp_task_wdt_reset();  // Reset watchdog after WiFi connection
    
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(handleMqttMessage);
    mqttClient.setBufferSize(1024);
    
    initializeDeviceMaps();
    
    debug_println("Initialization complete");
    esp_task_wdt_reset();
}

/**
 * Main Program Loop
 */
void loop() {
    // Reset watchdog at start of loop
    esp_task_wdt_reset();
    
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();
    
    // Handle periodic sensor readings
    if (millis() - lastReading >= readingInterval) {
        digitalWrite(RELAY_5V_PIN, HIGH);
        digitalWrite(RELAY_3V3_PIN, HIGH);
        delay(100);
        
        readSensors();
        
        digitalWrite(RELAY_5V_PIN, LOW);
        digitalWrite(RELAY_3V3_PIN, LOW);
        lastReading = millis();
        
        esp_task_wdt_reset();  // Reset after readings complete
    }
    
    // Handle MCP watchdog
    handleMCPWatchdog();
    
    // Handle LED status indicator
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink >= 1000) {
        if(mcpResponding) {
            mcp.digitalWrite(LED_BLINK_PIN, !mcp.digitalRead(LED_BLINK_PIN));
        } else {
            mcp.digitalWrite(LED_BLINK_PIN, LOW);
        }
        lastBlink = millis();
    }
    
    // Reset watchdog at end of loop if all operations completed successfully
    esp_task_wdt_reset();
}
