#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Adafruit_MCP23X17.h>
#include <esp_task_wdt.h>

// MQTT configuration
const char* mqtt_server = "mqtt.growa.ai";
const int mqtt_port = 8883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// Device configuration
#define DEVICE_ID "0001"
#define FIRMWARE_VERSION "1.0.1"

// Hardware pins
#define BATTERY_PIN 32
#define RELAY_5V_PIN 33
#define RELAY_3V3_PIN 19
#define MCP_FEEDBACK_PIN 14
#define DEBUG_RX 26
#define DEBUG_TX 27
#define LED_BUILTIN 2

// MCP23017 registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_GPIOA  0x12
#define MCP23017_GPIOB  0x13

// Global objects
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;
Adafruit_MCP23X17 mcp;
volatile bool watchdogFlag = false;

// Global variables
char systemTopic[50];
char configTopic[50];
int counter = 0;
int lastMsg = 0;

const uint8_t MCP23017_ADDRESS = 0x20;
uint8_t mcpStateA = 0;
uint8_t mcpStateB = 0;
unsigned long lastBlinkTime = 0;
bool blinkState = false;
bool feedbackState = false;
bool mcpInitialized = false;

// Mapping dei relay ai pin MCP23017
const uint8_t RELAY_TO_PIN[] = {
   9,   // Relay 1 -> GPB1 (pin 9)
   10,  // Relay 2 -> GPB2 (pin 10)
   11,  // Relay 3 -> GPB3 (pin 11)
   12,  // Relay 4 -> GPB4 (pin 12)
   13,  // Relay 5 -> GPB5 (pin 13)
   14,  // Relay 6 -> GPB6 (pin 14)
   1,   // Relay 7 -> GPA1 (pin 1)
   2,   // Relay 8 -> GPA2 (pin 2)
   3,   // Relay 9 -> GPA3 (pin 3)
   4,   // Relay 10 -> GPA4 (pin 4)
   5,   // Relay 11 -> GPA5 (pin 5)
   6    // Relay 12 -> GPA6 (pin 6)
};

const uint8_t ADC_PINS[] = {36, 39, 34, 35};
const char* ADC_VOLTAGE[4] = {"", "", "", ""};

const uint8_t PULSE_PINS[] = {17, 16};
volatile uint32_t pulseCount1 = 0;
volatile uint32_t pulseCount2 = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

#define WDT_TIMEOUT 60

// Debug function for MCP23017
void debugMCP() {
    // Leggiamo lo stato usando i pin
    Serial.println("MCP23017 Pins Status:");
    for(int i = 0; i < 16; i++) {
        int state = mcp.digitalRead(i);
        Serial.printf("Pin %d: %d\n", i, state);
    }
}

// Watchdog functions
void setupWatchdog() {
    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
}

void IRAM_ATTR resetModule() {
    ets_printf("Watchdog timer expired - rebooting\n");
    esp_restart();
}

void feedWatchdog() {
    esp_task_wdt_reset();
}

// Callback functions
void saveConfigCallback() {
    Serial.println("Configuration saved");
}

void configModeCallback(WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

// Interrupt handlers
void IRAM_ATTR pulse1_isr() {
    portENTER_CRITICAL_ISR(&mux);
    pulseCount1++;
    if (pulseCount1 >= UINT32_MAX) pulseCount1 = 0;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR pulse2_isr() {
    portENTER_CRITICAL_ISR(&mux);
    pulseCount2++;
    if (pulseCount2 >= UINT32_MAX) pulseCount2 = 0;
    portEXIT_CRITICAL_ISR(&mux);
}

// MCP23017 functions
void initMCP23017() {
    if (!mcp.begin_I2C(MCP23017_ADDRESS)) {
        Serial.println("Error initializing MCP23017");
        char errorMsg[] = "0x30+0+1+1";
        mqttClient.publish(
            (String(DEVICE_ID) + "/readings").c_str(),
            (const uint8_t*)errorMsg,
            strlen(errorMsg),
            true
        );
        return;
    }
    
    // Configura tutti i pin come output
    for(int i = 0; i < 16; i++) {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, LOW);
    }
    
    mcpInitialized = true;
    debugMCP();
}

void handleMCPConfig(byte* payload, unsigned int length) {
    Serial.println("Handling MCP config:");
    Serial.print("Raw payload: ");
    for(unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Cerca il primo '+' per isolare il primo token
    char* firstPlus = strchr((char*)payload, '+');
    if(!firstPlus) {
        Serial.println("Invalid format - no +");
        return;
    }

    // Verifica che il primo token inizi con "0x30"
    if(strncmp((char*)payload, "0x30", 4) != 0) {
        Serial.println("Invalid type - not 0x30");
        return;
    }

    // Cerca il secondo '+'
    char* secondPlus = strchr(firstPlus + 1, '+');
    if(!secondPlus) {
        Serial.println("Invalid format - no second +");
        return;
    }

    // Verifica che il secondo token sia "0"
    if(*(firstPlus + 1) != '0') {
        Serial.println("Invalid address - not 0");
        return;
    }

    // Cerca il terzo '+'
    char* thirdPlus = strchr(secondPlus + 1, '+');
    if(!thirdPlus) {
        Serial.println("Invalid format - no third +");
        return;
    }

    // Verifica che il terzo token sia "12"
    char countStr[3] = {0};
    strncpy(countStr, secondPlus + 1, thirdPlus - (secondPlus + 1));
    if(strcmp(countStr, "12") != 0) {
        Serial.println("Invalid count - not 12");
        return;
    }

    // Array per i valori dei relay
    uint8_t values[12] = {0};
    char* currentPos = thirdPlus + 1;
    int relayIndex = 0;

    // Leggi i 12 valori
    while(relayIndex < 12 && *currentPos) {
        if(*currentPos == '1') {
            values[relayIndex] = 1;
        } else if(*currentPos == '0') {
            values[relayIndex] = 0;
        } else if(*currentPos != '+') {
            Serial.printf("Invalid value character: %c\n", *currentPos);
            return;
        }
        
        if(*currentPos != '+') {
            relayIndex++;
        }
        currentPos++;
    }

    // Verifica che abbiamo tutti i 12 valori
    if(relayIndex != 12) {
        Serial.printf("Wrong number of values: %d\n", relayIndex);
        return;
    }

    // Imposta i relay
    for(int i = 0; i < 12; i++) {
        mcp.pinMode(RELAY_TO_PIN[i], OUTPUT);
        mcp.digitalWrite(RELAY_TO_PIN[i], values[i]);
        // Serial.printf("Setting relay %d to %d\n", i+1, values[i]);
    }

    delay(10);
    MCPReading();
}

void MCPReading() {
    Wire.beginTransmission(MCP23017_ADDRESS);
    if(Wire.endTransmission() != 0) {
        char errorMsg[] = "0x30+0+1+1";
        mqttClient.publish(
            (String(DEVICE_ID) + "/readings").c_str(),
            (const uint8_t*)errorMsg,
            strlen(errorMsg),
            true
        );
        return;
    }

    char outputBuffer[50];
    char tempBuffer[10];
    sprintf(outputBuffer, "0x30+0+12");
    
    // Leggi lo stato di ogni relay
    for(int relay = 0; relay < 12; relay++) {
        int state = mcp.digitalRead(RELAY_TO_PIN[relay]);
        sprintf(tempBuffer, "+%d", state);
        strcat(outputBuffer, tempBuffer);
    }
    
    mqttClient.publish(
        (String(DEVICE_ID) + "/readings").c_str(),
        (const uint8_t*)outputBuffer,
        strlen(outputBuffer),
        true
    );
}

void handleMCPBlink() {
    unsigned long currentTime = millis();
    static unsigned long lastBlinkTime = 0;
    static bool blinkState = false;
    static bool feedbackState = false;
    
    if(currentTime - lastBlinkTime >= 10000) {
        blinkState = true;
        feedbackState = true;
        mcp.digitalWrite(8, HIGH);
        mcp.digitalWrite(15, HIGH);
        mcp.digitalWrite(7, HIGH);
        lastBlinkTime = currentTime;
    }
    else if(currentTime - lastBlinkTime >= 2000 && blinkState) {
        blinkState = false;
        mcp.digitalWrite(8, LOW);
    }
    else if(currentTime - lastBlinkTime >= 1000 && feedbackState) {
        feedbackState = false;
        mcp.digitalWrite(15, LOW);
        mcp.digitalWrite(7, LOW);
    }
}

// Sensor reading functions
void ADCReading() {
    char outputBuffer[50];    
    char tempBuffer[10];      
    const char type[] = "0x20";
    const char address = '0';
    
    sprintf(outputBuffer, "%s+%c+4", type, address);
    
    for(int i = 0; i < 4; i++) {
        float voltage = analogRead(ADC_PINS[i]) * 10.0 / 4095.0;
        dtostrf(voltage, 1, 2, tempBuffer);
        
        char *ptr = tempBuffer;
        while(*ptr == ' ') ptr++;
        
        strcat(outputBuffer, "+");
        strcat(outputBuffer, ptr);
    }
    
    mqttClient.publish(
        (String(DEVICE_ID) + "/readings").c_str(),            
        (const uint8_t*)outputBuffer, 
        strlen(outputBuffer), 
        true
    );
}

void PULSEReading() {
    char outputBuffer[50];    
    char tempBuffer[10];      
    const char type[] = "0x10";
    const char address = '0';
    
    sprintf(outputBuffer, "%s+%c+2", type, address);
    
    portENTER_CRITICAL(&mux);
    uint32_t count1 = pulseCount1;
    uint32_t count2 = pulseCount2;
    portEXIT_CRITICAL(&mux);
    
    sprintf(tempBuffer, "%lu", count1);
    strcat(outputBuffer, "+");
    strcat(outputBuffer, tempBuffer);
    
    sprintf(tempBuffer, "%lu", count2);
    strcat(outputBuffer, "+");
    strcat(outputBuffer, tempBuffer);
    
    mqttClient.publish(
        (String(DEVICE_ID) + "/readings").c_str(),            
        (const uint8_t*)outputBuffer, 
        strlen(outputBuffer), 
        true
    );
}

#include <HTTPClient.h>
#include <Update.h>

#define MAX_URL_LENGTH 512
#define MIN_URL_LENGTH 30

bool performOTA(const char* url) {
    HTTPClient http;
    bool success = false;
    
    Serial.printf("Starting OTA from: %s\n", url);
    
    // Configura il timeout
    http.setTimeout(12000);  // 12 secondi di timeout
    
    if (!http.begin(url)) {
        Serial.println("Failed to connect to update server");
        return false;
    }
    
    // Get del file
    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("HTTP GET failed, error: %d\n", httpCode);
        http.end();
        return false;
    }
    
    // Ottieni la dimensione del file
    int contentLength = http.getSize();
    if (contentLength <= 0) {
        Serial.println("Invalid content length");
        http.end();
        return false;
    }
    
    Serial.printf("OTA file size: %d bytes\n", contentLength);
    
    // Prepara l'update
    if (!Update.begin(contentLength)) {
        Serial.printf("Not enough space for update: %s\n", Update.errorString());
        http.end();
        return false;
    }
    
    // Download e scrittura in flash
    WiFiClient *stream = http.getStreamPtr();
    size_t written = 0;
    uint8_t buff[1024] = {0};
    
    while (http.connected() && written < contentLength) {
        // Feed watchdog durante il processo
        feedWatchdog();
        
        // Leggi un chunk
        size_t len = stream->available();
        if (len > 0) {
            // Non leggere più di quanto possiamo gestire
            if (len > sizeof(buff)) len = sizeof(buff);
            
            // Leggi nel buffer
            int bytesRead = stream->readBytes(buff, len);
            if (bytesRead < 0) {
                Serial.println("Read error");
                break;
            }
            
            // Scrivi in flash
            if (Update.write(buff, bytesRead) != bytesRead) {
                Serial.printf("Write failed: %s\n", Update.errorString());
                break;
            }
            
            written += bytesRead;
            
            // Stampa progresso
            Serial.printf("OTA Progress: %d%%\n", (written * 100) / contentLength);
        }
        delay(1);  // Yield per il watchdog e altri task
    }
    
    // Verifica che tutto sia stato scritto
    if (written != contentLength) {
        Serial.println("Write incomplete");
        Update.abort();
        http.end();
        return false;
    }
    
    // Finalizza l'update
    if (!Update.end(true)) {
        Serial.printf("Update end failed: %s\n", Update.errorString());
        http.end();
        return false;
    }
    
    http.end();
    Serial.println("OTA Update successful!");
    return true;
}

void handleOTAUpdate(const char* url) {
    size_t urlLength = strlen(url);
    
    // Verifica lunghezza URL
    if(urlLength < MIN_URL_LENGTH || urlLength > MAX_URL_LENGTH) {
        char errorMsg[64];
        snprintf(errorMsg, sizeof(errorMsg), "URL length invalid (%d chars)", urlLength);
        mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), errorMsg, true);
        return;
    }

    // Verifica che l'URL sia raw.githubusercontent.com
    if(strncmp(url, "https://raw.githubusercontent.com/", 33) != 0) {
        mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                          "Invalid URL - must be raw.githubusercontent.com", true);
        return;
    }

    Serial.print("Starting OTA from URL: ");
    Serial.println(url);

    mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                       "OTA update started", true);

    bool success = performOTA(url);
    
    if(success) {
        mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                         "OTA update successful - rebooting", true);
        delay(1000);
        ESP.restart();
    } else {
        mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                         "OTA update failed", true);
    }
}

// MQTT functions
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] Payload: ");
    
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    String topicStr = String(topic);
    String configTopic = String(DEVICE_ID) + "/config";
    String systemTopic = String(DEVICE_ID) + "/system";

    if(topicStr == configTopic) {
        if(length >= 4 && strncmp((char*)payload, "0x30", 4) == 0) {
            handleMCPConfig(payload, length);
        }
    }
    else if(topicStr == systemTopic) {
        char* command = (char*)malloc(MAX_URL_LENGTH + 10);
        if(!command) {
            mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                             "Memory allocation failed", true);
            return;
        }

        if(length < MAX_URL_LENGTH + 10) {
            memset(command, 0, MAX_URL_LENGTH + 10);
            memcpy(command, payload, length);
            command[length] = 0;
            
            if(strcmp(command, "reset") == 0) {
                free(command);
                ESP.restart();
            }
            else if(strncmp(command, "ota:", 4) == 0) {
                handleOTAUpdate(command + 4);
            }
        } else {
            mqttClient.publish((String(DEVICE_ID) + "/status").c_str(), 
                             "Command too long", true);
        }
        
        free(command);
    }
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("connected");
            mqttClient.subscribe((String(DEVICE_ID) + "/system").c_str(), 1);
            mqttClient.subscribe((String(DEVICE_ID) + "/config").c_str(), 1);
            mqttClient.subscribe((String(DEVICE_ID) + "/readings").c_str(), 1);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(DEVICE_ID);
    
    Wire.begin();  // Inizializza I2C prima dell'MCP23017
    
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setConfigPortalTimeout(180);
    
    if (!wifiManager.autoConnect("GrowaAP")) {
        Serial.println("Failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
        delay(5000);
    }
    
    setupWatchdog();
    initMCP23017();
    
    espClient.setInsecure();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(callback);
    
    pinMode(RELAY_5V_PIN, OUTPUT);
    pinMode(RELAY_3V3_PIN, OUTPUT);
    pinMode(PULSE_PINS[0], INPUT_PULLUP);
    pinMode(PULSE_PINS[1], INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PULSE_PINS[0]), pulse1_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(PULSE_PINS[1]), pulse2_isr, FALLING);
}

void loop() {
    static unsigned long lastWdtFeed = 0;
    unsigned long now = millis();
    
    if (now - lastWdtFeed >= 30000) {
        feedWatchdog();
        lastWdtFeed = now;
    }
    
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    handleMCPBlink();
    
    if (now - lastMsg > 30000) {
        digitalWrite(RELAY_5V_PIN, HIGH);
        digitalWrite(RELAY_3V3_PIN, HIGH);
        
        if(!mcpInitialized) {
            initMCP23017();
        }
        
        ADCReading();
        PULSEReading();
        MCPReading();
        
        digitalWrite(RELAY_5V_PIN, LOW);
        digitalWrite(RELAY_3V3_PIN, LOW);
        lastMsg = now;
        ++counter;
    }
}
