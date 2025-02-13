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
const char* mqtt_user = "mqttroot";
const char* mqtt_password = "k<V<k3:n73mQCF7";

// Device configuration
#define DEVICE_ID "0001"
#define FIRMWARE_VERSION "1.0.7"

// Hardware pins
#define BATTERY_PIN 32
#define RELAY_5V_PIN 33
#define RELAY_3V3_PIN 19
#define MCP_FEEDBACK_PIN 14
#define DEBUG_RX 26
#define DEBUG_TX 27
#define LED_BUILTIN 2

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

#define WDT_TIMEOUT 60  // Timeout in secondi
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
    ets_printf("Watchdog timer expired - rebooting\n");
    esp_restart();
}

void setupWatchdog() {
    // Configura il watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);  // Aggiungi il task corrente al WDT
    
    // Configura il timer hardware come backup
    timer = timerBegin(1000000);  // 1MHz
    timerAttachInterrupt(timer, &resetModule);
    timerStart(timer);
}

void feedWatchdog() {
    esp_task_wdt_reset();  // Reset del watchdog
    timerWrite(timer, 0);  // Reset del timer hardware
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
   if (pulseCount1 >= UINT32_MAX) {
       pulseCount1 = 0;
   }
   portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR pulse2_isr() {
   portENTER_CRITICAL_ISR(&mux);
   pulseCount2++;
   if (pulseCount2 >= UINT32_MAX) {
       pulseCount2 = 0;
   }
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
   for(int i = 0; i < 16; i++) {
       mcp.pinMode(i, OUTPUT);
       mcp.digitalWrite(i, LOW);
   }
}

void handleMCPConfig(byte* payload, unsigned int length) {
   char payloadCopy[50];
   memcpy(payloadCopy, payload, length);
   payloadCopy[length] = '\0';
   
   char* token = strtok(payloadCopy, "+");
   if (token == NULL || strcmp(token, "0x30") != 0) return;
   
   token = strtok(NULL, "+");
   if (token == NULL || token[0] != '0') return;
   
   token = strtok(NULL, "+");
   if (token == NULL || strcmp(token, "12") != 0) return;
   
   uint8_t values[12];
   bool valid = true;
   
   for(int relay = 0; relay < 12 && valid; relay++) {
       token = strtok(NULL, "+");
       if(token == NULL) {
           valid = false;
           break;
       }
       uint8_t value = atoi(token);
       if(value > 1) {
           valid = false;
           break;
       }
       values[relay] = value;
   }
   
   if(valid) {
       for(int relay = 0; relay < 12; relay++) {
           mcp.digitalWrite(RELAY_TO_PIN[relay], values[relay]);
       }
   }
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
   
   for(int relay = 0; relay < 12; relay++) {
       int pinState = mcp.digitalRead(RELAY_TO_PIN[relay]);
       sprintf(tempBuffer, "+%d", pinState);
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

// MQTT functions
void callback(char* topic, byte* payload, unsigned int length) {
   Serial.print("Message arrived [");
   Serial.print(topic);
   Serial.print("] Payload: ");
   
   String topicStr = String(topic);
   String configTopic = String(DEVICE_ID) + "/config";
   String systemTopic = String(DEVICE_ID) + "/system";
   String readingsTopic = String(DEVICE_ID) + "/readings";
   
   for (int i = 0; i < length; i++) {
       Serial.print((char)payload[i]);
   }
   Serial.println();

   payload[length] = '\0';
   
   if(topicStr == configTopic) {
       char* payloadStr = (char*)payload;
       char* typeToken = strtok(payloadStr, "+");
       
       if(typeToken != NULL && strcmp(typeToken, "0x30") == 0) {
           handleMCPConfig(payload, length);
       }
   }
   else if(topicStr == systemTopic) {
       char* payloadStr = (char*)payload;
       if(strcmp(payloadStr, "reset") == 0) {
           ESP.restart();
       }
   }
   else if(topicStr == readingsTopic) {
       char* payloadStr = (char*)payload;
       char* typeToken = strtok(payloadStr, "+");
       
       if(typeToken != NULL) {
           if(strcmp(typeToken, "0x10") == 0) {
               Serial.println("Received PULSE reading");
           }
           else if(strcmp(typeToken, "0x20") == 0) {
               Serial.println("Received ADC reading");
           }
           else if(strcmp(typeToken, "0x30") == 0) {
               Serial.println("Received MCP status");
           }
       }
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
   
   setupWatchdog();
   
   wifiManager.setAPCallback(configModeCallback);
   wifiManager.setSaveConfigCallback(saveConfigCallback);
   wifiManager.setConfigPortalTimeout(180);
   
   if (!wifiManager.autoConnect("GrowaAP")) {
       Serial.println("Failed to connect and hit timeout");
       delay(3000);
       ESP.restart();
       delay(5000);
   }
   
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
   
   initMCP23017();
   
   espClient.setInsecure();
   mqttClient.setServer(mqtt_server, mqtt_port);
   mqttClient.setCallback(callback);
   
   pinMode(PULSE_PINS[0], INPUT_PULLUP);
   pinMode(PULSE_PINS[1], INPUT_PULLUP);
   
   attachInterrupt(digitalPinToInterrupt(PULSE_PINS[0]), pulse1_isr, FALLING);
   attachInterrupt(digitalPinToInterrupt(PULSE_PINS[1]), pulse2_isr, FALLING);
}

void loop() {
   feedWatchdog();
   
   if (!mqttClient.connected()) {
       reconnect();
   }
   mqttClient.loop();
   handleMCPBlink();
   
   unsigned long now = millis();
   if (now - lastMsg > 5000) {
       ADCReading();
       PULSEReading();
       MCPReading();
       lastMsg = now;
       ++counter;    
       Serial.println("Cycle " + String(counter));
   }
}
