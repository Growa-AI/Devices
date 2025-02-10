#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Configurazione WiFi
const char* ssid = "Growa";
const char* password = "Montblanc89!";

// Configurazione MQTT
const char* mqtt_server = "mqtt.growa.ai";  // Indirizzo del tuo broker
const int mqtt_port = 8883;               // Porta SSL
const char* mqtt_username = "mqttroot";
const char* mqtt_password = "k<V<k3:n73mQCF7";

// Topic MQTT per test
const char* mqtt_topic_pub = "test/message";
const char* mqtt_topic_sub = "test/response";

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char msg[50];

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Messaggio ricevuto [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentativo di connessione MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connesso");
      client.publish(mqtt_topic_pub, "ESP32 Online");
      client.subscribe(mqtt_topic_sub);
    } else {
      Serial.print("fallito, rc=");
      Serial.print(client.state());
      Serial.println(" riprovo tra 5 secondi");
      delay(5000);
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connessione a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connesso");
  Serial.println("Indirizzo IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  
  // Disabilita la verifica del certificato
  espClient.setInsecure();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    snprintf(msg, 50, "ESP32 test #%ld", now);
    Serial.print("Pubblico messaggio: ");
    Serial.println(msg);
    client.publish(mqtt_topic_pub, msg);
  }
}
