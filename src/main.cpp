#include "WiFi.h"
#include "Wire.h"
#include <PubSubClient.h>
#include <SPI.h>
#include <Adafruit_MCP2515.h>
#include <ArduinoOTA.h>
#include "secrets.h"

#define CS_PIN    7

#define CAN_BAUDRATE (1000000)

Adafruit_MCP2515 mcp(CS_PIN);

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32C3Client";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("battery_can/state", "on");
      // ... and resubscribe
      client.subscribe("battery_can/inTopic");
      IPAddress ip = WiFi.localIP();
      sprintf(msg, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
      client.publish("battery_can/IP", msg);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
//  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("MCP2515 Receiver Callback test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
  // start the WiFi OTA library with internal (flash) based storage
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // try to parse packet
  int packetSize = mcp.parsePacket();

  if (packetSize) {
//    sprintf(msg, "%02X", mcp.packetId());
//    client.publish("battery_can/packetidHEX", msg);
//    sprintf(msg, "%u", mcp.packetId());
//    client.publish("battery_can/packetid", msg);

    if (!mcp.packetRtr() && mcp.packetId() == 853) {
      // only print packet data for non-RTR packets
      if (mcp.available()) {
        sprintf(msg, "%u", mcp.read());
      }
    }
    while (mcp.available()) {
      int dummy = mcp.read();
    }
  }

  unsigned long now = millis();
  if (now - lastMsg > 60000) {
    lastMsg = now;
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("battery_can/SOC", msg);
    sprintf(msg, "%s", "0");
  }
}
