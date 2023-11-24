#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include "config.h"

const char ssid[] = "Aliyu";
const char pass[] = "macbooker";
unsigned long now = millis();
unsigned long lastTrigger = 0;
WiFiClient net;
MQTTClient client;
unsigned long lastMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("ESP32", CONFIG_BROKER_USERNAME, CONFIG_BROKER_PASSWORD)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  client.publish(CONFIG_TOPIC_SWITCH_STATE, payload);
}

void subscribeTopics() {
  client.subscribe(CONFIG_TOPIC_SWITCH_SET);
}
String makeJsonString(String key, int value) {
  StaticJsonDocument<128> doc;
  doc[key.c_str()] = value;
  String output;
  serializeJson(doc, output);
  Serial.println(output);
  return output;
}
void publishDummyData() {
  Serial.println("------------------------");
  int dummyTemp = random(20, 35);
  client.publish(CONFIG_TOPIC_SENSOR_DATA, makeJsonString(CONFIG_SENSOR_BUILDIN_TEMP_VALUE_KEY, dummyTemp));

  int dummyHummy = random(0, 50);
  client.publish(CONFIG_TOPIC_SENSOR_DATA, makeJsonString(CONFIG_SENSOR_BUILDIN_HUMIDITY_VALUE_KEY, dummyHummy));

  int dummyCO2 = random(900, 1200);
  client.publish(CONFIG_TOPIC_SENSOR_DATA, makeJsonString(CONFIG_SENSOR_BUILDIN_CO2_VALUE_KEY, dummyCO2));

  int dummyTVOC = random(0, 500);
  client.publish(CONFIG_TOPIC_SENSOR_DATA, makeJsonString(CONFIG_SENSOR_BUILDIN_TVOC_VALUE_KEY, dummyTVOC));

  Serial.println("-----Dummy Data Sent----");
}



void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  

  client.begin(CONFIG_BROKER_URL, 1883, net);
  client.onMessage(messageReceived);
  pinMode(2,OUTPUT);
  pinMode(4, INPUT);
  connect();
  subscribeTopics();
}

void loop() {
  // if(now - lastTrigger > 1000){
  //   lastTrigger = millis();
  //   int state = digitalRead(2);
  //   if(state ==0){

  //   }

  // }
  if(digitalRead(4)==0){
    digitalWrite(2,LOW);
    client.publish(CONFIG_TOPIC_SWITCH_STATE, "OFF");
  }
  if(digitalRead(4)==1){
    digitalWrite(2,HIGH);
    client.publish(CONFIG_TOPIC_SWITCH_STATE, "ON");
  }
  client.loop();
  delay(10);
  if (!client.connected()) {
    connect();
  } else {
    if (millis() - lastMillis > 3000) {
      lastMillis = millis();
      publishDummyData();
    }
  }
}