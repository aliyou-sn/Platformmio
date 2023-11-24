#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#define wifiLed   2   
#define BLYNK_TEMPLATE_ID "TMPL2Am2Lwl4q"
#define BLYNK_TEMPLATE_NAME "farm"
#define BLYNK_AUTH_TOKEN "AM7--IMieYXsaLndbA1iGgpWhvgP9qam"
#define VPIN_Text           V1
#define VPIN_currentTemp    V0
#define VPIN_currentHumi    V3
#define VPIN_Distance       V4
#define VPIN_Lux            V5
#define VPIN_pH            V6

#define DHTPIN 4
#define DHTTYPE DHT11
#define trigPin 12
#define echoPin 14
#define potPin 35

long duration;
int distance;
float lux;
float temp;
float Hum;
float ph;
float Value=0;

DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

char ssid[] = "Aliyu";
char pass[] = "macbooker";



BlynkTimer timer; 
BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_currentTemp);
  Blynk.syncVirtual(VPIN_currentHumi);
  Blynk.syncVirtual(VPIN_Distance);
  Blynk.syncVirtual(VPIN_Distance);
  Blynk.syncVirtual(VPIN_Lux);
  Blynk.syncVirtual(VPIN_pH);
}

void myTimerEvent()
{
  // Blynk.virtualWrite(V0, millis() / 100);
  Blynk.virtualWrite(V0, temp);
  Blynk.virtualWrite(V3, Hum);
  Blynk.virtualWrite(V4, distance);
  Blynk.virtualWrite(V5, lux);
  Blynk.virtualWrite(V6, ph);
}

void MeasureTemp(){
  temp = dht.readTemperature();
  Hum = dht.readHumidity();
  // Blynk.virtualWrite(VPIN_currentTemp, temp);
  // Blynk.virtualWrite(VPIN_currentHumi, Hum);
  Serial.print("Temperature : ");
  Serial.println(temp);
   Serial.print("Humidity  : ");
  Serial.println(Hum);
}

void GetDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2;
  // Blynk.virtualWrite(VPIN_Distance, distance);
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
}

void GetLux(){
  float lux = lightMeter.readLightLevel();
  // Blynk.virtualWrite(VPIN_Lux, lux);
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}

void GetpH(){
  Value= analogRead(potPin);
  // Serial.print(Value);
  // Serial.print(" | ");
  float voltage=Value*(3.3/4095.0);
  ph=(3.3*voltage);
  Serial.println(ph);
}

void setup()
{
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(wifiLed, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(potPin,INPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(500L, myTimerEvent);
  dht.begin();
  Wire.begin(); /// SCL --> 22, SDA --> 21
  lightMeter.begin();
}

void loop()
{
  GetDistance();
  MeasureTemp();
  GetLux();
  GetpH();
  Blynk.run();
  timer.run();
}
