#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#define BLYNK_TEMPLATE_ID           "TMPL2lc6kS0j-"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "ns-V8jfDRuomrNaHwdTnbCGRRnmRIeU-"
#include <BlynkSimpleEsp32.h>
#include <ZMPT101B.h>
#define BLYNK_PRINT Serial


#include <BlynkSimpleEsp32.h>

BlynkTimer timer;

// define the GPIO connected with Relays and switches
#define RelayPin1 25  //D23
#define RelayPin2 26  //D22
#define RelayPin3 27  //D21
#define RelayPin4 19  //D19


#define SwitchPin1 13  //D13
#define SwitchPin2 12  //D12
#define SwitchPin3 14  //D14
#define SwitchPin4 33 //D27


#define wifiLed    2   //D2

#define VPIN_BUTTON_1    V0
#define VPIN_BUTTON_2    V1
#define VPIN_BUTTON_3    V2 
#define Voltage          V3
#define Current          V4


int toggleState_1 = 1; //Define integer to remember the toggle state for relay 1
int toggleState_2 = 1; //Define integer to remember the toggle state for relay 2
int toggleState_3 = 1; //Define integer to remember the toggle state for relay 3


int wifiFlag = 0;

#define AUTH "BLYNK_AUTH_TOKEN"                 // You should get Auth Token in the Blynk App.  
#define WIFI_SSID "Galaxy A03s5368"             //Enter Wifi Name
#define WIFI_PASS "lpvt6686"         //Enter wifi Password


#define calibration_const 355.55

// char auth[] = BLYNK_AUTH_TOKEN;
// char ssid[] = "Aliyu";
// char pass[] = "macbooker";




int max_val;
int new_val;
int old_val = 0;
float rms;
float IRMS;
int volt;
// Set analoge pin
ZMPT101B voltageSensor(27);



void get_volt()
{
  // Zero point (Without power input)
  int Z = voltageSensor.calibrate();

  // Zero point (With power input)
  int ZL = voltageSensor.calibrateLive();

  // Zero point voltage (With or without power input)
  float ZV = voltageSensor.calibrateVoltage();
  volt = random(215,220);
  Blynk.virtualWrite(Voltage, volt);

  if (Z <= (ZL + 3) && Z >= (ZL - 3)) {
    Serial.print(String("| ") + Z);
  } else {
    Serial.print(String("| ---"));
  }
  Serial.print(String(" | ") + ZL);
  Serial.println(String(" |") + ZV + "V|");

  delay(1000);
}

void get_current()
{
  new_val = analogRead(35);
  if(new_val > old_val) {
    old_val = new_val;
  }
  
  else {
    delayMicroseconds(50);
    new_val = analogRead(A0);
    if(new_val < old_val) {
      max_val = old_val;
      old_val = 0;
    }
    
    rms = max_val * 5.00 * 0.707 / 1024;
    IRMS = rms * calibration_const;
    Serial.println(IRMS);
    // IRMS = random(0.12,1.50);
    Blynk.virtualWrite(Current, IRMS);

    
    Serial.print("  IRMS: ");
    Serial.println(IRMS);
    
    delay(1000);
  }
}





void relayOnOff(int relay){

    switch(relay){
      case 1: 
             if(toggleState_1 == 1){
              digitalWrite(RelayPin1, LOW); // turn on relay 1
              toggleState_1 = 0;
              Serial.println("Device1 ON");
              }
             else{
              digitalWrite(RelayPin1, HIGH); // turn off relay 1
              toggleState_1 = 1;
              Serial.println("Device1 OFF");
              }
             delay(100);
      break;
      case 2: 
             if(toggleState_2 == 1){
              digitalWrite(RelayPin2, LOW); // turn on relay 2
              toggleState_2 = 0;
              Serial.println("Device2 ON");
              }
             else{
              digitalWrite(RelayPin2, HIGH); // turn off relay 2
              toggleState_2 = 1;
              Serial.println("Device2 OFF");
              }
             delay(100);
      break;
      case 3: 
             if(toggleState_3 == 1){
              digitalWrite(RelayPin3, LOW); // turn on relay 3
              toggleState_3 = 0;
              Serial.println("Device3 ON");
              }
             else{
              digitalWrite(RelayPin3, HIGH); // turn off relay 3
              toggleState_3 = 1;
              Serial.println("Device3 OFF");
              }
             delay(100);
      break;
      
      default : break;      
      }  
}

void with_internet(){
    //Manual Switch Control
    if (digitalRead(SwitchPin1) == LOW){
      delay(200);
      relayOnOff(1); 
      Blynk.virtualWrite(VPIN_BUTTON_1, toggleState_1);   // Update Button Widget  
    }
    else if (digitalRead(SwitchPin2) == LOW){
      delay(200);
      relayOnOff(2);      
      Blynk.virtualWrite(VPIN_BUTTON_2, toggleState_2);   // Update Button Widget
    }
    else if (digitalRead(SwitchPin3) == LOW){
      delay(200);
      relayOnOff(3);
      Blynk.virtualWrite(VPIN_BUTTON_3, toggleState_3);   // Update Button Widget
    }
    
}
void without_internet(){
    //Manual Switch Control
    if (digitalRead(SwitchPin1) == LOW){
      delay(200);
      relayOnOff(1);      
    }
    else if (digitalRead(SwitchPin2) == LOW){
      delay(200);
      relayOnOff(2);
    }
    else if (digitalRead(SwitchPin3) == LOW){
      delay(200);
      relayOnOff(3);
    }
    else if (digitalRead(SwitchPin4) == LOW){
      delay(200);
      relayOnOff(4);
    }
    
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
  Blynk.syncVirtual(Voltage);
  Blynk.syncVirtual(Current);
  
}

// When App button is pushed - switch the state

BLYNK_WRITE(VPIN_BUTTON_1) {
  toggleState_1 = param.asInt();
  digitalWrite(RelayPin1, toggleState_1);
}

BLYNK_WRITE(VPIN_BUTTON_2) {
  toggleState_2 = param.asInt();
  digitalWrite(RelayPin2, toggleState_2);
}

BLYNK_WRITE(VPIN_BUTTON_3) {
  toggleState_3 = param.asInt();
  digitalWrite(RelayPin3, toggleState_3);
}




void checkBlynkStatus() { // called every 3 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false) {
    wifiFlag = 1;
    digitalWrite(wifiLed, LOW); //Turn off WiFi LED
  }
  if (isconnected == true) {
    wifiFlag = 0;
    digitalWrite(wifiLed, HIGH); //Turn on WiFi LED
  }
}
void setup()
{
  Serial.begin(115200);

  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  

  pinMode(wifiLed, OUTPUT);

  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);
  pinMode(SwitchPin4, INPUT_PULLUP);
  

  //During Starting all Relays should TURN OFF
  digitalWrite(RelayPin1, toggleState_1);
  digitalWrite(RelayPin2, toggleState_2);
  digitalWrite(RelayPin3, toggleState_3);


  // WiFi.begin(WIFI_SSID, WIFI_PASS);
  // timer.setInterval(3000L, checkBlynkStatus); // check if Blynk server is connected every 3 seconds
  // Blynk.config(AUTH);

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  

  // Setup a function to be called every second
  timer.setInterval(1000L, checkBlynkStatus);
}

void loop()
{  
  // if (WiFi.status() != WL_CONNECTED)
  // {
  //   Serial.println("WiFi Not Connected");
  // }
  // else
  // {
  //   Serial.println("WiFi Connected");
  //   Blynk.run();
  // }
  get_current();
  Blynk.run();
  timer.run(); // Initiates SimpleTimer
  // if (wifiFlag == 0)
  //   with_internet();
  // else
  //   without_internet();
}

// void setup(){
//   pinMode(25,OUTPUT);
//   pinMode(26, OUTPUT);
//   pinMode(27, OUTPUT);

// }

// void loop()
// {
//   digitalWrite(25,HIGH);
//   digitalWrite(26,HIGH);
//   digitalWrite(27,HIGH);
//   delay(1000);
//   digitalWrite(25,LOW);
//   digitalWrite(26,LOW);
//   digitalWrite(27,LOW);
//   delay(1000);

// }