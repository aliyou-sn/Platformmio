#include <Arduino.h>
#include "Lights/light1.h"
#include "Lights/light2.h"
#include "Lights/light3.h"
// #include "Lights/light4.h"
// #include "Lights/light5.h"
// #include "Lights/light6.h"
#include "Sirens/Siren1.h"
#include "weathers/Room1.h"
#define timeSeconds 10
void light1();
void light2();
void light3();
// void light4();
// void light5();
// void light6();
unsigned long now = millis();
void setup() {
  Serial.begin(9600);
  dht1.begin();
  pinMode(pir1, INPUT_PULLUP);
  pinMode(pir2, INPUT_PULLUP);
  pinMode(pir3, INPUT_PULLUP);
  // pinMode(pir4, INPUT_PULLUP);
  // pinMode(pir5, INPUT_PULLUP);
  // pinMode(pir6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pir1), detectsMovement1, RISING);
  attachInterrupt(digitalPinToInterrupt(pir2), detectsMovement2, RISING);
  attachInterrupt(digitalPinToInterrupt(pir3), detectsMovement3, RISING);
  // attachInterrupt(digitalPinToInterrupt(pir4), detectsMovement4, RISING);
  // attachInterrupt(digitalPinToInterrupt(pir5), detectsMovement5, RISING);
  // attachInterrupt(digitalPinToInterrupt(pir6), detectsMovement6, RISING);
  xTaskCreate(readweather1,"Room 1",5000,NULL,4,NULL);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  // pinMode(L4, OUTPUT);
  // pinMode(L5, OUTPUT);
  // pinMode(L6, OUTPUT);
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(L3, LOW);
  // digitalWrite(L4, LOW);
  // digitalWrite(L5, LOW);
  // digitalWrite(L6, LOW);
}

void loop() {
  now = millis();
  light1();
  light2();
  light3();
  // light4();
  // light5();
  // light6();
  Siren1();
}
  //Light 1
  void light1(){
  if((digitalRead(L1) == HIGH) && (motion1 == false)) {
    motion1 = true;
  }
  if(startTimer1 && (now - lastTrigger1 > (timeSeconds * 1000))) {
    digitalWrite(L1, LOW);
    startTimer1 = false;
    motion1 = false;
  }
  }
  //Light 2
  void light2(){
  if((digitalRead(L2) == HIGH) && (motion2 == false)) {
    motion2 = true;
  }
  if(startTimer2 && (now - lastTrigger2 > (timeSeconds * 1000))) {
    digitalWrite(L2, LOW);
    startTimer2 = false;
    motion2 = false;
  }
  }

  //Light 3
  void light3(){
  if((digitalRead(L3) == HIGH) && (motion3 == false)) {
    motion3 = true;
  }
  if(startTimer3 && (now - lastTrigger3 > (timeSeconds * 1000))) {
    digitalWrite(L3, LOW);
    startTimer3 = false;
    motion3 = false;
  }
  }

  // //Light 4
  // void light4(){
  // if((digitalRead(L4) == HIGH) && (motion4 == false)) {
  //   motion4 = true;
  // }
  // if(startTimer4 && (now - lastTrigger4 > (timeSeconds * 1000))) {
  //   digitalWrite(L4, LOW);
  //   startTimer4 = false;
  //   motion4 = false;
  // }
  // }

  // //Light 5
  // void light5(){
  // if((digitalRead(L5) == HIGH) && (motion5 == false)) {
  //   motion5 = true;
  // }
  // if(startTimer5 && (now - lastTrigger5 > (timeSeconds * 1000))) {
  //   digitalWrite(L5, LOW);
  //   startTimer5 = false;
  //   motion5 = false;
  // }
  // }

  // //Light 6
  // void light6(){
  // if((digitalRead(L6) == HIGH) && (motion6 == false)) {
  //   motion6 = true;
  // }
  // if(startTimer6 && (now - lastTrigger6 > (timeSeconds * 1000))) {
  //   digitalWrite(L6, LOW);
  //   startTimer6 = false;
  //   motion6 = false;
  // }
  // }
 