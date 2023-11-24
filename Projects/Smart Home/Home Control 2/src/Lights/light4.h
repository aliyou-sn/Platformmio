#ifndef light4_h
#define light4_h
#include <Arduino.h>
const int L4 = 27;
const int pir4 = 36;
unsigned long lastTrigger4 = 0;
bool startTimer4 = false;
bool motion4 = false;

void IRAM_ATTR detectsMovement4(){
  digitalWrite(L4, HIGH);
  startTimer4 = true;
  lastTrigger4 = millis();
}
#endif