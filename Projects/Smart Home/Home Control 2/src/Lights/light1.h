#ifndef light1_h
#define light1_h
#include <Arduino.h>
const int L1 = 13;
const int pir1 = 35;
unsigned long lastTrigger1 = 0;
bool startTimer1 = false;
bool motion1 = false;

void IRAM_ATTR detectsMovement1(){
  digitalWrite(L1, HIGH);
  startTimer1 = true;
  lastTrigger1 = millis();
}
#endif