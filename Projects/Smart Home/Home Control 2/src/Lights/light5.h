#ifndef light5_h
#define light5_h
#include <Arduino.h>
const int L5 = 26;
const int pir5 = 23;
unsigned long lastTrigger5 = 0;
bool startTimer5 = false;
bool motion5 = false;

void IRAM_ATTR detectsMovement5(){
  digitalWrite(L5, HIGH);
  startTimer5 = true;
  lastTrigger5 = millis();
}
#endif