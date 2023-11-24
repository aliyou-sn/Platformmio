#ifndef light6_h
#define light6_h
#include <Arduino.h>
const int L6 = 25;
const int pir6 = 22;
unsigned long lastTrigger6 = 0;
bool startTimer6 = false;
bool motion6 = false;

void IRAM_ATTR detectsMovement6(){
  digitalWrite(L6, HIGH);
  startTimer6 = true;
  lastTrigger6 = millis();
}
#endif