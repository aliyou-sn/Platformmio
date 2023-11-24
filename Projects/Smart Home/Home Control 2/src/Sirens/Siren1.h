#ifndef Siren1_h
#define Siren1_h
#include <Arduino.h>
const int TouchPin = 15; 
const int BuzzerPin = 21;
const int BuzzerFreq = 5000;
const int BuzzerDur = 500;
void Siren1(){
    int TouchValue = 0;
    TouchValue = touchRead(TouchPin);
    if (TouchValue < 10) {
        for(int i=0; i < 3; i++){
            tone(BuzzerPin, BuzzerFreq, BuzzerDur);
            vTaskDelay(BuzzerDur / portTICK_PERIOD_MS);
            noTone(BuzzerPin);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

#endif