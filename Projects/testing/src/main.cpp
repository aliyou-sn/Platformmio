#include <Arduino.h>
// #include <uMyo_BLE.h>

typedef union {
  float fp;
  byte binary[4];
} binaryFLoat;

void setup() {
  
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  
}

void loop() {
  binaryFLoat a[3] = {23.46,7.98,57.76};
  Serial.write(a[0].binary, 4);
  delay(1000);
}