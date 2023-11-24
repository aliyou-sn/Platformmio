#include <Arduino.h>

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(25, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}
