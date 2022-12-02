//WORKS ON TEENSY 4.1, as of 12/1/22

#include <Arduino.h>

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
}

void loop() {
  // nothing happens after setup
  Serial.println("Hello World");
}