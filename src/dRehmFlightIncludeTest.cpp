#include <Arduino.h>

#include <dRehmFlight_Teensy_BETA_1.3.cpp>

void setup() {
    dRehmFlightSetup();
}

void loop() {
    dRehmFlightLoop();
}