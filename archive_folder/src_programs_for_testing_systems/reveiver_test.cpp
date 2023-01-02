//WORKS AS OF 12/5/22
/*
#include <Arduino.h>

#include "src_group/dRehmFlight.h"
#include "sdreadwrite.h"

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    radioSetup();
    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds
    getCommands();
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    Serial.println(channel_2_pwm);
}*/
