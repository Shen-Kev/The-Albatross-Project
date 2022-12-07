//WORKS AS OF 12/5/22
/*
#include <Arduino.h>

#include "src_group/dRehmFlight.h"
#include "dataLog.h"

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    setupSD(); //needs to be at the start of the setup, to stop setup if SD isn't inserted
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink();    // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    // Get vehicle state
    logData(1, 3.2, 3241.234234, 'W', "This is a String");
}
