// works as of 12/16/22
/*
#include <Arduino.h>
#include "src_group/dRehmFlight.h"
#include "src_group/dataLog.h"
#include "src_group/ToF/VL53L1X.h"
#include "BMP180.h"
#include "src_group/ASPD4525.h"
// add other libraries when they come in

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
    pitotSetup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink();    // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    pitotLoop();
    Serial.print(" airspeed adjusted m/s: ");
    Serial.print(airspeed_adjusted);
    Serial.print(" airspeed adjusted mph: ");
    Serial.print(airspeed_adjusted * 2.23694);
    Serial.println();
}

*/