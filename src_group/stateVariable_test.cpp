
#include <Arduino.h>
#include "src_group/dRehmFlight.h" // modified dRehmFlight has stabilization code and actuation and IMU communication and reciever communication.
#include "src_group/dataLog.h"
#include "src_group/ToF/VL53L1X.h"
#include "BMP180.h"
#include "ASPD4525.h"
#include "stateVariable.h" //calculates the state variables of the UAV 

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);

    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink();    // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    // Get vehicle state




}
