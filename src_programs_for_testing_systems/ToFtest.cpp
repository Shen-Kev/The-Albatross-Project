//MAIN CODE, waiting to run some functions oh yeah
//works as of 12/10/22

#include <Arduino.h>
#include "src_group/dRehmFlight.h"
#include "src_group/ToF/VL53L1X.h"
//add other libraries when they come in

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
    VL53L1Xsetup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    VL35L1Xloop();
    Serial.print(" Distance: ");
    Serial.print(distance);
    Serial.print(" DistanceLP: ");
    Serial.print(distance_LP);
    Serial.println(" mm");
    //will need to make some code for when distance returns -1, that the kalman filter or whatever completely ignores it and just uses IMU and barometer
}
