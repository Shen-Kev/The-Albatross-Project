// WORKS AS OF 12/5/22!
// note, as the IMU passes 90 degrees rotation, it starts counting back down to 0
// note the yaw drift is pretty signinficant, I could tune it but I might need compass
// note when yaw goes around all the way it suddenly snaps from -180 to 180, might need to deal with that

#include <Arduino.h>

#include "src_group/dRehmFlight.h"
//#include "sdreadwrite.h"

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);

    // Initialize IMU communication
    IMUinit();
    calculate_IMU_error();
    delay(5);

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
    getIMUdata();                                                              // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

    Serial.print(AccX);
    Serial.print(" ");
    Serial.print(AccY);
    Serial.print(" ");
    Serial.println(AccZ);
}
