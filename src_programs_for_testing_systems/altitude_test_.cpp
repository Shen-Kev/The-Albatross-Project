/*//havent gotten to work really yet, but its getting to time consuming to do all this in separate files, im gonna just make the final cpp file now and then just edit the header files.
#include <Arduino.h>
#include "src_group/dRehmFlight.h" // modified dRehmFlight has stabilization code and actuation and IMU communication and reciever communication.
#include "src_group/dataLog.h"
#include "src_group/ToF/VL53L1X.h"
#include "BMP180.h"
#include "ASPD4525.h"
#include "altitude.h"

float gimbalServoGain = 2;

void setup()
{
  Serial.begin(500000); // USB serial
  delay(500);
  servo6.attach(servo6Pin, 900, 2100);
  IMUinit();
  VL53L1Xsetup();
  BMP180setup();
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
  calculate_IMU_error();
}

void loop()
{
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
  loopBlink();                                                               // Indicate we are in main loop with short blink every 1.5 seconds
  getIMUdata();                                                              // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  servo6.write(roll_IMU * gimbalServoGain + 90);
  VL35L1Xloop();
  BMP180loop();
  calculateAltitude();
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
                  // Get vehicle state
}
*/