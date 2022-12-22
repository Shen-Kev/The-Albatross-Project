/*
actually this kinda seems redundant...



#include "src_group/dRehmFlight.h"
#include "src_group/ToF/VL53L1X.h"
#include "BMP180.h"

//State variables:

attitude  (IMU orientation and compass heading)
rotation rate of change (IMU gyroscope)
lateral movement (IMU acceleration)
altitude (IMU, ToF, baro)
airspeed

//code also needs to calibrate the sensors, and point them in the right direction using servos

//FINAL STATE VECTOR VALUES:
float roll_angle;
float roll_rate;
float pitch_angle;
float pitch_rate;
float yaw_angle; //relative to magnetic north
float yaw_rate;
float horizontal_acceleration;  //always horizontal (parallel to ground), even when banking
float altitude; //distance to ground
 

//altitude related variables, like prev, and stuff, and angle of servo motor needed
//maybe kalman filter... idk 


void calibrateAltitude() {

}

void calculateAltitude() {

}

*/
