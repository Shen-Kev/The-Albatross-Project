/*
 *   Project Title:
 *   Wind Powered Flight: Exploring the Potential of Dynamic Soaring for Unmanned Aerial Vehicles
 *
 *   Code in this file fully written by Kevin Shen. Other files and libraries have been modified but are mostly unoriginal
 *   This is the main program that runs on the UAV's Teensy 4.1 flight computer
 *   During startup, the UAV should be level, unmoving, and pointed towards the wind
 *   Start with manual flight, then turn to stabilized flight to let the PID loops settle, then activate DS flight
 *
 *   Repository created November 30, 2022. Found at https://github.com/Shen-Kev/The-Albatross-Project
 *   File created December 21, 2022
 *   Project ends Feburary 11, 2023
 *
 */

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// INCLUDE FILES AND LIBRARIES
#include <Arduino.h>               //  Standard Arduino file
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu
                                   //

#include "BMP180nonblocking/BMP085NB.h" //https://github.com/Lithium366/BMP085
#include <Wire.h>
#include "pololuVL53L1x/VL53L1X.h"       //https://github.com/pololu/vl53l1x-arduino
#include "ASPD4525.h"                    //Library to interface with the ASPD4525 airspeed sensor
                                         // Sensor originally meant to work with Ardupilot flight computers, and thus needed to be experimentally tuned
#include <SD.h>                          // Library to read and write to the microSD card port
#include "AltitudeEstimation/altitude.h" // Library to combine barometric sensor and IMU in a two step Kalman-Complementary filter to estimate altitude
// #include "AltitudeEstimation/filters.cpp"

// https://github.com/sparkfun/BMP180_Breakout

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// FLIGHT COMPUTER SPECS AND PINOUT (edited on dRehmFlight.h)
/*
 * All sensors on I2C:
 * MPU6050 Inertial Measrement Unit (IMU)
 * BMP180 Barometric Pressure Sensor (baro)
 * GY-271 Magnometer (currently unused)
 * VL53L1X Time Of Flight (ToF) sensor
 * Matek ASPD4525 Pitot tube and airspeed sensor
 *
 * PWM signal from reciever:
 * Throttle channel: D2
 * Roll channel: D3
 * Pitch channel: D4
 * Yaw channel: D5
 * Mode1: D6
 * Mode2: D7
 *
 * Actuator outputs:
 * Electronic Speed Controller (ESC): D8
 * Left aileron (both ailerons) servo: D9
 * Right aileron (currently not used) servo: D10
 * Rudder servo: D24
 * Elevator servo: D25
 * Gimbal servo 1: D28
 * Gimbal servo 2: D29 (currently not used)
 */

// THINGS TO DO:
// find what data visualization program BPS uses or how to make it fancy
// edit csv data analyzing tool in python
// tune values that need to be tuned (also in drehmflight)
// get compass working if needed

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// DEBUG AND TESTING #IFS

// Throttle cut for safety:
#define MOTOR_ACTIVE 0
#define DATALOG 1

// Set only one of the below to TRURE to test. If all are false it runs the standard setup and loop. Can use this to test all systems, and also test flight mode 1 and 2 (servos, ESC, radio, PID, coordinated turns)

// Basic, individual systems ground test programs (with serial connection). Mostly for troubleshooting individual components
#define TEST_TOF 0            // Time of flight sensor test
#define TEST_AIRSPEED 0       // Airspeed sensor test
#define TEST_IMU 0            // IMU sensor test
#define TEST_BARO 0           // barometer sensor test
#define TEST_RADIO 0          // radio sensor test
#define TEST_RADIO_TO_SERVO 0 // servo test
#define TEST_SERIAL 0         // serial output test
#define TEST_SD 0             // SD write test

#define TEST_DREHMFLIGHT 0

// Combined systems ground test programs with serial connection
#define PID_TUNE 0               // Uses the gimbal rig to tune PID pitch and roll loops, and validate/tune airspeed, but monitor the throttle PID
#define TEST_ALTITUDE_RIG 0      // Uses the altitude rig to estimate the altitude of the UAV
#define IMU_ALTITUDE_TEST 0      // test IMU only for estimated altitude
#define TEST_HORIZONTAL_MOTION 0 // Tests the horizontal motion estimation

// DS flight test codes without serial connection. ALl of these are in flight mode 3 in the main flight code, flight mode 1 and 2 are standard and should be tested using the FULL_FLIGHT_CODE
#define LOW_ALTITUDE_FLIGHT 0       // Flies the UAV constantly at the low altitude and at constant flight speed
#define LOW_ALTITUDE_HORIZ_FLIGHT 0 // FLies the UAV constantly at low altitude, and completes the horizontal sine wave
#define DS_AT_ALTITUDE 0            // Flies the UAV as if it was DS, but at regular flight altitude. probabily will never use

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// PROGRAM VARIABLES AND OBJECTS

// Variables for the gimbaling servo that rotates the ToF sensor
const float gimbalServoGain = -1.5;          // The ratio between the angle the servo head moves to the angle outputted to it
const float gimbalServoTrim = 0;             // NEEDS TO BE ADJUSTED: Gimbal servo trim (degrees counterclockwise)
const float gimbalServoBound = 45;           // The angle the servo can rotate clockwise or counterclockwise from center (used to detect the gimbal 'maxing out')
const float halfWingspan = 0.75;             // The wingspan of the UAV / 2 (in meters)
const float gimbalDistanceFromCenter = 0.14; // The location of the ToF sensor and gimbal from the center of the wing (distance to the left in meters)
float gimbalRightBoundAngle;                 // The clockwise bound of the servo, based on the servo bound and trim constants
float gimbalLeftBoundAngle;                  // The counterclockwise bound of the servo, based on the servo bound and trim constants

// Variables for sensing airspeed
float airspeed_unadjusted;            // The raw airspeed reading from the ASPD4525 sensor, with a low pass filter applied
float airspeed_prev;                  // The previous raw airspeed reading from the ASPD4525 sensor (used in the low pass filter)
const float airspeed_LP_param = 0.02; // The low pass filter parameter for airspeed (smaller values means a more smooth signal but higher delay time)
float airspeed_offset = 0;            // The amount of airspeed the airspeed sensor detects when real airspeed is 0m/s. Used to zero the airspeed sensor
const float airspeed_scalar = 1.8;    // LIKELY NEEDS TO BE RE-ADJUSTED: The scalar applied to the unadjusted (but zeroed) airspeed to produce the adjusted airspeed
float airspeed_adjusted;              // the airspeed adjusted to be as accurate as possible (tuned to be best around flight speed)
float airspeed_adjusted_prev;         // the previous airspeed adjusted reading. Used to estimate the derivative (slope of secant line) in the PID loop

// Variables for altitde sensing
// float altitude_offset;                   // The difference between the barometer's altitude reading and the 'real' altitude (in m) which is assumed to be 0 at ground level
// const int altitude_offset_num_vals = 1000; // The number of sensor readings that get averaged to adjust the altitude offset
// int offset_loop_counter = 0;             // Tracks the number of sensor readings so far
// float altitude_offset_sum = 0.0;         // Tracks the sum of all the sensor reading so far
// float altitude_baro;                     // The altitude estimated from the barometer, with a low pass filter and offset adjustment applied
// float altitude_prev;                     // The previous reading of the barometric pressure sensor
// float altitude_LP_param = 0.05;          // The low pass filter parameter for altitude (smaller values means a more smooth signal but higher delay time)
// float altitudeMeasured;                  // The raw altitude reading from the barometric pressure sensor (in m)
// long pressure;                           // Pressure measured by the BMP180
// int temperature = 0.0;                   // set temperature just to make bmp180 happy, in reality its replaced by the offset feature
// double referencePressure;                // Pressure measured by the BMP180 at startup

// ToF altitude sensing
float ToFaltitude; // The estimated altitude by the Time of Flight sensor (in m). Derived from distance_LP, and therefore has low pass filter applied. Works up to 4m, but NEED TO TEST RANGE
float prevToFaltitude;
int16_t distance;
float distance_LP_param = 0.5;
float distancePrev;
float distance_LP;
float leftWingtipAltitude;  // The estimated and calculated altitude of the left wingtip based on roll angle and the ToF sensor
float rightWingtipAltitude; // The estimated and calculated altitude of the right wingtip based on roll angle and the ToF sensor
const int IRQpin = 35;
const int XSHUTpin = 34;

float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos; // the vertical acceleration, velocity, and position estimated by the IMU
// float IMU_horizontal_accel, IMU_horizontal_vel, IMU_horizontal_pos; // the horizontal acceleration, velocity, and position estimated by the IMU
float IMU_vertical_accel_LPparam = 0.02;
float IMU_vertical_accel_prev;

float estimated_altitude; // The estimated altitude of the UAV, as a combination of the ToF, IMU, and baro sensor
int altitudeTypeDataLog;  // To record the type of altitude the UAV is using as its estimated altitude. 0 is ToF within gimbal range, 1 is ToF too far left, 2 is ToF too far right, and 3 is using IMU only

float timeInMillis;      // Time in milliseconds since the flight controller was reset
int loopCounter = 0;     // A counter used to log data every certain number of loops
int loopCounterStep = 0; // counter used for gimbal mount test rig

// Dynamic soaring state variables
const float wind_heading = 0.0;                          // The heading the wind is coming from. For now it is assumed it is 0 degrees relative to the yaw IMU at startup. In the future can be manually set if absolute compass is added
const float DS_heading = 90.0;                           // The yaw heading of the overall DS flight path, perpendicular to the wind. Flying to the right relative to the wind.
const float heading_setup_tolerance = 5;                 //  NEEDS TO BE ADJUSTED: The tolerance of the heading allowed while setting up the DS flight path (deg)
const float heading_rate_of_change_setup_tolerance = 10; //  NEEDS TO BE ADJUSTED: The tolerance of the heading rate of change while setting up the DS flight path (deg/s)
const float pitch_rate_of_change_setup_tolerance = 10;   //  NEEDS TO BE ADJUSTED: The tolerance of the pitch rate of change while setting up the DS flight path (deg/s)
const float horizontal_vel_tolerance = 0.5;              //  NEEDS TO BE ADJUSTED: The tolerance of the horizontal velocity while setting up the DS flight path (m/s)

float DS_altitude_setpoint;              // The altitude setpoint while dynamic soaring
float DS_altitude_error;                 // The difference between the altitude setpoint and the estimated altitude
float DS_altitude_error_prev;            // The previous altitude error, used to differentiate the error curve
const float DS_altitude_min = 0.3;       // NEEDS TO BE ADJUSTED: The altitude (in meters) that the UAV should fly at while at the lowest phase of dynamic soaring. Could based on the bumpiness (std) of the water
const float DS_altitude_tolerance = 0.1; // NEEDS TO BE ADJUSTED: The altitude (in meters) that the UAV must be close to the setpoint to have met the setpoint
const float DS_altitude_max = 3.5;       // NEEDS TO BE ADJUSTED:The altitude (in meters) that the UAV should fly at to be most influenced by the wind shear layer
float DS_altitude_meanline;
float DS_altitude_amplitude;

const float DS_period = 5000;      // the period of the sine wave in milliseconds
const float DS_yaw_amplitude = 30; // the amount of degrees to yaw to either side while dynamic soaring
double DS_phase_timer = 0;
double DS_phase_start_time;

float DS_heading_rate_setpoint; // the desired heading while DS
float DS_heading_rate_mean_setpoint; //the mean desired heading while DS, set by pilot
float heading_rate_scalar = 100; //scales from -0.5 and 0.5 to 50 and 50 deg/s while DS
// float DS_horizontal_accel;                                // The global horizontal acceleration (perpendicular to the dynamic soaring line, parallel to the wind direction) (in g's)
// float DS_horizontal_accel_error;                          // The difference between the global horizontal acceleration setpoint and measured horizontal acceleration
// const float DS_horizontal_accel_setpoint_phase_2_3 = 2.0; //  NEEDS TO BE ADJUSTED: The setpoint acceleration (in g's) while accelerating in the wind (DS phase 2 and 3)

// float DS_horizontal_vel;                                 // The global horizontal velocity (perpendicular to the dynamic soaring line, parallel to the wind direction) (in m/s)
// float DS_horizontal_vel_error;                           // The differnece between the global horizontal velocity setpoint and the measured horizontal velocity
// float DS_horizontal_pos;                                 // The global horizontal position relative to the dynamic soaring line. Left of the line is negative, right of the line positive, and the center is set to be at 0 when the DS cycle starts
// const float DS_horizontal_vel_setpoint_phase_1_4 = -1.5; //  NEEDS TO BE ADJUSTED: The setpoint horizontal velocity (in m/s) while turning into the wind, below the shear layer

// float DS_horizontal_setpoint; // The global horizontal movement setpoint (can be velocity or acceleration, determined in the loop)
// int horiz_setpoint_type;      // variable to keep track what type of global horizontal setpoint is being used

enum horiz_setpoint_types
{
    setpoint_horiz_accel = 0, // Use global horizontal acceleration as the setpoint
    setpoint_horiz_vel = 1,   // Use global horizontal velocity as the setpoint
    setpoint_horiz_pos = 2,   // Use global horizontal acceleration as the setpoint

};

// Dynamic soaring phase variables
boolean DSifFirstRun = true; // The boolean which is true only if the mode switch goes from a non DS flight mode to the DS flight mode

// Flight mode variables
int flight_phase; // The flight mode (manual, stabilized, DS)
enum flight_phases
{
    // DS_phase_0 = 0, // DS Phase 0. UAV turns perpendicular to the wind flies low to the ground and stabilizes
    // DS_phase_1 = 1, // DS Phase 1. UAV turns into the wind, but below the shear layer to build up enough horizontal velocity
    // DS_phase_2 = 2, // DS Phase 2. Part of the DS cycle. UAV is facing the wind and above the shear layer, but is accelerating away from the wind, harvesting the wind's energy
    // DS_phase_3 = 3, // DS Phase 3. Part of the DS cycle. UAV is now facing away from the wind and descending towards the shear layer. Still accelerating away from the wind, havesting energy
    // DS_phase_4 = 4, // DS Phase 4. Part of the DS cycle. UAV is facing away from the wind and under the shear layer, but turning back towards the wind to build up velocity to start the DS cycle over
    // DS phase goes from 0 to 2pi, (6.28)
    manual_flight = 7,
    stabilized_flight = 8,
    dynamic_soaring_flight = 9,
    log_data_to_SD = 10
};

// // Horizontal PID controller values
// float K_horiz_accel = 1.0;                   //  NEEDS TO BE ADJUSTED: Global horizontal acceleration proportional gain
// float Kp_horiz_vel = 1.0;                    //  NEEDS TO BE ADJUSTED: Global horizontal velocity proportional gain
// float Ki_horiz_vel = 0.1;                    //  NEEDS TO BE ADJUSTED:Global horizontal velocity integral gain
// float horiz_vel_integral = 0.0;              //  Variable to keep track of global horizontal velocity integral
// float horiz_vel_integral_prev = 0.0;         //  The previous global horizontal velocity integral (to be integrated upon)
// float horiz_integral_saturation_limit = 2.0; //  NEEDS TO BE ADJUSTED:Altitude integral saturation limit (to prevent integral windup)

// Altitude PID controller values
const float Kp_altitude = 0.2;                         //  NEEDS TO BE ADJUSTED:// Altitude proportional gain
const float Ki_altitude = 0.3;                         //  NEEDS TO BE ADJUSTED:// Altitude integral gain
const float Kd_altitude = 0.0015;                      //  NEEDS TO BE ADJUSTED:// Altitude derivative gain
float altitude_integral = 0.0;                         //  Variable to keep track of altitude integral
float altitude_integral_prev = 0.0;                    //  Previous altitude integral (to be integrated upon)
const float altitude_integral_saturation_limit = 25.0; //  NEEDS TO BE ADJUSTED: Altitude integral saturation limit (to prevent integral windup)
float altitude_derivative;                             // Variable to keep track of altitude derivative

// // Coordinated turn variables
// float rudderCoordinatedCommand;                               // The command (-1 to 1) to give to the rudder to have a coordinate turn
// float acceleration_downwards_angle;                           // The roll angle of the downwards acceleration the UAV experiences (deg)
// float acceleration_downwards_angle_prev;                      // The previous roll angle of the downwards acceleration the UAV experiences (deg)
// float acceleration_downwards_angle_LP;                        // The roll angle of the downwards acceleration the UAV experiences (deg), with a low pass filter applied
// const float acceleration_downwards_angle_LP_param = 0.1;      //  NEEDS TO BE ADJUSTED: The low pass paramter for the roll angle of the downwards acceleration the UAv experiences
// float acceleration_downwards_magnitude;                       // The magnitude of the downwards acceleration about the roll axis the UAV experiences
// const float acceleration_downwards_magnitude_tolerance = 0.5; //  NEEDS TO BE ADJUSTED: The acceleration needed to be experienced by the UAV to legitimzie the acceleration angle (g's)

// const float Kp_coord = 0.2;                         //  NEEDS TO BE ADJUSTED:// Coordinated turn P gain
// const float Ki_coord = 0.3;                         //  NEEDS TO BE ADJUSTED:// Coordinated turn I gain
// const float Kd_coord = 0.0015;                      //  NEEDS TO BE ADJUSTED:// Coordinated turn D gain
// float coord_integral = 0.0;                         // Coordinated turn error PID controller integral
// float coord_integral_prev;                          // Previous coordinated turn PID controller integral
// const float coord_integral_saturation_limit = 25.0; //  NEEDS TO BE ADJUSTED: Coordinated turn saturation limit (to avoid integral windup)
// float coord_derivative;                             // Coordinated turn PID controller derivative

// Throttle PID controller values
float throttle_PID;                         // The throttle outputted by the PID loop to maintain constant airspeed (range between 0.0-1.0, where 0.0 is min throttle and 1.0 is max throttle)
float airspeed_setpoint;                    // The setpoint airspeed for the throttle PID loop
const float flight_speed = 20.0;            // NEEDS TO BE ADJUSTED: The airspeed in m/s for regular flight
boolean motorOn = false;                    // Activates or deactivates the motor
const float stall_speed = 10.0;             // NEEDS TO BE ADJUSTED: The airspeed in m/s to always stay above to avoid stall. Essentially the minimum airspeed to always stay above
float airspeed_error;                       // The error between setpoint airspeed and current airspeed
float airspeed_error_prev;                  // The previous error, used for derivative estimation
const float airspeed_error_tolerance = 1.0; // NEEDS TO BE ADJUSTED: The range of airspeeds to be within to be 'close enough' to the setpoint
float inputted_airspeed;

float Kp_throttle; //  NEEDS TO BE ADJUSTED:Throttle Proportional gain
float Ki_throttle; //  NEEDS TO BE ADJUSTED:Throttle Integral gain
float Kd_throttle; //  NEEDS TO BE ADJUSTED:Throttle Derivative gain

float throttle_integral = 0.0;                         // Throttle integral value (approximated with summation)
float throttle_integral_prev = 0.0;                    // Throttle previous integral value to be integrated upon
const float throttle_integral_saturation_limit = 50.0; // NEEDS TO BE ADJUSTED: Throttle integral saturation limit to prevent integral windup
float throttle_derivative;                             // Throttle derivative value
float throttle_PID_prev;
const float throttle_LP_param = 0.01;

// Roll, pitch, and yaw angles but in radians for easier math
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad;

// Variables for the Runge-Kutta integration method
// float k1_vel, k1_pos, k2_vel, k2_pos, k3_vel, k3_pos, k4_vel, k4_pos;

// used to estimate velocity of IMU
int ToFcounter = 0;
int ToFcounterNum = 200;

// Program Objects
BMP085NB bmp;
File dataFile;                                                // Object to interface with the microSD card
KalmanFilter kalmanHoriz(0.5, 0.01942384099, 0.001002176158); // Kalman filter for the horizontal

// Kalman filter for the vertical position
KalmanFilter kalmanVert(0.5, 0.01942384099, // standard deviation of gyroscope measurements
                        0.001002176158);    // standard deviation of accelerometer measurements
VL53L1X sensor;

float accelData[3];
float gyroData[3];

// Data logging variables
const int COLUMNS = 15;            // Columns in the datalog array
const int ROWS = 6400;             // Rows in the datalog array
float dataLogArray[ROWS][COLUMNS]; // Create the datalog array. Columns are the variables being printed, and rows are logs at different times
int currentRow = 0;                // Keeps track of the row the data should be logged into
const int datalogRate = 50;        // NEEDS TO BE ADJUSTED: Data logging rate in Hz
const int dataLogRateSlow = 10;    // Data logging rate when not important, just to show the setup and different phases of flight
boolean dataLogged = false;
// Altitude estimator to combine barometric pressure sensor (with low pass filter applied) with the gyroscope and acclerometer
// AltitudeEstimator altitudeLPbaro = AltitudeEstimator(0.001002176158, // Sigma (standard deviation of) the accelerometer
//                                                      0.01942384099,  // Sigma (standard deviation of) the gyroscope
//                                                      0.1674466677,   // sigma (standard deviation of) the barometer
//                                                      0.5,            // ca (don't touch)
//                                                      0.1);           // accel threshold (if there are many IMU acceleration values below this value, it is assumed the aircraft is not moving vertically)

// Timing variables
unsigned long ToF_test_time_in_micros;
unsigned long ToF_test_start_time;
unsigned long IMU_test_start_time;
unsigned long IMU_test_time_in_micros;
unsigned long baro_test_start_time;
unsigned long baro_test_time_in_micros;
unsigned long pitot_test_start_time;
unsigned long pitot_test_time_in_micros;
unsigned long all_sensors_start_time;
unsigned long all_sensors_time;

boolean toggle = false; // toggle between ToF and airspeed

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// FUNCTION DECLARATIONS

void dynamicSoar();
void DSattitude();
void coordinatedController();
void throttleController();
void horizontal();
void estimateAltitude();
void pitotSetup();
void pitotLoop();
// void BMP180setup();
// void BMP180loop();
void setupSD();
void logDataToRAM();
void clearDataInRAM();
void writeDataToSD();
void VL53L1Xsetup();
void VL53L1Xloop();

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// INDIVIDUAL TEST PROGRAMS
#if TEST_TOF
void setup()
{
    Serial.begin(500000);
    VL53L1Xsetup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopRate(2000);
    V53L1Xloop();
    if (loopCounter > 50)
    {
        Serial.print(" Distance: ");
        Serial.print(distance);
        Serial.print(" DistanceLP: ");
        Serial.print(distance_LP);
        Serial.println(" mm");
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }
}

#elif TEST_AIRSPEED
void setup()
{
    Serial.begin(500000); // USB serial
    Wire.begin();
    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

    pitotSetup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    pitotLoop();
    Serial.print(" airspeed unadjusted m/s: ");
    Serial.print(airspeed_unadjusted);
    Serial.print(" airspeed adjusted m/s: ");
    Serial.print(airspeed_adjusted);
    Serial.print(" airspeed adjusted mph: ");
    Serial.print(airspeed_adjusted * 2.23694);
    Serial.println();
    loopRate(2000);
}
#elif TEST_IMU

void setup()
{
    Serial.begin(500000); // USB serial
    Wire.begin();
    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

    IMUinit();
    delay(5);

    calculate_IMU_error();
    // AccErrorY = 0.04;
    // AccErrorZ = 0.11;
    // GyroErrorX = -3.20;
    // GyroErrorY = -0.14;
    // GyroErrorZ = -1.40;
    // delay(1000);
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopRate(2000);
    getIMUdata();

    Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);

    //  Madgwick6DOF(-GyroX, GyroY, GyroZ, AccX, -AccY, -AccZ, dt);

    Serial.print(roll_IMU);
    Serial.print(" ");
    Serial.print(pitch_IMU);
    Serial.print(" ");
    Serial.print(yaw_IMU);
    Serial.print("    ");
    Serial.print(AccX);
    Serial.print(" ");
    Serial.print(AccY);
    Serial.print(" ");
    Serial.print(AccZ);
    Serial.print("    ");
    Serial.print(GyroX);
    Serial.print(" ");
    Serial.print(GyroY);
    Serial.print(" ");
    Serial.print(GyroZ);
    Serial.println();
}

/*
#elif TEST_BARO
void setup()
{
    Serial.begin(500000); // USB serial
    BMP180setup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    loopBlink();
    loopRate(2000);
    BMP180loop();
    // Serial.print(pressureMeasured);
    // Serial.print(" ");
    // Serial.print(pressure);
    // Serial.print(" ");
    Serial.print(altitudeMeasured);
    Serial.print(" ");
    Serial.print(altitude_baro);
    Serial.println();
}
*/
#elif TEST_RADIO
void setup()
{
    Serial.begin(500000);
    radioSetup();
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getCommands();
    getDesState();
    // Serial.print(" channel 1: ");
    // Serial.print(throttle_channel);
    // Serial.print(" channel 2: ");
    // Serial.print(roll_channel);
    // Serial.print(" channel 3: ");
    // Serial.print(pitch_channel);
    // Serial.print("channel 4: ");
    // Serial.print(yaw_channel);
    // Serial.print(" channel 5: ");
    // Serial.print(mode1_channel);
    // Serial.print(" channel 6: ");
    // Serial.print(mode2_channel);
    Serial.print(" thro des: ");
    Serial.print(thro_des);
    Serial.print(" roll des: ");
    Serial.print(roll_des);
    Serial.print(" pitch des: ");
    Serial.print(pitch_des);
    Serial.print(" yaw des: ");
    Serial.print(yaw_des);
    Serial.println();
    // Serial.println();
    loopRate(2000);
}
#elif TEST_RADIO_TO_SERVO

void setup()
{
    Serial.begin(500000);
    radioSetup();
    // Attach actuators to PWM pins
    ESC.attach(ESCpin, 900, 2100);
    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    gimbalServo.attach(gimbal1ServoPin, 900, 2100);
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getCommands();
    getDesState();
    s1_command_scaled = thro_des;        // Between 0 and 1
    s2_command_scaled = roll_passthru;   // Between -0.5 and 0.5
    s3_command_scaled = pitch_passthru;  // Between -0.5 and 0.5
    s4_command_scaled = yaw_passthru;    // Between -0.5 and 0.5
    scaleCommands();                     // Scales commands to values that the servo and ESC can understand
    aileronServo.write(s2_command_PWM);  // aileron
    elevatorServo.write(s3_command_PWM); // elevator
    rudderServo.write(s4_command_PWM);   // rudder
    gimbalServo.write(s5_command_PWM);   // gimbal
    loopRate(2000);

    Serial.print("channel 1: ");
    Serial.print(thro_des);
    // Serial.print("channel 2: ");
    // Serial.print(roll_passthru);
    // Serial.print("channel 3: ");
    // Serial.print(pitch_passthru);
    // Serial.print("channel 4: ");
    // Serial.print(yaw_passthru);

    Serial.print("channel 2: ");
    Serial.print(s2_command_PWM);
    Serial.print("channel 3: ");
    Serial.print(s3_command_PWM);
    Serial.print("channel 4: ");
    Serial.print(s4_command_PWM);

    Serial.println();
}

#elif TEST_SERIAL
void setup()
{
    Serial.begin(500000);
}

void loop()
{
    Serial.println("Hello World!");
}

#elif TEST_SD
void setup()
{
    delay(500);
    setupSD();
}

void loop()
{

    loopRate(2000);
    dataFile = SD.open("flightData.txt", FILE_WRITE);
    dataFile.print(millis());
    dataFile.print(" ");
    dataFile.print("SD Write Test");
    dataFile.println();
    dataFile.close();
}
#elif TEST_DREHMFLIGHT // THIS IS OUT OF DATE< WILL NOT WORK

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    Wire.begin();
    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

    // ESC.attach(ESCpin, 900, 2100); // Pin, min PWM value, max PWM value
    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    gimbalServo.attach(gimbal1ServoPin, 900, 2100);
    servo6.attach(gimbal2ServoPin, 900, 2100);
    servo7.attach(servo7Pin, 900, 2100);

    // Set built in LED to turn on to signal startup
    digitalWrite(13, HIGH);

    delay(5);

    // Initialize radio communication
    radioSetup();

    // Set radio channels to default (safe) values before entering main loop
    throttle_channel = throttle_fs;
    roll_channel = roll_fs;
    pitch_channel = pitch_fs;
    yaw_channel = yaw_fs;
    mode1_channel = mode1_fs;
    mode2_channel = mode2_fs;

    // Initialize IMU communication
    IMUinit();

    delay(5);

    // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
    // calculate_IMU_error(); // Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
    AccErrorY = 0.04;
    AccErrorZ = 0.11;
    GyroErrorX = -3.20;
    GyroErrorY = -0.14;
    GyroErrorZ = -1.40;
    // delay(10000);
    // Arm servo channels
    ESC.write(0);           // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
    aileronServo.write(0);  // Set these to 90 for servos if you do not want them to briefly max out on startup
    elevatorServo.write(0); // Keep these at 0 if you are using servo outputs for motors
    rudderServo.write(0);
    gimbalServo.write(0);
    servo6.write(0);
    servo7.write(0);

    delay(5);

    // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
    // Code will not proceed past here if this function is uncommented!

    // Indicate entering main loop with 3 quick blinks
    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

    // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
    // calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section
}

void loop() // for the setup and loop, ill prob just use this as the start for the actual setup and
{
    // Keep track of what time it is and how much time has elapsed since the last loop
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

    // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    // printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
    // printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
    // printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    // printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    // printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
    // printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    // printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    // printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
    // printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
    // printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

    // Get vehicle state

    test_time_in_micros_start = micros();
    getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    test_time_in_micros = micros() - test_time_in_micros_start;
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

    // Compute desired state
    getDesState(); // Convert raw commands to normalized values based on saturated control limits

    // PID Controller - SELECT ONE:
    controlANGLE(); // Stabilize on angle setpoint
    // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
    // controlRATE(); //Stabilize on rate setpoint

    // Actuator mixing and scaling to PWM values
    controlMixer();  // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    // Throttle cut check
    // throttleCut(); // Directly sets motor commands to low based on state of ch5

    // Command actuators
    // commandMotors();           // Sends command pulses to each motor pin using OneShot125 protocol
    // ESC.write(s1_command_PWM); // Writes PWM value to servo object
    aileronServo.write(s2_command_PWM);
    elevatorServo.write(s3_command_PWM);
    rudderServo.write(s4_command_PWM);
    gimbalServo.write(s5_command_PWM);
    servo6.write(s6_command_PWM);
    servo7.write(s7_command_PWM);

    // Get vehicle commands for next loop iteration
    getCommands(); // Pulls current available radio commands
    failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
                   // printLoopRate();
    Serial.println(test_time_in_micros);
    // Regulate loop rate
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

#else

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// MICROCONTROLLER SETUP
void setup()
{
    // orientation PID parameters
    Kp_roll_angle = 0.3;
    Ki_roll_angle = 0.3;
    Kd_roll_angle = 0.3;

    Kp_pitch_angle = 0.5;
    Ki_pitch_angle = 0.3;
    Kd_pitch_angle = 0.3;

    Kp_yaw = 0.5;
    Ki_yaw = 0.3;
    Kd_yaw = 0.0015;

    Kp_throttle = 5.0;
    Ki_throttle = 1.0;
    Kd_throttle = 0.0;

    Serial.begin(500000); // USB serial
    Serial.println("serial works");
    Wire.begin();
    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

    //    delay(15 * 1000); // Delay to have enough time to push reset, close hatch, and place UAV flat on the ground and into the wind

    pinMode(13, OUTPUT); // LED on the Teensy 4.1 set to output

    // Attach actuators to PWM pins
    ESC.attach(ESCpin, 1100, 2100);
    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    gimbalServo.attach(gimbal1ServoPin, 900, 2100);
    Serial.println("passed attach");

    delay(100);

    // Setup and calibrate communciations
    radioSetup(); // R/c reciever
    Serial.println("passed radio setup");
    IMUinit(); // IMU init
    Serial.println("passed IMU init");
    // calculate_IMU_error(); // IMU calibrate
    // Serial.println("passed IMU calibration");

    // calibrated values on 1/12/23
    AccErrorY = 0.04;
    AccErrorZ = 0.11;
    GyroErrorX = -3.20;
    GyroErrorY = -0.14;
    GyroErrorZ = -1.40;
    delay(10000);

    // run the code a bunch of times to let IMU settle
    for (int i = 0; i < 1000; i++)
    {
        getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
        Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    }
    // BMP180setup(); // Barometer init and calibrate
    // Serial.println("passed baro setup");
    VL53L1Xsetup(); // ToF sensor init
    Serial.println("passed ToF setup");
    pitotSetup(); // Airspeed sensor init and calibrate
    Serial.println("passed pitot");
#if DATALOG || PID_TUNE
    clearDataInRAM();
    setupSD(); // microSD card read/write unit
#endif

    // Set R/c reciever channels to failsafe values
    throttle_channel = throttle_fs;
    roll_channel = roll_fs;
    pitch_channel = pitch_fs;
    yaw_channel = yaw_fs;
    mode1_channel = mode1_fs;
    mode2_channel = mode2_fs;

    delay(100);

    // calibrateESCs();

    // Set actuator values to safe values
    ESC.write(0); // ESC throttle off
    aileronServo.write(90);
    elevatorServo.write(90);
    rudderServo.write(90);
    gimbalServo.write(90);

    delay(100);

    // Set gimbal bounds
    gimbalRightBoundAngle = (0 - gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);
    gimbalLeftBoundAngle = (0 + gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);

    DS_altitude_meanline = (DS_altitude_max+DS_altitude_min)/2.0;
    DS_altitude_amplitude = (DS_altitude_max-DS_altitude_min)/2.0;
// #if DS_AT_ALTITUDE
//         // UAV does the DS at 10m up to avoid hitting the ground
//         DS_altitude_terrain_following += 10;
//     DS_altitude_in_wind += 10;
// #endif
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// MICROCONTROLLER LOOPS

void loop()
{

    // Timing
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0; // Time between loop iterations, in seconds
    timeInMillis = millis();

    all_sensors_start_time = micros();
    // Retrieve sensor data
    //  IMU_test_start_time = micros();
    getIMUdata(); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    //  IMU_test_time_in_micros = micros() - IMU_test_start_time;
    accelData[0] = AccX;
    accelData[1] = AccY;
    accelData[2] = AccZ;

    gyroData[0] = GyroX * DEG_TO_RAD;
    gyroData[1] = GyroY * DEG_TO_RAD;
    gyroData[2] = GyroZ * DEG_TO_RAD;
    // baro_test_start_time = micros();
    // BMP180loop(); // Retrieves barometric altitude and LP filters
    // baro_test_time_in_micros = micros() - baro_test_start_time;

    // toggles between the two functions every other loop
    if (toggle)
    {
        VL53L1Xloop(); // Retrieves ToF sensor distance
    }
    else
    {
        pitotLoop(); // Retrieves pitot tube airspeed
    }
    toggle = !toggle;

    getCommands(); // Retrieves radio commands
    failSafe();    // Failsafe in case of radio connection loss
    all_sensors_time = micros() - all_sensors_start_time;

    estimateAltitude();

    // Convert roll, pitch, and yaw from degrees to radians
    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD; // positive pitch is up
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;   // positive roll is roll to the right
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;     // positive yaw is yaw to the right

    getDesState(); // produces thro_des, roll_des, pitch_des, yaw_des, roll_passthru, pitch_passthru, yaw_passthru

#if PID_TUNE

    // use serial to get input. MAKE SURE USING "NO LINE ENDING" OPTION ON THE SERIAL MONITOR DROPDOWN

    // write the axis(r for roll, p for pitch, y for yaw, t for throttle) LOWERCASE then gain (P, I, D) UPPERCASE

    char axis = Serial.read();
    char gain = Serial.read();

    if (Serial.available())
    {
        // ROLL GAINS
        if (axis == 'r')
        {
            if (gain == 'P')
            {
                Kp_roll_angle = Serial.parseFloat();
            }
            else if (gain == 'I')
            {
                Ki_roll_angle = Serial.parseFloat();
            }
            else if (gain == 'D')
            {
                Kd_roll_angle = Serial.parseFloat();
            }

            dataFile = SD.open("PID.txt", FILE_WRITE);

            dataFile.print("\t time:\t");
            dataFile.print(timeInMillis);
            dataFile.print("\troll P gain\t");
            dataFile.print(Kp_roll_angle);
            dataFile.print("\troll I gain \t");
            dataFile.print(Ki_roll_angle);
            dataFile.print("\troll D gain\t");
            dataFile.print(Kd_roll_angle);
            dataFile.print("\tpitch P gain\t");
            dataFile.print(Kp_pitch_angle);
            dataFile.print("\tpitch I gain \t");
            dataFile.print(Ki_pitch_angle);
            dataFile.print("\tpitch D gain\t");
            dataFile.print(Kd_pitch_angle);
            dataFile.print("\tyaw P gain\t");
            dataFile.print(Kp_yaw);
            dataFile.print("\tyaw I gain \t");
            dataFile.print(Ki_yaw);
            dataFile.print("\tyaw D gain\t");
            dataFile.print(Kd_yaw);
            dataFile.print("\tthrottle P gain\t");
            dataFile.print(Kp_throttle);
            dataFile.print("\tthrottle I gain \t");
            dataFile.print(Ki_throttle);
            dataFile.print("\tthrottle D gain\t");
            dataFile.print(Kd_throttle);
            dataFile.println();

            dataFile.close();
        }
        // PITCH GAINS
        else if (axis == 'p')
        {
            if (gain == 'P')
            {
                Kp_pitch_angle = Serial.parseFloat();
            }
            else if (gain == 'I')
            {
                Ki_pitch_angle = Serial.parseFloat();
            }
            else if (gain == 'D')
            {
                Kd_pitch_angle = Serial.parseFloat();
            }

            dataFile = SD.open("PID.txt", FILE_WRITE);

            dataFile.print("\t time:\t");
            dataFile.print(timeInMillis);
            dataFile.print("\troll P gain\t");
            dataFile.print(Kp_roll_angle);
            dataFile.print("\troll I gain \t");
            dataFile.print(Ki_roll_angle);
            dataFile.print("\troll D gain\t");
            dataFile.print(Kd_roll_angle);
            dataFile.print("\tpitch P gain\t");
            dataFile.print(Kp_pitch_angle);
            dataFile.print("\tpitch I gain \t");
            dataFile.print(Ki_pitch_angle);
            dataFile.print("\tpitch D gain\t");
            dataFile.print(Kd_pitch_angle);
            dataFile.print("\tyaw P gain\t");
            dataFile.print(Kp_yaw);
            dataFile.print("\tyaw I gain \t");
            dataFile.print(Ki_yaw);
            dataFile.print("\tyaw D gain\t");
            dataFile.print(Kd_yaw);
            dataFile.print("\tthrottle P gain\t");
            dataFile.print(Kp_throttle);
            dataFile.print("\tthrottle I gain \t");
            dataFile.print(Ki_throttle);
            dataFile.print("\tthrottle D gain\t");
            dataFile.print(Kd_throttle);
            dataFile.println();

            dataFile.close();
        }
        // YAW GAINS
        else if (axis == 'y')
        {
            if (gain == 'P')
            {
                Kp_yaw = Serial.parseFloat();
            }
            else if (gain == 'I')
            {
                Ki_yaw = Serial.parseFloat();
            }
            else if (gain == 'D')
            {
                Kd_yaw = Serial.parseFloat();
            }

            dataFile = SD.open("PID.txt", FILE_WRITE);

            dataFile.print("\t time:\t");
            dataFile.print(timeInMillis);
            dataFile.print("\troll P gain\t");
            dataFile.print(Kp_roll_angle);
            dataFile.print("\troll I gain \t");
            dataFile.print(Ki_roll_angle);
            dataFile.print("\troll D gain\t");
            dataFile.print(Kd_roll_angle);
            dataFile.print("\tpitch P gain\t");
            dataFile.print(Kp_pitch_angle);
            dataFile.print("\tpitch I gain \t");
            dataFile.print(Ki_pitch_angle);
            dataFile.print("\tpitch D gain\t");
            dataFile.print(Kd_pitch_angle);
            dataFile.print("\tyaw P gain\t");
            dataFile.print(Kp_yaw);
            dataFile.print("\tyaw I gain \t");
            dataFile.print(Ki_yaw);
            dataFile.print("\tyaw D gain\t");
            dataFile.print(Kd_yaw);
            dataFile.print("\tthrottle P gain\t");
            dataFile.print(Kp_throttle);
            dataFile.print("\tthrottle I gain \t");
            dataFile.print(Ki_throttle);
            dataFile.print("\tthrottle D gain\t");
            dataFile.print(Kd_throttle);
            dataFile.println();

            dataFile.close();
        }
        // THROTTLE GAINS
        else if (axis == 't')
        {
            if (gain == 'P')
            {
                Kp_throttle = Serial.parseFloat();
            }
            else if (gain == 'I')
            {
                Ki_throttle = Serial.parseFloat();
            }
            else if (gain == 'D')
            {
                Kd_throttle = Serial.parseFloat();
            }

            dataFile = SD.open("PID.txt", FILE_WRITE);

            dataFile.print("\t time:\t");
            dataFile.print(timeInMillis);
            dataFile.print("\troll P gain\t");
            dataFile.print(Kp_roll_angle);
            dataFile.print("\troll I gain \t");
            dataFile.print(Ki_roll_angle);
            dataFile.print("\troll D gain\t");
            dataFile.print(Kd_roll_angle);
            dataFile.print("\tpitch P gain\t");
            dataFile.print(Kp_pitch_angle);
            dataFile.print("\tpitch I gain \t");
            dataFile.print(Ki_pitch_angle);
            dataFile.print("\tpitch D gain\t");
            dataFile.print(Kd_pitch_angle);
            dataFile.print("\tyaw P gain\t");
            dataFile.print(Kp_yaw);
            dataFile.print("\tyaw I gain \t");
            dataFile.print(Ki_yaw);
            dataFile.print("\tyaw D gain\t");
            dataFile.print(Kd_yaw);
            dataFile.print("\tthrottle P gain\t");
            dataFile.print(Kp_throttle);
            dataFile.print("\tthrottle I gain \t");
            dataFile.print(Ki_throttle);
            dataFile.print("\tthrottle D gain\t");
            dataFile.print(Kd_throttle);
            dataFile.println();

            dataFile.close();
        }

        // AIRSPEED NOT USED ANYMORE
        //  else if (axis == 'A' && gain == 'A')
        //  {
        //      inputted_airspeed = Serial.parseFloat();
        //      dataFile = SD.open("airspeed.txt", FILE_WRITE);
        //      dataFile.print("\treal airspeed inputted\t");
        //      dataFile.print(inputted_airspeed);
        //      dataFile.print("\tcurrent measured unadusted airspeed\t");
        //      dataFile.print(airspeed_unadjusted);
        //      dataFile.print("\tcurrent measured adusted airspeed\t");
        //      dataFile.println(airspeed_adjusted);
        //  }
    }

    // log to SD card, takes around 0.13 seconds
    if (mode2_channel < 1500 || currentRow >= ROWS)
    {
        // s1_command_scaled = 0;
        // s2_command_scaled = 0;
        // s3_command_scaled = 0;
        // s4_command_scaled = 0;
        // s5_command_PWM = 90;
        // float datalogtimer = micros();
        // Serial.println("about to datalog");
        if (!dataLogged)
        {
            writeDataToSD();
            clearDataInRAM();
        }
        //  datalogtimer = micros() - datalogtimer;
        //  Serial.println(datalogtimer);
        //    delay(10000);
        dataLogged = true;
    }
    else
    {
        dataLogged = false;
    }
    if (loopCounter > (2000 / datalogRate))
    {
        logDataToRAM(); // log motions of the UAV
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }

    airspeed_setpoint = 15;
    motorOn = true;

    controlANGLE(); // dRehmFlight for angle based (pitch and roll) PID loops
    throttleController();

    s1_command_scaled = throttle_PID;
    s2_command_scaled = roll_PID;  // Between -1 and 1
    s3_command_scaled = pitch_PID; // Between -1 and 1
    s4_command_scaled = yaw_PID;

    // Serial.print("IAS: "); // indicated airspeed
    // Serial.print(airspeed_unadjusted);
    Serial.print(" airspeed: "); // true airspeed
    Serial.print(airspeed_adjusted);

    /*
      Serial.print(" ROLL SET: ");
      Serial.print(roll_des);
      Serial.print(" ROLL CMD: ");
      Serial.print(aileron_command_PWM);
      Serial.print(" ROLL Kp: ");
      Serial.print(Kp_roll_angle);
      Serial.print(" ROLL Ki: ");
      Serial.print(Ki_roll_angle);
      Serial.print(" ROLL Kd: ");
      Serial.print(Kd_roll_angle);
      //  Serial.print("\troll I val \t");
      //  Serial.print(integral_roll);

      //  Serial.print("\troll D val\t");
      //  Serial.print(derivative_roll);
      Serial.print(" PITCH SET: ");
      Serial.print(pitch_des);
      Serial.print(" PITCH CMD: ");
      Serial.print(elevator_command_PWM);
      Serial.print(" PITCH Kp: ");
      Serial.print(Kp_pitch_angle);
      Serial.print(" PITCH Ki: ");
      Serial.print(Ki_pitch_angle);
      Serial.print(" PITCH Kd: ");
      Serial.print(Kd_pitch_angle);

      Serial.print(" YAW SET: ");
      Serial.print(yaw_des);
      Serial.print(" YAW CMD: ");
      Serial.print(rudder_command_PWM);
      Serial.print(" YAW Kp: ");
      Serial.print(Kp_yaw);
      Serial.print(" YAW Ki: ");
      Serial.print(Ki_yaw);
      Serial.print(" YAW Kd: ");
      Serial.print(Kd_yaw);
  */
    Serial.print(" THROTTLE SET: ");
    Serial.print(airspeed_setpoint);
    Serial.print(" THROTTLE CMD: ");
    Serial.print(ESC_command_PWM);
    Serial.print(" THROTTLE Kp: ");
    Serial.print(Kp_throttle);
    Serial.print(" THROTTLE Ki: ");
    Serial.print(Ki_throttle);
    Serial.print(" THROTTLE Kd: ");
    Serial.print(Kd_throttle);
    //   Serial.print("\tpitch I val \t");
    //   Serial.print(integral_pitch);

    //  Serial.print("\troll D val\t");
    // Serial.print(derivative_pitch);

    Serial.println();

    // // write the most recent PID values
    // if (loopCounter > (2000 / dataLogRateSlow))
    // {

    //     loopCounter = 0;
    // }
    // else
    // {
    //     loopCounter++;
    // }

#elif TEST_ALTITUDE_RIG

    // no pilot control or movements, just altitude detection
    // Serial.print("ToF Altitude\t");
    // Serial.print(ToFaltitude);
    // // Serial.print("\tBaro altitude\t");
    // // Serial.print(altitude);
    // // Serial.print("\tKalman altitude\t");
    // // Serial.print(altitudeLPbaro.getAltitude());
    // Serial.print("\tleft wingtip altitude\t");
    // Serial.print(leftWingtipAltitude);
    // Serial.print("\tright wingtip altitude\t");
    // Serial.print(rightWingtipAltitude);
    // Serial.print("\testimated altitude\t");

    Serial.print(IMU_vertical_accel);
    Serial.print(" m/s^2 ");
    Serial.print(IMU_vertical_vel);
    Serial.print(" m/s ");
    Serial.print(IMU_vertical_pos);
    Serial.print(" m ");
    Serial.print(estimated_altitude);
    Serial.print(" m ");

    // Serial.print("\taltitudeTypeDataLog \t");
    //   Serial.print(altitudeTypeDataLog);
    Serial.println();
#elif TEST_HORIZONTAL_MOTION

    // outputs horizontal motion of the uav
    Serial.print("AccX\t");
    Serial.print(AccX);
    Serial.print("\tAccY\t");
    Serial.print(AccY);
    Serial.print("\tAccZ\t");
    Serial.print(AccZ);
    Serial.print("\thorizontal accel:\t");
    Serial.print(DS_horizontal_accel);
    Serial.print("\thorizontal vel:\t");
    Serial.print(DS_horizontal_accel);
    Serial.print("\thorizontal pos:\t");
    Serial.print(DS_horizontal_pos);

#else

    if (mode1_channel < 1400)
    {
        flight_phase = manual_flight;
    }
    else if (mode1_channel < 1600)
    {
        flight_phase = stabilized_flight;
    }
    else
    {
        flight_phase = dynamic_soaring_flight;
    }

    // reciever_time = micros() - sensor_time;

    if (flight_phase == manual_flight)
    {

        // Flight mode 1 (manual flight). Directly puts the servo commands to the commands from the radio
        s1_command_scaled = thro_des;       // Between 0 and 1
        s2_command_scaled = roll_passthru;  // Between -0.5 and 0.5
        s3_command_scaled = pitch_passthru; // Between -0.5 and 0.5
        s4_command_scaled = yaw_passthru;   // Between -0.5 and 0.5
        DSifFirstRun = true;                // Resets the DS variable while not in the DS flight mode

        // Reset integrators to 0 so when the other two flight modes are triggered, they start out without integral windup
        integral_pitch = 0;
        integral_roll = 0;
        integral_yaw = 0;
        throttle_integral = 0;
        // coord_integral = 0;
        altitude_integral = 0;
        // horiz_vel_integral = 0;

        // Log data to RAM slowly because not in DS flight
        if (loopCounter > (2000 / dataLogRateSlow))
        {
#if DATALOG
            logDataToRAM(); // Logs the data to Teensy 4.1 RAM via a 2D array
#endif
            loopCounter = 0;
        }
        else
        {
            loopCounter++;
        }
    }
    else if (flight_phase == stabilized_flight)
    {
        // Flight mode 2 (stabilized flight, constant airspeed, and coordinated turns.
        controlANGLE(); // dRehmFlight for angle based (pitch and roll) PID loops
        motorOn = true;
        airspeed_setpoint = flight_speed;
        throttleController(); // PID loop for throttle control
                              //  coordinatedController(); // PID loop for coordinated turns

        s1_command_scaled = throttle_PID; // Between 0 and 1
        s2_command_scaled = roll_PID;     // Between -1 and 1
        s3_command_scaled = pitch_PID;    // Between -1 and 1
                                          // s4_command_scaled = rudderCoordinatedCommand; // Between -1 and 1
        s4_command_scaled = yaw_PID;      // Between -1 and 1

        DSifFirstRun = true; // Resets the DS variable while not in the DS flight mode
        // Log data to RAM slowly because not in DS flight
        if (loopCounter > (2000 / dataLogRateSlow))
        {
#if DATALOG
            logDataToRAM(); // Logs the data to Teensy 4.1 RAM via a 2D array
#endif
            loopCounter = 0;
        }
        else
        {
#if DATALOG
            loopCounter++;
#endif
        }
    }

    else if (flight_phase == dynamic_soaring_flight)
    {
        DS_heading_rate_mean_setpoint = yaw_passthru*heading_rate_scalar;

        // Flight mode 3 (Dynamic soaring)
        //horizontal();         // Estimates the global horizontal acceleration, velocity, and position of the UAV
        dynamicSoar();        // Creates the dynamic soaring setpoints
        DSattitude();         // Converts dynamic soaring setpoints to desired angles
        controlANGLE();       // dRehmFlight for angle based (pitch and roll) PID loops
        throttleController(); // PID loop for throttle control
                              //  coordinatedController(); // PID loop for coordinated turns
        

        s1_command_scaled = throttle_PID; // Between 0 and 1
        s2_command_scaled = roll_PID;     // Between -1 and 1
        s3_command_scaled = pitch_PID;    // Between -1 and 1
                                          // s4_command_scaled = rudderCoordinatedCommand; // Between -1 and 1
        s4_command_scaled = yaw_PID;      // Between -1 and 1

        DSifFirstRun = false; // False after the first loop
        // Log data to RAM datalogRate times per second
#if DATALOG
        if (loopCounter > (2000 / datalogRate))
        {
            logDataToRAM(); // Logs the data to Teensy 4.1 RAM via a 2D array
            loopCounter = 0;
        }
        else
        {
            loopCounter++;
        }

#endif
    }
#if DATALOG

    // Flight modes based on mode switch, or if datalog gets full it automatically logs to avoid data loss
    if (mode2_channel < 1500 || currentRow >= ROWS)
    {
        if (!dataLogged)
        {
            writeDataToSD();
            clearDataInRAM();
        }
        dataLogged = true;
    }
    else
    {
        dataLogged = false;
    }
#endif

#endif

    scaleCommands(); // Scales commands to values that the servo and ESC can understand
#if MOTOR_ACTIVE

    // EDITED SO THAT IT MAPS 15-170 TO 0-180
    ESC_command_PWM = ESC_command_PWM * 0.861 + 14;
    ESC.write(ESC_command_PWM);                 // ESC active
#else
    ESC.write(-100); // ESC inactive
#endif

#if TEST_ALTITUDE_RIG == 0
    aileronServo.write(aileron_command_PWM);    // aileron
    elevatorServo.write(elevator_command_PWM);  // elevator
    rudderServo.write(rudder_command_PWM);      // rudder
#endif
    gimbalServo.write(gimbalServo_command_PWM); // gimbal

    // Serial.print(ESC_command_PWM);
    //     Serial.print(" ");
    // Serial.println(rudder_command_PWM);

    //  Serial.println(ESC_command_PWM); // ESC active
    //   Serial.print(" ");
    //   Serial.print(pitch_IMU);
    //   Serial.print(" ");
    //   Serial.println(yaw_IMU);

    // Serial.print(" baro reading: \t");
    // Serial.print(altitudeMeasured);
    // Serial.print(" baro reading LP: \t");
    // Serial.print(altitude_baro);

    // Serial.print(" tof test time: \t");
    // Serial.print(ToF_test_time_in_micros);
    //  Serial.print("\t airspeed: \t");
    //  Serial.print(airspeed_adjusted);
    // Serial.print("\t imu test time: \t");
    // Serial.print(IMU_test_time_in_micros);
    //  Serial.print(" tof  : \t");
    //  Serial.print(distance_LP);

    // Serial.print("\t all sensors timme  : \t");
    // Serial.print(all_sensors_time);
    // if (bmp.newData)
    // {
    //     Serial.print("\t baro pressure: \t");
    //     Serial.print(pressure);
    //     // Serial.print("\t baro temp: \t");
    //     // Serial.print(temperature);
    //      Serial.print(" \t baro reading: \t");
    //      Serial.print(altitudeMeasured);
    //     // Serial.print(" baro reading LP: \t");
    //     // Serial.print(altitude_baro);
    //     Serial.println();
    // }

    // Serial.print(roll_IMU);
    // Serial.print(" ");
    // Serial.print(pitch_IMU);
    // Serial.print(" ");
    // Serial.print(yaw_IMU);

    // Serial.print("\t microseconds per loop: ");
    // Serial.print(dt * 1000000);
    // //    printLoopRate();
    // Serial.println(currentRow);
    // Regulate loop rate
    loopBlink();    // Blink every 1.5 seconds to indicate main loop
    loopRate(2000); // Loop runs at 2000 Hz
}
#endif
// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// DYNAMIC SOARING CONTROLLER FUNCTIONS
/*
// This function manages the phases of DS flight. It generates the setpoints (airspeed,altitude, and global horizontal motion) for each DS phase, and provides the conditions to move to the next phase
void dynamicSoar()
{

#if LOW_ALTITUDE_HORIZ_FLIGHT
    DS_altitude_in_wind = DS_altitude_terrain_following; // Effectively creates no vertical motion
#endif

#if LOW_ALTITUDE_FLIGHT
    // UAV does the DS at low altitude, straight forwards

    // Ground following setpoints
    DS_altitude_setpoint = DS_altitude_terrain_following;          // Fly below the wind shear layer
    DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
    DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

    // Global horizontal motion setpoints
    horiz_setpoint_type = setpoint_horiz_vel;                             // Setpoint is based on horizontal velocity
    DS_horizontal_setpoint = 0;                                           // Fly straight forwards
    DS_horizontal_vel_error = DS_horizontal_setpoint - DS_horizontal_vel; // Calculate the error

    // Airspeed setpoint
    motorOn = true;
    airspeed_setpoint = flight_speed; // Fly at normal flight speed

#else

    // Detects the first time the DS switch is activated by the pilot
    if (DSifFirstRun)
    {
        flight_phase = DS_phase_0; // Go-ahead by the pilot to dynamic soar: activate DS Phase 0
    }

    // All the dynamic soaring phases in a switch function to be most computaitonally efficent
    switch (flight_phase)
    {
    case DS_phase_0:
        // Phase 0 autonomously flies the UAV to be ready for dynamic soaring, but does not actually dynamic soar.
        // To safely activate the dynamic soaring cycle, the UAV must meet the following conditions, and the rest of DS Phase 0 is trying to meet these conditions.
        // Some conditions have been limited for reliability. Also, this is assuming that the low altitude is in range of the ToF sensor, or it will not DS at the right altitude.
        if (                                               // yaw_IMU < DS_heading - heading_setup_tolerance || yaw_IMU > DS_heading + heading_setup_tolerance // The UAV must be flying at the DS heading (within a tolerance), perpendicular to the wind, flying right (just using the IMU for yaw for now. If compass implemented, then the best estimate for heading will be used)
                                                           //|| abs(GyroZ) > heading_rate_of_change_setup_tolerance                                           // The UAV must not be changing yaw direciton (within a tolerance)
            abs(DS_altitude_error) > DS_altitude_tolerance // The UAV must be within the tolerance for terrain following altitude
            //|| abs(GyroY) > pitch_rate_of_change_setup_tolerance                                             // The UAV must not be chaning pitch (within a tolerance)
            || abs(DS_horizontal_vel) > horizontal_vel_tolerance // The UAV must not be moving horizontally (within a tolerance)
            || abs(airspeed_error) > airspeed_error_tolerance)   // The UAV must not be moving too fast or too slow (within a tolerance)
        {
            // Ground following setpoints in DS Phase 0
            DS_altitude_setpoint = DS_altitude_min;          // Fly below the wind shear layer
            DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
            DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

            // Global horizontal motion setpoints in DS Phase 0
            horiz_setpoint_type = setpoint_horiz_vel;                             // Setpoint is based on global horizontal velocity
            DS_horizontal_setpoint = 0.0;                                         // The horizontal velocity setpoint is 0, which automatically means fly parallel to the DS flight path
            DS_horizontal_vel_error = DS_horizontal_setpoint - DS_horizontal_vel; // Calculate the error

            // Airspeed setpoint in DS Phase 0
            motorOn = true;                   // The UAV may use its motor in this phase because it is below the wind shear layer and not harvesting wind energy
            airspeed_setpoint = flight_speed; // Fly at normal flight speed
        }
        else
        {
            flight_phase = DS_phase_1;
        }

        break;

    case DS_phase_1:
        // Phase 1 is the intial turn into the wind to start the dynamic soaring cycle. Because flying right relative to the wind, this turns left.

        // Ground following setpoints in DS Phase 1
        DS_altitude_setpoint = DS_altitude_min;          // Fly below the wind shear layer
        DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

        // Global horizontal motion setpoints in DS Phase 1
        horiz_setpoint_type = setpoint_horiz_vel;                             // Setpoint is based on horizontal velocity
        DS_horizontal_setpoint = DS_horizontal_vel_setpoint_phase_1_4;        // The setpoint velocity for flying towards the wind
        DS_horizontal_vel_error = DS_horizontal_setpoint - DS_horizontal_vel; // Calculate the error

        // Airspeed setpoint in DS Phase 1
        motorOn = true;                   // The UAV may use its motor in this phase because it is below the wind shear layer and not harvesting wind energy
        airspeed_setpoint = flight_speed; // Fly at normal flight speed

        // If the UAV is within horizontal velocity tolerance, proceed to the next DS phase. Making sure the velocity while under the shear layer is consistant is important because acceleration is used as the setpoint in the next to phases, and having significantly different sized DS cycles would not be good
        if (abs(DS_horizontal_vel) < horizontal_vel_tolerance)
        {
            flight_phase = DS_phase_2; // Go the DS Phase 2, starting the DS cycle
            DS_horizontal_pos = 0;     // Because the UAV should cross the DS line at the right velocity, set the DS line to be right here
        }

        break;

    case DS_phase_2:
        // Phase 2 is the energy harvesting climb above the wind shear layer

        // Ground following setpoints in DS Phase 2
        DS_altitude_setpoint = DS_altitude_max;                    // Fly above the wind shear layer
        DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

        // Global horizontal motion setpoints in DS Phase 2
        horiz_setpoint_type = setpoint_horiz_accel;                               // Setpoint is based on global horizontal acceleration
        DS_horizontal_setpoint = DS_horizontal_accel_setpoint_phase_2_3;          // The setpoint velocity for accelerating with the wind, harvesting energy
        DS_horizontal_accel_error = DS_horizontal_setpoint - DS_horizontal_accel; // Calcualte the error

        // Airspeed setpoint in DS Phase 2
        if (airspeed_adjusted < stall_speed)
        {
            // If about to stall, turn the motor on and get above the stall speed
            motorOn = true;
            airspeed_setpoint = stall_speed;
        }
        else
        {
            // If above stall speed, turn the motor off and let the wind do the work
            motorOn = false;
        }

        // Once the UAV reverses direction and begins to fly back towards the DS line, proceed to Phase 3 and begin descent
        if (DS_horizontal_vel > 0.0)
        {
            flight_phase = DS_phase_3;
        }

        break;

    case DS_phase_3:
        // Phase 3 is the energy harvesting descent above the wind shear layer

        // Ground following setpoints in DS Phase 3
        DS_altitude_setpoint = DS_altitude_min;          // Aim for below the shear layer, but at this phase is still above the shear layer
        DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

        // Global horizontal motion setpoints in DS Phase 3
        horiz_setpoint_type = setpoint_horiz_accel;                                               // Setpoint is based on global horizontal velocity
        DS_horizontal_setpoint = DS_horizontal_accel_setpoint_phase_2_3;                          // The setpoint velocity for accelerating with the wind, harvesting energy
        DS_horizontal_accel_error = DS_horizontal_accel_setpoint_phase_2_3 - DS_horizontal_accel; // Calcualte the error

        // Airspeed setpoint in DS Phase 3
        if (airspeed_adjusted < stall_speed)
        {
            // If about to stall, turn the motor on and get above the stall speed
            motorOn = true;
            airspeed_setpoint = stall_speed;
        }
        else
        {
            // If above stall speed, turn the motor off and let the wind do the work
            motorOn = false;
        }

        // Once the UAV passes the DS line and is below the shear layer, proceed to Phase 4 and begin to turn back to recover for leeway
        if (DS_horizontal_pos > 0.0 && abs(DS_altitude_error) < DS_altitude_tolerance)
        {
            flight_phase = DS_phase_4;
        }

        break;

    case DS_phase_4:
        // Phase 4 is the leeway recovery phase below the wind shear layer

        // Ground following setpoints in DS Phase 4
        DS_altitude_setpoint = DS_altitude_min;          // Fly below the wind shear layer
        DS_altitude_error_prev = DS_altitude_error;                    // Set the previous error before recalculating the new error
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude; // Calculate the error

        // Global horizontal motion setpoints in DS Phase 4
        horiz_setpoint_type = setpoint_horiz_vel;                             // Setpoint is based on horizontal velocity
        DS_horizontal_setpoint = DS_horizontal_vel_setpoint_phase_1_4;        // The setpoint velocity for flying towards the wind
        DS_horizontal_vel_error = DS_horizontal_setpoint - DS_horizontal_vel; // Calculate the error

        // Airspeed setpoint in DS Phase 4
        motorOn = true;                   // The UAV may use its motor in this phase because it is below the wind shear layer and not harvesting wind energy
        airspeed_setpoint = flight_speed; // Fly at normal flight speed

        // If the UAV is within horizontal velocity tolerance and crossed the DS flight path, loop back to Phase 2 to restart the cycle
        if (abs(DS_horizontal_vel_error) < horizontal_vel_tolerance && DS_horizontal_pos < 0.0)
        {
            flight_phase = DS_phase_2;
        }

        break;
    }
#endif
}

// This function creates the desired attitude (roll, pitch, and yaw angles) to meet the setpoint altitude and setpoint horizontal motion in a coordinated fashion to dynamic soar (keep in mind, this controller controlls the setpoint for the actual PID controller, and thus often does not need to be as complicated)
void DSattitude()
{
    // This is a very unconventional and novel? flight computer algorithm because unlike regular flight computer algorithms the most important thing for DS is the global horizontal acceleration, because that measures how much energy is being harvested from the

    // For horizontal motion, the increased bank angle increases the horizontal velocity. The elevator will automatically pitch up to maintain altitude, and the rudder will automatically move to coordinate the turn
    if (horiz_setpoint_type == setpoint_horiz_accel)
    {
        // If the horizontal setpoint is based on acceleration, the desired roll is just a simple proportional gain controller because it controlls the 'zeroth' integral of acceleration, which is just acceleration
        roll_des = K_horiz_accel * DS_horizontal_accel_error;
    }
    else if (horiz_setpoint_type == setpoint_horiz_vel)
    {
        // If the horizontal setpoint is based on velocity, the desired roll is just a PI (proportional-integral) controller, since it controlls the first integral of acceleration, velocity
        horiz_vel_integral_prev = horiz_vel_integral;
        horiz_vel_integral = horiz_vel_integral_prev + DS_horizontal_vel_error * dt;                                           // Integrate integral
        horiz_vel_integral = constrain(horiz_vel_integral, -horiz_integral_saturation_limit, horiz_integral_saturation_limit); // Saturate integrator to prevent unsafe buildup
        roll_des = 0.01 * (Kp_horiz_vel * DS_horizontal_vel_error + Ki_horiz_vel * horiz_vel_integral);                        // Scaled by .01 to bring within -1 to 1 range
    }

    // For pitch to reach altitude, it is a PID loop because it is based on the second integral of acceleration, position. Pitching too high or low is automatically prevented by the dRehmFlight pitch limitation to +-30 degrees
    altitude_integral_prev = altitude_integral;
    altitude_integral = altitude_integral_prev + DS_altitude_error * dt;                                                        // Integrate integral
    altitude_integral = constrain(altitude_integral, -altitude_integral_saturation_limit, altitude_integral_saturation_limit);  // Saturate integrator to prevent unsafe buildup
    altitude_derivative = (DS_altitude_error - DS_altitude_error_prev) / dt;                                                    // Derivative shouldn't need low pass filter because the altitude already has a low pass filter on it
    pitch_des = 0.01 * (Kp_altitude * DS_altitude_error + Ki_altitude * altitude_integral - Kd_altitude * altitude_derivative); // Scaled by .01 to bring within -1 to 1 range
}
*/

// Uses sine function to generate sine function and desried flight phase
void dynamicSoar()
{
    // generate generic sine wave which oscelates at set priod.

    if (DS_phase_timer >= DS_period)
    {
        DS_phase_start_time = millis();
    }
    else
    {
        DS_phase_timer = millis() - DS_phase_start_time;
    }
    // flight_phase acts as the sine wave phase in radians
    flight_phase = ((2 * PI) / DS_period) * DS_phase_timer;

    // generate altitude
    DS_altitude_setpoint = DS_altitude_meanline + DS_altitude_amplitude*sin(flight_phase);

    //generate yaw (cosine because its a derivative)
    DS_heading_rate_setpoint = cos(flight_phase)*DS_yaw_amplitude+DS_heading_rate_mean_setpoint;
}

// sets yaw pitch roll desired to follow sine function
void DSattitude()
{
}

// This function coordinates the UAV by to attempt to make the acceleration vector the UAV experinces point straight down about the roll axis. Outputs the rudder command rudderCoordinatedCommand
/*
void coordinatedController()
{
    acceleration_downwards_angle = atan2(AccY, AccZ) * RAD_TO_DEG; // Calculate the angle of the downwards vector

    // Low pass filter the angle, even though Acc is already low passed its good to be cautous around trig functions. Also to help prevent jitters in the PID derivative
    acceleration_downwards_angle_LP = (1.0 - acceleration_downwards_angle_LP_param) * acceleration_downwards_angle_prev + acceleration_downwards_angle_LP_param * acceleration_downwards_angle;
    acceleration_downwards_angle_prev = acceleration_downwards_angle_LP;

    acceleration_downwards_magnitude = sqrt(AccZ * AccZ + AccY * AccY); // Calculate the magnitude of the downwards vector

    // Check if vector angle is valid, since no matter now small the AccY and AccZ values are, the atan2 function will still produce an angle
    if (acceleration_downwards_magnitude > acceleration_downwards_magnitude_tolerance)
    {
        // Run PID loop
        coord_integral_prev = coord_integral;
        coord_integral = coord_integral_prev + acceleration_downwards_angle * dt;                                                              // The angle is the error
        coord_integral = constrain(coord_integral, -coord_integral_saturation_limit, coord_integral_saturation_limit);                         // Saturate integrator to prevent unsafe buildup
        coord_derivative = (acceleration_downwards_angle - acceleration_downwards_angle_prev) / dt;                                            // Derivative values already have low pass filter applied
        rudderCoordinatedCommand = 0.01 * (Kp_coord * acceleration_downwards_angle + Ki_coord * coord_integral - Kd_coord * coord_derivative); // Scaled by .01 to bring within -1 to 1 range
    }
    else
    {
        rudderCoordinatedCommand = 0; // If the vector is not good (if the UAV is momentarily in freefall and the wings aren't producing lift, there is nothing to coordinate) then set the rudder to the middle
    }
}*/

// This function controlls the throttle to maintain airspeed
void throttleController()
{
    if (motorOn)
    {
        // If the motor is on, run the PID loop
        throttle_integral_prev = throttle_integral;
        airspeed_error_prev = airspeed_error;
        airspeed_error = airspeed_setpoint - airspeed_adjusted;
        throttle_integral = throttle_integral_prev + airspeed_error * dt;
        throttle_integral = constrain(throttle_integral, 0, throttle_integral_saturation_limit);                                    // Saturate integrator to prevent unsafe buildup
        throttle_derivative = (airspeed_error - airspeed_error_prev) / dt;                                                          // Derivative shouldn't need low pass filter becasuse the airspeed already has low pass filter on it
        throttle_PID = 0.01 * (Kp_throttle * airspeed_error + Ki_throttle * throttle_integral - Kd_throttle * throttle_derivative); // Scaled by .01 to bring within -1 to 1 range
                                                                                                                                    // throttle_PID = (throttle_PID/2)+0.5;                                                                                  // Scale to 0 to 1 range.

        throttle_PID = (1.0 - throttle_LP_param) * throttle_PID_prev + throttle_LP_param * throttle_PID; // fetch_airspeed gets the raw airspeed
        throttle_PID_prev = throttle_PID;
    }
    else
    {
        throttle_PID = 0.0; // If the motor is off, set the throttle to zero.
    }
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// STATE VARIABLE ESTIMATION FUNCTIONS
/*
// This function estimates the global horizontal acceleration, velocity, and position of the UAV based on the IMU
void horizontal()
{

    // Use Kalman Filter libary to estimate horizontal velocity. Will need to test which horizontal it is, and if it needs to be rotated.
    float horizontal_acceleration_magnitude_any_direction;
    float angle_of_horizontal_acceleration;
    kalmanHoriz.estimateHorizontal(gyroData, accelData, dt, horizontal_acceleration_magnitude_any_direction, angle_of_horizontal_acceleration);

    // Calculate horizontal accel perpendicular to the DS flight path, assumed to be directly into the wind and assumed to be 0 degrees yaw angle relative to staring position
    DS_horizontal_accel = cos(angle_of_horizontal_acceleration) * horizontal_acceleration_magnitude_any_direction;
    // integrate to get horizontal velocity:
    DS_horizontal_vel += DS_horizontal_accel * dt;
    DS_horizontal_pos += DS_horizontal_vel * dt;

    // Integrate velocity and position using the Runge-Kutta method NOT USED RIGHT NOW

        // k1_vel = DS_horizontal_accel * dt;
        // k1_pos = DS_horizontal_vel * dt;
        // k2_vel = (DS_horizontal_accel + k1_vel / 2) * dt;
        // k2_pos = (DS_horizontal_vel + k1_pos / 2) * dt;
        // k3_vel = (DS_horizontal_accel + k2_vel / 2) * dt;
        // k3_pos = (DS_horizontal_vel + k2_pos / 2) * dt;
        // k4_vel = (DS_horizontal_accel + k3_vel) * dt;
        // k4_pos = (DS_horizontal_vel + k3_pos) * dt;
        // DS_horizontal_vel += (k1_vel + 2 * k2_vel + 2 * k3_vel + k4_vel) / 6;
        // DS_horizontal_pos += (k1_pos + 2 * k2_pos + 2 * k3_pos + k4_pos) / 6;

}
*/
// This function estimates the altitude of the UAV relative to the water using the IMU, baro, and ToF sensors
void estimateAltitude()
{
    // Use the Kalman filter to estimate the altitude of the UAV using only the IMU and barometer (both after having a low pass filter applied)

    // IMU_vertical_accel = (1.0 - IMU_vertical_accel_LPparam) * IMU_vertical_accel_prev + IMU_vertical_accel_LPparam * kalmanVert.estimate(gyroData, accelData, dt);
    // IMU_vertical_accel_prev = IMU_vertical_accel;
    // IMU_vertical_vel += IMU_vertical_accel * dt;
    // IMU_vertical_pos += IMU_vertical_vel * dt;
    // altitudeLPbaro.estimate(accelData, gyroData, altitudeMeasured - altitude_offset, dt);

    gimbalServo_command_PWM = roll_IMU * gimbalServoGain + 90; // Rotate the gimbal servo to point the ToF sensor straight down
    ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad); // Find the altitude of the UAV in meters based on the ToF sensor. Accounts for pitch.

#if IMU_ALTITUDE_TEST == 0
    if (ToFaltitude < 4.0 && distance > 0.0)

#else

    // also need to figure out how to get the range of the ToF sensor to 4m
    // If the distance being read is a valid number (it returns -1 if it cannot detect anything, and has a range up to 4m), use the ToF sensor as the altitude
    if (ToFaltitude < 0.5 && distance > 0.0)
#endif
    {
        // // Recalibrate the barometer based on the ToF sensor. Every 10 readings, find the offset and average
        // if (offset_loop_counter < altitude_offset_num_vals)
        // {
        //     offset_loop_counter++;
        //     altitude_offset_sum += altitudeMeasured - ToFaltitude;
        // }
        // else
        // {
        //     offset_loop_counter = 0;
        //     altitude_offset = (altitude_offset_sum / altitude_offset_num_vals);
        //     altitude_offset_sum = 0;
        // }


        // // recalibrate IMU
        // // occasionally rezero the velocity and position?
        // if (ToFcounter > ToFcounterNum)
        // {
        //     IMU_vertical_vel = (ToFaltitude - prevToFaltitude) / (dt * ToFcounterNum);
        //     prevToFaltitude = ToFaltitude;
        //     IMU_vertical_pos = ToFaltitude;

        //     ToFcounter = 0;
        // }
        // else
        // {
        //     ToFcounter++;
        // }

        // Calculate the distance of the wingtip to the ground. The UAV has a wingspan of 1.5m and the ToF sensor is located 0.14m to the left of the center of the fuselage
        if (roll_IMU > gimbalRightBoundAngle && roll_IMU < gimbalLeftBoundAngle)
        {
            // If the gimbal is within range (which it always should be, since the roll angle limit is 30 degrees)
            leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
            rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
            estimated_altitude = leftWingtipAltitude < rightWingtipAltitude ? leftWingtipAltitude : rightWingtipAltitude; // gets lesser of two values
            altitudeTypeDataLog = 0;                                                                                      // ToF sensor, within gimbal range
        }
        else
        {
            // If the roll somehow surpasses the gimbal servo bound, the altitude can still be calculated.
            if (roll_IMU > gimbalLeftBoundAngle)
            {
                // Altitude calculation when banking too far to the left
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
                altitudeTypeDataLog = 1; // Logs that the ToF sensor reading is based on a saturated gimbal (banking too far left)
            }
            else
            {
                // Altitude calculation when banking too far to the right
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
                altitudeTypeDataLog = 2; // Logs that the ToF sensor readin is based on a saturated gimbal (banking too far right)
            }
        }
        // estimated_altitude = estimated_altitude - (sin(roll_IMU_rad) * halfWingspan); //Not sure what this piece of code does, so it is commented out. Not deleting yet.
    }
    else
    {

        // If the ToF sensor is out of range, no altitude estimation, just hold the previous altitude. DS should never go above 3 or 4 meters anyways
      //  estimated_altitude = IMU_vertical_pos;
        altitudeTypeDataLog = 3; // Let the flight data show that there is no new altitude reading
    }
}

// This function calibrates the pitot tube by finding the average of 10 readings
void pitotSetup()
{
    for (int i = 0; i < 10; i++)
    {
        _status = fetch_airspeed(&P_dat);
        PR = (float)((P_dat - 819.15) / (14744.7));
        PR = (PR - 0.49060678);
        PR = abs(PR);
        V = ((PR * 13789.5144) / 1.225);
        airspeed_offset += (sqrt((V)));
        delay(100);
    }
    airspeed_offset = airspeed_offset / 10.0;
}

// This function retrieves the airspeed data, low pass filters it, and scales and offsets it
void pitotLoop()
{
    airspeed_unadjusted = (1.0 - airspeed_LP_param) * airspeed_prev + airspeed_LP_param * fetch_airspeed(&P_dat); // fetch_airspeed gets the raw airspeed
    airspeed_prev = airspeed_unadjusted;

    airspeed_adjusted_prev = airspeed_adjusted;
    airspeed_adjusted = (airspeed_unadjusted - airspeed_offset) * airspeed_scalar;
}

/*
// This function sets up and calibrates the barometric pressure sensor, and on startup offests the raw data to say the starting position is 0m
void BMP180setup()
{
    bmp.initialize();
    // bmp.setSeaLevelPressure(100600);

    // Get average offset, which should be close to 0. This is done because the offset will need to change, while the baseline pressure can't.
    for (int i = 0; i < altitude_offset_num_vals; i++)
    {
        while (!bmp.newData)
        {
            bmp.pollData(&temperature, &pressure, &altitudeMeasured);
        }
        bmp.pollData(&temperature, &pressure, &altitudeMeasured);
        altitude_offset_sum += altitudeMeasured;
        Serial.println(i);
    }
    altitude_offset = altitude_offset_sum / ((float)altitude_offset_num_vals);
    Serial.println(altitude_offset);
    delay(1000);
}

// This function offsets and low pass filters the barometric altitude reading
void BMP180loop()
{
    bmp.pollData(&temperature, &pressure, &altitudeMeasured);
    if (bmp.newData)
    {
        altitudeMeasured = altitudeMeasured - altitude_offset;
        altitude_baro = (1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured;
        altitude_prev = altitude_baro;
    }
}
*/

void VL53L1Xsetup()
{

    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1)
            ;
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    // sensor.startContinuous(50);
    sensor.startContinuous(50);
}

void VL53L1Xloop()
{

    distance = sensor.read(false);
    distance_LP = (1.0 - distance_LP_param) * distancePrev + distance_LP_param * distance;
    distancePrev = distance_LP;
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// DATA LOGGING FUNCTIONS

// This function checks if the SD card can be communicated with
void setupSD()
{
    // Setup won't proceed until it detects an SD card, avoids flying with no datalog
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }
}

void logDataToRAM()
{
    if (currentRow < ROWS)
    {
        // fill out the enitre row of data
        dataLogArray[currentRow][0] = timeInMillis;
        dataLogArray[currentRow][1] = flight_phase;

        dataLogArray[currentRow][2] = roll_IMU; // in degrees
        dataLogArray[currentRow][3] = roll_des; // in degrees
        dataLogArray[currentRow][4] = roll_PID; // in degrees

        dataLogArray[currentRow][5] = pitch_IMU;
        dataLogArray[currentRow][6] = pitch_des;
        dataLogArray[currentRow][7] = pitch_PID;

        dataLogArray[currentRow][8] = GyroZ;
        dataLogArray[currentRow][9] = yaw_des;
        dataLogArray[currentRow][10] = yaw_PID;

        dataLogArray[currentRow][11] = airspeed_adjusted;
        dataLogArray[currentRow][12] = airspeed_setpoint;
        dataLogArray[currentRow][13] = throttle_PID;

        dataLogArray[currentRow][14] = estimated_altitude;

        currentRow++; // Increment counter
    }
}

void clearDataInRAM() // set all values to 0
{
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLUMNS; j++)
        {
            dataLogArray[i][j] = 0.0;
        }
    }
    currentRow = 0;
}

// Writes the flight data in CSV format to the mciroSD card in this order by looping through the entire 2D array
void writeDataToSD()
{
    dataFile = SD.open("flightData.txt", FILE_WRITE);
    for (int i = 0; i < currentRow; i++)
    {
        for (int j = 0; j < COLUMNS; j++)
        {
            dataFile.print(dataLogArray[i][j]);
            dataFile.print(",");
        }
        dataFile.println();
    }
    dataFile.close();
}
