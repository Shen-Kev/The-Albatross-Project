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
#include <Arduino.h>                     //  Standard Arduino file
#include "src_group/dRehmFlight.h"       //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                         //  Credit to: Nicholas Rehm
                                         //  Department of Aerospace Engineering
                                         //  University of Maryland
                                         //  College Park 20742
                                         //  Email: nrehm@umd.edu
                                         //
#include "src_group/ToF/VL53L1X.h"       // Library to interface with the time-of-flight sensor VL53L1X
#include "BMP180/Adafruit_BMP085.h"      // Library to interface with the barometric sensor BMP180
#include "ASPD4525.h"                    //Library to interface with the ASPD4525 airspeed sensor
                                         // Sensor originally meant to work with Ardupilot flight computers, and thus needed to be experimentally tuned
#include <SD.h>                          // Library to read and write to the microSD card port
#include "AltitudeEstimation/altitude.h" // Library to combine barometric sensor and IMU in a two step Kalman-Complementary filter to estimate altitude

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
 * Mode2: D7 (currently unused)
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

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// DEBUG AND TESTING #IFS
#define SERIAL_CONNECTED TRUE
#define DATALOG TRUE
#define MOTOR_ACTIVE FALSE

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// PROGRAM VARIABLES AND OBJECTS

// Variables for the gimbaling servo that rotates the ToF sensor
const float gimbalServoGain = 2;             // The ratio between the angle the servo head moves to the angle outputted to it
const float gimbalServoTrim = 12;            // NEEDS TO BE ADJUSTED: Gimbal servo trim (degrees counterclockwise)
const float gimbalServoBound = 45;           // The angle the servo can rotate clockwise or counterclockwise from center (used to detect the gimbal 'maxing out')
const float halfWingspan = 0.75;             // The wingspan of the UAV / 2 (in meters)
const float gimbalDistanceFromCenter = 0.14; // The location of the ToF sensor and gimbal from the center of the wing (distance to the left in meters)
float gimbalRightBoundAngle;                 // The clockwise bound of the servo, based on the servo bound and trim constants
float gimbalLeftBoundAngle;                  // The counterclockwise bound of the servo, based on the servo bound and trim constants

// Variables for sensing airspeed
float airspeed_unadjusted;            // The raw airspeed reading from the ASPD4525 sensor, with a low pass filter applied
float airspeed_prev;                  // The previous raw airspeed reading from the ASPD4525 sensor (used in the low pass filter)
const float airspeed_LP_param = 0.05; // The low pass filter parameter for airspeed (smaller values means a more smooth signal but higher delay time)
float airspeed_offset = 0;            // The amount of airspeed the airspeed sensor detects when real airspeed is 0m/s. Used to zero the airspeed sensor
const float airspeed_scalar = 1.8;    // LIKELY NEEDS TO BE RE-ADJUSTED: The scalar applied to the unadjusted (but zeroed) airspeed to produce the adjusted airspeed
float airspeed_adjusted;              // the airspeed adjusted to be as accurate as possible (tuned to be best around flight speed)
float airspeed_adjusted_prev;         // the previous airspeed adjusted reading. Used to estimate the derivative (slope of secant line) in the PID loop

// Variables for altitde sensing
float altitude_offset;                   // The difference between the barometer's altitude reading and the 'real' altitude (in m) which is assumed to be 0 at ground level
const int altitude_offset_num_vals = 10; // The number of sensor readings that get averaged to adjust the altitude offset
int offset_loop_counter = 0;             // Tracks the number of sensor readings so far
float altitude_offset_sum = 0.0;         // Tracks the sum of all the sensor reading so far
float altitude_baro;                     // The altitude estimated from the barometer, with a low pass filter and offset adjustment applied
float altitude_prev;                     // The previous reading of the barometric pressure sensor
float altitude_LP_param = 0.05;          // The low pass filter parameter for altitude (smaller values means a more smooth signal but higher delay time)
float altitudeMeasured;                  // The raw altitude reading from the barometric pressure sensor (in m)

float ToFaltitude;          // The estimated altitude by the Time of Flight sensor (in m). Derived from distance_LP, and therefore has low pass filter applied. Works up to 4m, but NEED TO TEST RANGE
float leftWingtipAltitude;  // The estimated and calculated altitude of the left wingtip based on roll angle and the ToF sensor
float rightWingtipAltitude; // The estimated and calculated altitude of the right wingtip based on roll angle and the ToF sensor

// float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;       // the vertical acceleration, velocity, and position estimated by the IMU
// float IMU_horizontal_accel, IMU_horizontal_vel, IMU_horizontal_pos; // the horizontal acceleration, velocity, and position estimated by the IMU

float estimated_altitude; // The estimated altitude of the UAV, as a combination of the ToF, IMU, and baro sensor
int altitudeTypeDataLog;  // To record the type of altitude the UAV is using as its estimated altitude. 0 is ToF within gimbal range, 1 is ToF too far left, 2 is ToF too far right, and 3 is using IMU and barometer

float timeInMillis;         // Time in milliseconds since the flight controller was reset
int loopCounter = 0;        // A counter used to log data every certain number of loops
const int datalogRate = 50; // NEEDS TO BE ADJUSTED: Data logging rate in Hz

// Dynamic soaring state variables
const float wind_heading = 0.0;                          // The heading the wind is coming from. For now it is assumed it is 0 degrees relative to the yaw IMU at startup. In the future can be manually set if absolute compass is added
const float DS_heading = 90.0;                           // The yaw heading of the overall DS flight path, perpendicular to the wind. Flying to the right relative to the wind.
const float heading_setup_tolerance = 5;                 //  NEEDS TO BE ADJUSTED: The tolerance of the heading allowed while setting up the DS flight path (deg)
const float heading_rate_of_change_setup_tolerance = 10; //  NEEDS TO BE ADJUSTED: The tolerance of the heading rate of change while setting up the DS flight path (deg/s)
const float pitch_rate_of_change_setup_tolerance = 10;   //  NEEDS TO BE ADJUSTED: The tolerance of the pitch rate of change while setting up the DS flight path (deg/s)
const float horizontal_vel_tolerance = 0.5;              //  NEEDS TO BE ADJUSTED: The tolerance of the horizontal velocity while setting up the DS flight path (m/s)

float DS_altitude_setpoint;                      // The altitude setpoint while dynamic soaring
float DS_altitude_error;                         // The difference between the altitude setpoint and the estimated altitude
float DS_altitude_error_prev;                    // The previous altitude error, used to differentiate the error curve
const float DS_altitude_terrain_following = 0.3; // NEEDS TO BE ADJUSTED: The altitude (in meters) that the UAV should fly at while at the lowest phase of dynamic soaring. Could based on the bumpiness (std) of the water
const float DS_altitude_tolerance = 0.1;         // NEEDS TO BE ADJUSTED: The altitude (in meters) that the UAV must be close to the setpoint to have met the setpoint
const float DS_altitude_in_wind = 5.5;           // NEEDS TO BE ADJUSTED:The altitude (in meters) that the UAV should fly at to be most influenced by the wind shear layer

// float DSrotationMatrix[3][3]; // The rotation matrix to transform local angles (local: relative to the UAV) to global angles (global: relatie to the dynamic soaring line)

float DS_horizontal_accel;                                // The global horizontal acceleration (perpendicular to the dynamic soaring line, parallel to the wind direction) (in g's)
float DS_horizontal_accel_error;                          // The difference between the global horizontal acceleration setpoint and measured horizontal acceleration
const float DS_horizontal_accel_setpoint_phase_2_3 = 2.0; //  NEEDS TO BE ADJUSTED: The setpoint acceleration (in g's) while accelerating in the wind (DS phase 2 and 3)

float DS_horizontal_vel;                                 // The global horizontal velocity (perpendicular to the dynamic soaring line, parallel to the wind direction) (in m/s)
float DS_horizontal_vel_error;                           // The differnece between the global horizontal velocity setpoint and the measured horizontal velocity
float DS_horizontal_pos;                                 // The global horizontal position relative to the dynamic soaring line. Left of the line is negative, right of the line positive, and the center is set to be at 0 when the DS cycle starts
const float DS_horizontal_vel_setpoint_phase_1_4 = -1.5; //  NEEDS TO BE ADJUSTED: The setpoint horizontal velocity (in m/s) while turning into the wind, below the shear layer

float DS_horizontal_setpoint; // The global horizontal movement setpoint (can be velocity or acceleration, determined in the loop)
int horiz_setpoint_type;      // variable to keep track what type of global horizontal setpoint is being used

enum horiz_setpoint_types
{
    setpoint_horiz_accel = 0, // Use global horizontal acceleration as the setpoint
    setpoint_horiz_vel = 1,   // Use global horizontal velocity as the setpoint
    setpoint_horiz_pos = 2,   // Use global horizontal acceleration as the setpoint

};

// Dynamic soaring phase variables
boolean DSifFirstRun = true; // The boolean which is true only if the mode switch goes from a non DS flight mode to the DS flight mode

int DS_phase;  // The integer used to store the dynamic soaring phase
enum DS_phases // The different phases of DS stored as names to be easier to understand. Stored in DS_phase
{
    DS_phase_0 = 0, // DS Phase 0. UAV turns perpendicular to the wind flies low to the ground and stabilizes
    DS_phase_1 = 1, // DS Phase 1. UAV turns into the wind, but below the shear layer to build up enough horizontal velocity
    DS_phase_2 = 2, // DS Phase 2. Part of the DS cycle. UAV is facing the wind and above the shear layer, but is accelerating away from the wind, harvesting the wind's energy
    DS_phase_3 = 3, // DS Phase 3. Part of the DS cycle. UAV is now facing away from the wind and descending towards the shear layer. Still accelerating away from the wind, havesting energy
    DS_phase_4 = 4  // DS Phase 4. Part of the DS cycle. UAV is facing away from the wind and under the shear layer, but turning back towards the wind to build up velocity to start the DS cycle over
};

// Horizontal PID controller values
float K_horiz_accel = 1.0;                   //  NEEDS TO BE ADJUSTED: Global horizontal acceleration proportional gain
float Kp_horiz_vel = 1.0;                    //  NEEDS TO BE ADJUSTED: Global horizontal velocity proportional gain
float Ki_horiz_vel = 0.1;                    //  NEEDS TO BE ADJUSTED:Global horizontal velocity integral gain
float horiz_vel_integral = 0.0;              //  Variable to keep track of global horizontal velocity integral
float horiz_vel_integral_prev = 0.0;         //  The previous global horizontal velocity integral (to be integrated upon)
float horiz_integral_saturation_limit = 2.0; //  NEEDS TO BE ADJUSTED:Altitude integral saturation limit (to prevent integral windup)

// Altitude PID controller values
const float Kp_altitude = 0.2;                         //  NEEDS TO BE ADJUSTED:// Altitude proportional gain
const float Ki_altitude = 0.3;                         //  NEEDS TO BE ADJUSTED:// Altitude integral gain
const float Kd_altitude = 0.0015;                      //  NEEDS TO BE ADJUSTED:// Altitude derivative gain
float altitude_integral = 0.0;                         //  Variable to keep track of altitude integral
float altitude_integral_prev = 0.0;                    //  Previous altitude integral (to be integrated upon)
const float altitude_integral_saturation_limit = 25.0; //  NEEDS TO BE ADJUSTED: Altitude integral saturation limit (to prevent integral windup)
float altitude_derivative;                             // Variable to keep track of altitude derivative

// Coordinated turn variables
float rudderCoordinatedCommand;                               // The command (-1 to 1) to give to the rudder to have a coordinate turn
float acceleration_downwards_angle;                           // The roll angle of the downwards acceleration the UAV experiences (deg)
float acceleration_downwards_angle_prev;                      // The previous roll angle of the downwards acceleration the UAV experiences (deg)
float acceleration_downwards_angle_LP;                        // The roll angle of the downwards acceleration the UAV experiences (deg), with a low pass filter applied
const float acceleration_downwards_angle_LP_param = 0.1;      //  NEEDS TO BE ADJUSTED: The low pass paramter for the roll angle of the downwards acceleration the UAv experiences
float acceleration_downwards_magnitude;                       // The magnitude of the downwards acceleration about the roll axis the UAV experiences
const float acceleration_downwards_magnitude_tolerance = 0.5; //  NEEDS TO BE ADJUSTED: The acceleration needed to be experienced by the UAV to legitimzie the acceleration angle (g's)

const float Kp_coord = 0.2;                         //  NEEDS TO BE ADJUSTED:// Coordinated turn P gain
const float Ki_coord = 0.3;                         //  NEEDS TO BE ADJUSTED:// Coordinated turn I gain
const float Kd_coord = 0.0015;                      //  NEEDS TO BE ADJUSTED:// Coordinated turn D gain
float coord_integral = 0.0;                         // Coordinated turn error PID controller integral
float coord_integral_prev;                          // Previous coordinated turn PID controller integral
const float coord_integral_saturation_limit = 25.0; //  NEEDS TO BE ADJUSTED: Coordinated turn saturation limit (to avoid integral windup)
float coord_derivative;                             // Coordinated turn PID controller derivative

// Throttle PID controller values
float throttle_PID;                         // The throttle outputted by the PID loop to maintain constant airspeed (range between 0.0-1.0, where 0.0 is min throttle and 1.0 is max throttle)
float airspeed_setpoint;                    // The setpoint airspeed for the throttle PID loop
const float flight_speed = 20.0;            // NEEDS TO BE ADJUSTED: The airspeed in m/s for regular flight
boolean motorOn = false;                    // Activates or deactivates the motor
const float stall_speed = 10.0;             // NEEDS TO BE ADJUSTED: The airspeed in m/s to always stay above to avoid stall. Essentially the minimum airspeed to always stay above
float airspeed_error;                       // The error between setpoint airspeed and current airspeed
float airspeed_error_prev;                  // The previous error, used for derivative estimation
const float airspeed_error_tolerance = 1.0; // NEEDS TO BE ADJUSTED: The range of airspeeds to be within to be 'close enough' to the setpoint

const float Kp_throttle = 0.2;    //  NEEDS TO BE ADJUSTED:Throttle Proportional gain
const float Ki_throttle = 0.3;    //  NEEDS TO BE ADJUSTED:Throttle Integral gain
const float Kd_throttle = 0.0015; //  NEEDS TO BE ADJUSTED:Throttle Derivative gain

float throttle_integral = 0.0;                         // Throttle integral value (approximated with summation)
float throttle_integral_prev = 0.0;                    // Throttle previous integral value to be integrated upon
const float thorttle_integral_saturation_limit = 50.0; // NEEDS TO BE ADJUSTED: Throttle integral saturation limit to prevent integral windup
float throttle_derivative;                             // Throttle derivative value

// Roll, pitch, and yaw angles but in radians for easier math
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad;

// Variables for the Runge-Kutta integration method
// float k1_vel, k1_pos, k2_vel, k2_pos, k3_vel, k3_pos, k4_vel, k4_pos;

// Program Objects
Adafruit_BMP085 bmp; // Object to interface with the BMP180 barometric pressure sensor
File dataFile;       // Object to interface with the microSD card

// KalmanFilter kalmanHoriz; // Kalman filter for the horizontal

// float pastHorizGyro[3];
// float pastHorizAccel[3];

// Altitude estimator to combine barometric pressure sensor (with low pass filter applied) with the gyroscope and acclerometer
static AltitudeEstimator altitudeLPbaro = AltitudeEstimator(0.001002176158, // Sigma (standard deviation of) the accelerometer
                                                            0.01942384099,  // Sigma (standard deviation of) the gyroscope
                                                            0.1674466677,   // sigma (standard deviation of) the barometer
                                                            0.5,            // ca (don't touch)
                                                            0.1);           // accel threshold (if there are many IMU acceleration values below this value, it is assumed the aircraft is not moving vertically)

// Flight mode variables
int flight_mode; // The flight mode (manual, stabilized, DS)
enum flight_modes
{
    lost_connection = -1,
    manual_flight = 0,
    stabilized_flight = 1,
    dynamic_soaring_flight = 2
};

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// FUNCTION DECLARATIONS (in this order)

void dynamicSoar();
void DSattitude();
void coordinatedController();
void throttleController();
void horizontal();
void estimateAltitude();
void pitotSetup();
void pitotLoop();
void BMP180setup();
void BMP180loop();
void setupSD();
void writeDataToSD();

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// MICROCONTROLLER SETUP

void setup()
{
#if SERIAL_CONNECTED
    Serial.begin(500000); // Begin serial communication with computer via USB
#elif
    delay(20 * 1000); // Delay to have enough time to push reset, close hatch, and place UAV flat on the ground and into the wind
#endif

    pinMode(13, OUTPUT); // LED on the Teensy 4.1 set to output

    // Attach actuators to PWM pins
    ESC.attach(ESCpin, 900, 2100);
    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    gimbalServo.attach(gimbal1ServoPin, 900, 2100);

    delay(100);

    // Setup and calibrate communciations
    radioSetup();          // R/c reciever
    IMUinit();             // IMU init
    calculate_IMU_error(); // IMU calibrate
    BMP180setup();         // Barometer init and calibrate
    VL53L1Xsetup();        // ToF sensor init
    pitotSetup();          // Airspeed sensor init and calibrate
#if DATALOG
    setupSD(); // microSD card read/write unit
#endif

    // Set R/c reciever channels to failsafe values
    throttle_channel = throttle_fs;
    roll_channel = roll_fs;
    pitch_channel = pitch_fs;
    yaw_channel = yaw_fs;
    mode1_channel = mode1_fs;

    delay(100);

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
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// MICROCONTROLLER LOOP

void loop()
{
    // Timing
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0; // Time between loop iterations, in seconds
    timeInMillis = millis();

    // Retrieve sensor data
    getIMUdata();                                                              // Retrieves gyro and accelerometer data from IMU and LP filters
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (in deg)
    BMP180loop();                                                              // Retrieves barometric altitude and LP filters
    VL35L1Xloop();                                                             // Retrieves ToF sensor distance
    pitotLoop();                                                               // Retrieves pitot tube airspeed
    getCommands();                                                             // Retrieves radio commands
    failSafe();                                                                // Failsafe in case of radio connection loss

    // Convert roll, pitch, and yaw from degrees to radians
    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD;
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;

    getDesState(); // Scales throttle to between 0 and 1, and roll, pitch, and yaw to between -1 and 1. Produces thro_des, roll_des, pitch_des, yaw_des, roll_passthru, pitch_passthru, yaw_passthru

    // Flight modes based on mode switch
    if (mode1_channel < 1000)
    {
        flight_mode = lost_connection;
    }
    else if (mode1_channel < 1400)
    {
        flight_mode = manual_flight;
    }
    else if (mode1_channel < 1600)
    {
        flight_mode = stabilized_flight;
    }
    else
    {
        flight_mode = dynamic_soaring_flight;
    }

    if (flight_mode == manual_flight)
    {
        // Flight mode 1 (manual flight). Directly puts the servo commands to the commands from the radio
        s1_command_scaled = thro_des;       // Between 0 and 1
        s2_command_scaled = roll_passthru;  // Between -0.5 and 0.5
        s3_command_scaled = pitch_passthru; // Between -0.5 and 0.5
        s4_command_scaled = yaw_passthru;   // Between -0.5 and 0.5

        DSifFirstRun = true; // Resets the DS variable while not in the DS flight mode

        // Reset integrators to 0 so when the other two flight modes are triggered, they start out without integral windup
        integral_pitch = 0;
        integral_roll = 0;
        integral_yaw = 0;
        throttle_integral = 0;
        coord_integral = 0;
        altitude_integral = 0;
        horiz_vel_integral = 0;
    }
    else if (flight_mode == stabilized_flight)
    {
        // Flight mode 2 (stabilized flight, constant airspeed, and coordinated turns.
        controlANGLE();                               // dRehmFlight for angle based (pitch and roll) PID loops
        throttleController();                         // PID loop for throttle control
        coordinatedController();                      // PID loop for coordinated turns
        s1_command_scaled = throttle_PID;             // Between 0 and 1
        s2_command_scaled = roll_PID;                 // Between -1 and 1
        s3_command_scaled = pitch_PID;                // Between -1 and 1
        s4_command_scaled = rudderCoordinatedCommand; // Between -1 and 1

        DSifFirstRun = true; // Resets the DS variable while not in the DS flight mode
    }
    else if (flight_mode == dynamic_soaring_flight)
    {
        // Flight mode 3 (Dynamic soaring)
        horizontal();                                 // Estimates the global horizontal acceleration, velocity, and position of the UAV
        dynamicSoar();                                // Creates the dynamic soaring setpoints
        DSattitude();                                 // Converts dynamic soaring setpoints to desired angles
        controlANGLE();                               // dRehmFlight for angle based (pitch and roll) PID loops
        throttleController();                         // PID loop for throttle control
        coordinatedController();                      // PID loop for coordinated turns
        s1_command_scaled = throttle_PID;             // Between 0 and 1
        s2_command_scaled = roll_PID;                 // Between -1 and 1
        s3_command_scaled = pitch_PID;                // Between -1 and 1
        s4_command_scaled = rudderCoordinatedCommand; // Between -1 and 1

        DSifFirstRun = false; // False after the first loop
    }

    scaleCommands(); // Scales commands to values that the servo and ESC can understand
#if MOTOR_ACTIVE
    ESC.write(s1_command_PWM); // ESC active
#else
    ESC.write(-100);  // ESC inactive
#endif
    aileronServo.write(s2_command_PWM);  // aileron
    elevatorServo.write(s3_command_PWM); // elevator
    rudderServo.write(s4_command_PWM);   // rudder
    gimbalServo.write(s5_command_PWM);   // gimbal

#if DATALOG
    // Log data dataLogRate times per second
    if (loopCounter > (2000 / datalogRate))
    {
        writeDataToSD(); // Calls the function to open and write to the SDfile
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }
#endif

    // Regulate loop rate
    loopBlink();    // Blink every 1.5 seconds to indicate main loop
    loopRate(2000); // Loop runs at 2000 Hz
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// DYNAMIC SOARING CONTROLLER FUNCTIONS

// This function manages the phases of DS flight. It generates the setpoints (airspeed,altitude, and global horizontal motion) for each DS phase, and provides the conditions to move to the next phase
void dynamicSoar()
{
    // Detects the first time the DS switch is activated by the pilot
    if (DSifFirstRun)
    {
        DS_phase = DS_phase_0; // Go-ahead by the pilot to dynamic soar: activate DS Phase 0
    }

    // All the dynamic soaring phases in a switch function to be most computaitonally efficent
    switch (DS_phase)
    {
    case DS_phase_0:
        // Phase 0 autonomously flies the UAV to be ready for dynamic soaring, but does not actually dynamic soar.
        // To safely activate the dynamic soaring cycle, the UAV must meet the following conditions, and the rest of DS Phase 0 is trying to meet these conditions.
        if (yaw_IMU < DS_heading - heading_setup_tolerance || yaw_IMU > DS_heading + heading_setup_tolerance // The UAV must be flying at the DS heading (within a tolerance), perpendicular to the wind, flying right (just using the IMU for yaw for now. If compass implemented, then the best estimate for heading will be used)
            || abs(GyroZ) > heading_rate_of_change_setup_tolerance                                           // The UAV must not be changing yaw direciton (within a tolerance)
            || abs(DS_altitude_error) > DS_altitude_tolerance                                                // The UAV must be within the tolerance for terrain following altitude
            || abs(GyroY) > pitch_rate_of_change_setup_tolerance                                             // The UAV must not be chaning pitch (within a tolerance)
            || abs(DS_horizontal_vel) > horizontal_vel_tolerance                                             // The UAV must not be moving horizontally (within a tolerance)
            || abs(airspeed_error) > airspeed_error_tolerance)                                               // The AUV must not be moving too fast or too slow (within a tolerance)
        {
            // Ground following setpoints in DS Phase 0
            DS_altitude_setpoint = DS_altitude_terrain_following;          // Fly below the wind shear layer
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
            DS_phase = DS_phase_1;
        }

        break;

    case DS_phase_1:
        // Phase 1 is the intial turn into the wind to start the dynamic soaring cycle. Because flying right relative to the wind, this turns left.

        // Ground following setpoints in DS Phase 1
        DS_altitude_setpoint = DS_altitude_terrain_following;          // Fly below the wind shear layer
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
            DS_phase = DS_phase_2; // Go the DS Phase 2, starting the DS cycle
            DS_horizontal_pos = 0; // Because the UAV should cross the DS line at the right velocity, set the DS line to be right here
        }

        break;

    case DS_phase_2:
        // Phase 2 is the energy harvesting climb above the wind shear layer

        // Ground following setpoints in DS Phase 2
        DS_altitude_setpoint = DS_altitude_in_wind;                    // Fly above the wind shear layer
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
            DS_phase = DS_phase_3;
        }

        break;

    case DS_phase_3:
        // Phase 3 is the energy harvesting descent above the wind shear layer

        // Ground following setpoints in DS Phase 3
        DS_altitude_setpoint = DS_altitude_terrain_following;          // Aim for below the shear layer, but at this phase is still above the shear layer
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
            DS_phase = DS_phase_4;
        }

        break;

    case DS_phase_4:
        // Phase 4 is the leeway recovery phase below the wind shear layer

        // Ground following setpoints in DS Phase 4
        DS_altitude_setpoint = DS_altitude_terrain_following;          // Fly below the wind shear layer
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
            DS_phase = DS_phase_2;
        }

        break;
    }
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

// This function coordinates the UAV by to attempt to make the acceleration vector the UAV experinces point straight down about the roll axis. Outputs the rudder command rudderCoordinatedCommand
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
}

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
        throttle_integral = constrain(throttle_integral, 0, thorttle_integral_saturation_limit);                                        // Saturate integrator to prevent unsafe buildup
        throttle_derivative = (airspeed_error - airspeed_error_prev) / dt;                                                              // Derivative shouldn't need low pass filter becasuse the airspeed already has low pass filter on it
        throttle_PID = 0.01 * (Kp_roll_angle * airspeed_error + Ki_throttle * throttle_integral - Kd_roll_angle * throttle_derivative); // Scaled by .01 to bring within -1 to 1 range
        throttle_PID = map(throttle_PID, -1, 1, 0, 1);                                                                                  // Scale to 0 to 1 range.
    }
    else
    {
        throttle_PID = 0.0; // If the motor is off, set the throttle to zero.
    }
}

// ____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________//
// STATE VARIABLE ESTIMATION FUNCTIONS

// This function estimates the global horizontal acceleration, velocity, and position of the UAV based on the IMU
void horizontal()
{

    // OKAY THIS FUNCTION IS IDK WHATS GOING ON, DSOUHFOSDHFOUSDHF, FIGURE THIS OUT

    // ACTUALLY CHANGE THIS TO USE THE ALTITUDE ESTIMTION, BUT JUST FEED IT THE SIDEWAYS FORCE AS THE UPWARDS FORCE, ESSENTIALLY ROTATING IT, AND HAVE IT PREDCIT ALTITUDE WHICH IS ACTUALLY THE HORIZONTAL MOTION, SOMEHOW REMOVE OR SHIFT THE GRAVITY COMPONENT TO THE OTHER AXIS  AND MAKE A NEW FUNCTION LIKE THE ESTIMATE THAT DOESN"T TAKE INTO ACCOUNT ANOTHER SENSOR.. MAYBE NOT KALMAN THEN??
    // float horizGyro[3] = {GyroX, GyroZ-9.81, GyroY+9.81}; //Swap Z and Y, resulting in the vertical calulator calculating the horizontal? actually but then how do i tell wheich direction DS is/??
    // float horizAccel[3] = {AccX, AccZ-9.81, AccY+9.81};
    // DS_horizontal_accel = kalmanHoriz.estimate(pastHorizGyro, pastHorizAccel, dt);
    // copyVector(pastHorizGyro, horizGyro);
    // copyVector(pastHorizAccel, horizAccel);

    /*
        // create rotation matrix

        double DSrotationMatrix[3][3] = {
            {cos(pitch_IMU_rad) * cos(yaw_IMU_rad), -sin(roll_IMU_rad) * sin(pitch_IMU_rad) * cos(yaw_IMU_rad) + cos(roll_IMU_rad) * sin(yaw_IMU_rad), sin(roll_IMU_rad) * sin(yaw_IMU_rad) + cos(roll_IMU_rad) * sin(pitch_IMU_rad) * cos(yaw_IMU_rad)},
            {cos(pitch_IMU_rad) * sin(yaw_IMU_rad), cos(roll_IMU_rad) * cos(yaw_IMU_rad) + sin(roll_IMU_rad) * sin(pitch_IMU_rad) * sin(yaw_IMU_rad), -sin(roll_IMU_rad) * cos(yaw_IMU_rad) + cos(roll_IMU_rad) * sin(pitch_IMU_rad) * sin(yaw_IMU_rad)},
            {-sin(pitch_IMU_rad), sin(roll_IMU_rad) * cos(pitch_IMU_rad), cos(roll_IMU_rad) * cos(pitch_IMU_rad)}};
        // convert acceleration to global frame (centered on the DS path)
        float DSglobalAccel[3]; // Global accelerations
        // Transform local accelerations to global coordinates
        for (int i = 0; i < 3; i++)
        {
            DSglobalAccel[i] = 0;
            for (int j = 0; j < 3; j++)
            {
                DSglobalAccel[i] += DSrotationMatrix[i][j] * ((i == 0) ? AccX : ((i == 1) ? AccY : AccZ));
            }
        }

        // a_global[0] is the acceleration along the global pitch axis
        DS_horizontal_accel = DSglobalAccel[0];
    */

    // integrate to get horizontal velocity:
    // uncomment one at a time:
    DS_horizontal_vel += DS_horizontal_accel * dt; // direct
    DS_horizontal_pos += DS_horizontal_vel * dt;
    // Integrate velocity and position using the Runge-Kutta method
    /*
        k1_vel = DS_horizontal_accel * dt;
        k1_pos = DS_horizontal_vel * dt;
        k2_vel = (DS_horizontal_accel + k1_vel / 2) * dt;
        k2_pos = (DS_horizontal_vel + k1_pos / 2) * dt;
        k3_vel = (DS_horizontal_accel + k2_vel / 2) * dt;
        k3_pos = (DS_horizontal_vel + k2_pos / 2) * dt;
        k4_vel = (DS_horizontal_accel + k3_vel) * dt;
        k4_pos = (DS_horizontal_vel + k3_pos) * dt;
        DS_horizontal_vel += (k1_vel + 2 * k2_vel + 2 * k3_vel + k4_vel) / 6;
        DS_horizontal_pos += (k1_pos + 2 * k2_pos + 2 * k3_pos + k4_pos) / 6;
    */
}

// This function estimates the altitude of the UAV relative to the water using the IMU, baro, and ToF sensors
void estimateAltitude()
{
    // Use the Kalman filter to estimate the altitude of the UAV using only the IMU and barometer (both after having a low pass filter applied)
    float accelData[3] = {AccX, AccY, AccZ};
    float gyroData[3] = {GyroX * DEG_TO_RAD, GyroY * DEG_TO_RAD, GyroZ * DEG_TO_RAD};
    altitudeLPbaro.estimate(accelData, gyroData, altitudeMeasured - altitude_offset, dt);

    s5_command_PWM = roll_IMU * gimbalServoGain; // Rotate the gimbal servo to point the ToF sensor straight down

    ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad); // Find the altitude of the UAV in meters based on the ToF sensor. Accounts for pitch.
    // also need to figure out how to get the range of the ToF sensor to 4m
    // If the distance being read is a valid number (it returns -1 if it cannot detect anything, and has a range up to 4m), use the ToF sensor as the altitude
    if (ToFaltitude < 4.0 && ToFaltitude > 0.0)
    {
        // Recalibrate the barometer based on the ToF sensor. Every 10 readings, find the offset and average
        if (offset_loop_counter < altitude_offset_num_vals)
        {
            offset_loop_counter++;
            altitude_offset_sum += altitudeMeasured - ToFaltitude;
        }
        else
        {
            offset_loop_counter = 0;
            altitude_offset = (altitude_offset_sum / altitude_offset_num_vals);
            altitude_offset_sum = 0;
        }

        // Calculate the distance of the wingtip to the ground. The UAV has a wingspan of 1.5m and the ToF sensor is located 0.14m to the left of the center of the fuselage
        if (roll_IMU > gimbalRightBoundAngle && roll_IMU < gimbalLeftBoundAngle)
        {
            // If the gimbal is within range (which it always should be, since the roll angle limit is 30 degrees)
            leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
            rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
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
        // If the ToF sensor is out of range, estimate the altitude with the IMU and barometer only
        estimated_altitude = altitudeLPbaro.getAltitude();
        altitudeTypeDataLog = 3; // Let the flight data show that the altitude is based on the IMU and barometer only
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

// This function sets up and calibrates the barometric pressure sensor, and on startup offests the raw data to say the starting position is 0m
void BMP180setup()
{
    if (!bmp.begin())
    {
        while (1)
        {
        }
    }
    for (int i = 0; i < altitude_offset_num_vals; i++)
    {
        altitude_offset_sum += bmp.readAltitude();
    }
    altitude_offset = altitude_offset_sum / ((float)altitude_offset_num_vals);
}

// This function offsets and low pass filters the barometric altitude reading
void BMP180loop()
{
    altitudeMeasured = bmp.readAltitude() - altitude_offset;
    altitude_baro = (1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured;
    altitude_prev = altitude_baro;
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

// Writes the flight data in CSV format to the mciroSD card in this order
void writeDataToSD()
{
    dataFile = SD.open("flightData.txt", FILE_WRITE);

    // Time
    dataFile.print(timeInMillis + ',');

    // Pilot command
    dataFile.print(throttle_channel + ',');
    dataFile.print(roll_channel + ',');
    dataFile.print(pitch_channel + ',');
    dataFile.print(yaw_channel + ',');
    dataFile.print(flight_mode + ',');

    // Setpoints
    dataFile.print(airspeed_setpoint + ',');
    dataFile.print(DS_altitude_setpoint + ',');
    dataFile.print(DS_horizontal_setpoint + ',');
    dataFile.print(horiz_setpoint_type + ',');
    dataFile.print(rudderCoordinatedCommand + ',');

    // Altitude variables
    dataFile.print(altitude_baro + ',');
    dataFile.print(leftWingtipAltitude + ',');
    dataFile.print(rightWingtipAltitude + ',');
    dataFile.print(estimated_altitude + ',');
    dataFile.print(altitudeTypeDataLog + ',');
    dataFile.print(ToFaltitude + ',');

    // Airspeed
    dataFile.print(airspeed_adjusted + ',');

    // Orientation
    dataFile.print(roll_IMU + ',');
    dataFile.print(pitch_IMU + ',');
    dataFile.print(yaw_IMU + ',');

    //Global horizontal motion
    dataFile.print(DS_horizontal_accel + ',');
    dataFile.print(DS_horizontal_vel + ',');
    dataFile.print(DS_horizontal_pos + ',');

    // Booleans and enums
    dataFile.print(motorOn + ',');
    dataFile.print(DSifFirstRun + ',');
    dataFile.print(DS_phase + ',');

    // PID values
    dataFile.print(airspeed_error + ',');
    dataFile.print(throttle_integral + ',');
    dataFile.print(throttle_derivative + ',');

    dataFile.print(error_roll + ',');
    dataFile.print(integral_roll + ',');
    dataFile.print(derivative_roll + ',');

    dataFile.print(error_pitch + ',');
    dataFile.print(integral_pitch + ',');
    dataFile.print(derivative_pitch);

    dataFile.print(DS_altitude_error + ',');
    dataFile.print(altitude_integral + ',');
    dataFile.print(altitude_derivative + ',');

    dataFile.print(DS_horizontal_accel_error + ',');
    dataFile.print(DS_horizontal_vel_error + ',');
    dataFile.print(horiz_vel_integral + ',');

    dataFile.print(acceleration_downwards_magnitude + ',');
    dataFile.print(acceleration_downwards_angle + ',');
    dataFile.print(coord_integral + ',');
    dataFile.print(coord_derivative + ',');

    // Servo outputs
    dataFile.print(s1_command_PWM + ',');
    dataFile.print(s2_command_PWM + ',');
    dataFile.print(s3_command_PWM + ',');
    dataFile.print(s4_command_PWM + ',');
    dataFile.print(s5_command_PWM + ',');

    dataFile.print('\n');

    dataFile.close();
}
