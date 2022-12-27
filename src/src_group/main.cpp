// THIS FILE WILL CONTAIN ALL THE ORIGINAL CODE I WRITE, AND WILL BE WELL DOCUMENTED.
// WILL BE SEPARATED INTO FUNCTIONS FOR ORGANIZATION
// THE OTHER FILES THAT ARE INCLUDED ARE MODIFIED BUT MOSTLY UNORIGINAL

#include <Arduino.h>
#include "src_group/dRehmFlight.h" // modified dRehmFlight has stabilization code and actuation and IMU communication and reciever communication.
#include "src_group/ToF/VL53L1X.h"
#include "BMP180/Adafruit_BMP085.h"
#include "ASPD4525.h"
#include <SPI.h>
#include <SD.h>
#include "AltitudeEstimation/altitude.h"

// PROGRAM VARIABLES
float gimbalServoGain = 2;
float gimbalServoTrim = 12;            // degrees counterclockwise to get the servo pointed straight down.
float gimbalServoBound = 45;           // 45 degrees either side
float halfWingspan = 0.75;             // in meters
float gimbalDistanceFromCenter = 0.14; // distance left of center in meters
float gimbalRightBoundAngle;
float gimbalLeftBoundAngle;

float airspeed;
float airspeed_prev;
float airspeed_LP_param = 0.05;
float airspeed_zero = 0; // the calibrated airspeed to be 0m/s at startup to account for different pressrues
float airspeed_adjusted;
float airspeed_scalar = 1.8; // the scaled airspeed, tuned to be most accurate at about 10-15 m/s

float auto_throttle; // throttle outputted by autopilot to maintain constant airspeed (0.0-1.0)
float throttle_setpoint; //setpoint for throttle, it can change
const float flight_speed = 20.0; //m/s for regular flight, will need to be tested based on rc flight 
const float stall_speed = 10.0; //m/s to always stay above. When flying normally or in DS, treat this as a 'turn throttle off' variable
float airspeed_error; //error between setpoint airspeed and current airspeed


float altitude_offset;
const int altitude_offset_num_vals = 10;
int offset_loop_counter = 0;
float altitude_offset_sum = 0.0;
float altitude_baro;
float altitude_prev;
float altitude_LP_param = 0.05;
float altitudeMeasured;

float ToFaltitude;
float estimated_altitude; // the actual altitude used in the controller
float leftWingtipAltitude;
float rightWingtipAltitude;
int altitudeTypeDataLog; // 0 is ToF within gimbal range, 1 is ToF too far left, 2 is ToF too far right, and 3 is using IMU and barometer

float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;
float IMU_horizontal_accel, IMU_horizontal_vel, IMU_horizontal_pos;

float timeInMillis;
int loopCounter = 0;
int datalogRate = 50; // log data at 50Hz

// dynamic soaring variables
float wind_heading;                                // can be set manually set, but for now, assumed that it is 0 degrees relative to the yaw IMU (without compass for now)
float DS_heading;                                  // the yaw heading of the overall DS flight path, perpendicular to the wind, so 90 degrees (will be flying to the right)
float heading_setup_tolerance = 5;                 // within 5 degrees
float heading_rate_of_change_setup_tolerance = 10; // must be less than 10 degrees
float pitch_rate_of_change_setup_tolerance = 10;   // must be less than 10 degrees
float horizontal_vel_tolerance = 0.5;              // no more than 0.5m/s horizontal motion

float DS_altitude_setpoint; // altitude setpoint
float DS_altitude_error;
float DS_altitude_terrain_following = 0.3; // altitude in meters to NEVER GO BELOW (somehow tell the PID loops that this is realy bad)
float DS_altitude_in_wind = 5.5;           // altitude in meters to try and achieve when wanting to be influcenced by the wind

float DS_horizontal_accel;          // acceleration perpendicular to the heading
float DS_horizontal_accel_setpoint; // horizontal accelration setpoint
float DS_horizontal_accel_error;
float DS_horizontal_accel_setpoint_phase_2_3 = 2.0; // g's pulled while accelerating in the wind
float DS_horizontal_vel_setpoint_phase_1_4 = -1.5;  // horizontal velocity in m/s while turning back (to the left)
float DS_horizontal_vel;
float DS_horizontal_vel_setpoint;
float DS_horizontal_vel_error;
float DS_horizontal_pos; // left of the line should be negative, right of the line should be positive, the line is set to be at 0.

boolean DSifFirstRun = true;
boolean readyToDS = false;

float DSrotationMatrix[3][3]; // matrix to transform local coordinates to global (dynamic soaring line) coordinates

int DS_phase;
enum DS_phases // uses different sensors and different setpoints in different phases. Each phase should have constant acceleration
{
    DS_phase_0 = 0, // turn perpendicualr to the wind and descend
    DS_phase_1 = 1, // turn into the wind for about half the time of DS_phase_1, to get the sine wave started
    DS_phase_2 = 2, // climb into the wind, accelerating away from the wind
    DS_phase_3 = 3, // descent out of the wind, accelerating away from the wind, should take similar time to DS_phase_1
    DS_phase_4 = 4  // terrain following, accelerating towards the wind, should take the same time as DS_phase_1 and DS_phase_2 combined. Resets the IMU and barometer measurements
};

// angles but in radians bc im tired of constantly converting
float pitch_IMU_rad;
float roll_IMU_rad;
float yaw_IMU_rad;

// variables for the  Runge-Kutta method
float k1_vel;
float k1_pos;
float k2_vel;
float k2_pos;
float k3_vel;
float k3_pos;
float k4_vel;
float k4_pos;



// PROGRAM OBJECTS
Adafruit_BMP085 bmp; // altitude sensor object
File dataFile;       // SD data output object

// Altitude estimators
static AltitudeEstimator altitudeLPbaro = AltitudeEstimator(0.001002176158, // sigma Accel
                                                            0.01942384099,  // sigma Gyro
                                                            0.1674466677,   // sigma Baro
                                                            0.5,            // ca
                                                            0.1);           // accelThreshold

int horiz_setpoint_type;

enum horiz_setpoint_types
{
    setpoint_horiz_accel = 0,
    setpoint_horiz_vel = 1,
    setpoint_horiz_pos = 2,

};

// list all the functions
void estimateAltitude();
void estimateHorizonatalPosition();
void pitotSetup();
void pitotLoop();
void BMP180setup();
void BMP180loop();
void setupSD();
void logData();
void writeDataToSD();
void flightMode();
void dynamicSoar();
void horizontal();
void DSattitude();
void throttleController();

void setup()
{
    Serial.begin(500000);
    delay(500);

    pinMode(13, OUTPUT);
    servo1.attach(servo1Pin, 900, 2100);
    servo2.attach(servo2Pin, 900, 2100);
    servo3.attach(servo3Pin, 900, 2100);
    servo4.attach(servo4Pin, 900, 2100);
    servo5.attach(servo5Pin, 900, 2100);

    delay(500);

    radioSetup();
    IMUinit();
    BMP180setup();
    VL53L1Xsetup();
    setupSD();
    pitotSetup();

    // Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;

    delay(100);

    // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
    calculate_IMU_error(); // Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

    // DS setup
    wind_heading = 0.0;
    DS_heading = wind_heading + 90.0;
    gimbalRightBoundAngle = (0 - gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);
    gimbalLeftBoundAngle = (0 + gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);

    delay(100);

    servo1.write(0); // ESC set to 0
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
    servo5.write(90);

    delay(100);

    // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
    // Code will not proceed past here if this function is uncommented!
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    timeInMillis = millis();
    loopBlink();                                                               // Indicate we are in main loop with short blink every 1.5 seconds
    getIMUdata();                                                              // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD;
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;

    BMP180loop();
    VL35L1Xloop();
    logData();
    pitotLoop();

    getDesState();

    if (channel_5_pwm < 1400.0)
    {
        // flight mode 1, manual flight
        s1_command_scaled = thro_des;
        s2_command_scaled = roll_passthru;
        s3_command_scaled = pitch_passthru;
        s4_command_scaled = yaw_passthru;
        DSifFirstRun = true;
        readyToDS = false;
    }
    else if (channel_5_pwm < 1600.0)
    {
        // flight mode 2, stabilized flight and constant airspeed
        controlANGLE();
        throttleController();
        s1_command_scaled = auto_throttle;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;
        DSifFirstRun = true;
        readyToDS = false;
    }
    else
    {

        // flight mode 3, dynamic soaring
        horizontal();
        dynamicSoar();
        DSattitude();
        controlANGLE();
        throttleController();
        s1_command_scaled = auto_throttle;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;
        DSifFirstRun = false;
    }

    scaleCommands();
    servo1.write(s1_command_PWM); // ESC
    servo2.write(s2_command_PWM); // aileron
    servo3.write(s3_command_PWM); // elevator
    servo4.write(s4_command_PWM); // rudder
    servo5.write(s5_command_PWM); // gimbal

    getCommands(); // Pulls current available radio commands
    failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

    // Regulate loop rate
    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

// OTHER FUNCTIONS

// generates the setpoint altitude and horizontal accel and the error for each phase of DS flight
void dynamicSoar()
{
    // first time swtich goes on:
    if (DSifFirstRun)
    {
        DS_phase = DS_phase_0; // ready to dynamic soar, activate phase 0
    }

    switch (DS_phase)
    {
    case DS_phase_0:
        // do phase 0, turn and descent, wait until stable(using the IMU for now, use compass later with kalman filter class)
        if (yaw_IMU < DS_heading - heading_setup_tolerance || yaw_IMU > DS_heading + heading_setup_tolerance || abs(GyroZ) > heading_rate_of_change_setup_tolerance || estimated_altitude > DS_altitude_terrain_following || abs(GyroY) > pitch_rate_of_change_setup_tolerance || abs(DS_horizontal_vel) > horizontal_vel_tolerance)
        {
            // turn and descend somehow

            // ground follow
            DS_altitude_setpoint = DS_altitude_terrain_following;
            DS_altitude_error = DS_altitude_setpoint - estimated_altitude;
            // no horiz vel, this automatically means fly parallel to the DS flight path
            horiz_setpoint_type = setpoint_horiz_vel;
            DS_horizontal_vel_error = 0.0 - DS_horizontal_vel; // set horizontal velocity setpoint to 0
        }
        else
        {
            DS_phase = DS_phase_1;
        }
        break;
    case DS_phase_1:
        // do phase 1, inital turn into the wind (turning left for this code)
        // based on VELOCITY

        // ground follow
        DS_altitude_setpoint = DS_altitude_terrain_following;
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude;

        // towards the wind
        horiz_setpoint_type = setpoint_horiz_vel;
        DS_horizontal_vel_error = DS_horizontal_vel_setpoint_phase_1_4 - DS_horizontal_vel;

        if (abs(DS_horizontal_vel) < horizontal_vel_tolerance)
        {
            DS_phase = DS_phase_2; // start DS cycle
            DS_horizontal_pos = 0; // crosses the line at the right velocity, so sets the line to be here
        }

        break;
    case DS_phase_2:
        // do phase 2, climb and accelrate out of the wind

        // climb into wind
        DS_altitude_setpoint = DS_altitude_in_wind;
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude;

        // away from wind
        horiz_setpoint_type = setpoint_horiz_accel;
        DS_horizontal_accel_error = DS_horizontal_accel_setpoint_phase_2_3 - DS_horizontal_accel;

        if (DS_horizontal_vel > 0.0)
        { // once reached horizontal peak, go the other way and descend
            DS_phase = DS_phase_3;
        }

        break;
    case DS_phase_3:
        // do phase 3, descend and accelerate out of the wind

        // ground follow
        DS_altitude_setpoint = DS_altitude_terrain_following;
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude;

        // away from wind
        horiz_setpoint_type = setpoint_horiz_accel;
        DS_horizontal_accel_error = DS_horizontal_accel_setpoint_phase_2_3 - DS_horizontal_accel;

        if (DS_horizontal_pos > 0.0)
        { // passed from left to right side of the line
            DS_phase = DS_phase_4;
        }

        break;
    case DS_phase_4:
        // do phase 4, turn back into the wind BUT THIS TIME SETPOINT IS VELOCITY, SO THAT EACH CYCLE STARTS WITH THE SAME VELOCITY??

        // ground follow
        DS_altitude_setpoint = DS_altitude_terrain_following;
        DS_altitude_error = DS_altitude_setpoint - estimated_altitude;

        // towards the wind
        horiz_setpoint_type = setpoint_horiz_vel;
        DS_horizontal_vel_error = DS_horizontal_vel_setpoint_phase_1_4 - DS_horizontal_vel;

        // velocity constant, and crossed the line
        if (abs(DS_horizontal_vel_error) < horizontal_vel_tolerance && DS_horizontal_pos < 0.0)
        {
            DS_phase = DS_phase_2;
        }

        break;
    }
}

// finds the roll, pitch, and yaw required do coordinated turns and stuff to reach the angle requierd to reach the setpoint vertical and horizontal setpoints to do DS (take insp from ardupilot)
void DSattitude()
{
    // NEED TO DO:  in this funciton, call the dRehmFlight stuff, use tthe horiz_setpoint_type to see how to fly
}

void horizontal()
{
    // NEED TO DO: calculate this still, again, maybe horizVelocity instead
    // actually, i can do both, acceleration for DS, but to get it started have veloicty be 0

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

    // integrate to get horizontal velocity:
    // uncomment one at a time:
    // DS_horizontal_vel += DS_horizontal_accel * dt; // direct

    // Integrate velocity and position using the Runge-Kutta method

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
}

// takes in the IMU, baro, and ToF sensors
// If ToF sensor in range, just use this sensor and IMU
// If ToF sensor out of range, use baro and IMU
// IMU mostly just to smooth out the data, since it drifts over time
void estimateAltitude()
{
    float accelData[3] = {AccX, AccY, AccZ};
    float gyroData[3] = {GyroX * DEG_TO_RAD, GyroY * DEG_TO_RAD, GyroZ * DEG_TO_RAD};

    altitudeLPbaro.estimate(accelData, gyroData, altitudeMeasured - altitude_offset, dt);

    // might need to somehow smooth/interpolate between the two...
    // also need to figure out how to get the range of the ToF sensor to 4m
    if (!(distance_LP < 4000.0) && distance_LP > 0.0)
    {
        s5_command_PWM = roll_IMU * gimbalServoGain; // servo should have the same rotational angle as the UAV roll, but this servo only goes +- 45 degrees, so scaled up 2x

        ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad); // altitude in meters from the ToF sensor

        // just zeroing the baro, the IMU seems to automatically zero itself in the kalman and complementary filter
        // average offset from previous 10 value, make the value, then do the cycle again, no need to waste bunch of computational power on sliding window
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

        // experimental: estimate the distance the wingtip is to the ground
        // wingspan of 1.5m, half wingspan of .75m
        // sensor is located on the left wing .14m from the center
        // these two equations only work when bank angle within servo range of motion
        if (roll_IMU > gimbalRightBoundAngle && roll_IMU < gimbalLeftBoundAngle)
        {
            leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
            rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
            estimated_altitude = leftWingtipAltitude < rightWingtipAltitude ? leftWingtipAltitude : rightWingtipAltitude; // gets lesser of two values
            altitudeTypeDataLog = 0;                                                                                      // ToF sensor, within gimbal range
        }
        else
        {
            if (roll_IMU > gimbalLeftBoundAngle) // banking far to the left
            {
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
                altitudeTypeDataLog = 1; // ToF sensor, too far left
            }
            else // banking far to the right
            {
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
                altitudeTypeDataLog = 2; // ToF sensor, too far right
            }
        }
        estimated_altitude = estimated_altitude - (sin(roll_IMU_rad) * halfWingspan);
    }
    else
    {
        estimated_altitude = altitudeLPbaro.getAltitude();
        altitudeTypeDataLog = 3; // using IMU and barometer
    }
}

void pitotSetup()
{
    for (int i = 0; i < 10; i++)
    {
        _status = fetch_airspeed(&P_dat);
        PR = (float)((P_dat - 819.15) / (14744.7));
        PR = (PR - 0.49060678);
        PR = abs(PR);
        V = ((PR * 13789.5144) / 1.225);
        airspeed_zero += (sqrt((V)));
    }
    airspeed_zero = airspeed_zero / 10.0;
}
void pitotLoop()
{
    // adjust and filter the raw data into smooth and readable airspeed signal
    airspeed = (1.0 - airspeed_LP_param) * airspeed_prev + airspeed_LP_param * fetch_airspeed(&P_dat); // fetch_airspeed gets the raw airspeed
    airspeed_prev = airspeed;
    airspeed_adjusted = (airspeed - airspeed_zero) * airspeed_scalar;
}

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
    // calibrate to assume that startup is at sea level
}

void BMP180loop()
{
    altitudeMeasured = bmp.readAltitude() - altitude_offset;
    altitude_baro = (1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured;
    altitude_prev = altitude_baro;
}

void setupSD()
{
    // won't proceed until it detects an SD card, avoids flying with no datalog
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }
}

// edit to print all the appropritate data, perhaps a CSV to be easily read by microsoft excel or google sheets, maybe even modifiable by a python script to analyze the data
/*
A1,B1,C1,D1,E1,F1
A2,B2,C2,D2,E2,F2
A3,B3,C3,D3,E3,F3
A4,B4,C4,D4,E4,F4
A5,B5,C5,D5,E5,F5
A6,B6,C6,D6,E6,F6
A7,B7,C7,D7,E7,F7
A8,B8,C8,D8,E8,F8
A9,B9,C9,D9,E9,F9
A10,B10,C10,D10,E10,F10
A11,B11,C11,D11,E11,F11
A12,B12,C12,D12,E12,F12
A13,B13,C13,D13,E13,F13
A14,B14,C14,D14,E14,F14
A15,B15,C15,D15,E15,F15
A16,B16,C16,D16,E16,F16
*/
// commas separate the values, and new line separates datapoints
// record the actual template here:
//  time,

void logData()
{
    // loop runs 2000 times a second.
    // if the loop goes over 2000/50 times, it will log data
    if (loopCounter > (2000 / datalogRate))
    {
        writeDataToSD();
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }
}

void writeDataToSD()
{
    dataFile = SD.open("flightData.txt", FILE_WRITE);

    dataFile.print(current_time + ',');
    dataFile.print(current_time + ',');
    dataFile.print(current_time + ',');
    dataFile.print(current_time + ',');
    dataFile.print(current_time + ',');
    dataFile.print(current_time + ',');
    dataFile.print('\n');

    dataFile.close();
}

void throttleController() {

}