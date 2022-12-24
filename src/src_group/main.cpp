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

float airspeed;
float airspeed_prev;
float airspeed_LP_param = 0.05;
float airspeed_zero = 0; // the calibrated airspeed to be 0m/s at startup to account for different pressrues
float airspeed_adjusted;
float airspeed_scalar = 1.8; // the scaled airspeed, tuned to be most accurate at about 10-15 m/s

float altitude_offset;
int altitude_offset_num = 10;
float altitude_offset_sum = 0;
float altitude_baro;
float altitude_prev;
float altitude_LP_param = 0.05;
float altitudeMeasured;

float estimated_altitude; //the actual altitude used in the controller

float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;
float IMU_horizontal_accel, IMU_horizontal_vel, IMU_horizontal_pos;

unsigned long timeInMillis;
int loopCounter = 0;
int datalogRate = 50; // log data at 50Hz

// dynamic soaring variables
float wind_heading;        // can be set manually set, but for now, assumed that it is 0 degrees relative to the yaw IMU (without compass for now)
float DS_heading;          // the yaw heading of the overall DS flight path, perpendicular to the wind, so 90 degrees (will be flying to the right)
float DS_horizontal_accel; // acceleration perpendicular to the heading
enum DS_phase              // uses different sensors and different setpoints in different phases. Each phase should have constant acceleration
{
    DS_approach_and_setup_phase = 0, // turn perpendicular to the wind, descend until terrain following, and turn into the wind for about half the time of DS_phase_1, to get the sine wave started
    DS_phase_1 = 1,                  // climb into the wind, accelerating away from the wind
    DS_phase_2 = 2,                  // descent out of the wind, accelerating away from the wind, should take similar time to DS_phase_1
    DS_phase_3 = 3,                  // terrain following, accelerating towards the wind, should take the same time as DS_phase_1 and DS_phase_2 combined. Resets the IMU and barometer measurements
};
//each phase in the DS cycle takes a slightly different amount of time (in seconds), manually tuned, to adjust for leeway
float DS_approach_and_setup_phase_time = 2.0; //time for the first turn into the wind
float DS_phase_1_time = 2.0; //time for phase 1
float DS_phase_2_time = 2.0; //time for phase 2
float DS_phase_3_time = 4.0; //time for phase 3

// phase 1: terrain following, accelerating towards the
float DS_altitude_setpoint;         // altitude setpoint
float DS_horizontal_accel_setpoint; // horizontal accelration setpoint

// PROGRAM OBJECTS
Adafruit_BMP085 bmp; // altitude sensor object
File dataFile;       // SD data output object

// Altitude estimators
static AltitudeEstimator altitudeLPbaro = AltitudeEstimator(0.001002176158, // sigma Accel
                                                            0.01942384099,  // sigma Gyro
                                                            0.1674466677,   // sigma Baro
                                                            0.5,            // ca
                                                            0.1);           // accelThreshold

// list all the functions
void estimateAltitude();
void estimateHorizonatalPosition();
void pitotSetup();
void pitotLoop();
void BMP180setup();
void BMP180loop();
void setupSD();
void logData(); // can add as many parameters as needed for flight data

void setup()
{
    // runs a modified version of dRehmFlight's setup. Don't forget to calibrate the ESCs and IMU
    // note, acceleration is in g's
    dRehmFlightSetup();

    BMP180setup();
    VL53L1Xsetup();
    setupSD();
    pitotSetup();
}

void loop()
{
    // runs a modified version of dRehmFlight's loop.
    dRehmFlightLoop(); // might want to try figuring out how to use OneShot125 protocol for the ESC if the servo output is jittery

    BMP180loop();
    VL35L1Xloop();
    logData();
    pitotLoop();
}

// OTHER FUNCTIONS

// generates the horizontal and vertical setpoints
void dynamicSoarSetpointGeneration()
{
}

void horizontalAccel()
{
}

// takes in the IMU, baro, and ToF sensors
// If ToF sensor in range, just use this sensor and IMU
// If ToF sensor out of range, use baro and IMU
// IMU mostly just to smooth out the data, since it drifts over time
void estimateAltitude()
{
    float accelData[3] = {AccX, AccY, AccZ};
    float gyroData[3] = {GyroX / 57.2958, GyroY / 57.2958, GyroZ / 57.2958};

    altitudeLPbaro.estimate(accelData, gyroData, altitudeMeasured, dt);

    // might need to somehow smooth/interpolate between the two...
    // also need to figure out how to get the range of the ToF sensor to 4m
    if (!(0.0 < distance_LP < 4000.0))
    {
        estimated_altitude = (distance_LP / 1000.0) * cos(pitch_IMU / 57.2958); //altitude in meters from the ToF sensor
        // experimental: estimate the distance the wingtip is to the ground
        // wingspan of 1.5m, half wingspan of .75m
        estimated_altitude = estimated_altitude - (sin(roll_IMU / 57.2958) * 0.75);
        // somehow "zero" the IMU and barometer based on this reading
    }
    else
    {
        estimated_altitude = altitudeLPbaro.getAltitude();
    }
}

void pitotSetup()
{
    for (int i = 0; i < 10; i++)
    {
        _status = fetch_airspeed(&P_dat);
        PR = (double)((P_dat - 819.15) / (14744.7));
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

    for (size_t i = 0; i < altitude_offset_num; i++)
    {
        altitude_offset_sum += bmp.readAltitude();
    }

    altitude_offset = altitude_offset_sum / ((float)altitude_offset_num);
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