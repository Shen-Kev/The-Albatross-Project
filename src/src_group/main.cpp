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
float altitude;
float altitude_prev;
float altitude_LP_param = 0.2;
float altitudeMeasured;

float estimated_altitude;

float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;
float IMU_horizontal_accel, IMU_horizontal_vel, IMU_horizontal_pos;

unsigned long timeInMillis;
int loopCounter = 0;
int datalogRate = 50; // log data at 50Hz

// dynamic soaring variables
float DS_heading; // the yaw heading of the overall DS flight path

// PROGRAM OBJECTS
Adafruit_BMP085 bmp; // altitude sensor object
File dataFile;       // SD data output object

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
    dRehmFlightSetup();
    // note, acceleration is in g's


    /* NAW this is kinda sucky system theres gotta b something betr
//run the IMU for a lil bit to wait for it to stabilize for some reason
for(int i = 0; i < 2000; i++) {
        dRehmFlightLoop();
    Serial.println(roll_IMU);
    Serial.println("IN CALIBRATION LOOP THINGY NO1");
}
while (roll_IMU < -0.01 or roll_IMU > 0.01) {
    dRehmFlightLoop();
    Serial.println(roll_IMU);
    Serial.println("IN CALIBRATION LOOP THINGY NO2");
}
Serial.println("OUT OF CALIBRATION LOOP THINGY SOMEHOW");
IMU_vertical_vel=0;
IMU_vertical_pos=0;
delay(1000);

*/


    /*
    VL53L1Xsetup();

    setupSD();

    BMP180setup();

    pitotSetup();
*/
}

void loop()
{
    // runs a modified version of dRehmFlight's loop.
    dRehmFlightLoop();
    estimateAltitude();
    Serial.print(GyroErrorX);
    Serial.print(" ");
    Serial.print(roll_IMU);
    Serial.print(" ");
    Serial.print(cos(roll_IMU / 57.29577951));
    Serial.print(" ");
    Serial.print(cos(pitch_IMU / 57.29577951));
    Serial.print(" ");
    Serial.print(IMU_vertical_accel);
    Serial.print(" ");
    Serial.print(IMU_vertical_vel);
    Serial.print(" ");
    Serial.print(IMU_vertical_pos);
    Serial.println();
    /*
        VL35L1Xloop();

        // loop runs 2000 times a second.
        // if the loop goes over 2000/50 times, it will log data
        if (loopCounter > (2000 / datalogRate))
        {
            logData();
            loopCounter = 0;
        }
        else
        {
            loopCounter++;
        }

        BMP180loop();

        pitotLoop();
    */
}

// OTHER FUNCTIONS

// takes in the IMU, baro, and ToF sensors
// If ToF sensor in range, just use this sensor and IMU
// If ToF sensor out of range, use baro and IMU
// IMU mostly just to smooth out the data, since it drifts over time
void estimateAltitude()
{
    // first estimate IMU vertical velocity
    IMU_vertical_accel = (cos(roll_IMU / 57.29577951) * cos(pitch_IMU / 57.29577951) * AccZ) - 1.0;
    IMU_vertical_vel += (IMU_vertical_accel)*dt;
    IMU_vertical_pos += (IMU_vertical_vel)*dt;
    //this is not working
}

// estimates horizontal position relative to the straight dynamic soaring line set by using the IMU only
void estimateHorizontalPosition()
{
    IMU_horizontal_accel = cos(roll_IMU / 57.29577951) * cos(DS_heading / 57.29577951) * AccY;
    IMU_horizontal_vel += (IMU_horizontal_accel) / dt;
    IMU_horizontal_pos += IMU_horizontal_vel / dt;
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
    altitude = (1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured;
    altitude_prev = altitude;
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