#include <Arduino.h>               // Arduino library
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu

#include "BMP180nonblocking/BMP085NB.h"             // Barometer library
#include <Wire.h>                                   // I2C library
#include "pololuVL53L1x/VL53L1X.h"                  // ToF sensor library
#include "ASPD4525.h"                               // Pitot tube library
#include <SD.h>                                     // SD card library
#include "AltitudeEstimation/altitude.h"            // Altitude estimation library
#define MOTOR_ACTIVE 1                              // 1 = motor is active, 0 = motor is not active
#define DS_AUTO_GROUND_AVOIDANCE_TEST 1             // 1 = DS flight mode only avoids the ground
#define DS_AUTO_CIRCLING_NO_GROUND_AVOIDANCE_TEST 0 // 1 = DS flight mode circles the aircraft with manual pitch control

const int TRIGGER_PIN = 34;                // Trigger pin of the ultrasonic sensor
const int ECHO_PIN = 35;                   // Echo pin of the ultrasonic sensor
const unsigned long MEASURE_INTERVAL = 50; // Time between sensor readings (in milliseconds)

unsigned long lastMeasureTime = 0; // Time when the last sensor reading was taken
long ultrasonicDistance = 0;       // Variable to store the distance measured by the sensor

// Constants for Gimbal Servo
const float gimbalServoGain = -1.5;
const float gimbalServoTrim = 0;
const float gimbalServoBound = 60;           // The maximum angle the gimbal servo can move in either direction
const float halfWingspan = 0.75;             // The half wingspan of the aircraft (in m)
const float gimbalDistanceFromCenter = 0.14; // The distance from the center of the aircraft to the gimbal servo (in m)

// Variables for Gimbal Servo
float gimbalRightBoundAngle; // The angle of the gimbal servo when the gimbal is pointing to the right wingtip
float gimbalLeftBoundAngle;  // The angle of the gimbal servo when the gimbal is pointing to the left wingtip

// Constants and Variables for Airspeed
const float airspeed_LP_param = 0.02; // The low pass filter parameter for airspeed (smaller values means a more smooth signal but higher delay time)
const float airspeed_scalar = 1.8;    // The scalar to convert the raw airspeed reading to m/s
float airspeed_offset = 0;            // The offset for the airspeed sensor
float airspeed_unadjusted;            /// The raw airspeed reading from the pitot tube (in m/s)
float airspeed_prev;                  // The previous reading of the airspeed sensor
float airspeed_adjusted;              // The airspeed reading from the pitot tube, with a low pass filter and offset adjustment applied
float airspeed_adjusted_prev;         // The previous reading of the airspeed sensor, with a low pass filter and offset adjustment applied

// Constants and Variables for Altitude
const int datalogRate = 50;    // The rate at which data is logged to the SD card (in Hz)
float altitude_integral = 0.0; // The integral term of the altitude controller
float estimated_altitude;      // The estimated altitude of the aircraft (in m)
float estimated_altitude_prev; // The previous estimated altitude of the aircraft (in m)
float ToFaltitude;             // The altitude of the aircraft estimated from the ToF sensor (in m)

int16_t distance;               // The raw distance reading from the ToF sensor (in mm)
float distance_LP_param = 0.03; // The low pass filter parameter for distance (smaller values means a more smooth signal but higher delay time)
float distancePrev;             // The previous reading of the ToF sensor
float distance_LP;              // The distance reading from the ToF sensor, with a low pass filter applied
int altitudeTypeDataLog;        // The type of altitude data that is being logged to the SD card
float leftWingtipAltitude;      // The altitude of the left wingtip (in m)
float rightWingtipAltitude;     // The altitude of the right wingtip (in m)

// Constants and Variables for Forwards Acceleration
float forwardsAcceleration; // The acceleration in the forwards direction (in m/s^2)
// forwardsAcceleration low pass variables
float forwardsAcceleration_LP_param = 0.001; // The low pass filter parameter for forwardsAcceleration (smaller values means a more smooth signal but higher delay time)
float forwardsAcceleration_prev;             // The previous reading of the forwardsAcceleration

// Variables for Flight Control
float timeInMillis;                             // The time in milliseconds since the flight controller has started
int loopCounter = 0;                            // The number of times the loop has run
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad; // The raw pitch, roll, and yaw angles in radians from the IMU
float accelData[3];                             // The raw accelerometer data from the IMU
float gyroData[3];                              // The raw gyro data from the IMU

// Dynamic Soaring Variables
float DS_roll_angle = -30;       // The bank angle for Dynamic Soaring (in degrees) (turning left)
float DS_yaw_proportion = 0.008; // The proportion of yaw in degrees to roll 0-1 for Dynamic Soaring
float DS_pitch_angle;            // Normally just the pilot input, but is automatically adjusted to avoid the ground
float minimum_pitch_angle;       // The minimum pitch angle while close to the ground in DS
float minimum_altitude = 2;      // the altitude at which the min pitch angle starts to increase from -45
float slope_min_pitch_angle_function = -27.5;
float intercept_min_pitch_angle_function = 10;

// Variables for Data Logging
const int COLUMNS = 16;            // 16 columns of data to be logged to the SD card
const int ROWS = 6400;             // 7800 rows of data to be logged to the SD card
float dataLogArray[ROWS][COLUMNS]; // The array that stores the data to be logged to the SD card
boolean dataLogged = false;        // Used to determine if the data has been logged to the SD card
boolean toggle = false;            // Used to toggle the LED
int currentRow = 0;                // The current row of the data log array
boolean logSuccessful = false;     // Used to determine if the data has been successfully logged to the SD card

// Flight Phases
float flight_phase; // The current flight phase
enum flight_phases  // Flight phases for the flight controller
{
    manual_flight = 1,
    stabilized_flight = 2,
    DS_flight = 3
};

// Objects
File dataFile;  // File object for SD card
VL53L1X sensor; // ToF sensor object

// Functions
void estimateAltitude();
void pitotSetup();
void pitotLoop();
void setupSD();
void logDataToRAM();
void clearDataInRAM();
void writeDataToSD();
// void VL53L1Xsetup();
// void VL53L1Xloop();
void ultrasonicLoop();

float rate_of_climb_LP_param = 0.002;
float rate_of_climb_prev;

// Flight Controller Setup
// This function is run once when the flight controller is turned on
// It is used to initialize the flight controller and set the initial values of the variables and objects used in the flight controller loop function (loop())
void setup()
{
    // Constants for PID
    Kp_roll_angle = 1.0;
    Ki_roll_angle = 0.3;
    Kd_roll_angle = 0.2;
    Kp_pitch_angle = 2;
    Ki_pitch_angle = 0.3;
    Kd_pitch_angle = 0.5;
    // Kp_yaw = 1;
    // Ki_yaw = 0.3;
    // Kd_yaw = 0.0015;

    Serial.begin(500000);
    Serial.println("serial works");
    Wire.begin();
    Wire.setClock(1000000);
    Serial.println("wire works");
    pinMode(13, OUTPUT);

    ESC.attach(ESCpin, 1100, 2100);
    aileronServo.attach(aileronServoPin, 900, 2100);
    elevatorServo.attach(elevatorServoPin, 900, 2100);
    rudderServo.attach(rudderServoPin, 900, 2100);
    gimbalServo.attach(gimbal1ServoPin, 900, 2100);
    Serial.println("passed attach");
    delay(100);
    radioSetup();
    Serial.println("passed radio setup");
    IMUinit();
    Serial.println("passed IMU init");

    AccErrorY = 0.04;
    AccErrorZ = 0.11;
    GyroErrorX = -3.20;
    GyroErrorY = -0.14;
    GyroErrorZ = -1.40;
    delay(10000);
    for (int i = 0; i < 1000; i++)
    {
        getIMUdata();
        Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    }
    //    VL53L1Xsetup();
    // Serial.println("passed ToF setup");
    pitotSetup();
    Serial.println("passed pitot");

    // Initialize the ultrasonic sensor pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    clearDataInRAM();
    setupSD();
    throttle_channel = throttle_fs;
    roll_channel = roll_fs;
    pitch_channel = pitch_fs;
    yaw_channel = yaw_fs;
    mode1_channel = mode1_fs;
    mode2_channel = mode2_fs;
    delay(100);
    ESC.write(0);
    aileronServo.write(90);
    elevatorServo.write(90);
    rudderServo.write(90);
    gimbalServo.write(90);
    delay(100);
    gimbalRightBoundAngle = (0 - gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);
    gimbalLeftBoundAngle = (0 + gimbalServoBound) + (gimbalServoTrim / gimbalServoGain);
    calibrateAttitude(); // runs IMU for a few seconds to allow it to stabilize
}
void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    timeInMillis = millis();
    getIMUdata();
    Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);

    // estimate forwards acceleration (in g's) (because the AccX,Y,Z are in g's for easy computing and reasy represtnation)
    forwardsAcceleration = AccX - 1 * sin(pitch_IMU_rad);
    // low pass forwards accel
    forwardsAcceleration = forwardsAcceleration * forwardsAcceleration_LP_param + (forwardsAcceleration_prev * (1 - forwardsAcceleration_LP_param));
    forwardsAcceleration_prev = forwardsAcceleration;

    accelData[0] = AccX;
    accelData[1] = AccY;
    accelData[2] = AccZ;
    gyroData[0] = GyroX * DEG_TO_RAD;
    gyroData[1] = GyroY * DEG_TO_RAD;
    gyroData[2] = GyroZ * DEG_TO_RAD;
    if (toggle)
    {
        ultrasonicLoop();
        // VL53L1Xloop();
    }
    else
    {
        pitotLoop();
    }
    estimateAltitude();

    toggle = !toggle;
    getCommands();
    failSafe();
    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD;
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;
    getDesState();

    // Flight Modes
    if (mode1_channel < 1400)
    {
        flight_phase = manual_flight;
        s1_command_scaled = thro_des;
        s2_command_scaled = roll_passthru;
        s3_command_scaled = pitch_passthru;
        s4_command_scaled = yaw_passthru;
        integral_pitch = 0;
        integral_roll = 0;
        roll_PID = 0;
        pitch_PID = 0;
        // throttle_integral = 0;
        altitude_integral = 0;
    }
    else if (mode1_channel < 1600)
    {
        flight_phase = stabilized_flight;
        controlANGLE();
        s1_command_scaled = thro_des;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = roll_des * DS_yaw_proportion; // no yaw stick input
    }
    // Dynamic Soaring Flight
    else
    {
        flight_phase = DS_flight;

        // Adjust elevator to avoid ground. above -0.9 is ok because -1 is the ToF code that something is wrong, and the altitude might go slightly below 0 because of the wingtip calculations being slightly off
        if (estimated_altitude < minimum_altitude && estimated_altitude > -0.9)
        {
            minimum_pitch_angle = slope_min_pitch_angle_function * estimated_altitude + intercept_min_pitch_angle_function; // linear function

            // if pitch angle is above the minimum pitch angle, thats ok
            if (pitch_des > minimum_pitch_angle)
            {
                DS_pitch_angle = pitch_des;
            }
            else
            {
                DS_pitch_angle = minimum_pitch_angle;
            }
        }
        else
        {
            DS_pitch_angle = pitch_des; // maybe in the future, have this be a sinusoidal function. because the derivative of the altitude if the altitude is sinusoidal is sinuosidal, and the derivative of the pitch angle is the pitch rate, so the pitch rate should be sinusoidal as well
        }

#if DS_AUTO_GROUND_AVOIDANCE_TEST == 1
        pitch_des = DS_pitch_angle; // set the desired pitch to the DS pitch angle
                                    // leaves roll_des as is
#elif DS_AUTO_CIRCLING_NO_GROUND_AVOIDANCE_TEST == 1
        roll_des = DS_roll_angle; // set the desired roll to DS roll angle
                                  // leaves pitch_des as is
#else // normal DS flight
        pitch_des = DS_pitch_angle; // set the desired pitch to the DS pitch angle
        roll_des = DS_roll_angle;   // set the desired roll to DS roll angle
#endif

        controlANGLE();                                        // run the PID loops for roll and pitch
        s1_command_scaled = 0;                                 // throttle to 0
        s2_command_scaled = roll_PID;                          // roll to DS roll angle
        s3_command_scaled = pitch_PID;                         // pitch to DS pitch angle
        s4_command_scaled = DS_roll_angle * DS_yaw_proportion; // yaw to a proportion of the roll angle
        // roll angle is 30 deg, so divide by 100 to get 0.3
    }

    // Log data to RAM
    if (loopCounter > (2000 / datalogRate)) // 2000 is the loop rate in microseconds
    {
        logDataToRAM();
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }

    // Log data to SD in flight if needed
    if (currentRow >= ROWS)
    {
        writeDataToSD();
        delay(5);
        clearDataInRAM();
    }

    // Log data to SD using switch (for use on the ground only)
    else if (mode2_channel < 1500)
    {
        if (!dataLogged)
        {
            writeDataToSD();
            delay(5);
            clearDataInRAM();
            // blink the LED 3 times
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(13, HIGH);
                delay(100);
                digitalWrite(13, LOW);
                delay(100);
            }
        }
        dataLogged = true;
    }
    else
    {
        dataLogged = false;
    }

    scaleCommands();
#if MOTOR_ACTIVE
    ESC_command_PWM = ESC_command_PWM * 0.861 + 14;
    ESC.write(ESC_command_PWM);
#else
    ESC.write(-100);
#endif
    aileronServo.write(aileron_command_PWM);
    elevatorServo.write(elevator_command_PWM);
    rudderServo.write(rudder_command_PWM);
    gimbalServo.write(gimbalServo_command_PWM);
    loopBlink();
    loopRate(2000);
}
/*
void estimateAltitude()
{

    gimbalServo_command_PWM = roll_IMU * gimbalServoGain + 90;
    ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad);

    // above -0.9 because ToF sends -1 as a code that it is out of range or not working
    if (ToFaltitude < 4.0 && ToFaltitude > -0.9 && roll_IMU < gimbalLeftBoundAngle && roll_IMU > gimbalRightBoundAngle) // if the ToF is in range and the gimbal is in range
    {
        leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
        rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
        estimated_altitude = leftWingtipAltitude < rightWingtipAltitude ? leftWingtipAltitude : rightWingtipAltitude;
    }
    else
    {                            // if the ToF is out of range or the gimbal is out of range
        estimated_altitude = -1; // to log that it is out of range.
    }
}*/

//using ultrasonic sensor instead
void estimateAltitude() {
    
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
        airspeed_offset += (sqrt((V)));
        delay(100);
    }
    airspeed_offset = airspeed_offset / 10.0;
}
void pitotLoop()
{
    airspeed_unadjusted = (1.0 - airspeed_LP_param) * airspeed_prev + airspeed_LP_param * fetch_airspeed(&P_dat);
    airspeed_prev = airspeed_unadjusted;
    airspeed_adjusted_prev = airspeed_adjusted;
    airspeed_adjusted = (airspeed_unadjusted - airspeed_offset) * airspeed_scalar;
}
/*
void VL53L1Xsetup()
{
    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1)
            ;
    }
    sensor.setDistanceMode(VL53L1X::Medium); // short has a range of 1m, medium has a range of 2m, long has a range of 4m
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
}
void VL53L1Xloop()
{
    distance = sensor.read(false);
    distance_LP = (1.0 - distance_LP_param) * distancePrev + distance_LP_param * distance;
    distancePrev = distance_LP;
}
*/

void ultrasonicLoop()
{
    // Check if it's time to take a sensor reading
    if (millis() - lastMeasureTime >= MEASURE_INTERVAL)
    {
        // Reset the last measurement time
        lastMeasureTime = millis();

        // Trigger a new sensor reading
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);

        // Wait for the echo signal
        long duration = pulseIn(ECHO_PIN, HIGH);

        // Calculate the distance in centimeters
        ultrasonicDistance = duration / 58;
        ultrasonicDistance /= 100.0; // convert to meters

        // Print the distance to the serial port
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" m");
    }
}
void setupSD()
{
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }

    // write a line of sample data to the SD card
    dataFile = SD.open("flightData.txt", FILE_WRITE);
    dataFile.print("TEST DATA");
    dataFile.println();
    dataFile.close();

    delay(100);

    // read the line of sample data from the SD card, only continue if it the data is correct
    while (!logSuccessful)
    {
        dataFile = SD.open("flightData.txt");
        if (dataFile)
        {
            String dataString = dataFile.readStringUntil('\r'); // read the first line of the file
            if (dataString == "TEST DATA")
            {
                Serial.println("SD card initialized correctly");
                logSuccessful = true;
            }
            else
            {
                Serial.println("SD card initialized incorrectly");
                logSuccessful = false;
            }
        }
        else
        {
            Serial.println("SD card failed to open");
            logSuccessful = false;
        }
        dataFile.print(""); // clear dataFile
        dataFile.close();
    }

    // blink LED 10 times to indicate SD card is ready
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
    }
}

void logDataToRAM()
{
    // log data to RAM

    if (currentRow < ROWS)
    {
        // time and fight phase
        dataLogArray[currentRow][0] = timeInMillis; // time in milliseconds
        dataLogArray[currentRow][1] = flight_phase; // flight phase

        // roll variables
        dataLogArray[currentRow][2] = roll_IMU;                 // roll angle from IMU in degrees
        dataLogArray[currentRow][3] = roll_des;                 // desired roll angle in degrees
        dataLogArray[currentRow][4] = aileron_command_PWM - 90; // aileron command in degrees (90 is neutral)

        // pitch variables
        dataLogArray[currentRow][5] = pitch_IMU;                 // pitch angle from IMU in degrees
        dataLogArray[currentRow][6] = pitch_des;                 // pilot desired pitch angle in degrees
        dataLogArray[currentRow][7] = DS_pitch_angle;            // DS desired pitch angle in degrees
        dataLogArray[currentRow][8] = elevator_command_PWM - 90; // elevator command in degrees (90 is neutral)

        // yaw
        dataLogArray[currentRow][9] = yaw_IMU;                  // heading in degrees (summated from gyroZ)
        dataLogArray[currentRow][10] = rudder_command_PWM - 90; // rudder command in degrees (90 is neutral)

        // speed
        dataLogArray[currentRow][11] = airspeed_adjusted;    // airspeed in m/s
        dataLogArray[currentRow][12] = s1_command_scaled;    // throttle command in percent
        dataLogArray[currentRow][13] = forwardsAcceleration; // acceleration in m/s^2

        // altitude
        dataLogArray[currentRow][14] = estimated_altitude; // altitude in meters
        dataLogArray[currentRow][15] = 0;                  // not used yet

        currentRow++;

        Serial.println(estimated_altitude);
    }
}

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

void clearDataInRAM()
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
