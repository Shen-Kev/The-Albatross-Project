

#include <Arduino.h>               // Arduino library
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu

#include "BMP180nonblocking/BMP085NB.h"  // Barometer library
#include <Wire.h>                        // I2C library
#include "pololuVL53L1x/VL53L1X.h"       // ToF sensor library
#include "ASPD4525.h"                    // Pitot tube library
#include <SD.h>                          // SD card library
#include "AltitudeEstimation/altitude.h" // Altitude estimation library
#define MOTOR_ACTIVE 1                   // 1 = motor is active, 0 = motor is not active
#define DS_LOW_ALTITUDE_CIRCLE 0         // 1 = Dynamic Soaring in low altitude circle, 0 = Dynamic Soaring in high altitude circle

// Constants for Gimbal Servo
const float gimbalServoGain = -1.5;
const float gimbalServoTrim = 0;
const float gimbalServoBound = 45;           // The maximum angle the gimbal servo can move in either direction
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
const float DS_altitude_min = 1;                       // The minimum altitude for Dynamic Soaring (in m)
const float DS_altitude_max = 5;                       // The maximum altitude for Dynamic Soaring (in m)
const float Kp_altitude = 0.2;                         // The proportional gain for altitude control
const float Ki_altitude = 0.3;                         // The integral gain for altitude control
const float Kd_altitude = 0.0015;                      // The derivative gain for altitude control
const float altitude_integral_saturation_limit = 25.0; // The saturation limit for the integral term of the altitude controller
const int datalogRate = 50;                            // The rate at which data is logged to the SD card (in Hz)
float altitude_integral = 0.0;                         // The integral term of the altitude controller
float altitude_integral_prev = 0.0;                    // The previous value of the integral term of the altitude controller
float altitude_derivative;                             // The derivative term of the altitude controller
float estimated_altitude;                              // The estimated altitude of the aircraft (in m)
float ToFaltitude;                                     // The altitude of the aircraft estimated from the ToF sensor (in m)

int16_t distance;                  // The raw distance reading from the ToF sensor (in mm)
float distance_LP_param = 0.03;    // The low pass filter parameter for distance (smaller values means a more smooth signal but higher delay time)
float distancePrev;                // The previous reading of the ToF sensor
float distance_LP;                 // The distance reading from the ToF sensor, with a low pass filter applied
int altitudeTypeDataLog;           // The type of altitude data that is being logged to the SD card
float leftWingtipAltitude;         // The altitude of the left wingtip (in m)
float rightWingtipAltitude;        // The altitude of the right wingtip (in m)
float DS_altitude_setpoint;        // The altitude setpoint for Dynamic Soaring (in m)
float DS_altitude_error;           // The altitude error for Dynamic Soaring (in m)
float DS_altitude_meanline;        // The altitude of the meanline for Dynamic Soaring (in m)
float DS_altitude_amplitude;       // The amplitude of the altitude oscillation for Dynamic Soaring (in m)
float safe_circling_altitude = 30; // The altitude at which the aircraft will circle for testing (in m)

// Barometer Variables
int temperature = 0;
long pressure = 0;
float alti = 0;
unsigned long timer = 0;                 // The time at which the barometer was last read (in ms)
const int altitude_offset_num_vals = 10; // The number of altitude readings to take to calculate the offset
float altitudeMeasured;                  // The raw altitude reading from the barometric pressure sensor (in m). iS LP FILTERED
float altitude_offset;                   // The offset for the barometric pressure sensor
float altitude_baro;                     // The altitude estimated from the barometer, with a low pass filter and offset adjustment applied/ float altitude_prev;                     // The previous reading of the barometric pressure sensor
float altitude_LP_param = 0.1;           // The low pass filter parameter for altitude (smaller values means a more smooth signal but higher delay time)
float altitude_prev;                     // The previous reading of the barometric pressure sensor
int offset_loop_counter = 0;             // The number of times the loop has run while calculating the offset
float altitude_offset_sum = 0;           // The sum of the altitude readings to calculate the offset

// Constants and Variables for Dynamic Soaring
const float DS_cycle_radius = 15; // The radius of the circle that the aircraft will circle in Dynamic Soaring (in m)
const float DS_yaw_setpoint_scalar = 1.0;
const float DS_roll_setpoint_scalar = 0.2;
const float DS_pitch_setpoint_scalar = 0.2;
float flight_throttle = 0.6;
float wind_offset;
float DS_heading_rate_setpoint;
float DS_heading_rate_mean_setpoint;
float pilot_adjusted_leeway;
float pilot_adjusted_leeway_scalar = 2;
float forwardsAcceleration; // The acceleration in the forwards direction (in m/s^2)

// Variables for Flight Control
float timeInMillis;                             // The time in milliseconds since the flight controller has started
int loopCounter = 0;                            // The number of times the loop has run
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad; // The raw pitch, roll, and yaw angles in radians from the IMU
float accelData[3];                             // The raw accelerometer data from the IMU
float gyroData[3];                              // The raw gyro data from the IMU

// Variables for Data Logging
const int COLUMNS = 16;            // 16 columns of data to be logged to the SD card
const int ROWS = 6400;             //
float dataLogArray[ROWS][COLUMNS]; // The array that stores the data to be logged to the SD card
boolean dataLogged = false;        // Used to determine if the data has been logged to the SD card
boolean toggle = false;            // Used to toggle the LED
int currentRow = 0;                // The current row of the data log array

// Flight Phases
boolean DSifFirstRun = true; // Used to determine if the first run of the dynamic soaring loop has been completed
float flight_phase;          // The current flight phase
enum flight_phases           // Flight phases for the flight controller
{
    // 1-2pi are for DS
    manual_flight = 7,
    stabilized_flight = 8,
    log_data_to_SD = 10
};

// Objects
File dataFile;  // File object for SD card
VL53L1X sensor; // ToF sensor object
BMP085NB bmp;   // Barometer object

// Functions
void dynamicSoar();
void coordinatedController();
void throttleController();
void horizontal();
void estimateAltitude();
void pitotSetup();
void pitotLoop();
void setupSD();
void logDataToRAM();
void clearDataInRAM();
void writeDataToSD();
void VL53L1Xsetup();
void VL53L1Xloop();
void BMP180setup();
void BMP180loop();

// Flight Controller Setup
// This function is run once when the flight controller is turned on
// It is used to initialize the flight controller and set the initial values of the variables and objects used in the flight controller loop function (loop())
void setup()
{

    // Constants for PID
    Kp_roll_angle = 0.5;
    Ki_roll_angle = 0.3;
    Kd_roll_angle = 0.3;
    Kp_pitch_angle = 1;
    Ki_pitch_angle = 0.3;
    Kd_pitch_angle = 0.3;
    Kp_yaw = 0.5;
    Ki_yaw = 0.3;
    Kd_yaw = 0.0015;

    Serial.begin(500000);
    Serial.println("serial works");
    Wire.begin();
    Wire.setClock(1000000);
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
    BMP180setup();
    Serial.println("passed baro init");

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
    VL53L1Xsetup();
    Serial.println("passed ToF setup");
    pitotSetup();
    Serial.println("passed pitot");
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
    DS_altitude_meanline = (DS_altitude_max + DS_altitude_min) / 2.0;
    DS_altitude_amplitude = (DS_altitude_max - DS_altitude_min) / 2.0;

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

    accelData[0] = AccX;
    accelData[1] = AccY;
    accelData[2] = AccZ;
    gyroData[0] = GyroX * DEG_TO_RAD;
    gyroData[1] = GyroY * DEG_TO_RAD;
    gyroData[2] = GyroZ * DEG_TO_RAD;
    if (toggle)
    {
        VL53L1Xloop();
    }
    else
    {
        pitotLoop();
    }
    BMP180loop(); // BMP only takes around 3microseconds per loop.

    toggle = !toggle;
    getCommands();
    failSafe();
    estimateAltitude();
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
        DSifFirstRun = true;
        integral_pitch = 0;
        integral_roll = 0;
        integral_yaw = 0;
        // throttle_integral = 0;
        altitude_integral = 0;
    }
    else if (mode1_channel < 1600)
    {
        flight_phase = stabilized_flight;

        controlANGLE();

        s1_command_scaled = flight_throttle;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;
        DSifFirstRun = true;
    }
    else // DS flight. in the fligtht mode it will show up as 0-2pi
    {
        //        DS_heading_rate_mean_setpoint = yaw_passthru * heading_rate_scalar;
        pilot_adjusted_leeway = throttle_channel * pilot_adjusted_leeway_scalar;

        dynamicSoar();
        controlANGLE();
        //  throttleController();

        s1_command_scaled = flight_throttle;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;
        DSifFirstRun = false;
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

    // Log data to SD in flight
    if (currentRow >= ROWS)
    {
        writeDataToSD();
        delay(5);
        clearDataInRAM();
    }

    // Log data to SD using switch (for use on the ground only)
    else if (mode2_channel < 1500 && !dataLogged)
    {
        writeDataToSD();
        delay(5);
        clearDataInRAM();
        // blink the LED 5 times
        for (int i = 0; i < 5; i++)
        {
            digitalWrite(13, HIGH);
            delay(100);
            digitalWrite(13, LOW);
            delay(100);
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

void dynamicSoar()
{
    // turning counterclockwise, because thats how unit circle works and also puts the left wing with the sensor always closer to the ground
    if (DSifFirstRun)
    {
        flight_phase = 0; /// is the angle relative to the wind in radians
        wind_offset = yaw_IMU_rad;
    }
    // offest the yaw IMU to align with the starting direction
    flight_phase = yaw_IMU_rad - wind_offset;

#if DS_LOW_ALTITUDE_CIRCLE
    DS_heading_rate_setpoint = safe_circling_altitude; // a safe altitude
#else
    // heading rate based on airspeed, flight, the radius of the circle, and the cos(flightphase)*pilot_adjusted is to adjust the arispeed to estimate groundspeed through pilot input.
    DS_heading_rate_setpoint = (airspeed_adjusted - (cos(flight_phase) * pilot_adjusted_leeway)) / DS_cycle_radius;
#endif

    // altitude based on the phase. counterclockwise flight, flight phase 0 when into the wind, so at pi/2 radians to counterclockwise should be max height
    DS_altitude_setpoint = DS_altitude_amplitude * sin(flight_phase + (PI / 2)) + DS_altitude_meanline;

    // convert altitude and heading, global frame,  into pitch and yaw, local frame
    pitch_des = DS_altitude_setpoint * cos(roll_IMU_rad) - DS_heading_rate_setpoint * sin(roll_IMU_rad);
    yaw_des = DS_heading_rate_setpoint * cos(roll_IMU_rad) - DS_altitude_setpoint * sin(roll_IMU_rad);

    // convert to degrees
    roll_des *= RAD_TO_DEG;
    pitch_des *= RAD_TO_DEG;
    yaw_des *= RAD_TO_DEG;

    // scale to reasonable commands that couldve been given by pilot, to be sent to PID
    roll_des *= DS_roll_setpoint_scalar;
    pitch_des *= DS_pitch_setpoint_scalar;
    yaw_des *= DS_yaw_setpoint_scalar;
}

void estimateAltitude()
{
    gimbalServo_command_PWM = roll_IMU * gimbalServoGain + 90;
    ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad);

    // maybhe have a hevaily LP of estimated altitude of ToF altitude PID loop for altitude

    if (ToFaltitude < 4.0 && distance > 0.0 && roll_IMU < gimbalLeftBoundAngle && roll_IMU > gimbalRightBoundAngle)
    {
        leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
        rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
        estimated_altitude = leftWingtipAltitude < rightWingtipAltitude ? leftWingtipAltitude : rightWingtipAltitude;
        altitudeTypeDataLog = 0;

        if (offset_loop_counter < altitude_offset_num_vals)
        {
            offset_loop_counter++;
            altitude_offset_sum += (altitudeMeasured - estimated_altitude);
        }
        else
        {
            offset_loop_counter = 0;
            altitude_offset = (altitude_offset_sum / altitude_offset_num_vals);
            altitude_offset_sum = 0;
        }
    }
    else
    {
        if (roll_IMU > gimbalLeftBoundAngle)
        {
            altitudeTypeDataLog = 1;
        }
        else if (roll_IMU < gimbalRightBoundAngle)
        {
            altitudeTypeDataLog = 2;
        }
        else
        {
            altitudeTypeDataLog = 4;
        }

        estimated_altitude = altitude_baro;
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
void VL53L1Xsetup()
{
    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1)
            ;
    }
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
}
void VL53L1Xloop()
{
    distance = sensor.read(false);
    distance_LP = (1.0 - distance_LP_param) * distancePrev + distance_LP_param * distance;
    distancePrev = distance_LP;
}

void BMP180setup()
{
    bmp.initialize();
    // Get average offset, which should be close to 0. This is done because the offset will need to change, while the baseline pressure can't.
    for (int i = 0; i < altitude_offset_num_vals; i++)
    {
        while (!bmp.newData)
        {
            bmp.pollData(&temperature, &pressure, &altitudeMeasured);
        }
        bmp.pollData(&temperature, &pressure, &altitudeMeasured);
        altitude_offset += altitudeMeasured;
        Serial.println(i);
    }
    altitude_offset = altitude_offset / ((float)altitude_offset_num_vals);
    Serial.print("altitude offset: ");
    Serial.println(altitude_offset);
    delay(1000);
}

// This function offsets and low pass filters the barometric altitude reading
void BMP180loop()
{
    bmp.pollData(&temperature, &pressure, &altitudeMeasured);
    if (bmp.newData)
    {
        altitudeMeasured = ((1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured);
        altitude_prev = altitudeMeasured;
    }
    altitude_baro = altitudeMeasured - altitude_offset;
}

void setupSD()
{
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }
}

void logDataToRAM()
{
    // log data to RAM

    if (currentRow < ROWS)
    {
        dataLogArray[currentRow][0] = timeInMillis;
        dataLogArray[currentRow][1] = flight_phase;
        dataLogArray[currentRow][2] = roll_IMU;
        dataLogArray[currentRow][3] = roll_des;
        dataLogArray[currentRow][4] = roll_PID;
        dataLogArray[currentRow][5] = pitch_IMU;
        dataLogArray[currentRow][6] = pitch_des;
        dataLogArray[currentRow][7] = pitch_PID;
        dataLogArray[currentRow][8] = GyroZ;
        dataLogArray[currentRow][9] = yaw_des;
        dataLogArray[currentRow][10] = yaw_PID;
        dataLogArray[currentRow][11] = airspeed_adjusted;
        dataLogArray[currentRow][12] = s1_command_scaled;
        dataLogArray[currentRow][13] = estimated_altitude;
        dataLogArray[currentRow][14] = altitudeTypeDataLog;
        dataLogArray[currentRow][15] = forwardsAcceleration;
        currentRow++;
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
