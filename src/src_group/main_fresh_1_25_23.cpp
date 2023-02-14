// TO DO
// implement altitude estimation with IMU, maybe baro, maybe with the pitch of the plane???
// do unit testing of the code functions and calulations
// reorgnaize and recomment code (much later)

#include <Arduino.h>
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu

#include "BMP180nonblocking/BMP085NB.h"
#include <Wire.h>
#include "pololuVL53L1x/VL53L1X.h"
#include "ASPD4525.h"
#include <SD.h>
#include "AltitudeEstimation/altitude.h"
#define MOTOR_ACTIVE 1
#define DS_LOW_ALTITUDE_CIRCLE 0

// const int IRQpin = 35;
// const int XSHUTpin = 34;

// Constants for Gimbal Servo
const float gimbalServoGain = -1.5;
const float gimbalServoTrim = 0;
const float gimbalServoBound = 45;
const float halfWingspan = 0.75;
const float gimbalDistanceFromCenter = 0.14;

// Variables for Gimbal Servo
float gimbalRightBoundAngle;
float gimbalLeftBoundAngle;

// Constants and Variables for Airspeed
const float airspeed_LP_param = 0.02;
const float airspeed_scalar = 1.8;
float airspeed_offset = 0;
float airspeed_unadjusted;
float airspeed_prev;
float airspeed_adjusted;
float airspeed_adjusted_prev;

// Constants and Variables for Altitude
const float DS_altitude_min = 1;
const float DS_altitude_max = 5;
const float Kp_altitude = 0.2;
const float Ki_altitude = 0.3;
const float Kd_altitude = 0.0015;
const float altitude_integral_saturation_limit = 25.0;
const int datalogRate = 50;
float altitude_integral = 0.0;
float altitude_integral_prev = 0.0;
float altitude_derivative;
float estimated_altitude;
float ToFaltitude;
int16_t distance;
float distance_LP_param = 0.5;
float distancePrev;
float distance_LP;
int altitudeTypeDataLog;
float leftWingtipAltitude;
float rightWingtipAltitude;
float DS_altitude_setpoint;
float DS_altitude_error;
float DS_altitude_meanline;
float DS_altitude_amplitude;
float safe_circling_altitude = 3;

// Barometer Variables
int temperature = 0;
long pressure = 0;
float alti = 0;
unsigned long timer = 0;
const int altitude_offset_num_vals = 10;
float altitudeMeasured; // The raw altitude reading from the barometric pressure sensor (in m)
float altitude_offset;
float altitude_baro;           // The altitude estimated from the barometer, with a low pass filter and offset adjustment applied/ float altitude_prev;                     // The previous reading of the barometric pressure sensor
float altitude_LP_param = 0.1; // The low pass filter parameter for altitude (smaller values means a more smooth signal but higher delay time)
float altitude_prev;
int offset_loop_counter = 0;
float altitude_offset_sum = 0;

// Constants and Variables for Dynamic Soaring
const float DS_cycle_radius = 15;
const float DS_yaw_setpoint_scalar = 1.0;
const float DS_roll_setpoint_scalar = 0.2;
const float DS_pitch_setpoint_scalar = 0.2;
float flight_throttle = 0.6;
float wind_offset;
float DS_heading_rate_setpoint;
float DS_heading_rate_mean_setpoint;
float pilot_adjusted_leeway;
float pilot_adjusted_leeway_scalar = 2;

// Variables for Flight Control
float timeInMillis;
int loopCounter = 0;
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad;
float accelData[3];
float gyroData[3];

// float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;
// float IMU_vertical_accel_LPparam = 0.02;
// float IMU_vertical_accel_prev;

// float airspeed_setpoint;
// const float flight_speed = 20.0;
// float airspeed_error;
// float airspeed_error_prev;
// const float stall_speed = 10.0;
// boolean motorOn = false;

// Variables for Data Logging
const int COLUMNS = 16;
const int ROWS = 6400;
float dataLogArray[ROWS][COLUMNS];
boolean dataLogged = false;
boolean toggle = false;
int currentRow = 0;


// Flight Phases
boolean DSifFirstRun = true;
float flight_phase;
enum flight_phases
{
    manual_flight = 7,
    stabilized_flight = 8,
    //    dynamic_soaring_flight = 9,
    log_data_to_SD = 10
};

// Objects
File dataFile;
KalmanFilter kalmanVert(0.5, 0.01942384099, 0.001002176158);
VL53L1X sensor;
BMP085NB bmp;

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

///@brief yo this is setup @note yo this is note   MUST ADD THESE AT THE END, also use them in the readme
void setup()
{
    Kp_roll_angle = 0.3;
    Ki_roll_angle = 0.3;
    Kd_roll_angle = 0.3;
    Kp_pitch_angle = 0.5;
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
}
void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    timeInMillis = millis();
    getIMUdata();
    Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);

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
    if (loopCounter > (2000 / datalogRate))
    {
        logDataToRAM();
        loopCounter = 0;
    }
    else
    {
        loopCounter++;
    }
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
    if (ToFaltitude < 4.0 && distance > 0.0)
    {
        if (roll_IMU > gimbalRightBoundAngle && roll_IMU < gimbalLeftBoundAngle)
        {
            leftWingtipAltitude = ToFaltitude - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
            rightWingtipAltitude = ToFaltitude + sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
            estimated_altitude = leftWingtipAltitude < rightWingtipAltitude ? leftWingtipAltitude : rightWingtipAltitude;
            altitudeTypeDataLog = 0;
        }
        else
        {
            if (roll_IMU > gimbalLeftBoundAngle)
            {
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan - gimbalDistanceFromCenter);
                altitudeTypeDataLog = 1;
            }
            else
            {
                estimated_altitude = ToFaltitude * cos(roll_IMU_rad - (gimbalLeftBoundAngle * DEG_TO_RAD)) - sin(roll_IMU_rad) * (halfWingspan + gimbalDistanceFromCenter);
                altitudeTypeDataLog = 2;
            }
        }

        // recalibrate barometer, every 10 times reset it
        if (offset_loop_counter < altitude_offset_num_vals)
        {
            offset_loop_counter++;
            altitude_offset_sum += altitudeMeasured - ToFaltitude;
        }
        else
        {
            offset_loop_counter = 0;
            altitude_offset = (altitude_offset / altitude_offset_num_vals);
            altitude_offset_sum = 0;
        }
    }
    else
    {
        // just barometer, but prevent it from drifting too LOW, if too high thats ok the uav will descend until in range of ToF, but if too low itll just keep flying upp

        if (altitude_baro < 4.0)
        {
            estimated_altitude = 4.0;
            altitudeTypeDataLog = 3;
        }
        else
        {
            estimated_altitude = altitude_baro;
            altitudeTypeDataLog = 4;
        }
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
void setupSD()
{
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }
}
void logDataToRAM()
{
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
        dataLogArray[currentRow][15] = AccX; // forwards acceleration
        currentRow++;
    }
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
    //    Serial.println(altitude_offset);
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