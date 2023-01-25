#include <Arduino.h>
#include "src_group/dRehmFlight.h"

#include "BMP180nonblocking/BMP085NB.h"
#include <Wire.h>
#include "pololuVL53L1x/VL53L1X.h"
#include "ASPD4525.h"
#include <SD.h>
#include "AltitudeEstimation/altitude.h"

#define MOTOR_ACTIVE 0
#define DATALOG 1

#define TEST_TOF 0
#define TEST_AIRSPEED 0
#define TEST_IMU 0
#define TEST_BARO 0
#define TEST_RADIO 0
#define TEST_RADIO_TO_SERVO 0
#define TEST_SERIAL 0
#define TEST_SD 0

#define TEST_DREHMFLIGHT 0

#define PID_TUNE 0
#define TEST_ALTITUDE_RIG 0
#define IMU_ALTITUDE_TEST 0
#define TEST_HORIZONTAL_MOTION 0

#define LOW_ALTITUDE_FLIGHT 0
#define LOW_ALTITUDE_HORIZ_FLIGHT 0
#define DS_AT_ALTITUDE 0

const float gimbalServoGain = -1.5;
const float gimbalServoTrim = 0;
const float gimbalServoBound = 45;
const float halfWingspan = 0.75;
const float gimbalDistanceFromCenter = 0.14;
float gimbalRightBoundAngle;
float gimbalLeftBoundAngle;

float airspeed_unadjusted;
float airspeed_prev;
const float airspeed_LP_param = 0.02;
float airspeed_offset = 0;
const float airspeed_scalar = 1.8;
float airspeed_adjusted;
float airspeed_adjusted_prev;

float ToFaltitude;
float prevToFaltitude;
int16_t distance;
float distance_LP_param = 0.5;
float distancePrev;
float distance_LP;
float leftWingtipAltitude;
float rightWingtipAltitude;
const int IRQpin = 35;
const int XSHUTpin = 34;

float IMU_vertical_accel, IMU_vertical_vel, IMU_vertical_pos;
float IMU_vertical_accel_LPparam = 0.02;
float IMU_vertical_accel_prev;

float estimated_altitude;
int altitudeTypeDataLog;

float timeInMillis;
int loopCounter = 0;
int loopCounterStep = 0;

const float wind_heading = 0.0;
const float DS_heading = 90.0;
const float heading_setup_tolerance = 5;
const float heading_rate_of_change_setup_tolerance = 10;
const float pitch_rate_of_change_setup_tolerance = 10;
const float horizontal_vel_tolerance = 0.5;

float DS_altitude_setpoint;
float DS_altitude_error;
float DS_altitude_error_prev;
const float DS_altitude_min = 0.3;
const float DS_altitude_tolerance = 0.1;
const float DS_altitude_max = 3.5;
float DS_altitude_meanline;
float DS_altitude_amplitude;

const float DS_period = 5000;
const float DS_yaw_amplitude = 30;
double DS_phase_timer = 0;
double DS_phase_start_time;

float DS_heading_rate_setpoint;
float DS_heading_rate_mean_setpoint;
float heading_rate_scalar = 100;

enum horiz_setpoint_types
{
    setpoint_horiz_accel = 0,
    setpoint_horiz_vel = 1,
    setpoint_horiz_pos = 2,

};

boolean DSifFirstRun = true;

int flight_phase;
enum flight_phases
{
    manual_flight = 7,
    stabilized_flight = 8,
    dynamic_soaring_flight = 9,
    log_data_to_SD = 10
};

const float Kp_altitude = 0.2;
const float Ki_altitude = 0.3;
const float Kd_altitude = 0.0015;
float altitude_integral = 0.0;
float altitude_integral_prev = 0.0;
const float altitude_integral_saturation_limit = 25.0;
float altitude_derivative;

float throttle_PID;
float airspeed_setpoint;
const float flight_speed = 20.0;
boolean motorOn = false;
const float stall_speed = 10.0;
float airspeed_error;
float airspeed_error_prev;
const float airspeed_error_tolerance = 1.0;
float inputted_airspeed;

float Kp_throttle;
float Ki_throttle;
float Kd_throttle;

float throttle_integral = 0.0;
float throttle_integral_prev = 0.0;
const float throttle_integral_saturation_limit = 50.0;
float throttle_derivative;
float throttle_PID_prev;
const float throttle_LP_param = 0.01;

float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad;

int ToFcounter = 0;
int ToFcounterNum = 200;

BMP085NB bmp;
File dataFile;
KalmanFilter kalmanHoriz(0.5, 0.01942384099, 0.001002176158);

KalmanFilter kalmanVert(0.5, 0.01942384099,
                        0.001002176158);
VL53L1X sensor;

float accelData[3];
float gyroData[3];

const int COLUMNS = 15;
const int ROWS = 6400;
float dataLogArray[ROWS][COLUMNS];
int currentRow = 0;
const int datalogRate = 50;
const int dataLogRateSlow = 10;
boolean dataLogged = false;

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

boolean toggle = false;

void dynamicSoar();
void DSattitude();
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

    Kp_throttle = 5.0;
    Ki_throttle = 1.0;
    Kd_throttle = 0.0;

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
#if DATALOG || PID_TUNE
    clearDataInRAM();
    setupSD();
#endif

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

    all_sensors_start_time = micros();
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
    toggle = !toggle;

    getCommands();
    failSafe();
    all_sensors_time = micros() - all_sensors_start_time;

    estimateAltitude();

    pitch_IMU_rad = pitch_IMU * DEG_TO_RAD;
    roll_IMU_rad = roll_IMU * DEG_TO_RAD;
    yaw_IMU_rad = yaw_IMU * DEG_TO_RAD;

    getDesState();

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

    if (flight_phase == manual_flight)
    {

        s1_command_scaled = thro_des;
        s2_command_scaled = roll_passthru;
        s3_command_scaled = pitch_passthru;
        s4_command_scaled = yaw_passthru;
        DSifFirstRun = true;

        integral_pitch = 0;
        integral_roll = 0;
        integral_yaw = 0;
        throttle_integral = 0;
        altitude_integral = 0;

        if (loopCounter > (2000 / dataLogRateSlow))
        {
#if DATALOG
            logDataToRAM();
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
        controlANGLE();
        motorOn = true;
        airspeed_setpoint = flight_speed;
        throttleController();

        s1_command_scaled = throttle_PID;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;

        DSifFirstRun = true;
        if (loopCounter > (2000 / dataLogRateSlow))
        {
#if DATALOG
            logDataToRAM();
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
        DS_heading_rate_mean_setpoint = yaw_passthru * heading_rate_scalar;

        dynamicSoar();
        DSattitude();
        controlANGLE();
        throttleController();

        s1_command_scaled = throttle_PID;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = yaw_PID;

        DSifFirstRun = false;
#if DATALOG
        if (loopCounter > (2000 / datalogRate))
        {
            logDataToRAM();
            loopCounter = 0;
        }
        else
        {
            loopCounter++;
        }

#endif
    }
#if DATALOG

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

    scaleCommands();
#if MOTOR_ACTIVE

    ESC_command_PWM = ESC_command_PWM * 0.861 + 14;
    ESC.write(ESC_command_PWM);
#else
    ESC.write(-100);
#endif

#if TEST_ALTITUDE_RIG == 0
    aileronServo.write(aileron_command_PWM);
    elevatorServo.write(elevator_command_PWM);
    rudderServo.write(rudder_command_PWM);
#endif
    gimbalServo.write(gimbalServo_command_PWM);

    loopBlink();
    loopRate(2000);
}
#endif

void dynamicSoar()
{

    if (DS_phase_timer >= DS_period)
    {
        DS_phase_start_time = millis();
    }
    else
    {
        DS_phase_timer = millis() - DS_phase_start_time;
    }
    flight_phase = ((2 * PI) / DS_period) * DS_phase_timer;

    DS_altitude_setpoint = DS_altitude_meanline + DS_altitude_amplitude * sin(flight_phase);

    DS_heading_rate_setpoint = cos(flight_phase) * DS_yaw_amplitude + DS_heading_rate_mean_setpoint;
}

void DSattitude()
{
}

void throttleController()
{
    if (motorOn)
    {
        throttle_integral_prev = throttle_integral;
        airspeed_error_prev = airspeed_error;
        airspeed_error = airspeed_setpoint - airspeed_adjusted;
        throttle_integral = throttle_integral_prev + airspeed_error * dt;
        throttle_integral = constrain(throttle_integral, 0, throttle_integral_saturation_limit);
        throttle_derivative = (airspeed_error - airspeed_error_prev) / dt;
        throttle_PID = 0.01 * (Kp_throttle * airspeed_error + Ki_throttle * throttle_integral - Kd_throttle * throttle_derivative);

        throttle_PID = (1.0 - throttle_LP_param) * throttle_PID_prev + throttle_LP_param * throttle_PID;
        throttle_PID_prev = throttle_PID;
    }
    else
    {
        throttle_PID = 0.0;
    }
}

void estimateAltitude()
{

    gimbalServo_command_PWM = roll_IMU * gimbalServoGain + 90;
    ToFaltitude = (distance_LP / 1000.0) * cos(pitch_IMU_rad);

#if IMU_ALTITUDE_TEST == 0
    if (ToFaltitude < 4.0 && distance > 0.0)

#else

    if (ToFaltitude < 0.5 && distance > 0.0)
#endif
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
    }
    else
    {

        altitudeTypeDataLog = 3;
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
        dataLogArray[currentRow][12] = airspeed_setpoint;
        dataLogArray[currentRow][13] = throttle_PID;

        dataLogArray[currentRow][14] = estimated_altitude;

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