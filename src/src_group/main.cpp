#include <Arduino.h>               // Arduino library
#include "src_group/dRehmFlight.h" //  Modified and used dRehmFlight: https://github.com/nickrehm/dRehmFlight
                                   //  Credit to: Nicholas Rehm
                                   //  Department of Aerospace Engineering
                                   //  University of Maryland
                                   //  College Park 20742
                                   //  Email: nrehm@umd.edu

#include <Wire.h>      // I2C library
#include "ASPD4525.h"  // Pitot tube library
#include <SD.h>        // SD card library
#define MOTOR_ACTIVE 1 // 1 = motor is active, 0 = motor is not active
#define CONTROL_TEST 1 // 1 = control test, 0 = DS test

// Constants and Variables for Airspeed
const float airspeed_LP_param = 0.02; // The low pass filter parameter for airspeed (smaller values means a more smooth signal but higher delay time)
const float airspeed_scalar = 1.8;    // The scalar to convert the raw airspeed reading to m/s
float airspeed_offset = 0;            // The offset for the airspeed sensor
float airspeed_unadjusted;            /// The raw airspeed reading from the pitot tube (in m/s)
float airspeed_prev;                  // The previous reading of the airspeed sensor
float airspeed_adjusted;              // The airspeed reading from the pitot tube, with a low pass filter and offset adjustment applied
float airspeed_adjusted_prev;         // The previous reading of the airspeed sensor, with a low pass filter and offset adjustment applied

const int datalogRate = 50; // The rate at which data is logged to the SD card (in Hz)

// Constants and Variables for Forwards Acceleration
float forwardsAcceleration;                  // The acceleration in the forwards direction (in m/s^2)
float forwardsAcceleration_LP_param = 0.001; // The low pass filter parameter for forwardsAcceleration (smaller values means a more smooth signal but higher delay time)
float forwardsAcceleration_prev;             // The previous reading of the forwardsAcceleration

// Variables for Flight Control
float timeInMillis;                             // The time in milliseconds since the flight controller has started
int loopCounter = 0;                            // The number of times the loop has run
float pitch_IMU_rad, roll_IMU_rad, yaw_IMU_rad; // The raw pitch, roll, and yaw angles in radians from the IMU
float accelData[3];                             // The raw accelerometer data from the IMU
float gyroData[3];                              // The raw gyro data from the IMU
float yaw_IMU_rad_prev;
float heading_changed_last_loop;

// Dynamic Soaring Variables
float DS_roll_angle = 30;        // The bank angle for Dynamic Soaring (in degrees) (turning RIGHT)
float DS_yaw_proportion = 0.005; // The proportion of yaw in degrees to roll 0-1 for Dynamic Soaring
float DS_pitch_max = 20;         // The maximum pitch angle for Dynamic Soaring (in degrees)
float DS_pitch_exit = 15;
float DS_throttle_exit = 0.5; // throttle exiting the DS
boolean DS_turn = false;
boolean DS_first_activated = false;
boolean DS_speed_met = false;
float DS_speed = 10; // m/s
float DS_start_heading;
float DS_pitch_offset = 5; // at all times the angle with be 5 deg more than just the raw cos wave to account for gravity pulling the UAV down
float yaw_commmand_scaled;
float angle_turned_radians;
float throttle_scaled;
float totalTurnAngle = 135; // degrees the UAV should turn
float totalTurnAngleRadians;
float DSstartTime;
boolean needToLogDSdata = false;
boolean rollMetDes = false;    // true if the roll has met the desired roll at at least once in the DS cycle
float rollMetDesTolerance = 5; // degrees

// Variables for Data Logging
const int COLUMNS = 13;            // 16 columns of data to be logged to the SD card
const int ROWS = 7900;             // 7800 rows of data to be logged to the SD card
float dataLogArray[ROWS][COLUMNS]; // The array that stores the data to be logged to the SD card
boolean dataLogged = false;        // Used to determine if the data has been logged to the SD card
boolean toggle = false;            // Used to toggle the LED
int currentRow = 0;                // The current row of the data log array
boolean logSuccessful = false;     // Used to determine if the data has been successfully logged to the SD card
float accelSum = 0;                // the sum of the accelerations in a DS cycle
int accelNum;                      // the number of acceleation values in a DS cycle
float accelAvg;                    // the average acceleration in a DS cycle

// Flight Phases
float flight_phase; // The current flight phase
enum flight_phases  // Flight phases for the flight controller
{
    manual_flight = 1,
    stabilized_flight = 2,
    DS_flight = 3,
    control_flight = 4
};

File dataFile; // File object for SD card

// Functions
void pitotSetup();
void pitotLoop();
void setupSD();
void logDataToRAM();
void clearDataInRAM();
void writeDataToSD();

// Flight Controller Setup
// This function is run once when the flight controller is turned on
// It is used to initialize the flight controller and set the initial values of the variables and objects used in the flight controller loop function (loop())
void setup()
{
    // Constants for PID
    Kp_roll_angle = 1.0;
    Ki_roll_angle = 0.3;
    Kd_roll_angle = 0.2;
    Kp_pitch_angle = 2.0;
    Ki_pitch_angle = 0.5;
    Kd_pitch_angle = 0.4;

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

    AccErrorX = 0.07;
    AccErrorY = 0.03;
    AccErrorZ = 0.11;
    GyroErrorX = -3.59;
    GyroErrorY = 0.07;
    GyroErrorZ = 1.26;

    for (int i = 0; i < 1000; i++)
    {
        getIMUdata();
        Madgwick6DOF(GyroX, GyroY, GyroZ, -AccX, AccY, AccZ, dt);
    }
    //    VL53L1Xsetup();
    // Serial.println("passed ToF setup");
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
    calibrateAttitude(); // runs IMU for a few seconds to allow it to stabilize
    totalTurnAngleRadians = totalTurnAngle * DEG_TO_RAD;
}
void loop()
{
    // time per loop
    // printLoopRate();

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

    pitotLoop();
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
        angle_turned_radians = 0;
        DS_turn = true;
        rollMetDes = false;
        DS_speed_met = false;
        DSstartTime = timeInMillis;
        accelSum = 0;
        accelNum = 0;
    }
    else if (mode1_channel < 1600)
    {
        flight_phase = stabilized_flight;
        controlANGLE();
        s1_command_scaled = thro_des;
        s2_command_scaled = roll_PID;
        s3_command_scaled = pitch_PID;
        s4_command_scaled = roll_des * DS_yaw_proportion; // no yaw stick input
        DS_first_activated = true;
        angle_turned_radians = 0;
        DS_turn = true;
        rollMetDes = false;
        DS_speed_met = false;
        DSstartTime = timeInMillis;
        accelSum = 0;
        accelNum = 0;
    }

#if CONTROL_TEST == 1
    //================================================================================================================================
    // Control Flight Start ==================================================================================================
    //================================================================================================================================

    else
    {
        flight_phase = control_flight;
        // fly in a turn which pitches up to maintain altitude. Every totalTurnAngle, log accel data, then accelerate back up to 10m/s

        angle_turned_radians += GyroZ * DEG_TO_RAD * dt;
        yaw_IMU_rad_prev = yaw_IMU_rad;

        // if DS has turned enough, DS turn is over
        if (DS_turn && abs(angle_turned_radians) > totalTurnAngleRadians)
        {
            DS_turn = false;
        }
        // Don't start DS until airspeed is met (going too fast not a problem, too slow and it will stall)
        if (airspeed_adjusted > DS_speed)
        {
            DS_speed_met = true; // only should change once per cycle, from false to true
        }

        if (DS_turn && DS_speed_met)
        {
            pitch_des = 20; // to maintain alt
            roll_des = DS_roll_angle; //DS turn essentially 
            yaw_commmand_scaled = DS_roll_angle * DS_yaw_proportion;
            throttle_scaled = 0;
            accelSum += forwardsAcceleration;
            accelNum++;
            needToLogDSdata = true;
        }
        else if (needToLogDSdata && !DS_turn)
        {
            // RUNS ONCE AFTER DS TURN
            needToLogDSdata = false;
            accelAvg = accelSum / float(accelNum);
            accelSum = 0;
            accelNum = 0;

            dataFile = SD.open("accelDataControl.txt", FILE_WRITE);
            dataFile.print(DSstartTime);
            dataFile.print(",");
            dataFile.print(timeInMillis); // the end time
            dataFile.print(",");
            dataFile.print(accelAvg);
            dataFile.println();
            dataFile.close();
        }
        else if (!DS_turn)
        {
            //restart cycle, so these tests can be done quickly and repeatedly
            angle_turned_radians = 0;
            DS_turn = true;
            DS_speed_met = false;
            DSstartTime = timeInMillis;
            accelSum = 0;
            accelNum = 0;  
        }
        else
        {
            // BEFORE DS TURN.
            // set motor to 80% power, pitch and roll to 0
            throttle_scaled = 0.8;
            roll_PID = 0;
            pitch_PID = -5;
            yaw_commmand_scaled = 0;
        }
        pitch_des += pitch_passthru * 60;        // add in pitch stick input, goes from -0.5 to 0.5, so multiply by 60 to get to -30 to 30 (to avoid crashes)
        controlANGLE();                          // run the PID loops for roll and pitch
        s1_command_scaled = throttle_scaled;     // throttle to 0
        s2_command_scaled = roll_PID;            // roll to DS roll angle
        s3_command_scaled = pitch_PID;           // pitch to DS pitch angle
        s4_command_scaled = yaw_commmand_scaled; // yaw to a proportion of the roll angle

        DS_first_activated = false;
    }
    //================================================================================================================================
    // Control Flight End ==================================================================================================
    //================================================================================================================================

#elif
    //================================================================================================================================
    // Dynamic Soaring Flight Start ==================================================================================================
    //================================================================================================================================
    else
    {
        flight_phase = DS_flight;

        angle_turned_radians += GyroZ * DEG_TO_RAD * dt;
        yaw_IMU_rad_prev = yaw_IMU_rad;

        // if DS has turned enough, DS turn is over
        if (DS_turn && abs(angle_turned_radians) > totalTurnAngleRadians)
        {
            DS_turn = false;
        }
        // Don't start DS until airspeed is met (going too fast not a problem, too slow and it will stall)
        if (airspeed_adjusted > DS_speed)
        {
            DS_speed_met = true; // only should change once per cycle, from false to true
        }

        if (DS_turn && DS_speed_met)
        {
            Serial.println(pitch_passthru);

            // DS TURN
            if (abs(roll_IMU - DS_roll_angle) < rollMetDesTolerance)
            {
                rollMetDes = true;
            }
            roll_des = DS_roll_angle;
            if (rollMetDes)
            {
                pitch_des = DS_pitch_max * cos(angle_turned_radians) + DS_pitch_offset;
            }
            else
            {
                pitch_des = DS_pitch_offset;
            }
            yaw_commmand_scaled = DS_roll_angle * DS_yaw_proportion;
            throttle_scaled = 0;
            accelSum += forwardsAcceleration;
            accelNum++;
            needToLogDSdata = true;
        }
        else if (needToLogDSdata && !DS_turn)
        {
            // RUNS ONCE AFTER DS TURN
            needToLogDSdata = false;
            accelAvg = accelSum / float(accelNum);
            accelSum = 0;
            accelNum = 0;

            dataFile = SD.open("accelData.txt", FILE_WRITE);
            dataFile.print(DSstartTime);
            dataFile.print(",");
            dataFile.print(timeInMillis); // the end time
            dataFile.print(",");
            dataFile.print(accelAvg);
            dataFile.println();
            dataFile.close();
        }
        else if (!DS_turn)
        {
            // AFTER DS TURN.
            throttle_scaled = DS_throttle_exit;
            roll_des = 0;
            pitch_des = DS_pitch_exit;
            yaw_commmand_scaled = 0;
        }
        else
        {
            // BEFORE DS TURN.
            // set motor to 80% power, pitch and roll to 0
            throttle_scaled = 0.8;
            roll_PID = 0;
            pitch_PID = 0;
            yaw_commmand_scaled = 0;
        }
        pitch_des += pitch_passthru * 60;        // add in pitch stick input, goes from -0.5 to 0.5, so multiply by 60 to get to -30 to 30
        controlANGLE();                          // run the PID loops for roll and pitch
        s1_command_scaled = throttle_scaled;     // throttle to 0
        s2_command_scaled = roll_PID;            // roll to DS roll angle
        s3_command_scaled = pitch_PID;           // pitch to DS pitch angle
        s4_command_scaled = yaw_commmand_scaled; // yaw to a proportion of the roll angle

        DS_first_activated = false;

        //================================================================================================================================
        // Dynamic Soaring Flight End =====================================================================================================
        //================================================================================================================================
    }
#endif
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

void setupSD()
{
    while (!SD.begin(BUILTIN_SDCARD))
    {
        delay(1000);
    }

    // write a line of sample data to the SD card
    dataFile = SD.open("SDtest.txt", FILE_WRITE);
    dataFile.print("TEST DATA");
    dataFile.println();
    dataFile.close();

    delay(100);

    // read the line of sample data from the SD card, only continue if it the data is correct
    while (!logSuccessful)
    {
        dataFile = SD.open("SDtest.txt");
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
        dataLogArray[currentRow][7] = elevator_command_PWM - 90; // elevator command in degrees (90 is neutral)

        // yaw
        dataLogArray[currentRow][8] = angle_turned_radians * RAD_TO_DEG; // yaw angle from DS in degrees
        dataLogArray[currentRow][9] = 180 - rudder_command_PWM;          // rudder command in degrees (90 is neutral)

        // speed
        dataLogArray[currentRow][10] = airspeed_adjusted;    // airspeed in m/s
        dataLogArray[currentRow][11] = s1_command_scaled;    // throttle command in percent
        dataLogArray[currentRow][12] = forwardsAcceleration; // acceleration in m/s^2

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
