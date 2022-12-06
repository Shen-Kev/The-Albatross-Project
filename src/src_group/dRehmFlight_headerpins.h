// dRhemFlight Code- edited down to basics needed for the Albatross Project. All credit for this code goes to Nichlosas Rhem, and this code will be kept separate from other code in this project.

// Uncomment only one receiver type
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX

// Uncomment only one IMU
#define USE_MPU6050_I2C // Default
//#define USE_MPU9250_SPI

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

//========================================================================================================================//

// REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer

#if defined USE_SBUS_RX
#include "src/SBUS/SBUS.h" //sBus interface
#endif

#if defined USE_DSM_RX
#include "src/DSMRX/DSMRX.h"
#endif

#if defined USE_MPU6050_I2C
#include "MPU6050/MPU6050.h"
MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
#include "src/MPU9250/MPU9250.h"
MPU9250 mpu9250(SPI2, 36);
#else
#error No MPU defined...
#endif

//========================================================================================================================//

// Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

// Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; // thro
unsigned long channel_2_fs = 1500; // ail
unsigned long channel_3_fs = 1500; // elev
unsigned long channel_4_fs = 1500; // rudd
unsigned long channel_5_fs = 2000; // gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; // aux1

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04; // Madgwick filter parameter
float B_accel = 0.14;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;      // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;       // Magnetometer LP filter parameter

// Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0;
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

// IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

// Controller parameters (take note of defaults before modifying!):
float i_limit = 25.0;  // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;  // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 30.0; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;  // Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;   // Roll P-gain - angle mode
float Ki_roll_angle = 0.3;   // Roll I-gain - angle mode
float Kd_roll_angle = 0.05;  // Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;     // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;  // Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;  // Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05; // Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;    // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    // Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     // Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   // Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    // Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;     // Yaw P-gain
float Ki_yaw = 0.05;    // Yaw I-gain
float Kd_yaw = 0.00015; // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
// Radio:
// Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
const int ch1Pin = 2; // throttle
const int ch2Pin = 3; // ail
const int ch3Pin = 4; // ele
const int ch4Pin = 5; // rudd
const int ch5Pin = 6; // gear (throttle cut)
const int ch6Pin = 7; // aux1 (free aux channel)
const int PPM_Pin = 23;
//OneShot125 ESC pin outputs: NOT USED
const int m1Pin = 100;
const int m2Pin = 1100;
const int m3Pin = 2100;
const int m4Pin = 3100;
const int m5Pin = 4100;
const int m6Pin = 5100;
// PWM servo or ESC outputs:
const int servo1Pin = 14;
const int servo2Pin = 15;
const int servo3Pin = 18;
const int servo4Pin = 19;
const int servo5Pin = 22;
const int servo6Pin = 23;
const int servo7Pin = 29;
PWMServo servo1; // Create servo objects to control a servo or ESC with PWM
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

//========================================================================================================================//

// DECLARE GLOBAL VARIABLES

// General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

// Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

// PWM stuff
unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

#if defined USE_SBUS_RX
SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
DSM1024 DSM;
#endif

// IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; // Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//function prototypes https://forum.arduino.cc/t/functions-at-the-end-or-beggining/530377/5

void controlMixer();
void IMUinit();
void getIMUdata();
void calculate_IMU_error();
void calibrateAltitude();
void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
void getDesState();
void controlANGLE();
void controlANGLE2();
void controlRATE();
void scaleCommands();
void getCommands();
void failSafe();
void commandMotors();
void armMotors();
void calibrateESCs();
float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq);
void switchRollYaw(int reverseRoll, int reverseYaw);
void throttleCut();
void calibrateMagnetometer();
void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks, int upTime, int downTime);
void printRadioData();
void printDesiredState();
void printGyroData();
void printAccelData();
void printMagData();
void printRollPitchYaw();
void printPIDoutput();
void printMotorCommands();
void printServoCommands();
void printLoopRate();
float invSqrt(float x);
void radioSetup();
unsigned long getRadioPWM(int ch_num);
void getCh1();
void getCh2();
void getCh3();
void getCh4();
void getCh5();
void getCh6();

