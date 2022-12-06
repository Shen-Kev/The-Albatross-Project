#include "src_group/dRehmFlight_headerpins.h"
#include "src_group/dRehmFlight_functions.h"

void dRehmFlightSetup()
{
  Serial.begin(500000); // USB serial
  delay(500);

  // Initialize all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); // Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  // Initialize radio communication
  radioSetup();

  // Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  // Initialize IMU communication
  IMUinit();

  delay(5);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  // calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  // Arm servo channels
  servo1.write(0); // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); // Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); // Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);

  delay(5);

  // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  // Code will not proceed past here if this function is uncommented!

  // Arm OneShot125 motors
  m1_command_PWM = 125; // Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  armMotors(); // Loop over commandMotors() until ESCs happily arm

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

  // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  // calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section
}

void dRehmFlightLoop()
{
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  // printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  // printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  // printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  // printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  // printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  // printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  // printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  // printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  // printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  // printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  // Get vehicle state
  getIMUdata();                                                              // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  // Compute desired state
  getDesState(); // Convert raw commands to normalized values based on saturated control limits

  // PID Controller - SELECT ONE:
  controlANGLE(); // Stabilize on angle setpoint
  // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  // controlRATE(); //Stabilize on rate setpoint

  // Actuator mixing and scaling to PWM values
  controlMixer();  // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  // Throttle cut check
  throttleCut(); // Directly sets motor commands to low based on state of ch5

  // Command actuators
  commandMotors();              // Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); // Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);

  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}