#include <Arduino.h>

const int TRIGGER_PIN = 34; // Trigger pin of the ultrasonic sensor
const int ECHO_PIN = 35;    // Echo pin of the ultrasonic sensor

volatile unsigned long pulseStartTime; // Time when the ultrasonic pulse is transmitted
volatile unsigned long pulseEndTime;   // Time when the ultrasonic pulse is received
volatile boolean pulseInProgress;      // Flag to indicate whether a pulse is in progress

const unsigned long MEASURE_INTERVAL = 50; // Time between sensor readings (in milliseconds)

unsigned long lastMeasureTime = 0; // Time when the last sensor reading was taken
long distance = 0; // Variable to store the distance measured by the sensor
void pulseInISR();

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(9600);

  // Initialize the ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the pulse variables
  pulseStartTime = 0;
  pulseEndTime = 0;
  pulseInProgress = false;

  // Attach an interrupt to the ECHO_PIN to detect the start and end of the ultrasonic pulse
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), pulseInISR, CHANGE);
}

void loop() {
  // Check if it's time to take a sensor reading
  if (millis() - lastMeasureTime >= MEASURE_INTERVAL) {
    // Reset the last measurement time
    lastMeasureTime = millis();

    // Trigger a new sensor reading
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Wait for the distance to be calculated by the ISR, but not longer than 50 microseconds
    unsigned long waitStartTime = micros();
    while (pulseInProgress && (micros() - waitStartTime <= 50)) {
      // Do nothing while the ultrasonic pulse is in progress or the time limit hasn't been reached
    }

    // If the time limit has been reached and the pulse is still in progress, return -1
    if (pulseInProgress) {
      distance = -1;
    } else {
      // Calculate the distance in centimeters
      distance = (pulseEndTime - pulseStartTime) / 58;
    }
    

    // Print the distance to the serial port
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Other code can go here, as this is a non-blocking implementation
}

// Interrupt Service Routine to handle the start and end of the ultrasonic pulse
void pulseInISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    // The ultrasonic pulse is starting
    pulseStartTime = micros();
    pulseInProgress = true;
  } else {
    // The ultrasonic pulse is ending
    pulseEndTime = micros();
    pulseInProgress = false;
  }
}
