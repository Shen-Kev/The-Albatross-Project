#include <Wire.h>
#include "BMP180nonblocking/BMP085NB.h"

BMP085NB bmp;

int temperature = 0;
long pressure = 0;
float alti = 0;
unsigned long timer = 0;

// new vars
const int altitude_offset_num_vals = 10;
float altitudeMeasured; // The raw altitude reading from the barometric pressure sensor (in m)
float altitude_offset;
float altitude_baro;            // The altitude estimated from the barometer, with a low pass filter and offset adjustment applied/ float altitude_prev;                     // The previous reading of the barometric pressure sensor
float altitude_LP_param = 0.1; // The low pass filter parameter for altitude (smaller values means a more smooth signal but higher delay time)
float altitude_prev;

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

void setup()
{
    Serial.begin(500000); // USB serial
    Wire.begin();
    Wire.setClock(1000000);
    BMP180setup();
}

void loop()
{
  //  timer = micros();
    BMP180loop();
  //  Serial.print("alt: ");
    Serial.println(altitude_baro);
    delay(10);
    // Serial.print(" ");
    // Serial.print(micros() - timer);
    // Serial.println("micros"); // even tho it occasionally takes 8000 microsd thats ok
}