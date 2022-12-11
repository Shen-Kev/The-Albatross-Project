#include "BMP180/Adafruit_BMP085.h"

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
float pressure;
float pressure_prev; 
float pressure_LP_param = 0.2;
float pressureMeasured;

float altitude_offset;
int altitude_offset_num = 10;
float altitude_offset_sum = 0;
float altitude;
float altitude_prev;
float altitude_LP_param = 0.2;
float altitudeMeasured;

void BMP180setup() {
  if (!bmp.begin()) {
	while (1) {}
  }
 for (size_t i = 0; i < altitude_offset_num; i++)
 {
    altitude_offset_sum+= bmp.readAltitude();
 }
 
  altitude_offset = altitude_offset_sum/((float)altitude_offset_num);
  //calibrate to assume that startup is at sea level
}
  
void BMP180loop() {
    pressureMeasured = bmp.readPressure();
    pressure = (1.0 - pressure_LP_param) * pressure_prev + pressure_LP_param * pressureMeasured;
    pressure_prev = pressure;
   
    altitudeMeasured = bmp.readAltitude()-altitude_offset;
    altitude = (1.0 - altitude_LP_param) * altitude_prev + altitude_LP_param * altitudeMeasured;
    altitude_prev = altitude;
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
 //   Serial.print("Altitude = ");
 //   Serial.print(bmp.readAltitude());
  //  Serial.print(" meters ");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  
  //101320 WORKS WELL
   // Serial.print("Real altitude = ");
    //Serial.print(bmp.readAltitude(101320));
//Serial.println(" meters");
}