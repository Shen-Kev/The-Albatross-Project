//NOT WORKING

/************************************************************************

  Wireless Meteo Station

*************************************************************************
  Description: Wireless_Meteo_Station
  The station measures the temperature, the relative humidity and the wind speed
  and transmits the data with the help of a Bluetooth Low Energy module

  Material
  1. Arduino Due
  2. Pmod HYGRO (library needed)
  3. Pmod TMP3
  4. Pmod DPG1
  5. Pmod BLE

************************************************************************/

//used connections and user settings
#include <Arduino.h>

#define DPG1_CS 19          //pin used as CS with Pmod DPG1

#define BAUD_RATE 115200   //define the baud rate of serial communication (for debugging)

#define DPG1_OFFSET 100    //rough offset of the Pmod DPG1 in Pa

/*---------------------------------------------------------------------*/

//used libraries
#include <SPI.h>                //library for SPI communication
#include <Wire.h>               //library for I2C communication

/*---------------------------------------------------------------------*/

//function prototypes
void DPG1_init(void);                     //initializes the Pmod DPG1
float DPG1_read(void);                    //reads wind speed from the Pmod DPG1

/*---------------------------------------------------------------------*/

//initialization functions: setting up the device
void setup()
{
  Serial.begin(BAUD_RATE);            //start the serial communication
  Serial.flush();                     //discard data junk in buffer
  Serial.println("Initialization:");  //display a message

  DPG1_init();                                //initialize the Pmod DPG1
  Serial.println("\tPmod DPG1 initialized");  //display a message
}

/*---------------------------------------------------------------------*/

//main program loop
void loop()
{

  float WindSpeed = DPG1_read();           //read wind speed
  String wsp = "wind: ";                   //create string to store the message
  Serial.println(WindSpeed); //append decoded wind type
  wsp.concat("\n\n");                      //append endilnes

  delay(2000);  //wait 2 seconds until next measurement
}

/*---------------------------------------------------------------------*/
//initializes the Pmod DPG1
void DPG1_init(void)
{
  SPI.begin();                          //initialization of SPI port
  SPI.setDataMode(SPI_MODE0);           //configuration of SPI communication in mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16); //configuration of clock at 1MHz
  pinMode(DPG1_CS, OUTPUT);             //setting the chip select pin as output
  digitalWrite(DPG1_CS, HIGH);          //sending a HIGH signal (idle) on the chip select line

  return;
}

//reads wind speed from the Pmod DPG1
float DPG1_read(void)
{
  digitalWrite(DPG1_CS, LOW);               //activate the device
  delayMicroseconds(20);                    //wait 20us
  int measurement = SPI.transfer(0) << 8;   //read high byte and shift it left 8 spaces
  measurement |= SPI.transfer(0);           //append low byte
  delayMicroseconds(20);                    //wait 20us
  digitalWrite(DPG1_CS, HIGH);              //deactivate the device

  float difference = (measurement * 1.0 / 4096 - 0.08) / 0.09; //calculate pressure difference in kPa (check: https://reference.digilentinc.com/reference/pmod/pmoddpg1/reference-manual )
  difference *= 1000.0;                                        //convert kPa to Pa (base unit)
  difference -= DPG1_OFFSET;                                   //correct offset
  //Serial.println(difference);                                //display for debugging

  if (difference < 0)                                             //if the result is negative
  {
    difference *= (-1);                                           //invert its sign
  }
  float airDensity = 1.225;                                       //air density in kg/(m^3)
  float windSpeed = sqrt(2 * difference / airDensity);            //wind speed in m/s
  return (windSpeed * 3.6);                                       //convert m/s to km/h and return wind speed
}