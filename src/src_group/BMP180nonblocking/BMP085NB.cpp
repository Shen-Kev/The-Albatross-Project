#include "BMP085NB.h"
#include <Wire.h>

void BMP085NB::pollData(int *temperature, long *pressure, float *alti)
{
  *temperature = 0; // TEMPERATURE ALWAYS AT 0, going to get calibrated anyways

  switch (pressureState)
  {
  case 0:
    pressureState = 1;
    newData = false;
    timer = millis();
    StartUT();
    break;
  case 1:
    if (millis() - timer >= 5)
    {
      pressureState = 2;
      ut = ReadUT();
      *temperature = 0; // TEMPERATURE ALWAYS AT 0, going to get calibrated anyways
    }
    break;
  case 2:
    timer1 = millis();
    StartUP();
    pressureState = 3;
    break;
  case 3:
    if (millis() - timer1 >= 2 + (3 << OSS))
    {
      up = ReadUP();
      *pressure = Pressure(up);
      *alti = Altitude(*pressure);
      newData = true;
      pressureState = 0;
    }
    break;
  default:
    break;
  }
}

void BMP085NB::setSeaLevelPressure(long seaLevelPressure)
{
  slp = seaLevelPressure;
}

short BMP085NB::Temperature(unsigned int ut)
{
  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

long BMP085NB::Pressure(unsigned long up)
{

  // calculate true pressure
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

float BMP085NB::Altitude(long pres)
{
  return (float)44330 * (1 - pow(((float)pres / slp), 0.190295));
}

void BMP085NB::StartUT()
{
  writeRegister(0xf4, 0x2e);
}

void BMP085NB::StartUP()
{
  writeRegister(0xf4, 0x34 + (OSS << 6));
}

unsigned int BMP085NB::ReadUT()
{
  return readIntRegister(0xf6);
}

unsigned long BMP085NB::ReadUP()
{
  unsigned char msb, lsb, xlsb;

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0xf6); // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 3); // request three bytes
  while (Wire.available() < 3)
    ; // wait until data available
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  return (((unsigned long)msb << 16) | ((unsigned long)lsb << 8) | (unsigned long)xlsb) >> (8 - OSS);
}

void BMP085NB::initialize()
{
  pressureState = 0;
  newData = false;
  slp = SEA_LEVEL_PRESSURE;

  ac1 = readIntRegister(0xAA);
  ac2 = readIntRegister(0xAC);
  ac3 = readIntRegister(0xAE);
  ac4 = readIntRegister(0xB0);
  ac5 = readIntRegister(0xB2);
  ac6 = readIntRegister(0xB4);
  b1 = readIntRegister(0xB6);
  b2 = readIntRegister(0xB8);
  mb = readIntRegister(0xBA);
  mc = readIntRegister(0xBC);
  md = readIntRegister(0xBE);
}

void BMP085NB::writeRegister(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

int BMP085NB::readIntRegister(unsigned char r)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); // register to read
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS, 2); // request two bytes
  while (!Wire.available())
    ; // wait until data available
  msb = Wire.read();
  while (!Wire.available())
    ; // wait until data available
  lsb = Wire.read();
  return (((int)msb << 8) | ((int)lsb));
}