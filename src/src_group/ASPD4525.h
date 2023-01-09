//#include <Wire.h>                              //I2C library 0x28H
#include <Arduino.h>
//byte fetch_pressure(unsigned int *p_P_dat); // convert value to byte data type
//#define TRUE 1
//#define FALSE 0
byte _status;
unsigned int P_dat;
float PR;
float V;
float VV;


//FASTER, OPTIMIZED CODE
byte fetch_airspeed(unsigned int *p_P_dat)
{
    byte address, Press_H, Press_L;
    unsigned int P_dat;

    address = 0x28;
    Wire.beginTransmission(address);
    Wire.requestFrom((int)address, (int)2); // Only request 2 bytes (Press_H and Press_L)
    Press_H = Wire.read();
    Press_L = Wire.read();
    Wire.endTransmission();

    _status = Press_H >> 6; // Use bit shifting instead of & operator
    Press_H = Press_H & 0x3f;
    P_dat = (((unsigned int)Press_H) << 8) | Press_L;
    *p_P_dat = P_dat;
    PR = (double)((P_dat - 819.15) / (14744.7));
    PR = (PR - 0.49060678);
    PR = abs(PR);
    V = ((PR * 13789.5144) / 1.225);
    VV = (sqrt((V)));
    return (VV);
}


/*

//ORIGGINAL CODE
byte fetch_airspeed(unsigned int *p_P_dat)
{
    byte address, Press_H, Press_L;
    unsigned int P_dat;

    address = 0x28;
    Wire.beginTransmission(address);
    Wire.requestFrom((int)address, (int)4); // Request 4 bytes need 4 bytes are read
    Press_H = Wire.read();
    Press_L = Wire.read();
    Wire.endTransmission();

    _status = (Press_H >> 6) & 0x03;
    Press_H = Press_H & 0x3f;
    P_dat = (((unsigned int)Press_H) << 8) | Press_L;
    *p_P_dat = P_dat;
    PR = (double)((P_dat - 819.15) / (14744.7));
    PR = (PR - 0.49060678);
    PR = abs(PR);
    V = ((PR * 13789.5144) / 1.225);
    VV = (sqrt((V)));
    return (VV);
}
*/