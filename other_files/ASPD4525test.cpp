//WORKS AS OF 12/16/22
#include <Arduino.h>
#include <Wire.h>                              //I2C library 0x28H
byte fetch_pressure(unsigned int *p_P_dat); // convert value to byte data type

#define TRUE 1
#define FALSE 0
byte _status;
unsigned int P_dat;
float PR;
float V;
float VV;
float airspeed;
float airspeed_prev;
float airspeed_LP_param = 0.05;
float airspeed_zero=0; //the calibrated airspeed to be 0m/s at startup to account for different pressrues
float airspeed_adjusted; 
float airspeed_scalar = 1.8; // the scaled airspeed, tuned to be most accurate at about 10-15 m/s

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    for (int i = 0; i < 10; i++)
    {
        _status = fetch_pressure(&P_dat);
        PR = (double)((P_dat - 819.15) / (14744.7));
        PR = (PR - 0.49060678);
        PR = abs(PR);
        V = ((PR * 13789.5144) / 1.225);
        airspeed_zero += (sqrt((V)));
    }
    airspeed_zero = airspeed_zero/10.0;
}

void loop()
{

    _status = fetch_pressure(&P_dat);
    /*
        switch (_status)
        {
          case 0: Serial.println("Ok ");
            break;
          case 1: Serial.println("Busy");
            break;
          case 2: Serial.println("Slate");
            break;
          default: Serial.println("Error");
            break;
        }

    */
    PR = (double)((P_dat - 819.15) / (14744.7));
    PR = (PR - 0.49060678);
    PR = abs(PR);
    V = ((PR * 13789.5144) / 1.225);
    VV = (sqrt((V)));
    airspeed = (1.0 - airspeed_LP_param) * airspeed_prev + airspeed_LP_param * VV;
    airspeed_prev = airspeed;
    airspeed_adjusted = (airspeed-airspeed_zero)*airspeed_scalar;
    //   TR = (double)((T_dat*0.09770395701));
    //    TR = TR-50;

    // Serial.print("raw Pressure:");
    //  Serial.println(P_dat);
    // Serial.println(P_dat,DEC);
    // Serial.println(P_dat,BIN);
    //  Serial.print("pressure psi:");
    //  Serial.println(PR,10);
    //  Serial.print(" ");
    // Serial.print("raw Temp:");
    // Serial.println(T_dat);
    // Serial.print("temp:");
    // Serial.println(TR);
      //Serial.print(" raw measurement: ");
     //Serial.print(VV);
    //Serial.print(" raw measurement filtered and zeroed: ");
    //Serial.print(airspeed);
    Serial.print(" airspeed adjusted m/s: ");
    Serial.print(airspeed_adjusted);
    Serial.print(" airspeed adjusted mph: ");
    Serial.print(airspeed_adjusted * 2.23694);
    Serial.println();

    delay(10);
}

byte fetch_pressure(unsigned int *p_P_dat)
{
    byte address, Press_H, Press_L, _status;
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

    return (_status);
}