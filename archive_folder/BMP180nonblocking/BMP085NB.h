#ifndef BMP085NB_h
#define BMP085NB_h

#include <Arduino.h>

#define I2C_ADDRESS 0x77
#define OSS 0x03
#define SEA_LEVEL_PRESSURE 101325

class BMP085NB {
	public:
		void initialize();
		void pollData(int *temperature, long *pressure, float *alti);
		void setSeaLevelPressure(long);
		short Temperature(unsigned int);
		float Altitude(long);
		long Pressure(unsigned long);
		void StartUT();
		void StartUP();
		unsigned int ReadUT();
		unsigned long ReadUP();		
		void writeRegister(unsigned char, unsigned char);
		int readIntRegister(unsigned char);
		boolean newData;
	private:		
		long slp;
		int ac1;
		int ac2; 
		int ac3; 
		unsigned int ac4;
		unsigned int ac5;
		unsigned int ac6;
		int b1; 
		int b2;
		int mb;
		int mc;
		int md;
		unsigned long timer;
		unsigned long timer1;
		int pressureState;
		long x1;
		long x2;
		long x3;
		long b3;
		long b5;
		long b6;
		long p;
		unsigned long b4;
		unsigned long b7;
		unsigned int ut;
		unsigned long up;
};

#endif