//WORKS as of 12/4/22
#include <Arduino.h>

#include <SPI.h>
#include <SD.h>

int i = 0;

File myFile;

void setup()
{
    SD.begin(BUILTIN_SDCARD);
}

void loop()
{
    myFile = SD.open("test.txt", FILE_WRITE);
    myFile.println(i);
    myFile.close();
    i++;
    delay(1000);
}