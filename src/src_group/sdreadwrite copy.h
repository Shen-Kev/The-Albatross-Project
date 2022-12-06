//WORKS AS OF 12/4/22
#include <SPI.h>
#include <SD.h>

File myFile;

void writeSD(int i);
void setupSD();

void setupSD()
{
  SD.begin(BUILTIN_SDCARD);

}

void writeSD(int i)
{
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println(i);
  myFile.close();
}