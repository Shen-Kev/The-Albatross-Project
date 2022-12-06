//WORKS AS OF 12/4/22

//responsible for writing all the data 

#include <SPI.h>
#include <SD.h>

File myFile;

void writeSD(int i); //number, just if needed or for testing or maybe an error code idk
void logData(int a, float b, double c, char d, String e); //can add as many parameters as needed for flight data

void setupSD()
{
  SD.begin(BUILTIN_SDCARD);
  // add some warning system or something if it doesn't work 
}

void logData(int a, float b, double c, char d, String e)
{
  myFile = SD.open("test.txt", FILE_WRITE);
  
  myFile.print("this is the integer: ");
  myFile.print(a);
  myFile.print("\t");
  
  myFile.print("this is the float: ");
  myFile.print(b);
  myFile.print("\t");
  
  myFile.print("this is the double: ");
  myFile.print(c);
  myFile.print("\t");
  
  myFile.print("this is the char: ");
  myFile.print(d);
  myFile.print("\t");
  
  myFile.print("this is the String: ");
  myFile.print(e);
  myFile.print("\t");

  myFile.print("\n");

  myFile.close();
}

void writeSD(int i)
{
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println(i);
  myFile.close();
}
