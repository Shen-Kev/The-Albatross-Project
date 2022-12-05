//WORKS AS OF 12/4/22

#include <Arduino.h>

#include "sdreadwrite.h"

int i = 0; 

void setup()
{
    setupSD();
}

void loop()
{
    writeSD(i);
    i++;
    delay(1000);
}
