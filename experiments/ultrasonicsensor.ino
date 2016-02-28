#include "Ultrasonic.h"

#define serial Serial 

Ultrasonic ultrasonic(12,13);

void setup() {
serial.begin(9600);
}

void loop()
{
  serial.println(ultrasonic.Ranging(CM));   
  delay(100);
}

