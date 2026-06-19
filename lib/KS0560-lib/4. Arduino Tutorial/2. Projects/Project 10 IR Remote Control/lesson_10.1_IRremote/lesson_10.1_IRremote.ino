//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 10.1
  IRremote
  http://www.keyestudio.com
*/

#include "ir.h"

IR IRreceive(A3);//IR receiver is connected to A3

void setup(){
  Serial.begin(9600); //Set baud rate to 9600
}

void loop() {
  int key = IRreceive.getKey();
  if (key != -1) {
    Serial.println(key);
  }
}
//*************************************************************************
