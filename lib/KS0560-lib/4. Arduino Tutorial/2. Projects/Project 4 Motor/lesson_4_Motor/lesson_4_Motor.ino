//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 4
  Motor
  http://www.keyestudio.com
*/

#include "MecanumCar_v2.h"
mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2

void setup() {
  mecanumCar.Init();  //Initialize the drive
}

void loop() {
  mecanumCar.Advance();
  delay(2000);            //Wait for 2s
  
  mecanumCar.Back();      //Move back
  delay(2000);
  
  mecanumCar.Turn_Left(); //Turn left
  delay(2000);

  mecanumCar.Turn_Right();//Turn right
  delay(2000);

  mecanumCar.Stop();      //Stop
  delay(1000);
}
