//*******************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 1
  Color LED
  http://www.keyestudio.com
*/
#include "MecanumCar_v2.h"
mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2

void setup() {
  mecanumCar.Init();//Initialize the motors and the seven-color LEDs
}

void loop() {
  mecanumCar.right_led(1);//Turn on the right seven-color LED
  mecanumCar.left_led(1); //Turn on the left seven-color LED
  delay(3000);            //Delay in 3000 ms
  mecanumCar.right_led(0); //Turn off the right seven-color LED
  mecanumCar.left_led(0);  //Turn off the left seven-color LED
  delay(1000);             //Delay in 1000 ms
}
//*******************************************************************
