//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 6
  Line Tracking Robot
  http://www.keyestudio.com
*/

#include "MecanumCar_v2.h"
mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2

/*******Define the pin of Line Tracking Sensor**********/
#define SensorLeft    A0   //input pin of left sensor
#define SensorMiddle  A1   //input pin of middle sensor
#define SensorRight   A2   //input pin of right sensor


void setup() {
  /****Set all the interface of the line tracking sensor to input mode***/
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);
  mecanumCar.Init(); //Initialize the seven-color leds and motor drive
}

void loop() {
  uint8_t SL = digitalRead(SensorLeft);   //Read the value of the left line tracking sensor
  uint8_t SM = digitalRead(SensorMiddle); //Read the value of the middle line tracking sensor
  uint8_t SR = digitalRead(SensorRight);  //Read the value of the right line tracking sensor
  if (SM == HIGH) {
    if (SL == LOW && SR == HIGH) {  // black on right, white on left, turn right
      mecanumCar.Turn_Right();
    }
    else if (SR == LOW && SL == HIGH) {  // black on left, white on right, turn left
      mecanumCar.Turn_Left();
    }
    else {  // white on both sides, going forward
      mecanumCar.Advance();
    }
  }
  else {
    if (SL == LOW && SR == HIGH) { // black on right, white on left, turn right
      mecanumCar.Turn_Right();
    }
    else if (SR == LOW && SL == HIGH) {  // white on right, black on left, turn left
      mecanumCar.Turn_Left();
    }
    else { // all white, stop
      mecanumCar.Stop();
    }
  }
}
//*************************************************************************
