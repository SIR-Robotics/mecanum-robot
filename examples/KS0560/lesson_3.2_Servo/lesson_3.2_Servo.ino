//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 3.2
  Servo
  http://www.keyestudio.com
*/
#include <Servo.h>
Servo myservo;    //Define a servo instance

void setup() {
  myservo.attach(9);    //The servo pin is connected to D9
}

void loop() {
  for (uint8_t angle = 0; angle < 180; angle++)//From 0 to 180 degrees
  {
    myservo.write(angle); //Rotate to angle
    delay(15);  //Wait for a while
  }
  for (uint8_t angle = 180; angle > 0; angle--)//From 180 to 0 degrees
  {
    myservo.write(angle); //Rotate to angle
    delay(15);
  }
}
//*************************************************************************
