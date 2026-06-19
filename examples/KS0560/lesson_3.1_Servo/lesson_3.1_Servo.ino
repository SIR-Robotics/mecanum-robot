//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 3.1
  Servo
  http://www.keyestudio.com
*/
#define  servo_pin  9      //The servo is connected to D9

void setup() {
  pinMode(servo_pin, OUTPUT);    //Set the servo pin mode to output
}
void loop() {
  for (uint8_t angle = 0; angle < 180; angle++)
  {
    servopulse(servo_pin, angle);
  }
  for (uint8_t angle = 180; angle > 0; angle--)
  {
    servopulse(servo_pin, angle);
  }
}

void servopulse(int pin, int myangle) {   //Impulse function 
  int pulsewidth = map(myangle, 0, 180, 500, 2500); //Map angle to pulse width
  //Output pulse
  digitalWrite(pin, HIGH);        //Set the servo interface to high level 
  delayMicroseconds(pulsewidth);  //Delay the number of ms of pulse width value
  digitalWrite(pin, LOW);         //Set the servo interface to low level 
  delay(20 - pulsewidth / 1000);  //Cycle is 20 ms
}
//*************************************************************************
