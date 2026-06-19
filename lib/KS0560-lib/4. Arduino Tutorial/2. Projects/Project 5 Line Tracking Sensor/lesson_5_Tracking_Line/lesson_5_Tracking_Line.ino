//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 5
  Line Tracking Sensor
  http://www.keyestudio.com
*/
/*******Define the pin of Line Tracking Sensor**********/
#define SensorLeft    A0   //input pin of left sensor
#define SensorMiddle  A1   //input pin of middle sensor
#define SensorRight   A2   //input pin of right sensor


void setup() {
  Serial.begin(9600); //Start the serial monitor and set the baud rate to 9600
  /****All Line Tracking Sensor interface are set to input mode***/
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);
}

void loop() {
  uint8_t SL = digitalRead(SensorLeft); //Read the value of the left line tracking sensor
  uint8_t SM = digitalRead(SensorMiddle); //Read the value of the intermediate line tracking sensor
  uint8_t SR = digitalRead(SensorRight); //Read the value of the right line tracking sensor
  Serial.print("Left:");  //Serial port prints the information  
  Serial.print(SL);
  Serial.print("  Middle:");
  Serial.print(SM);
  Serial.print("  Right:");
  Serial.print(SR);
  Serial.println(); //Line feed
  delay(100);
}
//*************************************************************************
