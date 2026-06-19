//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 8
  Ultrasonic follow Robot
  http://www.keyestudio.com
*/
#include "MecanumCar_v2.h"
#include "Servo.h"

mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2
Servo myservo;    //Define an instance of a servo

/*******Ultrasonic sensor interface*****/
#define EchoPin  13  //ECHO to D13
#define TrigPin  12  //TRIG to D12

void setup() {
  Serial.begin(9600); //Start the serial monitor and set the baud rate to 9600
  pinMode(EchoPin, INPUT);    //The ECHO pin is set to input mode
  pinMode(TrigPin, OUTPUT);   //The TRIG pin is set to output mode
  myservo.attach(9);  // attaches the servo on D9 to the servo object
  myservo.write(90); //Rotate to 90 degrees
  delay(300);//Delay in 0.3s
  mecanumCar.Init(); //Initialize the seven-color LED and the motor drive

}

void loop() {
  int distance = get_distance();  //Get the distance and save in the distance variable  
  Serial.println(distance);
  if (distance <= 15)  //Receding range
  {
    mecanumCar.Back();
  }
  else if (distance <= 25)  //Stop range
  {
    mecanumCar.Stop();
  }
  else if (distance <= 45) //Range of advance
  {
    mecanumCar.Advance();
  }
  else  //Other conditions stop
  {
    mecanumCar.Stop();
  }
}


/*********************Ultrasonic detects the distance*******************************/
int get_distance(void) {    //Ultrasonic detects the distance
  int dis;
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH); //Give the TRIG a high level of at least 10 µ s to trigger
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  dis = pulseIn(EchoPin, HIGH) / 58.2; //Work out the distance
  delay(30);
  return dis;
}
//*************************************************************************
