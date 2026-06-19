//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 7
  Ultrasonic sensor
  http://www.keyestudio.com
*/

/*******Ultrasonic Sensor interface*****/
#define EchoPin  13  //ECHO to D13
#define TrigPin  12  //TRIG to D12

void setup() {
  Serial.begin(9600); //Set baud rate to 9600
  pinMode(EchoPin, INPUT);    //The ECHO pin is set to input mode
  pinMode(TrigPin, OUTPUT);   //The TRIG pin is set to output mode
}

void loop() {
  float distance = Get_Distance();  //Get the distance and save in the distance variable
  Serial.print("ditance:");
  Serial.print(distance);
  Serial.println("cm");
  delay(100);
}

float Get_Distance(void) {    //Ultrasonic detects the distance
  float dis;
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH); //Give the TRIG a high level of at least 10 µ s to trigger
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  dis = pulseIn(EchoPin, HIGH) /58.2;  //Work out the distance
  delay(50);
  return dis;
}
//*************************************************************************
