//*************************************************************************
/*
  Keyestudio 4WD Mecanum Robot for Arduino
  lesson 12.1
  APP Test
  http://www.keyestudio.com
*/

void setup() {
  Serial.begin(9600);//Start the serial monitor and set the baud rate to 9600
}

void loop() {
  if (Serial.available())//If the receiving area is not empty
  {
    char ble_val = Serial.read();//Read the data of the bluetooth
    Serial.println(ble_val);//The serial monitor prints the read data
  }
  }
}
//*************************************************************************
