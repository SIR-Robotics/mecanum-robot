#include "MecanumCar_v2.h"

mecanumCar robot(3, 2);  // SDA, SCL pins

void setup() {
  robot.Init();
}

void loop() {
  // Strafe left
  robot.L_Move();
  delay(1000);
  
  // Strafe right
  robot.R_Move();
  delay(1000);
  
  // Stop
  robot.Stop();
  delay(500);
}
