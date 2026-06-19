#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <Servo.h>
#include "ir.h"

IR IRreceive(A3);
mecanumCar robot(3, 2);  // SDA=D3, SCL=D2
Servo myServo;

#define SERVO_PIN      9
#define DEFAULT_SPEED  150
#define TURN_90_MS     750

enum Move {
  FWD, BWD,
  STRAFE_L, STRAFE_R,
  TURN_L, TURN_R,
  DRIFT_L, DRIFT_R,
  DIAG_FWD_L, DIAG_FWD_R,
  DIAG_BWD_L, DIAG_BWD_R,
  PAUSE
};

struct Step {
  Move     move;
  uint8_t  speed;
  uint16_t duration;
};

Step route[] = {
  { FWD,      DEFAULT_SPEED, 2000         },
  { STRAFE_R, DEFAULT_SPEED, 10000        },
  { BWD,      DEFAULT_SPEED, 2000         },
  { DRIFT_R,  DEFAULT_SPEED, TURN_90_MS*2 },
  { STRAFE_L, DEFAULT_SPEED, 10000        },
  { DRIFT_R,  DEFAULT_SPEED, 3000         },
  { DRIFT_L,  DEFAULT_SPEED, 3000         },
  { PAUSE,    0,             500          },
};

const uint8_t TOTAL_STEPS = sizeof(route) / sizeof(route[0]);

bool     running   = false;
uint8_t  stepIndex = 0;
uint32_t stepStart = 0;

const int servoAngles[] = {0, 180};
uint8_t   servoPos       = 0;

void setSpeed(uint8_t spd) {
  speed_Upper_L = speed_Lower_L = spd;
  speed_Upper_R = speed_Lower_R = spd;
}

void startStep(uint8_t i) {
  setSpeed(route[i].speed);
  switch (route[i].move) {
    case FWD:        robot.Advance();     break;
    case BWD:        robot.Back();        break;
    case STRAFE_L:   robot.L_Move();      break;
    case STRAFE_R:   robot.R_Move();      break;
    case TURN_L:     robot.Turn_Left();   break;
    case TURN_R:     robot.Turn_Right();  break;
    case DRIFT_L:    robot.drift_left();  break;
    case DRIFT_R:    robot.drift_right(); break;
    case DIAG_FWD_L: robot.LU_Move();    break;
    case DIAG_FWD_R: robot.RU_Move();    break;
    case DIAG_BWD_L: robot.LD_Move();    break;
    case DIAG_BWD_R: robot.RD_Move();    break;
    case PAUSE:      robot.Stop();        break;
  }
  stepStart = millis();
}

void setup() {
  Serial.begin(9600);
  robot.Init();
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngles[0]);
  delay(2000);
}

void loop() {
  int key = IRreceive.getKey();
  if (key != -1) {
    Serial.println(key);

    // OK button (64): start route if idle, stop if running
    if (key == 64) {
      if (!running) {
        running   = true;
        stepIndex = 0;
        startStep(0);
      } else {
        running = false;
        robot.Stop();
      }
    }

    // Button 1 (22): cycle servo 0deg -> 90deg -> 180deg -> 0deg
    if (key == 22) {
      servoPos = (servoPos + 1) % 3;
      myServo.write(servoAngles[servoPos]);
    }
  }

  // Non-blocking route advancement
  if (running && millis() - stepStart >= route[stepIndex].duration) {
    robot.Stop();
    stepIndex++;
    if (stepIndex >= TOTAL_STEPS) {
      running = false;
    } else {
      delay(100);
      startStep(stepIndex);
    }
  }
}
