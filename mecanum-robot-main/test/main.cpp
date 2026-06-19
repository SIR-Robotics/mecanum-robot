#include <Arduino.h>
#include <MecanumCar_v2.h>
#include "ir.h"

IR IRreceive(A3);//IR receiver is connected to A3


mecanumCar robot(3, 2);  // SDA=D3, SCL=D2

// ── Calibration ───────────────────────────────────────────────────────────────
#define DEFAULT_SPEED  150   // 0–255
#define TURN_90_MS     750   // ms to spin ~90° at DEFAULT_SPEED
// ─────────────────────────────────────────────────────────────────────────────

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
  uint16_t duration;  // ms
};

// ── Route ─────────────────────────────────────────────────────────────────────
Step route[] = {
  { FWD,      DEFAULT_SPEED, 2000         },  // straight 2s
  { STRAFE_R, DEFAULT_SPEED, 10000        },  // strafe right 10s
  { BWD,      DEFAULT_SPEED, 2000         },  // reverse 2s
  { DRIFT_R,  DEFAULT_SPEED, TURN_90_MS*2 },  // drift 180°
  { STRAFE_L, DEFAULT_SPEED, 10000        },  // strafe left 10s
  { DRIFT_R,  DEFAULT_SPEED, 3000         },  // drift right 3s
  { DRIFT_L,  DEFAULT_SPEED, 3000         },  // drift left 3s
  { PAUSE,    0,             500          },
};
// ─────────────────────────────────────────────────────────────────────────────

void setSpeed(uint8_t spd) {
  speed_Upper_L = speed_Lower_L = spd;
  speed_Upper_R = speed_Lower_R = spd;
}

void executeStep(const Step& s) {
  setSpeed(s.speed);
  switch (s.move) {
    case FWD:        robot.Advance();    break;
    case BWD:        robot.Back();       break;
    case STRAFE_L:   robot.L_Move();     break;
    case STRAFE_R:   robot.R_Move();     break;
    case TURN_L:     robot.Turn_Left();  break;
    case TURN_R:     robot.Turn_Right(); break;
    case DRIFT_L:    robot.drift_left(); break;
    case DRIFT_R:    robot.drift_right();break;
    case DIAG_FWD_L: robot.LU_Move();   break;
    case DIAG_FWD_R: robot.RU_Move();   break;
    case DIAG_BWD_L: robot.LD_Move();   break;
    case DIAG_BWD_R: robot.RD_Move();   break;
    case PAUSE:      robot.Stop();       break;
  }
  delay(s.duration);
  robot.Stop();
  delay(100);
}

void setup() {
  robot.Init();
   Serial.begin(9600); //Set baud rate to 9600
  delay(2000);
}

void loop() {
    int key = IRreceive.getKey();
    if (key != -1) {
    Serial.println(key);
    }

    if (key == 64) {

        const uint8_t steps = sizeof(route) / sizeof(route[0]);
        for (uint8_t i = 0; i < steps; i++) {
          executeStep(route[i]);
        }
    }
  // while (true);
}