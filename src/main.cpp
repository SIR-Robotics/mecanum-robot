#include <Arduino.h>
#include <MecanumCar_v2.h>
#include "ir.h"

mecanumCar robot(3, 2);
IR IRreceive(A3);

const uint8_t LINE_LEFT_PIN   = A0;
const uint8_t LINE_MIDDLE_PIN = A1;
const uint8_t LINE_RIGHT_PIN  = A2;

const uint8_t  CORRECTION_SPEED     = 60;
const uint16_t LINE_LOG_INTERVAL_MS = 200;
const uint8_t  TARGET_LINES         = 5;
const uint8_t  LEFT_SPEED           = 33;
const uint8_t  RIGHT_SPEED          = 35;
const uint8_t  NUDGE_SPEED          = 35;
const uint16_t NUDGE_DURATION_MS    = 50;
const uint16_t CROSS_DRIVE_MS       = 800;
const uint16_t CROSS_PAUSE_MS       = 200;

unsigned long lastLineLogMs   = 0;
unsigned long crossingStartMs = 0;
int           numLines        = 0;
bool          isRunning       = false;
bool          wasOnFullLine   = false;
bool          isCrossing      = false;

// ─────────────────────────────────────────────────────────────────────────────

void driveForward(uint8_t leftSpd, uint8_t rightSpd) {
  robot.Motor_Upper_L(1, leftSpd);
  robot.Motor_Lower_L(1, leftSpd);
  robot.Motor_Upper_R(1, rightSpd);
  robot.Motor_Lower_R(1, rightSpd);
}

void turnRight90() {
 Serial.println("Turning right 90 degrees...");
  robot.Advance();
  delay(300);   // Move forward a bit before turning  
  
  robot.Motor_Upper_L(1, CORRECTION_SPEED);
  robot.Motor_Lower_L(1, CORRECTION_SPEED);
  robot.Motor_Upper_R(0, CORRECTION_SPEED);
  robot.Motor_Lower_R(0, CORRECTION_SPEED);
  delay(500);   // Tune this until it hits ~90 degrees on your surface

//     while (digitalRead(LINE_MIDDLE_PIN) != 1) {
//     // Keep turning, waiting for middle sensor to find the line

//   }
//   delay(100);   // Small delay to ensure the robot is centered on the line
  robot.Stop();
  robot.L_Move();
  delay(400);   // Move forward a bit to ensure it's centered on the line
  robot.Stop();
}

void turnLeft() {
  robot.Motor_Upper_L(0, CORRECTION_SPEED);
  robot.Motor_Lower_L(0, CORRECTION_SPEED);
  robot.Motor_Upper_R(1, CORRECTION_SPEED);
  robot.Motor_Lower_R(1, CORRECTION_SPEED);
}

void turnRight() {
  robot.Motor_Upper_L(1, CORRECTION_SPEED);
  robot.Motor_Lower_L(1, CORRECTION_SPEED);
  robot.Motor_Upper_R(0, CORRECTION_SPEED);
  robot.Motor_Lower_R(0, CORRECTION_SPEED);
}

void logLineSensors(uint8_t sl, uint8_t sm, uint8_t sr) {
  unsigned long now = millis();
  if (now - lastLineLogMs < LINE_LOG_INTERVAL_MS) return;
  Serial.print("L:"); Serial.print(sl);
  Serial.print(" M:"); Serial.print(sm);
  Serial.print(" R:"); Serial.println(sr);
  lastLineLogMs = now;
}

// ─────────────────────────────────────────────────────────────────────────────

void handleLineTracking() {
  uint8_t sl = digitalRead(LINE_LEFT_PIN);
  uint8_t sm = digitalRead(LINE_MIDDLE_PIN);
  uint8_t sr = digitalRead(LINE_RIGHT_PIN);

  logLineSensors(sl, sm, sr);

  bool allBlack = (sl == 1 && sm == 1 && sr == 1);
  bool allWhite = (sl == 0 && sm == 0 && sr == 0);

  // ── Non-blocking crossing drive-through ───────────────────────────────────
  if (isCrossing) {
    if (millis() - crossingStartMs < CROSS_DRIVE_MS) {
      driveForward(LEFT_SPEED, RIGHT_SPEED);
    } else if (millis() - crossingStartMs < CROSS_DRIVE_MS + CROSS_PAUSE_MS) {
      robot.Stop();
    } else {
      isCrossing = false;
    }
    return;
  }

  // ── Full line crossing ────────────────────────────────────────────────────
  if (allBlack) {
    if (!wasOnFullLine) {
      numLines++;
      wasOnFullLine = true;

      Serial.print("Full line crossed! Count: ");
      Serial.println(numLines);

            if (numLines >= TARGET_LINES) {
        robot.Stop();
        delay(200);         // Brief pause before turning
        turnRight90();      // 90 degree turn
        isRunning = false;
        robot.right_led(true);
        robot.left_led(true);
        Serial.println("Target reached. Robot stopped.");
        return;
        }

      isCrossing      = true;
      crossingStartMs = millis();
      driveForward(LEFT_SPEED, RIGHT_SPEED);
    }
    return;

  // ── Lost line → nudge forward, bail early if line reacquired ─────────────
  } else if (allWhite) {
    wasOnFullLine = false;
    Serial.println("Line lost. Nudging forward...");
    unsigned long nudgeStart = millis();
    while (millis() - nudgeStart < NUDGE_DURATION_MS) {
      driveForward(NUDGE_SPEED, NUDGE_SPEED);
      if (digitalRead(LINE_MIDDLE_PIN) == 1) break;
    }
    robot.Stop();

  // ── Normal tracking with turning correction ───────────────────────────────
  } else {
    wasOnFullLine = false;

    if      (sm == 1 && sl == 0 && sr == 0) { driveForward(LEFT_SPEED, RIGHT_SPEED); }  // Mid only → straight
    else if (sl == 1 && sm == 0 && sr == 0) { turnLeft();                             }  // Left only → turn left
    else if (sr == 1 && sm == 0 && sl == 0) { turnRight();                            }  // Right only → turn right
    else if (sm == 1 && sl == 1)            { turnLeft();                             }  // Mid + left → turn left
    else if (sm == 1 && sr == 1)            { turnRight();                            }  // Mid + right → turn right
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  pinMode(LINE_LEFT_PIN,   INPUT);
  pinMode(LINE_MIDDLE_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN,  INPUT);

  Serial.begin(9600);
  robot.Init();
  delay(1000);
  Serial.println("Ready. Press IR to start.");
}

void loop() {
  int key = IRreceive.getKey();

  if (key != -1 && !isRunning) {
    numLines      = 0;
    wasOnFullLine = false;
    isCrossing    = false;
    isRunning     = true;
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Program started!");
  }

  if (isRunning) handleLineTracking();
}