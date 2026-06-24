#include <Arduino.h>
#include <MecanumCar_v2.h>
#include "ir.h"

mecanumCar robot(3, 2);
IR IRreceive(A3);

const uint8_t LINE_LEFT_PIN   = A0;
const uint8_t LINE_MIDDLE_PIN = A1;
const uint8_t LINE_RIGHT_PIN  = A2;

const uint8_t  CORRECTION_SPEED     = 65;
const uint8_t  TARGET_LINES         = 4;
const uint8_t  LEFT_SPEED           = 45;
const uint8_t  RIGHT_SPEED          = 47;
const uint8_t  NUDGE_SPEED          = 50;
const uint16_t NUDGE_DURATION_MS    = 30;
const uint16_t HUNT_DURATION_MS     = 60;
const uint16_t CROSS_DRIVE_MS       = 800;
const uint16_t CROSS_PAUSE_MS       = 200;

unsigned long crossingStartMs = 0;
int           numLines        = 0;
uint8_t       targetLines     = TARGET_LINES;
bool          isRunning       = false;
bool          wasOnFullLine   = false;
bool          isCrossing      = false;

// ─────────────────────────────────────────────────────────────────────────────

void turnRight90() {
  Serial.println("Turning right 90 degrees...");
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = CORRECTION_SPEED;
  robot.Advance();
  delay(300);
  robot.Turn_Right();
  delay(500);
  robot.Stop();
  robot.L_Move();
  delay(400);
  robot.Stop();
}

void strafeLeft(int ms) {
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = CORRECTION_SPEED;
  robot.L_Move();
  delay(ms);
  robot.Stop();
}

// ─────────────────────────────────────────────────────────────────────────────

// ── Pure line-follow movement. Handles: allWhite hunt, normal tracking. ──────
// Caller passes pre-read sensor values from handleLineTracking.

void followLine(uint8_t sl, uint8_t sm, uint8_t sr) {
  bool allWhite = (sl == 0 && sm == 0 && sr == 0);

  // ── Lost line → nudge + hunt (inlined sensor re-check) ──────────────────
  if (allWhite) {
    wasOnFullLine = false;

    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = NUDGE_SPEED;

    // Phase 1: nudge forward with inline re-check
    unsigned long start = millis();
    while (millis() - start < NUDGE_DURATION_MS) {
      robot.Advance();
      if (digitalRead(LINE_MIDDLE_PIN) == 1) { robot.Stop(); return; }
    }

    // Phase 2: hunt left
    start = millis();
    while (millis() - start < HUNT_DURATION_MS) {
      robot.Turn_Left();
      if (digitalRead(LINE_MIDDLE_PIN) == 1) { robot.Stop(); return; }
    }

    // Phase 3: hunt right
    start = millis();
    while (millis() - start < HUNT_DURATION_MS) {
      robot.Turn_Right();
      if (digitalRead(LINE_MIDDLE_PIN) == 1) { robot.Stop(); return; }
    }

    robot.Stop();
    return;
  }

  // ── Normal tracking ─────────────────────────────────────────────────────
  wasOnFullLine = false;

  if (sm == 1 && sl == 0 && sr == 0) {
    speed_Upper_L = speed_Lower_L = LEFT_SPEED;
    speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
    robot.Advance();
  } else {
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = CORRECTION_SPEED;
    if (sl == 1 || (sm == 1 && sl == 1)) {
      robot.Turn_Left();
    } else {
      robot.Turn_Right();
    }
  }
}

// ── Non-blocking line-follow with crossing counting (called from loop) ──────

void handleLineTracking() {
  uint8_t sl = digitalRead(LINE_LEFT_PIN);
  uint8_t sm = digitalRead(LINE_MIDDLE_PIN);
  uint8_t sr = digitalRead(LINE_RIGHT_PIN);

  bool allBlack = (sl == 1 && sm == 1 && sr == 1);

  // ── Non-blocking crossing drive-through ─────────────────────────────────
  if (isCrossing) {
    if (millis() - crossingStartMs < CROSS_DRIVE_MS) {
      speed_Upper_L = speed_Lower_L = LEFT_SPEED;
      speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
      robot.Advance();
    } else if (millis() - crossingStartMs < CROSS_DRIVE_MS + CROSS_PAUSE_MS) {
      robot.Stop();
    } else {
      isCrossing = false;
    }
    return;
  }

  // ── Full line crossing ──────────────────────────────────────────────────
  if (allBlack) {
    if (!wasOnFullLine) {
      numLines++;
      wasOnFullLine = true;

      Serial.print("Full line crossed! Count: ");
      Serial.println(numLines);

      if (numLines >= targetLines) {
        robot.Stop();
        delay(200);
        turnRight90();
        isRunning = false;
        robot.right_led(true);
        robot.left_led(true);
        Serial.println("Target reached. Robot stopped.");
        return;
      }

      isCrossing      = true;
      crossingStartMs = millis();
      speed_Upper_L = speed_Lower_L = LEFT_SPEED;
      speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
      robot.Advance();
    }
    return;
  }

  // ── Delegate movement to pure line-follow ───────────────────────────────
  followLine(sl, sm, sr);
}

// ── Blocking line-follow with parameterized target ───────────────────────────

void followLineWithTarget(int targetCount) {
  numLines      = 0;
  wasOnFullLine = false;
  isCrossing    = false;

  while (numLines < targetCount) {
    uint8_t sl = digitalRead(LINE_LEFT_PIN);
    uint8_t sm = digitalRead(LINE_MIDDLE_PIN);
    uint8_t sr = digitalRead(LINE_RIGHT_PIN);

    bool allBlack = (sl == 1 && sm == 1 && sr == 1);

    // ── Non-blocking crossing drive-through ───────────────────────────────
    if (isCrossing) {
      if (millis() - crossingStartMs < CROSS_DRIVE_MS) {
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      } else if (millis() - crossingStartMs < CROSS_DRIVE_MS + CROSS_PAUSE_MS) {
        robot.Stop();
      } else {
        isCrossing = false;
      }
      continue;
    }

    // ── Full line crossing ────────────────────────────────────────────────
    if (allBlack) {
      if (!wasOnFullLine) {
        numLines++;
        wasOnFullLine = true;

        Serial.print("Line crossed: ");
        Serial.print(numLines);
        Serial.print("/");
        Serial.println(targetCount);

        if (numLines >= targetCount) {
          robot.Stop();
          Serial.println("Target reached.");
          return;
        }

        isCrossing      = true;
        crossingStartMs = millis();
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      }
      continue;
    }

    // ── Delegate movement to pure line-follow ─────────────────────────────
    followLine(sl, sm, sr);
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void rotate180() {
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = CORRECTION_SPEED;
  robot.Turn_Right();
  delay(1000);
  robot.Stop();
}

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

  // challenge 1
  if (key == 22) {
    numLines      = 0;
    wasOnFullLine = false;
    isCrossing    = false;
    isRunning     = true;
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Program started!");

    while (isRunning) handleLineTracking();

    robot.right_led(true);
    robot.left_led(true);
    Serial.println("Challenge 1 done.");
  }

  // challenge 2
  if (key == 25) {
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Following 2 lines...");
    followLineWithTarget(2);
    strafeLeft(1000);
    followLineWithTarget(4);
    rotate180();
    followLineWithTarget(3);
    robot.right_led(true);
    robot.left_led(true);
    Serial.println("Done.");
  }

  // stop everything
  if (key == 70) {
    robot.Stop();
    isRunning   = false;
    isCrossing  = false;
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Stopped.");
  }
}