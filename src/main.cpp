// ==========================================================================
//  HARDWARE, PINS, CONSTANTS & GLOBAL STATE
// ==========================================================================

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include "ir.h"
#include <Servo.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

mecanumCar robot(3, 2);
IR IRreceive(A3);
Servo servo;
SoftwareSerial espSerial(10, 11); // Uno RX, TX

const uint8_t LINE_LEFT_PIN   = A0;
const uint8_t LINE_MIDDLE_PIN = A1;
const uint8_t LINE_RIGHT_PIN  = A2;

// Color Sensor TCS3200 (S0=GND, S1=5V, scaling=2%)
const uint8_t CS_S2  = 7;
const uint8_t CS_S3  = 6;
const uint8_t CS_OUT = 8;

// Ultrasonic Sensor Pin
const uint8_t ECHO_PIN = 13;
const uint8_t TRIG_PIN = 12;

// Servo Pin
const uint8_t SERVO_PIN = 9;

// Speeds are raw PWM, 0-255 (see MecanumCar_v2.cpp: "val为写入的pwm值0~255").
// The library's own default is 80. Below roughly 30 the motors sit in their stall
// band, where they don't all break free from a standstill and small PWM
// differences turn into large speed differences — keep every speed here well
// clear of that floor.
const uint8_t  DRIVE_SPEED   = 41;   // forward speed while following a line
const uint8_t  TURN_SPEED    = 50;   // in-place spin speed (rotate90 / searching)
const uint8_t  TURN_SPEED_180 = 40;  // 180 spins twice as far and builds twice the momentum, so it coasts past the line at full TURN_SPEED — spin it slower
const uint8_t  SLOW_SPEED    = 35;   // final approach to a block / drop

// Instructor's line follower adds 10 PWM while correcting. Tune this if the
// robot turns too sharply or too slowly while recovering the middle sensor.
const uint8_t  LINE_CORRECT_BOOST = 13;

const uint8_t  TICK_MS       = 5;    // line-follow loop tick
const uint8_t  TURN_TICK_MS  = 12;   // turn / positioning loop tick
const uint16_t ROTATE_SENSOR_GRACE_MS = 200;  // ignore the sensors this long after a spin starts
const uint16_t BRAKE_MS = 40;                 // counter-pulse to cancel spin momentum — shorter now that the robot moves slower and carries less of it (too long = kicks back the other way)

const uint16_t OBSTACLE_DISTANCE_CM = 7;
const uint16_t APPROACH_DISTANCE_CM = 30;  // switch to the slow creep this far out — raise to start slowing earlier
const uint16_t ULTRASONIC_SAMPLE_MS = 50;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;
const uint8_t  ULTRASONIC_CONFIRM_READS = 3;
const uint8_t  ULTRASONIC_STOP_HYSTERESIS_CM = 2;

const uint8_t  GRIPPER_OPEN_ANGLE   = 2;
const uint8_t  GRIPPER_CLOSE_ANGLE  = 180;
const uint8_t  GRIPPER_STEP_DELAY_MS = 10;



// alignOnCrossLine: after stopping on a crossing marker, momentum only ever
// carries the robot forward past it, so back up gently until all 3 sensors
// read it solidly again before dropping the object.
const uint8_t  ALIGN_CONFIRM_TICKS = 5;   // consecutive all-on-line ticks required
const uint16_t ALIGN_TIMEOUT_MS    = 1000;

bool isGripperOpen = true; // true = open, false = closed
uint8_t gripperAngle = GRIPPER_OPEN_ANGLE;

// Color sensor calibration (white=255, black=0)
int whiteR = 949, whiteG = 967, whiteB = 881;
int blackR = 5109, blackG = 4806, blackB = 4189;

int           numLines        = 0;
bool          wasOnFullLine   = false;
bool          stopAll         = false;
unsigned long statusLedOffAtMs = 0;
int           pendingEspKey   = -1;

const uint8_t  STOP_KEY             = 64;

// ==========================================================================
//  DATA TYPES
// ==========================================================================

struct RGB { uint8_t r, g, b; };

enum class ColorLabel { Red, Blue, Yellow };

// ==========================================================================
//  FORWARD DECLARATIONS
// ==========================================================================

// Forward declarations — every function is declared here so the definitions
// below can be grouped by topic regardless of call order. Default arguments
// live only in these declarations.
int        readRawColor(bool s2, bool s3);
RGB        readColor();
ColorLabel classifyColor();

void       stopEverything();
bool       stopRequested();
bool       waitOrStop(uint16_t ms);

void       logEsp32Messages();
void       handleEsp32Line(String line);
void       actionLog(const char* message);
void       actionLog(const __FlashStringHelper* message);
void       sendArmCommand(int colorRes);

void       brakePulse(void (mecanumCar::*counterMove)());
bool       moveShort(uint16_t durationMs = 300);
bool       moveForwardWheelSpeeds(uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint16_t durationMs = 3000);
bool       reverseShort(uint16_t durationMs = 300);

bool       rotate90(uint8_t turnSpeed = TURN_SPEED, uint16_t timeoutMs = 3000);
bool       rotate90Left(uint8_t turnSpeed = TURN_SPEED, uint16_t timeoutMs = 3000);
bool       rotate180(uint8_t turnSpeed = TURN_SPEED_180, uint16_t timeoutMs = 5000);

void       followLineWithTarget(int targetCount);
void       moveSlowly(int targetCount);

uint16_t   readUltrasonic();
uint16_t   readUltrasonicMedian();

void       followLineWithDistance();
void       moveSlowlyToObject();

bool       searchAndCenterLine(uint16_t timeoutMs = 1500, int8_t initialLastSeenSide = 0);
bool       alignOnCrossLine(uint16_t timeoutMs = ALIGN_TIMEOUT_MS);

void       openGripper(bool state);
int        gripAndIdentifyColor(bool gOpen);

bool       returnToCheckpoint();

void       path1();
void       path2();
void       path3();
void       runCommandKey(int key, const char* source);

// ==========================================================================
//  COLOR SENSOR (TCS3200)
// ==========================================================================

// Read raw pulse width (µs) for given photodiode filter
int readRawColor(bool s2, bool s3) {
  digitalWrite(CS_S2, s2);
  digitalWrite(CS_S3, s3);
  delay(10);  // Let filter settle
  return pulseIn(CS_OUT, LOW, 100000);
}

// Read normalized RGB (0-255) using calibration values
RGB readColor() {
  int rawR = readRawColor(LOW,  LOW);
  int rawG = readRawColor(HIGH, HIGH);
  int rawB = readRawColor(LOW,  HIGH);

  RGB c;
  c.r = constrain(map(rawR, blackR, whiteR, 0, 255), 0, 255);
  c.g = constrain(map(rawG, blackG, whiteG, 0, 255), 0, 255);
  c.b = constrain(map(rawB, blackB, whiteB, 0, 255), 0, 255);

  Serial.print(F("R: ")); Serial.print(c.r);
  Serial.print(F(" G: ")); Serial.print(c.g);
  Serial.print(F(" B: ")); Serial.println(c.b);
  return c;
}

ColorLabel classifyColor() {
  RGB color = readColor(); // already logs R/G/B internally

  if (color.b > color.r && color.b > color.g) return ColorLabel::Blue;
  if (color.r > color.g && color.r > color.b) return ColorLabel::Red;
  return ColorLabel::Yellow;
}

// ==========================================================================
//  SYSTEM CONTROL / STOP
// ==========================================================================

void stopEverything() {
  robot.Stop();
  stopAll = true;
  robot.right_led(false);
  robot.left_led(false);
  Serial.println(F("Stopped."));
}

bool stopRequested() {
  if (stopAll) return true;
  logEsp32Messages();
  if (pendingEspKey == STOP_KEY || pendingEspKey == 70) {
    pendingEspKey = -1;
    actionLog("force stop");
    stopEverything();
    return true;
  }
  if (IRreceive.getKey() == STOP_KEY) {
    stopEverything();
    return true;
  }
  return false;
}

bool waitOrStop(uint16_t ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (stopRequested()) return true;
    delay(1);
  }
  return false;
}

// ==========================================================================
//  ESP32 WIFI + ROBOT-ARM COMMS
// ==========================================================================

void handleEsp32Line(String line) {
  line.trim();
  if (line.startsWith("IR_KEY")) {
    pendingEspKey = line.substring(6).toInt();
    Serial.print(F("ESP32 command key: "));
    Serial.println(pendingEspKey);
    return;
  }

  // Connection status arrives whenever the ESP32 gets around to sending it —
  // setup() no longer waits for it. Flash the LEDs to acknowledge; loop() turns
  // them back off once statusLedOffAtMs passes.
  if (line.startsWith("WIFI_CONNECTED") || line.startsWith("MQTT_CONNECTED")) {
    robot.right_led(true);
    robot.left_led(true);
    statusLedOffAtMs = millis() + 2000;
  }

  Serial.print(F("ESP32: "));
  Serial.println(line);
}

void logEsp32Messages() {
  static String line;

  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) handleEsp32Line(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}

void actionLog(const char* message) {
  Serial.print(F("action: "));
  Serial.println(message);
  espSerial.listen();
  espSerial.print(F("ACTION_LOG "));
  espSerial.println(message);
  espSerial.flush();
}

void actionLog(const __FlashStringHelper* message) {
  Serial.print(F("action: "));
  Serial.println(message);
  espSerial.listen();
  espSerial.print(F("ACTION_LOG "));
  espSerial.println(message);
  espSerial.flush();
}

void sendArmCommand(int colorRes) {
  const char* command = nullptr;
  if (colorRes == 0) command = "RUN_BLUE";
  else if (colorRes == 1) command = "RUN_RED";
  else if (colorRes == 2) command = "RUN_YELLOW";
  if (!command) return;

  while (espSerial.available()) espSerial.read();
  Serial.println(command);
  espSerial.listen();
  espSerial.println(command);
  espSerial.flush();
}

// ==========================================================================
//  MOTION PRIMITIVES
// ==========================================================================

// Counter-pulse brake: briefly drive the opposite direction at the speed already
// set by the caller to cancel momentum, then coast-stop. No encoders, so the pulse
// can't be computed from actual speed — BRAKE_MS is hand-tuned. `counterMove` is the
// motor primitive that spins/strafes opposite to the motion being braked.
void brakePulse(void (mecanumCar::*counterMove)()) {
  Serial.print(F("Brake pulse "));
  Serial.print(BRAKE_MS);
  Serial.println(F("ms"));
  (robot.*counterMove)();
  delay(BRAKE_MS);
  robot.Stop();
}

bool moveShort(uint16_t durationMs) {
  Serial.println(F("Moving short distance..."));

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = DRIVE_SPEED;
  robot.Advance();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;
  }

  robot.Stop();
  return true;
}

// Drive forward with explicit per-wheel speeds — lets the remote's wheel-test
// button try values on the fly without re-flashing.
bool moveForwardWheelSpeeds(uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint16_t durationMs) {
  Serial.print(F("Wheel speeds — UL:")); Serial.print(upperL);
  Serial.print(F(" LL:")); Serial.print(lowerL);
  Serial.print(F(" UR:")); Serial.print(upperR);
  Serial.print(F(" LR:")); Serial.println(lowerR);

  speed_Upper_L = upperL;
  speed_Lower_L = lowerL;
  speed_Upper_R = upperR;
  speed_Lower_R = lowerR;
  robot.Advance();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;
  }

  robot.Stop();
  return true;
}

bool reverseShort(uint16_t durationMs) {
  Serial.println(F("Reversing short distance..."));

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = DRIVE_SPEED;
  robot.Back();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;   // stop was requested mid-reverse
  }

  robot.Stop();
  return true;
}

// ==========================================================================
//  ROTATION (90°)
// ==========================================================================

bool rotate90(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println(F("Rotating 90 degrees until right sensor detects line..."));

  unsigned long startMs = millis();

  speed_Upper_L = speed_Lower_L = speed_Lower_R = turnSpeed;
  speed_Upper_R = turnSpeed; // slight trim for asymmetry

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    if (millis() - startMs >= ROTATE_SENSOR_GRACE_MS && SR == HIGH) {
      brakePulse(&mecanumCar::Turn_Left);   // cancel spin momentum
      Serial.println(F("Right sensor detected line."));
      return true;
    }

    robot.Turn_Right();
    delay(TURN_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println(F("Rotate 90 timeout. Right sensor did not detect line."));
  return false;
}

// Single continuous 180° spin — the alternative to calling rotate90() twice.
// Same shape as rotate90 (spin right, grace period, brake, return), but counts
// right-sensor line crossings and stops on the 2nd: the 1st crossing is the ~90°
// line, the 2nd is the ~180° line. Timeout is larger since the spin is twice as
// long. Spins slower than a 90 by default — a half-turn builds up twice the
// momentum, so at full TURN_SPEED it tends to coast straight past the line.
bool rotate180(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println(F("Rotating 180 degrees until 2nd right-line crossing..."));

  unsigned long startMs = millis();

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;

  uint8_t crossings = 0;
  bool onLine = false;   // true while the right sensor is currently over a line

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    if (millis() - startMs >= ROTATE_SENSOR_GRACE_MS) {
      if (SR == HIGH && !onLine) {        // rising edge → a new line crossing
        onLine = true;
        crossings++;
        Serial.print(F("Crossing "));
        Serial.println(crossings);
        if (crossings >= 2) {
          brakePulse(&mecanumCar::Turn_Left);   // cancel spin momentum
          Serial.println(F("180 reached."));
          return true;
        }
      } else if (SR == LOW) {             // left the line; ready for the next one
        onLine = false;
      }
    }

    robot.Turn_Right();
    delay(TURN_TICK_MS);
  }
  robot.Stop();
  Serial.println(F("Rotate 180 timeout. 2nd crossing not detected."));
  return false;
}

bool rotate90Left(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println(F("Rotating 90 degrees until left sensor detects line..."));

  unsigned long startMs = millis();

  speed_Upper_L = turnSpeed; // slight trim for asymmetry
  speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);

    if (millis() - startMs >= ROTATE_SENSOR_GRACE_MS && SL == HIGH) {
      brakePulse(&mecanumCar::Turn_Right);   // cancel spin momentum before centering
      Serial.println(F("Left sensor detected line."));
      return searchAndCenterLine(1500, -1);
    }

    robot.Turn_Left();
    delay(TURN_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println(F("Rotate 90 timeout. Left sensor did not detect line."));
  return false;
}

// ==========================================================================
//  LINE FOLLOWING
// ==========================================================================
//
// Instructor's 3-sensor pattern: an outer sensor lighting up alone means turn
// toward it; equal outer readings mean continue. The caller reads the middle
// sensor too when detecting full-width intersections.

// One steering step. HIGH = sensor is over black.
static void steerLine(uint8_t speed) {
  uint8_t L = digitalRead(LINE_LEFT_PIN);
  uint8_t R = digitalRead(LINE_RIGHT_PIN);
  uint8_t correctionSpeed = speed + LINE_CORRECT_BOOST;

  if (L == LOW && R == HIGH) {          // 001 or 011: line is to the right
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = correctionSpeed;
    robot.Turn_Right();
  } else if (L == HIGH && R == LOW) {   // 100 or 110: line is to the left
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = correctionSpeed;
    robot.Turn_Left();
  } else {                              // 000, 010, 101 or 111: continue forward
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = speed;
    robot.Advance();
  }
}

// Follow the line until targetCount crossings have gone by, then stop.
// A crossing is all three sensors on black at once, counted once per line.
static void followLine(int targetCount, uint8_t speed) {
  numLines      = 0;
  wasOnFullLine = false;

  Serial.print(F("Line follow. Target: "));
  Serial.println(targetCount);

  while (numLines < targetCount) {
    if (stopRequested()) return;

    bool onCrossLine = digitalRead(LINE_LEFT_PIN)   == HIGH &&
                       digitalRead(LINE_MIDDLE_PIN) == HIGH &&
                       digitalRead(LINE_RIGHT_PIN)  == HIGH;

    if (onCrossLine && !wasOnFullLine) {
      numLines++;
      Serial.print(F("Line crossed: "));
      Serial.print(numLines);
      Serial.print(F("/"));
      Serial.println(targetCount);

      if (numLines >= targetCount) {
        robot.Stop();
        Serial.println(F("Target reached."));
        return;
      }
    }
    wasOnFullLine = onCrossLine;

    steerLine(speed);
    delay(TICK_MS);
  }
}

void followLineWithTarget(int targetCount) { followLine(targetCount, DRIVE_SPEED); }
void moveSlowly(int targetCount)           { followLine(targetCount, SLOW_SPEED); }

// ==========================================================================
//  ULTRASONIC SENSOR
// ==========================================================================

uint16_t readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return 999;
  return duration / 58.2;
}

uint16_t readUltrasonicMedian() {
  uint16_t a = readUltrasonic();
  delay(3);
  uint16_t b = readUltrasonic();
  delay(3);
  uint16_t c = readUltrasonic();

  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// ==========================================================================
//  DRIVE UNTIL OBSTACLE
// ==========================================================================

// Follow the line until the ultrasonic confirms something within stopDistanceCm.
static bool driveUntilObstacle(uint16_t stopDistanceCm, uint8_t speed) {
  Serial.println(F("Driving until obstacle..."));

  unsigned long lastDistanceSampleMs = millis() - ULTRASONIC_SAMPLE_MS;
  uint8_t closeReadCount = 0;

  while (true) {
    if (stopRequested()) return false;

    unsigned long now = millis();
    if (now - lastDistanceSampleMs >= ULTRASONIC_SAMPLE_MS) {
      lastDistanceSampleMs = now;
      uint16_t distanceCm = readUltrasonicMedian();

      if (distanceCm <= stopDistanceCm) closeReadCount++;
      else if (distanceCm > stopDistanceCm + ULTRASONIC_STOP_HYSTERESIS_CM) closeReadCount = 0;

      if (closeReadCount >= ULTRASONIC_CONFIRM_READS) {
        robot.Stop();
        Serial.print(F("Obstacle at "));
        Serial.print(distanceCm);
        Serial.println(F(" cm. Stopping."));
        return true;
      }
    }

    steerLine(speed);
    delay(TICK_MS);
  }
}

// Approach an object: drive up to APPROACH_DISTANCE_CM, pause, then creep in.
void followLineWithDistance() {
  if (driveUntilObstacle(APPROACH_DISTANCE_CM, DRIVE_SPEED)) {
    delay(1000);
    moveSlowlyToObject();
  }
}

void moveSlowlyToObject() {
  driveUntilObstacle(OBSTACLE_DISTANCE_CM, SLOW_SPEED);
}

// ==========================================================================
//  LINE SEARCH & CENTERING
// ==========================================================================

// Rotate until the middle sensor alone is on the line. Used after a turn to
// square up before driving off. Spin toward whichever side sees the line; if
// nothing sees it, keep spinning the way we were last told.
bool searchAndCenterLine(uint16_t timeoutMs, int8_t initialLastSeenSide) {
  Serial.println(F("Centering on line..."));

  unsigned long startMs = millis();
  uint8_t centeredTicks = 0;
  int8_t  lastSeenSide  = (initialLastSeenSide < 0) ? -1 : +1;

  while (timeoutMs == 0 || millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    bool L = digitalRead(LINE_LEFT_PIN)   == HIGH;
    bool M = digitalRead(LINE_MIDDLE_PIN) == HIGH;
    bool R = digitalRead(LINE_RIGHT_PIN)  == HIGH;

    if (M && !L && !R) {                       // centred
      robot.Stop();
      if (++centeredTicks >= ALIGN_CONFIRM_TICKS) {
        Serial.println(F("Centered."));
        return true;
      }
      delay(TURN_TICK_MS);
      continue;
    }
    centeredTicks = 0;

    if (L && !R)      lastSeenSide = -1;
    else if (R && !L) lastSeenSide = +1;

    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TURN_SPEED;
    (lastSeenSide < 0) ? robot.Turn_Left() : robot.Turn_Right();
    delay(TURN_TICK_MS);
  }

  robot.Stop();
  Serial.println(F("Center timeout."));
  return false;
}

// After stopping on a crossing, motor coast always carries the robot slightly
// past it. Back up until all 3 sensors read the crossing again, squaring the
// robot up before a drop.
bool alignOnCrossLine(uint16_t timeoutMs) {
  Serial.println(F("Aligning on cross line..."));

  uint8_t confirmedTicks = 0;
  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    bool onCrossLine = digitalRead(LINE_LEFT_PIN)   == HIGH &&
                       digitalRead(LINE_MIDDLE_PIN) == HIGH &&
                       digitalRead(LINE_RIGHT_PIN)  == HIGH;

    if (onCrossLine) {
      robot.Stop();
      if (++confirmedTicks >= ALIGN_CONFIRM_TICKS) {
        Serial.println(F("Aligned on cross line."));
        return true;
      }
    } else {
      confirmedTicks = 0;
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = SLOW_SPEED;
      robot.Back();
    }

    delay(TICK_MS);
  }

  robot.Stop();
  Serial.println(F("Align timeout."));
  return false;
}

// ==========================================================================
//  GRIPPER
// ==========================================================================

void openGripper(bool state) {
  uint8_t targetAngle = state ? GRIPPER_CLOSE_ANGLE : GRIPPER_OPEN_ANGLE;

  while (gripperAngle != targetAngle) {
    if (stopRequested()) return;
    gripperAngle += gripperAngle < targetAngle ? 1 : -1;
    servo.write(gripperAngle);
    delay(GRIPPER_STEP_DELAY_MS);
  }

  Serial.println(state ? "Gripper: close" : "Gripper: open");
}

int gripAndIdentifyColor(bool gOpen) {
  if (stopRequested()) return -1;

  Serial.println(F("Checking TCS3200 color..."));
  ColorLabel label = classifyColor();

  openGripper(gOpen);
  delay(1000);

  if (label == ColorLabel::Blue) {
    Serial.println(F("Blue detected. Gripper closed at 180."));
    return 0;
  } else if (label == ColorLabel::Red) {
    Serial.println(F("Red detected. Gripper closed at 180."));
    return 1;
  } else {
    Serial.println(F("Yellow detected. Gripper closed at 180."));
    return 2;
  }
}

// ==========================================================================
//  NAVIGATION HELPERS
// ==========================================================================

// for path 1
bool returnToCheckpoint() {
  reverseShort(300);
  if (!rotate90()) return false;
  delay(500);
  reverseShort(200);
  if (!rotate90()) return false;
  return true;
}

// ==========================================================================
//  CHALLENGE PATHS
// ==========================================================================

// path1 — pick up the object straight ahead, ferry it to the drop lane,
// release it, signal the arm which colour it was, then return to the checkpoint.
void path1() {
  // ── Phase 1: approach the object, grip & identify its colour ──
  actionLog(F("challenge3: Path 1 - approaching object"));
  followLineWithDistance();
  if (stopAll) return;
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  delay(5000);
  isGripperOpen = !isGripperOpen;

  // ── Phase 2: turn around onto the drop lane ──
  actionLog(F("challenge3: Path 1 - turning to drop lane"));
  // Two ways to turn around. Run one, comment out the other.

  // METHOD A — two discrete 90° turns. Each stops on its own line crossing, so
  // the robot is never more than a quarter-turn of momentum from a known heading.
  // reverseShort(200);
  if (!rotate90()) return;
  delay(500);
  reverseShort(200);
  delay(500);
  if (!rotate90()) return;

  // METHOD B — single continuous 180° spin (currently active). Counts two
  // right-sensor crossings and stops on the 2nd. Spins at TURN_SPEED_180 (slower
  // than a 90) because a half-turn carries twice the momentum into the stop.

  // ── Phase 3: drive to the drop zone and release the object ──
  actionLog(F("challenge3: Path 1 - delivering object"));
  followLineWithTarget(5);
  delay(1000);
  moveSlowly(2);
  delay(1000);
  alignOnCrossLine();
  openGripper(isGripperOpen);
  delay(5000);
  isGripperOpen = !isGripperOpen;

  // ── Phase 4: signal the arm, then return to the checkpoint ──
  actionLog(F("challenge3: Path 1 - returning to checkpoint"));
  sendArmCommand(colorRes);
  if (!returnToCheckpoint()) return;

}

// path2 — LEFT-side run. Fetches the object from the left branch and brings
// it to the drop zone (mirror of path3; distances are hand-tuned per side).
void path2() {
  // ── Phase 1: turn onto the object lane (left, then in) ──
  actionLog(F("challenge3: Path 2 - entering left branch"));
  followLineWithTarget(3);
  delay(1000);
  moveShort(300);
  delay(500);
  if (!rotate90Left()) return;
  moveShort(300);
  delay(1000);
  followLineWithTarget(2);
  delay(500);
  moveShort(300);
  delay(500);
  if (!rotate90()) return;
  delay(500);

  // ── Phase 2: approach the object, grip & identify its colour ──
  actionLog(F("challenge3: Path 2 - approaching object"));
  followLineWithDistance();
  delay(500);
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  delay(500);
  delay(1000);
  isGripperOpen = !isGripperOpen;
  delay(1000);

  // ── Phase 3: back off and navigate to the drop lane ──
  actionLog(F("challenge3: Path 2 - navigating to drop lane"));
  reverseShort(100);
  delay(500);
  if (!rotate90()) return;
  delay(1000);
  followLineWithTarget(2);
  delay(500);
  moveShort(300);
  delay(500);
  if (!rotate90()) return;

  // ── Phase 4: final approach and release the object ──
  actionLog(F("challenge3: Path 2 - delivering object"));
  followLineWithTarget(5);
  delay(1000);
  moveSlowly(2);
  delay(1000);
  alignOnCrossLine();
  openGripper(isGripperOpen);
  delay(5000);
  isGripperOpen = !isGripperOpen;

  // ── Phase 5: return to the checkpoint ──
  actionLog(F("challenge3: Path 2 - returning to checkpoint"));
  if (!returnToCheckpoint()) return;
  if (!searchAndCenterLine()) return;
}

// path3 — RIGHT-side run: mirror of path2. Fetches the object from the right
// branch and brings it to the drop zone (hand-tuned distances differ from path2).
void path3() {
  // ── Phase 1: turn onto the object lane (right, then in) ──
  actionLog(F("challenge3: Path 3 - entering right branch"));
  followLineWithTarget(3);
  delay(500);
  moveShort(300);
  delay(500);
  if (!rotate90()) return;
  delay(500);
  followLineWithTarget(2);
  delay(500);
  moveShort(300);
  delay(500);
  if (!rotate90Left()) return;
  delay(500);

  // ── Phase 2: approach the object, grip & identify its colour ──
  actionLog(F("challenge3: Path 3 - approaching object"));
  followLineWithDistance();
  if (stopAll) return;

  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  delay(5000);
  isGripperOpen = !isGripperOpen;

  // ── Phase 3: back off and navigate to the drop lane ──
  actionLog(F("challenge3: Path 3 - navigating to drop lane"));
  reverseShort(150);
  delay(500);
  if (!rotate90Left()) return;
  delay(1000);
  followLineWithTarget(2);
  delay(500);
  moveShort(400);
  delay(500);
  if (!rotate90Left()) return;

  // ── Phase 4: final approach and release the object ──
  actionLog(F("challenge3: Path 3 - delivering object"));
  if (!searchAndCenterLine()) return;
  followLineWithTarget(4);
  delay(1000);
  moveSlowly(2);
  delay(1000);
  alignOnCrossLine();
  openGripper(isGripperOpen);
  delay(5000);
  isGripperOpen = !isGripperOpen;
  // ── Phase 5: return to the checkpoint ──
  actionLog(F("challenge3: Path 3 - returning to checkpoint"));
  if (!returnToCheckpoint()) return;
}

void runCommandKey(int key, const char* source) {
  if (key == -1) return;

  char logMessage[40];
  snprintf(logMessage, sizeof(logMessage), "%s key %d", source, key);
  actionLog(logMessage);

  // A fresh command key re-arms the robot after an emergency stop. The stop
  // buttons are excluded so they never clear their own flag.
  if (key != 70 && key != STOP_KEY) {
    stopAll = false;
  }

  switch (key) {
    case 13: { // challenge 3
      actionLog(F("challenge3: Starting Challenge 3"));
      path1();
      if (stopRequested() || waitOrStop(1000)) break;
      path2();
      if (stopRequested() || waitOrStop(1000)) break;
      path3();
      if (stopRequested()) break;
      robot.Stop();
      Serial.println(F("Done."));
      actionLog("Done.");
      break;
    }

    case 70: // front button / stop everything
      stopEverything();
      break;

    case 90: // number 6 — wheel test: drive forward at DRIVE_SPEED on all four
      moveForwardWheelSpeeds(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
      break;

    default:
      break;
  }
}

// ==========================================================================
//  ARDUINO ENTRY POINTS
// ==========================================================================

void setup() {
  pinMode(LINE_LEFT_PIN,   INPUT);
  pinMode(LINE_MIDDLE_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN,  INPUT);
  pinMode(CS_S2,  OUTPUT);
  pinMode(CS_S3,  OUTPUT);
  pinMode(CS_OUT, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  Serial.begin(9600);
  espSerial.begin(9600);
  robot.Init();
  servo.attach(SERVO_PIN);
  servo.write(gripperAngle);
  // The ESP32 is not waited for: it reports WIFI/MQTT status on its own schedule
  // and loop() picks it up via logEsp32Messages(). Blocking setup() on it just
  // delayed the robot becoming IR-responsive.
  Serial.println(F("Ready. Press IR to start."));
}

void loop() {
  logEsp32Messages();

  // init success
  if (statusLedOffAtMs > 0 && millis() >= statusLedOffAtMs) {
    robot.right_led(false);
    robot.left_led(false);
    statusLedOffAtMs = 0;
  }

  if (pendingEspKey >= 0) {
    int key = pendingEspKey;
    pendingEspKey = -1;
    runCommandKey(key, "favoriot");
  } else {
    runCommandKey(IRreceive.getKey(), "ir");
  }
}
