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

const uint8_t  TURNING_SPEED        = 54;
const uint8_t  LEFT_SPEED           = 35;   // advance speed, left side
const uint8_t  RIGHT_SPEED          = 35;   // advance speed, right side (trim for asymmetry)
const uint8_t  LINE_TURN_SPEED      = 25;   // spin speed for line-follow corrections
const uint8_t  SLOW_LEFT_SPEED      = 30;
const uint8_t  SLOW_RIGHT_SPEED     = 30;
const uint8_t  SLOW_LINE_TURN_SPEED = 28;
const uint8_t  LINE_TICK_MS         = 12;   // follow-loop tick delay
const uint16_t OBSTACLE_DISTANCE_CM = 7;
const uint16_t APPROACH_DISTANCE_CM = 20;
const uint16_t ULTRASONIC_SAMPLE_MS = 50;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;
const uint8_t  ULTRASONIC_CONFIRM_READS = 3;
const uint8_t  ULTRASONIC_STOP_HYSTERESIS_CM = 2;
const uint8_t  LINE_SEARCH_SPEED    = 40;   // aggressive turning when line is lost
const uint8_t  GRIPPER_OPEN_ANGLE   = 2;
const uint8_t  GRIPPER_CLOSE_ANGLE  = 180;
const uint8_t  GRIPPER_STEP_DELAY_MS = 10;
const uint16_t REVERSE_BLIND_MS     = 500;
const uint16_t ROTATE_SENSOR_GRACE_MS = 200;
const uint16_t BRAKE_MS = 50;   // counter-pulse duration to cancel turn momentum — tune empirically (too long = kicks back the other way)

// Tuning constants — adjust to taste (for searchAndCenterLine)
const uint16_t SEARCH_SWEEP_INITIAL_MS = 300;   // first sweep half-width
const uint16_t SEARCH_SWEEP_GROWTH_MS  = 300;   // how much wider each sweep gets
const uint16_t SEARCH_SWEEP_MAX_MS     = 1500;  // cap on sweep width

bool isGripperOpen = true; // true = open, false = closed
uint8_t gripperAngle = GRIPPER_OPEN_ANGLE;

// Color sensor calibration (white=255, black=0)
int whiteR = 949, whiteG = 967, whiteB = 881;
int blackR = 5109, blackG = 4806, blackB = 4189;

int           numLines        = 0;
bool          wasOnFullLine   = false;
bool          stopAll         = false;
unsigned long statusLedOffAtMs = 0;

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
bool       connectEsp32Wifi();
bool       runArmCommand(const char* command, const char* okResponse, const char* failResponse);
bool       fetchRed();
bool       fetchBlue();
bool       fetchYellow();
void       sendArmCommand(int colorRes);

void       turnRight90();
void       brakePulse(void (mecanumCar::*counterMove)());
bool       moveShort(uint16_t durationMs = 300);
bool       reverseShort(uint16_t durationMs = 300);
bool       robotReverse(uint16_t timeoutMs = 2500, uint16_t reverseBlindMs = REVERSE_BLIND_MS);
void       strafeLeft(int targetCount);

bool       rotate90(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 3000);
bool       rotate90Left(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 3000);
bool       rotate180(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 5000);

void       steerAlongLine(uint8_t SL, uint8_t SM, uint8_t SR,
                          uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed,
                          int8_t &lastSeenSide);
void       followLineWithTarget(int targetCount, uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed);
void       followLineWithTarget(int targetCount);
bool       followLineForMs(uint16_t ms);
void       moveSlowly(int targetCount);

uint16_t   readUltrasonic();
uint16_t   readUltrasonicMedian();

bool       followLineWithDistance(uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed, uint16_t stopDistanceCm);
void       followLineWithDistance();
void       moveSlowlyToObject();

bool       searchAndCenterLine(uint16_t timeoutMs = 1500, int8_t initialLastSeenSide = 0);

void       openGripper(bool state);
int        gripAndIdentifyColor(bool gOpen);

bool       returnToCheckpoint();

void       path1();
void       path2();
void       path3();

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

  Serial.print("R: "); Serial.print(c.r);
  Serial.print(" G: "); Serial.print(c.g);
  Serial.print(" B: "); Serial.println(c.b);
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
  Serial.println("Stopped.");
}

bool stopRequested() {
  if (stopAll) return true;
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

void logEsp32Messages() {
  static String line;

  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) {
        Serial.print("ESP32: ");
        Serial.println(line);
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}

bool connectEsp32Wifi() {
  Serial.println("Waiting for ESP32 WiFi...");

  unsigned long startMs = millis();
  bool wifiConnected = false;
  String line;
  while (millis() - startMs < 20000) {
    while (espSerial.available()) {
      char c = espSerial.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          Serial.print("ESP32: ");
          Serial.println(line);
          if (line.startsWith("WIFI_CONNECTED")) wifiConnected = true;
          if (line.startsWith("MQTT_CONNECTED")) return true;
          if (line.startsWith("MQTT_FAILED")) return wifiConnected;
          if (line.startsWith("WIFI_FAILED")) return false;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }
  if (!wifiConnected) Serial.println("ESP32 not responding.");
  return wifiConnected;
}

bool runArmCommand(const char* command, const char* okResponse, const char* failResponse) {
  while (espSerial.available()) espSerial.read();

  Serial.println(command);
  espSerial.listen();
  espSerial.println(command);
  espSerial.flush();

  unsigned long startMs = millis();
  String line;
  while (millis() - startMs < 5000) {
    if (stopRequested()) return false;

    while (espSerial.available()) {
      char c = espSerial.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          Serial.print("ESP32: ");
          Serial.println(line);
          if (line == okResponse) return true;
          if (line == failResponse) return false;
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  Serial.println("ESP32 arm command timeout.");
  return false;
}

bool fetchRed() {
  return runArmCommand("RUN_RED", "ARM_RED_OK", "ARM_RED_FAIL");
}

bool fetchBlue() {
  return runArmCommand("RUN_BLUE", "ARM_BLUE_OK", "ARM_BLUE_FAIL");
}

bool fetchYellow() {
  return runArmCommand("RUN_YELLOW", "ARM_YELLOW_OK", "ARM_YELLOW_FAIL");
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
  Serial.print("Brake pulse ");
  Serial.print(BRAKE_MS);
  Serial.println("ms");
  (robot.*counterMove)();
  delay(BRAKE_MS);
  robot.Stop();
}

void turnRight90() {
  Serial.println("Turning right 90 degrees...");
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TURNING_SPEED;
  robot.Advance();
  delay(300);
  robot.Turn_Right();
  delay(500);
  brakePulse(&mecanumCar::Turn_Left);   // cancel rotation momentum
  robot.L_Move();
  delay(400);
  brakePulse(&mecanumCar::R_Move);      // cancel strafe momentum
}

bool moveShort(uint16_t durationMs) {
  Serial.println("Moving short distance...");

  speed_Upper_L = speed_Lower_L = LEFT_SPEED;
  speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
  robot.Advance();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;
  }

  robot.Stop();
  return true;
}

bool reverseShort(uint16_t durationMs) {
  Serial.println("Reversing short distance...");

  speed_Upper_L = speed_Lower_L = LEFT_SPEED;
  speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
  robot.Back();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;   // stop was requested mid-reverse
  }

  robot.Stop();
  return true;
}

bool robotReverse(uint16_t timeoutMs, uint16_t reverseBlindMs) {
  bool leftStartPoint = !(digitalRead(LINE_LEFT_PIN) == HIGH &&
                          digitalRead(LINE_MIDDLE_PIN) == HIGH &&
                          digitalRead(LINE_RIGHT_PIN) == HIGH);

  speed_Upper_L = speed_Lower_L = LEFT_SPEED;
  speed_Upper_R = speed_Lower_R = RIGHT_SPEED + 2; // slight trim for asymmetry
  robot.Back();

  if (waitOrStop(reverseBlindMs)) {
    robot.Stop();
    return false;
  }

  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    bool onPoint = digitalRead(LINE_LEFT_PIN) == HIGH &&
                   digitalRead(LINE_MIDDLE_PIN) == HIGH &&
                   digitalRead(LINE_RIGHT_PIN) == HIGH;

    if (leftStartPoint && onPoint) {
      robot.Stop();
      Serial.println("Reverse point reached.");
      return true;
    }

    if (!onPoint) leftStartPoint = true;
    delay(LINE_TICK_MS);
  }

  robot.Stop();
  Serial.println("Reverse timeout. Point not found.");
  return false;
}

void strafeLeft(int targetCount) {
  int detectedLines = 0;
  bool wasOnLeftLine = false;

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = 60;

  Serial.print("Strafing left. Target left lines: ");
  Serial.println(targetCount);

  while (detectedLines < targetCount) {
    if (stopRequested()) {
      robot.Stop();
      return;
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);

    if (SL == HIGH) {
      if (!wasOnLeftLine) {
        detectedLines++;
        wasOnLeftLine = true;
        Serial.print("Left line detected: ");
        Serial.print(detectedLines);
        Serial.print("/");
        Serial.println(targetCount);
      }
    } else {
      wasOnLeftLine = false;
    }

    robot.L_Move();
    delay(LINE_TICK_MS);
  }

  robot.Stop();
}

// ==========================================================================
//  ROTATION (90°)
// ==========================================================================

bool rotate90(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println("Rotating 90 degrees until right sensor detects line...");

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
      brakePulse(&mecanumCar::Turn_Left);   // cancel spin momentum before centering
      Serial.println("Right sensor detected line.");
      return searchAndCenterLine(1500, +1);
    }

    robot.Turn_Right();
    delay(LINE_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println("Rotate 90 timeout. Right sensor did not detect line.");
  return false;
}

bool rotate90Left(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println("Rotating 90 degrees until left sensor detects line...");

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
      Serial.println("Left sensor detected line.");
      return searchAndCenterLine(1500, -1);
    }

    robot.Turn_Left();
    delay(LINE_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println("Rotate 90 timeout. Left sensor did not detect line.");
  return false;
}

// Single continuous 180° spin — an alternative to calling rotate90() twice.
// Same logic as rotate90 (spin right, grace period, brake pulse, then center),
// but counts right-sensor line crossings and stops on the 2nd: the first crossing
// is the ~90° line, the second is the ~180° line. Added for A/B comparison against
// the two-90 method in path1; timeout is larger since the spin is twice as long.
bool rotate180(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println("Rotating 180 degrees (single spin) until 2nd right-line crossing...");

  unsigned long startMs = millis();

  speed_Upper_L = speed_Lower_L = speed_Lower_R = turnSpeed;
  speed_Upper_R = turnSpeed;

  uint8_t crossings = 0;
  bool onLine = false;   // true while the right sensor is currently over a line

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    if (millis() - startMs >= ROTATE_SENSOR_GRACE_MS) {
      if (SR == HIGH && !onLine) {          // rising edge → a new line crossing
        onLine = true;
        crossings++;
        if (crossings >= 2) {
          brakePulse(&mecanumCar::Turn_Left);   // cancel spin momentum before centering
          Serial.println("Second line crossing — 180° reached.");
          return searchAndCenterLine(1500, +1);
        }
      } else if (SR == LOW) {               // left the line; ready for next crossing
        onLine = false;
      }
    }

    robot.Turn_Right();
    delay(LINE_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println("Rotate 180 timeout. Second line crossing not detected.");
  return false;
}

// ==========================================================================
//  LINE FOLLOWING
// ==========================================================================

// One steering step shared by every line-follower. Middle sensor owns the lane;
// side sensors correct only when the middle is off; if the line is lost entirely,
// keep turning toward whichever side saw it last. `lastSeenSide` is updated in
// place (+1 = line last seen on right, -1 = left).
void steerAlongLine(uint8_t SL, uint8_t SM, uint8_t SR,
                    uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed,
                    int8_t &lastSeenSide) {
  if (SM == HIGH || (SL == HIGH && SR == HIGH)) {
    speed_Upper_L = speed_Lower_L = leftSpeed;
    speed_Upper_R = speed_Lower_R = rightSpeed;
    robot.Advance();
  } else if (SL == LOW && SR == HIGH) {
    lastSeenSide = +1;
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
    robot.Turn_Right();
  } else if (SR == LOW && SL == HIGH) {
    lastSeenSide = -1;
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
    robot.Turn_Left();
  } else {
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
    (lastSeenSide < 0) ? robot.Turn_Left() : robot.Turn_Right();
  }
}

// ── Basic 3-sensor line-follow (KS0560 lesson_6 pattern) ─────────────────────
// Follow the line, counting all-black crossings; stop when count hits targetCount.
void followLineWithTarget(int targetCount, uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed) {
  numLines      = 0;
  wasOnFullLine = false;

  unsigned long lastIRCheckMs = 0;
  int8_t lastSeenSide = +1; // +1 = line last seen on right, -1 = left

  Serial.print("Basic line follow. Target lines: ");
  Serial.println(targetCount);

  while (numLines < targetCount) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return;
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    // Count crossings — all three sensors on black at once, one-shot per line
    if (SL == HIGH && SM == HIGH && SR == HIGH) {
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
      }
    } else {
      wasOnFullLine = false;
    }

    steerAlongLine(SL, SM, SR, leftSpeed, rightSpeed, turnSpeed, lastSeenSide);
    delay(LINE_TICK_MS);
  }
}

void followLineWithTarget(int targetCount) {
  followLineWithTarget(targetCount, LEFT_SPEED, RIGHT_SPEED, LINE_TURN_SPEED);
}

bool followLineForMs(uint16_t ms) {
  unsigned long start = millis();
  unsigned long lastIRCheckMs = 0;
  int8_t lastSeenSide = +1; // +1 = line last seen on right, -1 = left

  while (millis() - start < ms) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return false;
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    steerAlongLine(SL, SM, SR, LEFT_SPEED, RIGHT_SPEED, LINE_TURN_SPEED, lastSeenSide);
    delay(LINE_TICK_MS);
  }

  robot.Stop();
  return true;
}

void moveSlowly(int targetCount) {
  followLineWithTarget(targetCount, SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED);
}

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
//  LINE FOLLOWING WITH OBSTACLE STOP
// ==========================================================================

bool followLineWithDistance(uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed, uint16_t stopDistanceCm) {
  Serial.println("Following line with distance check...");

  unsigned long lastIRCheckMs      = 0;
  unsigned long lastDistanceSampleMs = millis() - ULTRASONIC_SAMPLE_MS;
  unsigned long lastDistanceReportMs = millis() - 250;
  uint16_t distanceCm = 999;
  uint8_t closeReadCount = 0;
  int8_t lastSeenSide = +1; // +1 = line last seen on right, -1 = left

  while (true) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return false;
    }

    if (now - lastDistanceSampleMs >= ULTRASONIC_SAMPLE_MS) {
      lastDistanceSampleMs = now;
      distanceCm = readUltrasonicMedian();

      if (now - lastDistanceReportMs >= 250) {
        lastDistanceReportMs = now;
        if (distanceCm >= 999) {
          Serial.println("Distance: out of range");
        } else {
          Serial.print("Distance: ");
          Serial.print(distanceCm);
          Serial.println(" cm");
        }
      }

      if (distanceCm <= stopDistanceCm) {
        closeReadCount++;
      } else if (distanceCm > stopDistanceCm + ULTRASONIC_STOP_HYSTERESIS_CM) {
        closeReadCount = 0;
      }

      if (closeReadCount >= ULTRASONIC_CONFIRM_READS) {
        robot.Stop();
        Serial.print("Obstacle detected at ");
        Serial.print(distanceCm);
        Serial.println(" cm. Stopping.");
        return true;
      }
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    steerAlongLine(SL, SM, SR, leftSpeed, rightSpeed, turnSpeed, lastSeenSide);
    delay(LINE_TICK_MS);
  }
}

// Same tracking logic as followLineWithTarget; runs until an obstacle is
// detected within OBSTACLE_DISTANCE_CM or STOP is pressed.
void followLineWithDistance() {
  if (followLineWithDistance(LEFT_SPEED, RIGHT_SPEED, LINE_TURN_SPEED, APPROACH_DISTANCE_CM)) {
    if (waitOrStop(1000)) return;
    moveSlowlyToObject();
  }
}

void moveSlowlyToObject() {
  followLineWithDistance(SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED, OBSTACLE_DISTANCE_CM);
}

// ==========================================================================
//  LINE SEARCH & CENTERING
// ==========================================================================

bool searchAndCenterLine(uint16_t timeoutMs, int8_t initialLastSeenSide) {
  Serial.println("Searching and centering on line...");

  const uint8_t CENTER_CONFIRM_TICKS = 5;
  uint8_t centeredTicks = 0;

  unsigned long startMs = millis();

  // Search state
  enum SearchPhase : uint8_t { SWEEP_TOWARD, SWEEP_AWAY };
  SearchPhase phase = SWEEP_TOWARD;
  unsigned long phaseStartMs = 0;
  uint16_t sweepMs = SEARCH_SWEEP_INITIAL_MS;
  bool searching = false;        // are we currently in lost-line mode?
  int8_t lastSeenSide = initialLastSeenSide; // +1 = right, -1 = left, 0 = neutral

  while (true) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    if (timeoutMs > 0 && millis() - startMs >= timeoutMs) {
      robot.Stop();
      Serial.println("Search timeout. Line not found.");
      return false;
    }

    bool L = digitalRead(LINE_LEFT_PIN)   == HIGH;
    bool M = digitalRead(LINE_MIDDLE_PIN) == HIGH;
    bool R = digitalRead(LINE_RIGHT_PIN)  == HIGH;

    // --- Centered: middle only ---
    if (!L && M && !R) {
      robot.Stop();
      if (++centeredTicks >= CENTER_CONFIRM_TICKS) {
        Serial.println("Line found and centered.");
        return true;
      }
      delay(LINE_TICK_MS);
      continue;
    }
    centeredTicks = 0;

    // --- Some sensor sees the line: correct toward center ---
    if (L || M || R) {
      searching = false;  // reset lost-line state machine
      sweepMs = SEARCH_SWEEP_INITIAL_MS;

      // Remember which side the line is on for future searches
      if (L && !R)      lastSeenSide = -1;
      else if (R && !L) lastSeenSide = +1;

      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_TURN_SPEED;

      if (L && R) {
        (lastSeenSide < 0) ? robot.Turn_Left() : robot.Turn_Right();
      } else if (L && !R) {
        robot.Turn_Left();          // line is left of center
      } else if (R && !L) {
        robot.Turn_Right();         // line is right of center
      } else {
        // L && R (crossing / thick line), or middle-only-unstable:
        // advance slowly until readings normalize
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      }

      delay(LINE_TICK_MS);
      continue;
    }

    // --- All white: line lost. Expanding sweep toward last-seen side ---
    if (!searching) {
      searching = true;
      phase = SWEEP_TOWARD;
      phaseStartMs = millis();
    }

    unsigned long stepElapsed = millis() - phaseStartMs;
    int8_t sweepSide = (lastSeenSide == 0) ? -1 : lastSeenSide;
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_SEARCH_SPEED;

    switch (phase) {
      case SWEEP_TOWARD:
        // Sweep toward the side where the line was last seen
        (sweepSide < 0) ? robot.Turn_Left() : robot.Turn_Right();
        if (stepElapsed >= sweepMs) {
          phase = SWEEP_AWAY;
          phaseStartMs = millis();
        }
        break;

      case SWEEP_AWAY:
        // Sweep back through center to the other side (double width)
        (sweepSide < 0) ? robot.Turn_Right() : robot.Turn_Left();
        if (stepElapsed >= (uint16_t)(sweepMs * 2)) {
          uint16_t nextSweep = sweepMs + SEARCH_SWEEP_GROWTH_MS;
          sweepMs = (nextSweep > SEARCH_SWEEP_MAX_MS) ? SEARCH_SWEEP_MAX_MS : nextSweep;
          phase = SWEEP_TOWARD;
          phaseStartMs = millis();
        }
        break;
    }

    delay(LINE_TICK_MS);
  }
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

  Serial.println("Checking TCS3200 color...");
  ColorLabel label = classifyColor();

  openGripper(gOpen);
  if (waitOrStop(1000)) return -1;

  if (label == ColorLabel::Blue) {
    Serial.println("Blue detected. Gripper closed at 180.");
    return 0;
  } else if (label == ColorLabel::Red) {
    Serial.println("Red detected. Gripper closed at 180.");
    return 1;
  } else {
    Serial.println("Yellow detected. Gripper closed at 180.");
    return 2;
  }
}

// ==========================================================================
//  NAVIGATION HELPERS
// ==========================================================================

// for path 1
bool returnToCheckpoint() {
  reverseShort(300);
  if (!searchAndCenterLine()) return false;
  if (!rotate90()) return false;
  if (!searchAndCenterLine()) return false;
  if (waitOrStop(500)) return false;
  reverseShort(200);
  if (!searchAndCenterLine()) return false;
  if (!rotate90()) return false;
  return searchAndCenterLine();
}

// ==========================================================================
//  CHALLENGE PATHS
// ==========================================================================

// path1 — pick up the object straight ahead, ferry it to the drop lane,
// release it, signal the arm which colour it was, then return to the checkpoint.
void path1() {
  // ── Phase 1: approach the object, grip & identify its colour ──
  followLineWithDistance();
  if (stopAll) return;
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (waitOrStop(5000)) return;
  if (waitOrStop(300)) return;
  isGripperOpen = !isGripperOpen;

  // ── Phase 2: turn around onto the drop lane ──
  // METHOD A — two discrete 90° turns (currently active).
  //            Comment this block out to try METHOD B.
  // if (!rotate90()) return;
  // if (waitOrStop(500)) return;
  // if (!searchAndCenterLine()) return;
  // reverseShort(200);
  // if (waitOrStop(500)) return;
  // if (!rotate90()) return;
  // if (!searchAndCenterLine()) return;

  // METHOD B — single continuous 180° spin.
  //            Uncomment these two lines and comment out METHOD A above.
  if (!rotate180()) return;
  if (!searchAndCenterLine()) return;

  // ── Phase 3: drive to the drop zone and release the object ──
  followLineWithTarget(5);
  if (waitOrStop(1000)) return;
  moveSlowly(2);
  if (waitOrStop(1000)) return;
  openGripper(isGripperOpen);
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;

  // ── Phase 4: signal the arm, then return to the checkpoint ──
  sendArmCommand(colorRes);
  if (!returnToCheckpoint()) return;
  if (!searchAndCenterLine()) return;
}

// path2 — LEFT-side run. Fetches the object from the left branch and brings
// it to the drop zone (mirror of path3; distances are hand-tuned per side).
void path2() {
  // ── Phase 1: turn onto the object lane (left, then in) ──
  followLineWithTarget(3);
  if (waitOrStop(1000)) return;
  moveShort(400);
  if (waitOrStop(500)) return;
  if (!rotate90Left()) return;
  if (waitOrStop(1000)) return;
  followLineWithTarget(2);
  if (waitOrStop(500)) return;
  moveShort(100);
  if (waitOrStop(500)) return;
  if (!rotate90()) return;
  if (waitOrStop(500)) return;

  // ── Phase 2: approach the object, grip & identify its colour ──
  followLineWithDistance();
  if (waitOrStop(500)) return;
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  if (waitOrStop(500)) return;
  if (waitOrStop(1000)) return;
  isGripperOpen = !isGripperOpen;
  if (waitOrStop(1000)) return;

  // ── Phase 3: back off and navigate to the drop lane ──
  reverseShort(150);
  if (waitOrStop(500)) return;
  if (!rotate90()) return;
  if (waitOrStop(1000)) return;
  followLineWithTarget(2);
  if (waitOrStop(500)) return;
  moveShort(300);
  if (waitOrStop(500)) return;
  if (!rotate90()) return;

  // ── Phase 4: final approach and release the object ──
  followLineWithTarget(4);
  if (waitOrStop(1000)) return;
  moveSlowly(2);
  if (waitOrStop(1000)) return;
  openGripper(isGripperOpen);
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;

  // ── Phase 5: return to the checkpoint ──
  if (!returnToCheckpoint()) return;
  if (!searchAndCenterLine()) return;
}

// path3 — RIGHT-side run: mirror of path2. Fetches the object from the right
// branch and brings it to the drop zone (hand-tuned distances differ from path2).
void path3() {
  // ── Phase 1: turn onto the object lane (right, then in) ──
  followLineWithTarget(3);
  if (waitOrStop(500)) return;
  moveShort(150);
  if (waitOrStop(500)) return;
  if (!rotate90()) return;
  if (waitOrStop(500)) return;
  followLineWithTarget(2);
  if (waitOrStop(500)) return;
  moveShort(300);
  if (waitOrStop(500)) return;
  if (!rotate90Left()) return;
  if (waitOrStop(500)) return;

  // ── Phase 2: approach the object, grip & identify its colour ──
  followLineWithDistance();
  if (stopAll) return;

  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;

  // ── Phase 3: back off and navigate to the drop lane ──
  reverseShort(150);
  if (waitOrStop(500)) return;
  if (!rotate90Left()) return;
  if (waitOrStop(1000)) return;
  followLineWithTarget(2);
  if (waitOrStop(500)) return;
  moveShort(400);
  if (waitOrStop(500)) return;
  if (!rotate90Left()) return;

  // ── Phase 4: final approach and release the object ──
  if (!searchAndCenterLine()) return;
  followLineWithTarget(4);
  if (waitOrStop(1000)) return;
  moveSlowly(2);
  if (waitOrStop(1000)) return;
  openGripper(isGripperOpen);
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;
  // ── Phase 5: return to the checkpoint ──
  if (!returnToCheckpoint()) return;
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
  bool wifiConnected = connectEsp32Wifi();
  robot.right_led(wifiConnected);
  robot.left_led(wifiConnected);
  if (wifiConnected) statusLedOffAtMs = millis() + 2000;
  servo.attach(SERVO_PIN);
  servo.write(gripperAngle);
  Serial.println("Ready. Press IR to start.");
}

void loop() {
  logEsp32Messages();

  // init success
  if (statusLedOffAtMs > 0 && millis() >= statusLedOffAtMs) {
    robot.right_led(false);
    robot.left_led(false);
    statusLedOffAtMs = 0;
  }

  int key = IRreceive.getKey();

  // A fresh command key (getKey returns -1 when idle) re-arms the robot after an
  // emergency stop. The stop buttons are excluded so they never clear their own flag.
  if (key != -1 && key != 70 && key != STOP_KEY) {
    stopAll = false;
  }

  switch (key) {
    case 22: // challenge 1
      returnToCheckpoint();
      if (!searchAndCenterLine()) return;
      followLineWithTarget(3);
      break;

    case 25: // challenge 2
      reverseShort(300);
      if (!searchAndCenterLine()) return;
      break;

    case 13: { // challenge 3
      path1();
      if (waitOrStop(1000)) break;
      path2();
      if (waitOrStop(1000)) break;
      path3();
      robot.Stop();
      Serial.println("Done.");
      break;
    }

    case 68: // left button
      if (!returnToCheckpoint()) return;
      if (!searchAndCenterLine()) return;
      if (waitOrStop(1000)) return;
      followLineWithTarget(2);
      strafeLeft(2);
      if (!searchAndCenterLine()) return;
      followLineWithDistance();
      break;

    case 70: // front button / stop everything
      stopEverything();
      break;

    case 12: // number 4
      path2();
      break;

    case 24: // button 5
      path3();
      break;

    case 94:
      returnToCheckpoint();
      break;

    case 67: // right button
      rotate90();
      break;

    case 82: { // Button to test out code - gripper
      static bool gripperOpen = false;
      gripperOpen = !gripperOpen;
      openGripper(gripperOpen);
      break;
    }

    case 74: // Test for ultrasonic sensor
      Serial.println("Ultrasonic reading started...");
      while (IRreceive.getKey() != 70) {
        uint16_t dist = readUltrasonic();
        if (dist >= 999)
          Serial.println("Distance: out of range");
        else {
          Serial.print("Distance: ");
          Serial.print(dist);
          Serial.println(" cm");
        }
        delay(250);
      }
      Serial.println("Ultrasonic stopped.");
      break;

    case 28: // test run robot
      fetchBlue();
      break;

    default:
      break;
  }
}
