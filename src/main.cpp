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
const uint8_t  LINE_TURN_SPEED      = 32;   // spin speed for the lost-line recovery search only — normal tracking no longer spins (see steerAlongLine)

// Line-follow correction gains. Tracking corrects by strafing the chassis back
// onto the line (heading untouched) plus a small yaw to fix actual heading
// error — it never pivots in place, so forward speed never drops to zero.
// Error is -2..+2 (see lineError). Tune on hardware.
const uint8_t  LINE_STRAFE_GAIN     = 14;  // strafe PWM per unit of line error — the main correction authority
const uint8_t  LINE_YAW_GAIN        = 1;   // yaw PWM per unit of line error — keep small; this is what used to cause the zigzag
const uint8_t  LINE_CORRECT_SLOWDOWN = 6;  // forward PWM shed per unit of error; sized so vy stays well above zero at |error| = 2
const uint8_t  SLOW_LEFT_SPEED      = 33;
const uint8_t  SLOW_RIGHT_SPEED     = 33;
const uint8_t  SLOW_LINE_TURN_SPEED = 31;
const uint8_t  LINE_TICK_MS         = 2;   // follow-loop tick delay (line-follow correction only)
const uint8_t  SLOW_LINE_TICK_MS    = 2;   // tick delay for the slow-approach phase — tighter than LINE_TICK_MS to keep per-correction coverage small at low speed
const uint8_t  TURN_TICK_MS         = 12;  // tick delay for dedicated turn/positioning maneuvers (rotate90/90Left/180, searchAndCenterLine, robotReverse, strafeLeft) — decoupled from LINE_TICK_MS since these weren't validated at the aggressive correction tick
const uint16_t OBSTACLE_DISTANCE_CM = 7;
const uint16_t APPROACH_DISTANCE_CM = 20;
const uint16_t ULTRASONIC_SAMPLE_MS = 50;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;
const uint8_t  ULTRASONIC_CONFIRM_READS = 3;
const uint8_t  ULTRASONIC_STOP_HYSTERESIS_CM = 2;
const uint8_t  LINE_SEARCH_SPEED    = 40;   // aggressive turning when line is lost

// Per-wheel advance speeds — calibrated via the IR remote's wheel-test button
// (case 90, moveForwardWheelSpeeds) to compensate for motor-to-motor variance.
// This is now the single source of truth for normal-speed forward driving.
const uint8_t  WHEEL_SPEED_UPPER_L   = 42;
const uint8_t  WHEEL_SPEED_LOWER_L   = 44;
const uint8_t  WHEEL_SPEED_UPPER_R   = 45;
const uint8_t  WHEEL_SPEED_LOWER_R   = 45;
const uint8_t  GRIPPER_OPEN_ANGLE   = 2;
const uint8_t  GRIPPER_CLOSE_ANGLE  = 180;
const uint8_t  GRIPPER_STEP_DELAY_MS = 10;
const uint16_t REVERSE_BLIND_MS     = 500;
const uint16_t ROTATE_SENSOR_GRACE_MS = 200;
const uint16_t BRAKE_MS = 50;   // counter-pulse duration to cancel turn momentum — tune empirically (too long = kicks back the other way)
const uint16_t DRIFT_CORRECT_MS = 120;  // drift_left pulse after the 180° spin to pull the overshoot back onto the line — tune empirically

// Tuning constants — adjust to taste (for searchAndCenterLine)
const uint16_t SEARCH_SWEEP_INITIAL_MS = 300;   // first sweep half-width
const uint16_t SEARCH_SWEEP_GROWTH_MS  = 300;   // how much wider each sweep gets
const uint16_t SEARCH_SWEEP_MAX_MS     = 1500;  // cap on sweep width

// If steerAlongLine loses the line on all 3 sensors and keeps spinning one way
// this long without reacquiring it, flip direction instead of committing to
// that turn indefinitely — tune empirically.
const uint16_t LOST_LINE_FLIP_MS = 400;

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
bool       connectEsp32Wifi();
bool       runArmCommand(const char* command, const char* okResponse, const char* failResponse);
bool       fetchRed();
bool       fetchBlue();
bool       fetchYellow();
void       sendArmCommand(int colorRes);

void       turnRight90();
void       brakePulse(void (mecanumCar::*counterMove)());
bool       moveShort(uint16_t durationMs = 300);
bool       moveForwardWheelSpeeds(uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint16_t durationMs = 2000);
bool       reverseShort(uint16_t durationMs = 300);
bool       robotReverse(uint16_t timeoutMs = 2500, uint16_t reverseBlindMs = REVERSE_BLIND_MS);
void       strafeLeft(int targetCount);

bool       rotate90(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 3000);
bool       rotate90Left(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 3000);
bool       rotate180(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 5000);

void       steerAlongLine(uint8_t SL, uint8_t SM, uint8_t SR,
                          uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed,
                          int8_t &lastSeenSide);
void       followLineWithTarget(int targetCount, uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed, uint8_t tickMs = LINE_TICK_MS);
void       followLineWithTarget(int targetCount);
bool       followLineForMs(uint16_t ms);
void       moveSlowly(int targetCount);

uint16_t   readUltrasonic();
uint16_t   readUltrasonicMedian();

bool       followLineWithDistance(uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed, uint16_t stopDistanceCm, uint8_t tickMs = LINE_TICK_MS);
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

bool connectEsp32Wifi() {
  Serial.println(F("Waiting for ESP32 WiFi..."));

  unsigned long startMs = millis();
  bool wifiConnected = false;
  String line;
  while (millis() - startMs < 20000) {
    while (espSerial.available()) {
      char c = espSerial.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          Serial.print(F("ESP32: "));
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
  if (!wifiConnected) Serial.println(F("ESP32 not responding."));
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
          Serial.print(F("ESP32: "));
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

  Serial.println(F("ESP32 arm command timeout."));
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
  Serial.print(F("Brake pulse "));
  Serial.print(BRAKE_MS);
  Serial.println(F("ms"));
  (robot.*counterMove)();
  delay(BRAKE_MS);
  robot.Stop();
}

void turnRight90() {
  Serial.println(F("Turning right 90 degrees..."));
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
  Serial.println(F("Moving short distance..."));

  speed_Upper_L = WHEEL_SPEED_UPPER_L;
  speed_Lower_L = WHEEL_SPEED_LOWER_L;
  speed_Upper_R = WHEEL_SPEED_UPPER_R;
  speed_Lower_R = WHEEL_SPEED_LOWER_R;
  robot.Advance();

  if (waitOrStop(durationMs)) {
    robot.Stop();
    return false;
  }

  robot.Stop();
  return true;
}

// Drive forward with explicit per-wheel speeds, bypassing the WHEEL_SPEED_*
// constants — lets a caller (e.g. the remote's wheel-test button) try out
// values on the fly without re-flashing.
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

  speed_Upper_L = WHEEL_SPEED_UPPER_L;
  speed_Lower_L = WHEEL_SPEED_LOWER_L;
  speed_Upper_R = WHEEL_SPEED_UPPER_R;
  speed_Lower_R = WHEEL_SPEED_LOWER_R;
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

  speed_Upper_L = WHEEL_SPEED_UPPER_L;
  speed_Lower_L = WHEEL_SPEED_LOWER_L;
  speed_Upper_R = WHEEL_SPEED_UPPER_R;
  speed_Lower_R = WHEEL_SPEED_LOWER_R;
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
      Serial.println(F("Reverse point reached."));
      return true;
    }

    if (!onPoint) leftStartPoint = true;
    delay(TURN_TICK_MS);
  }

  robot.Stop();
  Serial.println(F("Reverse timeout. Point not found."));
  return false;
}

void strafeLeft(int targetCount) {
  int detectedLines = 0;
  bool wasOnLeftLine = false;

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = 60;

  Serial.print(F("Strafing left. Target left lines: "));
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
        Serial.print(F("Left line detected: "));
        Serial.print(detectedLines);
        Serial.print(F("/"));
        Serial.println(targetCount);
      }
    } else {
      wasOnLeftLine = false;
    }

    robot.L_Move();
    delay(TURN_TICK_MS);
  }

  robot.Stop();
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
      brakePulse(&mecanumCar::Turn_Left);   // cancel spin momentum before centering
      Serial.println(F("Right sensor detected line."));
      return searchAndCenterLine(1500, +1);
    }

    robot.Turn_Right();
    delay(TURN_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println(F("Rotate 90 timeout. Right sensor did not detect line."));
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

// Single continuous 180° spin — an alternative to calling rotate90() twice.
// Same logic as rotate90 (spin right, grace period, brake pulse, then center),
// but counts right-sensor line crossings and stops on the 2nd: the first crossing
// is the ~90° line, the second is the ~180° line. Added for A/B comparison against
// the two-90 method in path1; timeout is larger since the spin is twice as long.
bool rotate180(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println(F("Rotating 180 degrees (single spin) until 2nd right-line crossing..."));

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
          robot.Stop();
          Serial.println(F("Second line crossing — 180° reached."));
          return true;
        }
      } else if (SR == LOW) {               // left the line; ready for next crossing
        onLine = false;
      }
    }

    robot.Turn_Right();
    delay(TURN_TICK_MS);
  }
  robot.Stop();
  delay(1000);
  Serial.println(F("Rotate 180 timeout. Second line crossing not detected."));
  return false;
}

// ==========================================================================
//  LINE FOLLOWING
// ==========================================================================

// Mecanum mixer: drive an arbitrary (forward, strafe, yaw) velocity vector.
// Wheel signs are taken from the library's own primitives — Advance() drives all
// four forward, R_Move() runs (+UL, -LL, -UR, +LR), Turn_Right() runs
// (+UL, +LL, -UR, -LR) — which gives:
//   Upper_L = vy + vx + w      Upper_R = vy - vx - w
//   Lower_L = vy - vx + w      Lower_R = vy + vx - w
// vy is per-wheel so the existing forward-speed trim (WHEEL_SPEED_*) survives.
// vx is positive to the right, w is positive clockwise.
static void driveMecanum(int vyUL, int vyLL, int vyUR, int vyLR, int vx, int w) {
  int ul = vyUL + vx + w;
  int ll = vyLL - vx + w;
  int ur = vyUR - vx - w;
  int lr = vyLR + vx - w;
  robot.Motor_Upper_L(ul >= 0, constrain(abs(ul), 0, 255));
  robot.Motor_Lower_L(ll >= 0, constrain(abs(ll), 0, 255));
  robot.Motor_Upper_R(ur >= 0, constrain(abs(ur), 0, 255));
  robot.Motor_Lower_R(lr >= 0, constrain(abs(lr), 0, 255));
}

// Position of the line relative to the robot, from the 3 binary sensors.
// Positive = line is off to the right, so the robot must move right to re-centre.
// Reading the middle sensor together with the sides gives 5 levels instead of the
// 3 the sides alone would — enough for a proportional correction.
//   -2 = line under left sensor only      +2 = line under right sensor only
//   -1 = line between left and middle     +1 = line between middle and right
//    0 = centred, or straddling a crossing (both sides lit)
static int8_t lineError(uint8_t SL, uint8_t SM, uint8_t SR) {
  if (SL == HIGH && SR == HIGH) return 0;                 // crossing — hold the lane
  if (SR == HIGH) return (SM == HIGH) ? +1 : +2;
  if (SL == HIGH) return (SM == HIGH) ? -1 : -2;
  return 0;                                               // middle only, or lost
}

// One steering step shared by every line-follower.
//
// Corrections strafe the chassis sideways back onto the line rather than pivoting
// in place. The old version stopped dead and spun whenever the middle sensor fell
// off the line, and because that spin ended when the *sensor* re-acquired the line
// — while the sensor sits ahead of the centre of rotation — the *body* always
// over-rotated past parallel, drove off the far edge, and spun back. That geometric
// overshoot is what produced the zigzag, and no gain or tick tuning removes it.
// Strafing has no such overshoot: lateral error is corrected without changing
// heading at all. A small yaw term rides along to fix genuine heading error, and
// forward speed is only reduced (never zeroed) while correcting.
//
// If the line is lost entirely, keep turning toward whichever side saw it last.
// `lastSeenSide` is updated in place (+1 = line last seen on right, -1 = left).
void steerAlongLine(uint8_t SL, uint8_t SM, uint8_t SR,
                    uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed,
                    int8_t &lastSeenSide) {
  // Tracks how long we've been spinning one way with the line fully lost —
  // static because this only makes sense as a running timer across
  // consecutive ticks of the same follow-loop, not per-call state.
  static unsigned long lostSinceMs = 0;
  static int8_t lostSpinDir = 0;

  bool lineVisible = (SL == HIGH || SM == HIGH || SR == HIGH);

  if (lineVisible) {
    lostSinceMs = 0;
    int8_t err = lineError(SL, SM, SR);
    if (err > 0) lastSeenSide = +1;
    else if (err < 0) lastSeenSide = -1;

    int mag = abs(err);
    int vx  = LINE_STRAFE_GAIN * err;   // strafe toward the line — the real correction
    int w   = LINE_YAW_GAIN * err;      // gentle heading alignment, not a pivot
    int cut = LINE_CORRECT_SLOWDOWN * mag;

    driveMecanum(max(upperL - cut, 1), max(lowerL - cut, 1),
                 max(upperR - cut, 1), max(lowerR - cut, 1), vx, w);
  } else {
    // Line fully lost. Spin toward lastSeenSide, but if that doesn't
    // reacquire it within LOST_LINE_FLIP_MS, flip direction instead of
    // committing to one turn indefinitely (e.g. a swerve that overshoots
    // past the line shouldn't just spin left forever).
    if (lostSinceMs == 0) {
      lostSinceMs = millis();
      lostSpinDir = (lastSeenSide < 0) ? -1 : +1;
    } else if (millis() - lostSinceMs >= LOST_LINE_FLIP_MS) {
      lostSinceMs = millis();
      lostSpinDir = -lostSpinDir;
    }
    speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
    (lostSpinDir < 0) ? robot.Turn_Left() : robot.Turn_Right();
  }
}

// ── Basic 3-sensor line-follow (KS0560 lesson_6 pattern) ─────────────────────
// Follow the line, counting all-black crossings; stop when count hits targetCount.
void followLineWithTarget(int targetCount, uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed, uint8_t tickMs) {
  numLines      = 0;
  wasOnFullLine = false;

  unsigned long lastIRCheckMs = 0;
  int8_t lastSeenSide = +1; // +1 = line last seen on right, -1 = left

  Serial.print(F("Basic line follow. Target lines: "));
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
    } else {
      wasOnFullLine = false;
    }

    steerAlongLine(SL, SM, SR, upperL, lowerL, upperR, lowerR, turnSpeed, lastSeenSide);
    delay(tickMs);
  }
}

void followLineWithTarget(int targetCount) {
  followLineWithTarget(targetCount, WHEEL_SPEED_UPPER_L, WHEEL_SPEED_LOWER_L, WHEEL_SPEED_UPPER_R, WHEEL_SPEED_LOWER_R, LINE_TURN_SPEED, LINE_TICK_MS);
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

    steerAlongLine(SL, SM, SR, WHEEL_SPEED_UPPER_L, WHEEL_SPEED_LOWER_L, WHEEL_SPEED_UPPER_R, WHEEL_SPEED_LOWER_R, LINE_TURN_SPEED, lastSeenSide);
    delay(LINE_TICK_MS);
  }

  robot.Stop();
  return true;
}

void moveSlowly(int targetCount) {
  followLineWithTarget(targetCount, SLOW_LEFT_SPEED, SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED, SLOW_LINE_TICK_MS);
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

bool followLineWithDistance(uint8_t upperL, uint8_t lowerL, uint8_t upperR, uint8_t lowerR, uint8_t turnSpeed, uint16_t stopDistanceCm, uint8_t tickMs) {
  Serial.println(F("Following line with distance check..."));

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
          Serial.println(F("Distance: out of range"));
        } else {
          Serial.print(F("Distance: "));
          Serial.print(distanceCm);
          Serial.println(F(" cm"));
        }
      }

      if (distanceCm <= stopDistanceCm) {
        closeReadCount++;
      } else if (distanceCm > stopDistanceCm + ULTRASONIC_STOP_HYSTERESIS_CM) {
        closeReadCount = 0;
      }

      if (closeReadCount >= ULTRASONIC_CONFIRM_READS) {
        robot.Stop();
        Serial.print(F("Obstacle detected at "));
        Serial.print(distanceCm);
        Serial.println(F(" cm. Stopping."));
        return true;
      }
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    steerAlongLine(SL, SM, SR, upperL, lowerL, upperR, lowerR, turnSpeed, lastSeenSide);
    delay(tickMs);
  }
}

// Same tracking logic as followLineWithTarget; runs until an obstacle is
// detected within OBSTACLE_DISTANCE_CM or STOP is pressed.
void followLineWithDistance() {
  if (followLineWithDistance(WHEEL_SPEED_UPPER_L, WHEEL_SPEED_LOWER_L, WHEEL_SPEED_UPPER_R, WHEEL_SPEED_LOWER_R, LINE_TURN_SPEED, APPROACH_DISTANCE_CM, LINE_TICK_MS)) {
    delay(1000);
    moveSlowlyToObject();
  }
}

void moveSlowlyToObject() {
  followLineWithDistance(SLOW_LEFT_SPEED, SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED, OBSTACLE_DISTANCE_CM, SLOW_LINE_TICK_MS);
}

// ==========================================================================
//  LINE SEARCH & CENTERING
// ==========================================================================

bool searchAndCenterLine(uint16_t timeoutMs, int8_t initialLastSeenSide) {
  Serial.println(F("Searching and centering on line..."));

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
      Serial.println(F("Search timeout. Line not found."));
      return false;
    }

    bool L = digitalRead(LINE_LEFT_PIN)   == HIGH;
    bool M = digitalRead(LINE_MIDDLE_PIN) == HIGH;
    bool R = digitalRead(LINE_RIGHT_PIN)  == HIGH;

    // --- Centered: middle only ---
    if (!L && M && !R) {
      robot.Stop();
      if (++centeredTicks >= CENTER_CONFIRM_TICKS) {
        Serial.println(F("Line found and centered."));
        return true;
      }
      delay(TURN_TICK_MS);
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
        speed_Upper_L = WHEEL_SPEED_UPPER_L;
        speed_Lower_L = WHEEL_SPEED_LOWER_L;
        speed_Upper_R = WHEEL_SPEED_UPPER_R;
        speed_Lower_R = WHEEL_SPEED_LOWER_R;
        robot.Advance();
      }

      delay(TURN_TICK_MS);
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

    delay(TURN_TICK_MS);
  }
}

// After stopping on a crossing marker (all 3 sensors briefly HIGH), motor
// coast only ever carries the robot forward past it — never short of it,
// since the stop is triggered by first detecting the crossing. So back up
// in small steps until all 3 sensors read the line solidly again, confirming
// for a few ticks to debounce noise, squaring the robot up before a drop.
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
      speed_Upper_L = speed_Lower_L = SLOW_LEFT_SPEED;
      speed_Upper_R = speed_Lower_R = SLOW_RIGHT_SPEED;
      robot.Back();
    }

    delay(SLOW_LINE_TICK_MS);
  }

  robot.Stop();
  Serial.println(F("Align timeout — cross line not reacquired."));
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
  if (!searchAndCenterLine()) return false;
  if (!rotate90()) return false;
  if (!searchAndCenterLine()) return false;
  delay(500);
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
  delay(5000);
  delay(300);
  isGripperOpen = !isGripperOpen;

  // ── Phase 2: turn around onto the drop lane ──
  // METHOD A — two discrete 90° turns. Uncomment this block and comment out METHOD B.
  // if (!rotate90()) return;
  // delay(500);
  // if (!searchAndCenterLine()) return;
  // reverseShort(200);
  // delay(500);
  // if (!rotate90()) return;
  // if (!searchAndCenterLine()) return;

  // METHOD B — single continuous 180° spin (currently active).
  if (!rotate180()) return;
  // Drift left to straighten onto the line after the 180° (combats overshoot).
  // Tune DRIFT_CORRECT_MS; swap to robot.drift_right() if it drifts the wrong way.
  speed_Lower_L = speed_Lower_R = TURNING_SPEED;
  robot.drift_left();
  delay(DRIFT_CORRECT_MS);
  robot.Stop();
  if (!searchAndCenterLine()) return;

  // ── Phase 3: drive to the drop zone and release the object ──
  followLineWithTarget(5);
  delay(1000);
  moveSlowly(2);
  delay(1000);
  alignOnCrossLine();
  openGripper(isGripperOpen);
  delay(5000);
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
  followLineWithTarget(2);
  delay(1000);
  moveShort(200);
  delay(500);
  if (!rotate90Left()) return;
  delay(1000);
  followLineWithTarget(2);
  delay(500);
  moveShort(100);
  delay(500);
  if (!rotate90()) return;
  delay(500);

  // if (!rotate180()) return;
  // // Drift left to straighten onto the line after the 180° (combats overshoot).
  // // Tune DRIFT_CORRECT_MS; swap to robot.drift_right() if it drifts the wrong way.
  // speed_Lower_L = speed_Lower_R = TURNING_SPEED;
  // robot.drift_left();
  // delay(DRIFT_CORRECT_MS);
  // robot.Stop();
  // if (!searchAndCenterLine()) return;

  // ── Phase 2: approach the object, grip & identify its colour ──
  followLineWithDistance();
  delay(500);
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  delay(500);
  delay(1000);
  isGripperOpen = !isGripperOpen;
  delay(1000);

  // ── Phase 3: back off and navigate to the drop lane ──
  reverseShort(100);
  delay(500);
  if (!rotate90()) return;
  delay(1000);
  followLineWithTarget(3);
  delay(500);
  moveShort(300);
  delay(500);
  if (!rotate90()) return;

  // ── Phase 4: final approach and release the object ──
  followLineWithTarget(5);
  delay(1000);
  moveSlowly(3);
  delay(1000);
  alignOnCrossLine();
  openGripper(isGripperOpen);
  delay(5000);
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
  delay(500);
  moveShort(150);
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
  followLineWithDistance();
  if (stopAll) return;

  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  delay(5000);
  isGripperOpen = !isGripperOpen;

  // ── Phase 3: back off and navigate to the drop lane ──
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
    // case 22: // challenge 1
    //   returnToCheckpoint();
    //   if (!searchAndCenterLine()) return;
    //   followLineWithTarget(3);
    //   break;

    // case 25: // challenge 2
    //   reverseShort(300);
    //   if (!searchAndCenterLine()) return;
    //   break;

    case 13: { // challenge 3
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

    // case 68: // left button
    //   if (!returnToCheckpoint()) return;
    //   if (!searchAndCenterLine()) return;
    //   delay(1000);
    //   followLineWithTarget(2);
    //   strafeLeft(2);
    //   if (!searchAndCenterLine()) return;
    //   followLineWithDistance();
    //   break;

    // case 70: // front button / stop everything
    //   stopEverything();
    //   break;

    // case 12: // number 4
    //   path2();
    //   break;

    // case 24: // button 5
    //   path3();
    //   break;

    // case 94:
    //   returnToCheckpoint();
    //   break;

    // case 67: // right button
    //   rotate90();
    //   break;

    // case 82: { // Button to test out code - gripper
    //   static bool gripperOpen = false;
    //   gripperOpen = !gripperOpen;
    //   openGripper(gripperOpen);
    //   break;
    // }

    // case 74: // Test for ultrasonic sensor
    //   Serial.println(F("Ultrasonic reading started..."));
    //   while (IRreceive.getKey() != 70) {
    //     logEsp32Messages();
    //     if (pendingEspKey == 70 || pendingEspKey == STOP_KEY) {
    //       pendingEspKey = -1;
    //       break;
    //     }

    //     uint16_t dist = readUltrasonic();
    //     if (dist >= 999)
    //       Serial.println(F("Distance: out of range"));
    //     else {
    //       Serial.print(F("Distance: "));
    //       Serial.print(dist);
    //       Serial.println(F(" cm"));
    //     }
    //     delay(250);
    //   }
    //   Serial.println(F("Ultrasonic stopped."));
    //   break;

    // case 28: // test run robot
    //   fetchBlue();
    //   break;

    case 90: // number 6 — drive forward at the production per-wheel speeds (see WHEEL_SPEED_* constants)
      moveForwardWheelSpeeds(WHEEL_SPEED_UPPER_L, WHEEL_SPEED_LOWER_L, WHEEL_SPEED_UPPER_R, WHEEL_SPEED_LOWER_R);
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
  bool wifiConnected = connectEsp32Wifi();
  robot.right_led(wifiConnected);
  robot.left_led(wifiConnected);
  if (wifiConnected) statusLedOffAtMs = millis() + 2000;
  servo.attach(SERVO_PIN);
  servo.write(gripperAngle);
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
