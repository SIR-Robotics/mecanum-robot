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

const uint8_t  TURNING_SPEED        = 42;
const uint16_t ROTATE180_MS         = 1000; // tune manually — how long TURNING_SPEED gives you 180°
const uint8_t  CORRECTION_SPEED     = 25;   // used by strafeLeft
const uint8_t  LEFT_SPEED           = 40;   // advance speed, left side
const uint8_t  RIGHT_SPEED          = 42;   // advance speed, right side (trim for asymmetry)
const uint8_t  LINE_TURN_SPEED      = 25;   // spin speed for line-follow corrections
const uint8_t  SLOW_LEFT_SPEED      = 30;
const uint8_t  SLOW_RIGHT_SPEED     = 32;
const uint8_t  SLOW_LINE_TURN_SPEED = 28;
const uint8_t  LINE_TICK_MS         = 10;   // follow-loop tick delay
const uint16_t OBSTACLE_DISTANCE_CM = 6;
const uint16_t APPROACH_DISTANCE_CM = 20;
const uint16_t ULTRASONIC_SAMPLE_MS = 50;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;
const uint8_t  LINE_SEARCH_SPEED    = 40;   // aggressive turning when line is lost

bool isGripperOpen = true; // true = open, false = closed

// Color sensor calibration (white=255, black=0)
int whiteR = 949, whiteG = 967, whiteB = 881;
int blackR = 5109, blackG = 4806, blackB = 4189;

int           numLines        = 0;
bool          wasOnFullLine   = false;
bool          stopAll         = false;

volatile unsigned long echoRiseUs = 0;
volatile unsigned long echoDurationUs = 0;
volatile bool echoDurationReady = false;

const uint8_t  STOP_KEY             = 64;

struct RGB { uint8_t r, g, b; };

enum class ColorLabel { Red, Blue, Yellow };

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

// ── Color Sensor ──────────────────────────────────────────────────────────────


// ─────────────────────────────────────────────────────────────────────────────

// Forward declarations (defined further down)
bool stopRequested();
bool waitOrStop(uint16_t ms);
void stopEverything();
bool searchAndCenterLine(uint16_t timeoutMs = 0);
bool rotate90(uint8_t turnSpeed = TURNING_SPEED, uint16_t timeoutMs = 2500);
void moveSlowlyToObject();

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

void turnRight90() {
  Serial.println("Turning right 90 degrees...");
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TURNING_SPEED;
  robot.Advance();
  delay(300);
  robot.Turn_Right();
  delay(500);
  robot.Stop();
  robot.L_Move();
  delay(400);
  robot.Stop();
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

void strafeRight(int ms) {
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = 60;
  robot.R_Move();
  delay(ms);
  robot.Stop();
}

// ─────────────────────────────────────────────────────────────────────────────

// ── Basic 3-sensor line-follow (KS0560 lesson_6 pattern) ─────────────────────
// Advance when middle sensor is on the line and sides are clear.
// Spin toward whichever side sensor picks up the line.
// Count all-black as a full line crossing; stop when count hits targetCount.
void followLineWithTarget(int targetCount, uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed) {
  numLines      = 0;
  wasOnFullLine = false;

  unsigned long lastIRCheckMs = 0;

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

    // Basic 3-sensor tracking (from KS0560 lesson_6). HIGH = black, LOW = white.
    // Fallback in every ambiguous state = Advance, so a single-tick sensor
    // flicker never stalls the robot mid-track.
    if (SL == LOW && SR == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
      robot.Turn_Right();
    } else if (SR == LOW && SL == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
      robot.Turn_Left();
    } else {
      speed_Upper_L = speed_Lower_L = leftSpeed;
      speed_Upper_R = speed_Lower_R = rightSpeed;
      robot.Advance();
    }

    delay(LINE_TICK_MS);
  }
}

void followLineWithTarget(int targetCount) {
  followLineWithTarget(targetCount, LEFT_SPEED, RIGHT_SPEED, LINE_TURN_SPEED);
}

bool followLineForMs(uint16_t ms) {
  unsigned long start = millis();
  unsigned long lastIRCheckMs = 0;

  while (millis() - start < ms) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return false;
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    if (SL == LOW && SR == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_TURN_SPEED;
      robot.Turn_Right();
    } else if (SR == LOW && SL == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_TURN_SPEED;
      robot.Turn_Left();
    } else {
      speed_Upper_L = speed_Lower_L = LEFT_SPEED;
      speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
      robot.Advance();
    }

    delay(LINE_TICK_MS);
  }

  robot.Stop();
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

bool rotate180(int ms) {
  (void)ms;
  Serial.println("Rotating 180 degrees...");
  return rotate90() && rotate90();
}

bool rotate90(uint8_t turnSpeed, uint16_t timeoutMs) {
  Serial.println("Rotating 90 degrees until right sensor detects line...");

  unsigned long startMs = millis();

  speed_Upper_L = speed_Lower_L = speed_Lower_R = turnSpeed;
  speed_Upper_R = 44;

  while (millis() - startMs < timeoutMs) {
    if (stopRequested()) {
      robot.Stop();
      return false;
    }

    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    if (SR == HIGH) {
      robot.Stop();
      Serial.println("Right sensor detected line.");
      return searchAndCenterLine();
    }

    robot.Turn_Right();
    delay(LINE_TICK_MS);
  }
  robot.Stop();
  Serial.println("Rotate 90 timeout. Right sensor did not detect line.");
  return false;
}

// ── Servo Gripper ─────────────────────────────────────────────────────────

void openGripper(bool state) {
  if (state) {
    servo.write(180);
    Serial.println("Gripper: close");
  } else {
    servo.write(2);
    Serial.println("Gripper: open");
  }
}

// - color detection --------------------------------------------

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

// void gripAndIdentifyColor(bool gOpen) {
//   if (stopRequested()) return;

//   Serial.println("Checking TCS3200 color...");
//   ColorLabel label = classifyColor();

//   delay(1000);
//   // servo.write(180);
//   openGripper(gOpen);

//   if (label == ColorLabel::Blue) {
//     Serial.println("Blue detected. Gripper closed at 180.");
//   } else if (label == ColorLabel::Red) {
//     Serial.println("Red detected. Gripper closed at 180.");
//   } else {
//     Serial.println("Yellow detected. Gripper closed at 180.");
//   }
// }

int gripAndIdentifyColor(bool gOpen) {
  if (stopRequested()) return -1;

  Serial.println("Checking TCS3200 color...");
  ColorLabel label = classifyColor();

  // servo.write(180);
  openGripper(gOpen);
  delay(1000);

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



// ── Ultrasonic Sensor ─────────────────────────────────────────────────────

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

enum UltrasonicState : uint8_t {
  ULTRASONIC_IDLE,
  ULTRASONIC_TRIGGER_LOW,
  ULTRASONIC_TRIGGER_HIGH,
  ULTRASONIC_WAIT_ECHO
};

struct UltrasonicReading {
  UltrasonicState state;
  unsigned long   lastSampleMs;
  unsigned long   stateStartUs;
  uint16_t        distanceCm;
  bool            sampleReady;
};

// Advances one ultrasonic measurement without blocking line tracking.
void updateUltrasonic(UltrasonicReading &sensor) {
  unsigned long nowUs = micros();
  sensor.sampleReady = false;

  switch (sensor.state) {
    case ULTRASONIC_IDLE:
      if (millis() - sensor.lastSampleMs >= ULTRASONIC_SAMPLE_MS) {
        sensor.lastSampleMs = millis();
        sensor.stateStartUs = nowUs;
        digitalWrite(TRIG_PIN, LOW);
        sensor.state = ULTRASONIC_TRIGGER_LOW;
      }
      break;

    case ULTRASONIC_TRIGGER_LOW:
      if (nowUs - sensor.stateStartUs >= 2) {
        digitalWrite(TRIG_PIN, HIGH);
        sensor.stateStartUs = nowUs;
        sensor.state = ULTRASONIC_TRIGGER_HIGH;
      }
      break;

    case ULTRASONIC_TRIGGER_HIGH:
      if (nowUs - sensor.stateStartUs >= 10) {
        noInterrupts();
        echoRiseUs = 0;
        echoDurationReady = false;
        interrupts();
        digitalWrite(TRIG_PIN, LOW);
        sensor.stateStartUs = nowUs;
        sensor.state = ULTRASONIC_WAIT_ECHO;
      }
      break;

    case ULTRASONIC_WAIT_ECHO: {
      noInterrupts();
      bool echoReady = echoDurationReady;
      unsigned long durationUs = echoDurationUs;
      echoDurationReady = false;
      interrupts();

      if (echoReady) {
        sensor.distanceCm = durationUs / 58.2;
        sensor.sampleReady = true;
        sensor.state = ULTRASONIC_IDLE;
      } else if (nowUs - sensor.stateStartUs >= ULTRASONIC_TIMEOUT_US) {
        noInterrupts();
        echoRiseUs = 0;
        interrupts();
        sensor.distanceCm = 999;
        sensor.sampleReady = true;
        sensor.state = ULTRASONIC_IDLE;
      }
      break;
    }
  }
}

// ── Line follow with ultrasonic obstacle stop ─────────────────────────────

// ── Basic 3-sensor follow with ultrasonic obstacle stop ──────────────────────
bool followLineWithDistance(uint8_t leftSpeed, uint8_t rightSpeed, uint8_t turnSpeed, uint16_t stopDistanceCm) {
  Serial.println("Following line with distance check...");

  unsigned long lastIRCheckMs      = 0;
  unsigned long lastDistanceSampleMs = millis() - ULTRASONIC_SAMPLE_MS;
  unsigned long lastDistanceReportMs = millis() - 250;
  uint16_t distanceCm = 999;

  while (true) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return false;
    }

    if (now - lastDistanceSampleMs >= ULTRASONIC_SAMPLE_MS) {
      lastDistanceSampleMs = now;
      distanceCm = readUltrasonic();

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
        robot.Stop();
        Serial.print("Obstacle detected at ");
        Serial.print(distanceCm);
        Serial.println(" cm. Stopping.");
        return true;
      }
    }

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    // Basic 3-sensor tracking. Fallback = Advance so brief sensor blips
    // don't stall the robot; ultrasonic stop is what ends this function.
    if (SL == LOW && SR == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
      robot.Turn_Right();
    } else if (SR == LOW && SL == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = turnSpeed;
      robot.Turn_Left();
    } else {
      speed_Upper_L = speed_Lower_L = leftSpeed;
      speed_Upper_R = speed_Lower_R = rightSpeed;
      robot.Advance();
    }

    delay(LINE_TICK_MS);
  }
}

// Same tracking logic as followLineWithTarget; runs until an obstacle is
// detected within OBSTACLE_DISTANCE_CM or STOP is pressed.
void followLineWithDistance() {
  if (followLineWithDistance(LEFT_SPEED, RIGHT_SPEED, LINE_TURN_SPEED, APPROACH_DISTANCE_CM)) {
    delay(1000);
    moveSlowlyToObject();
  }
}

void moveSlowlyToObject() {
  followLineWithDistance(SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED, OBSTACLE_DISTANCE_CM);
}

void robotReverse(int ms) {
  speed_Upper_L = speed_Lower_L = LEFT_SPEED;
  speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
  robot.Back();
  delay(ms);
  robot.Stop();
}

bool searchAndCenterLine(uint16_t timeoutMs) {
  Serial.println("Searching and centering on line...");

  const uint8_t CENTER_CONFIRM_TICKS = 5;
  uint8_t centeredTicks = 0;

  unsigned long startMs = millis();
  unsigned long searchStepStartMs = millis();

  uint8_t searchStep = 0;

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

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    // Perfect: middle sensor is on black, left/right are white
    if (SL == LOW && SM == HIGH && SR == LOW) {
      robot.Stop();
      centeredTicks++;

      if (centeredTicks >= CENTER_CONFIRM_TICKS) {
        Serial.println("Line found and centered.");
        return true;
      }

      delay(LINE_TICK_MS);
      continue;
    }

    centeredTicks = 0;

    // Any sensor sees black: correct toward center
    if (SL == HIGH || SM == HIGH || SR == HIGH) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_TURN_SPEED;

      if (SL == HIGH && SM == LOW && SR == LOW) {
        // Line is on left
        robot.Turn_Left();
      } 
      else if (SL == HIGH && SM == HIGH && SR == LOW) {
        // Line is slightly left
        robot.Turn_Left();
      } 
      else if (SL == LOW && SM == LOW && SR == HIGH) {
        // Line is on right
        robot.Turn_Right();
      } 
      else if (SL == LOW && SM == HIGH && SR == HIGH) {
        // Line is slightly right
        robot.Turn_Right();
      } 
      else if (SL == HIGH && SM == HIGH && SR == HIGH) {
        // On a crossing / thick line, move forward slowly until it becomes normal
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      } 
      else {
        // Middle detects black, but not stable yet
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      }

      delay(LINE_TICK_MS);
      continue;
    }

 // All sensors are white: line is lost.
// Aggressive search pattern:
// 1. fast turn right
// 2. brief turn left if it overshot
// 3. fast turn right longer
// 4. creep forward slightly
// repeat
unsigned long stepElapsed = millis() - searchStepStartMs;

speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_SEARCH_SPEED;

if (searchStep == 0) {
  robot.Turn_Right();

  if (stepElapsed >= 500) {
    searchStep = 1;
    searchStepStartMs = millis();
  }
} 
else if (searchStep == 1) {
  robot.Turn_Left();

  if (stepElapsed >= 250) {
    searchStep = 2;
    searchStepStartMs = millis();
  }
} 
else if (searchStep == 2) {
  robot.Turn_Right();

  if (stepElapsed >= 1000) {
    searchStep = 3;
    searchStepStartMs = millis();
  }
} 
else {
  // Small forward push in case robot is just behind/beside the line
  speed_Upper_L = speed_Lower_L = 30;
  speed_Upper_R = speed_Lower_R = 30;
  robot.Advance();

  if (stepElapsed >= 250) {
    searchStep = 0;
    searchStepStartMs = millis();
  }
}

delay(LINE_TICK_MS);
  }
}

bool returnToCheckpoint() {
  robotReverse(500);
  if (!rotate180(700)) return false;
  return searchAndCenterLine();
}

void moveSlowly(int targetCount) {
  followLineWithTarget(targetCount, SLOW_LEFT_SPEED, SLOW_RIGHT_SPEED, SLOW_LINE_TURN_SPEED);
}

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
  servo.attach(SERVO_PIN);
  servo.write(30);
  // delay(1000);
  Serial.println("Ready. Press IR to start.");
}

void path1() {
  followLineWithDistance();
  if (stopAll) return;
  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (waitOrStop(5000)) return;
  delay(300);
  isGripperOpen = !isGripperOpen;

  if (!rotate180(800)) return;
  if (!searchAndCenterLine()) return;

  followLineWithTarget(5);
  delay(1000);
  if (stopAll) return;
  moveSlowly(2);
  delay(1000);
  openGripper(isGripperOpen);
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;
  sendArmCommand(colorRes);
  if (!returnToCheckpoint()) return;
  if (!searchAndCenterLine()) return;
  followLineWithTarget(2);
}

void path2() {
  strafeLeft(2);
  if (!searchAndCenterLine()) return;
  followLineWithDistance();
  if (stopAll) return;

  int colorRes = gripAndIdentifyColor(isGripperOpen);
  if (colorRes < 0) return;
  if (waitOrStop(1000)) return;
  isGripperOpen = !isGripperOpen;

  if (!rotate180(100)) return;
  if (!searchAndCenterLine()) return;
  followLineWithTarget(2);
  delay(500);
  if (stopAll) return;
  strafeLeft(3);
  delay(500);
  if (!searchAndCenterLine()) return;
  followLineWithTarget(3);
  delay(1000);
  moveSlowly(2);
  if (stopAll) return;
  openGripper(isGripperOpen);
  if (waitOrStop(5000)) return;
  isGripperOpen = !isGripperOpen;
  if (!returnToCheckpoint()) return;
  if (!searchAndCenterLine()) return;
  followLineWithTarget(2);
}

void loop() {
  logEsp32Messages();

  int key = IRreceive.getKey();

  switch (key) {
    case 22: // challenge 1
      stopAll = false;
      followLineWithTarget(7);
      break;

    case 25: // challenge 2
      stopAll = false;
      robot.right_led(false);
      robot.left_led(false);
      Serial.println("Following 2 lines...");
      followLineWithTarget(2);
      strafeLeft(2);
      followLineWithTarget(4);
      if (!rotate180(1000)) return;
      followLineWithTarget(3);
      robot.right_led(true);
      robot.left_led(true);
      Serial.println("Done.");
      break;

    case 13: { // challenge 3
      stopAll = false;
      path1();

      // // path 3 -- start
      // strafeRight(800);
      // if (!searchAndCenterLine()) return;
      // followLineWithDistance();
      // if (stopAll) return;
      // colorRes = gripAndIdentifyColor(isGripperOpen);
      // if (colorRes < 0) return;
      // if (waitOrStop(5000)) return;
      // isGripperOpen = !isGripperOpen;

      // if (!rotate180(1000)) return;
      // if (!searchAndCenterLine()) return;
      // followLineWithTarget(2);
      // if (stopAll) return;
      // strafeRight(800);
      // if (!searchAndCenterLine()) return;
      // followLineWithTarget(5);
      // if (stopAll) return;
      // openGripper(isGripperOpen);
      // if (waitOrStop(5000)) return;
      // isGripperOpen = !isGripperOpen;
      // // sendArmCommand(colorRes);
      // if (!returnToCheckpoint()) return;
      // // path 3 -- end

      robot.Stop();
      Serial.println("Done.");
      break;
    }

    case 68: // left button
      if (!returnToCheckpoint()) return;
      if (!searchAndCenterLine()) return;
      // if (!followLineForMs(500)) return;
      delay(1000);
      followLineWithTarget(2);
      strafeLeft(2);
      if (!searchAndCenterLine()) return;
      followLineWithDistance();
      break;

    case 70: // front button / stop everything
      robot.Stop();
      stopAll = true;
      robot.right_led(false);
      robot.left_led(false);
      Serial.println("Stopped.");
      break;

    case 12: // number 4
      // stopAll = false;
      // gripAndIdentifyColor(isGripperOpen);
      path2();
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
