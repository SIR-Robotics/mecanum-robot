#include <Arduino.h>
#include <MecanumCar_v2.h>
#include "ir.h"
#include <Servo.h>
#include <avr/interrupt.h>

mecanumCar robot(3, 2);
IR IRreceive(A3);
Servo servo;

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

const uint8_t  CORRECTION_SPEED     = 45;
const uint8_t  TARGET_LINES         = 4;
const uint8_t  LEFT_SPEED           = 45;
const uint8_t  RIGHT_SPEED          = 45;
const uint8_t  NUDGE_SPEED          = 40;
const uint16_t NUDGE_DURATION_MS    = 20;
const uint16_t HUNT_DURATION_MS     = 200;
const uint16_t CROSS_DRIVE_MS       = 500;
const uint16_t CROSS_PAUSE_MS       = 200;
const uint16_t OBSTACLE_DISTANCE_CM = 15;
const uint16_t ULTRASONIC_SAMPLE_MS = 60;
const uint32_t ULTRASONIC_TIMEOUT_US = 12000;

// Color sensor calibration (white=255, black=0)
int whiteR = 949, whiteG = 967, whiteB = 881;
int blackR = 5109, blackG = 4806, blackB = 4189;

unsigned long crossingStartMs = 0;
int           numLines        = 0;
uint8_t       targetLines     = TARGET_LINES;
bool          isRunning       = false;
bool          wasOnFullLine   = false;
bool          isCrossing      = false;

volatile unsigned long echoRiseUs = 0;
volatile unsigned long echoDurationUs = 0;
volatile bool echoDurationReady = false;

// Arduino Uno D13 is PB5 / PCINT5. Capture echo edges even while driving.
ISR(PCINT0_vect) {
  if (PINB & _BV(PB5)) {
    echoRiseUs = micros();
  } else if (echoRiseUs != 0) {
    echoDurationUs = micros() - echoRiseUs;
    echoDurationReady = true;
    echoRiseUs = 0;
  }
}

// ── Color Sensor ──────────────────────────────────────────────────────────────

struct RGB { uint8_t r, g, b; };

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

// ── Servo Gripper ─────────────────────────────────────────────────────────

void openGripper(bool state) {
  if (state) {
    servo.write(180);
    Serial.println("Gripper: open");
  } else {
    servo.write(0);
    Serial.println("Gripper: closed");
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

void followLineWithDistance () {
  Serial.println("Following line with distance check...");

  UltrasonicReading ultrasonic = {
    ULTRASONIC_IDLE,
    millis() - ULTRASONIC_SAMPLE_MS,
    0,
    999,
    false
  };
  bool wasOnFullLine = false;
  bool isCrossing = false;
  unsigned long crossingStartMs = 0;
  uint8_t recoveryState = 0;
  unsigned long recoveryStartMs = 0;
  unsigned long lastDistanceReportMs = millis() - 250;

  while (true) {
    updateUltrasonic(ultrasonic);
    if (ultrasonic.sampleReady) {
      unsigned long now = millis();
      if (now - lastDistanceReportMs >= 250) {
        lastDistanceReportMs = now;
        if (ultrasonic.distanceCm >= 999) {
          Serial.println("Distance: out of range");
        } else {
          Serial.print("Distance: ");
          Serial.print(ultrasonic.distanceCm);
          Serial.println(" cm");
        }
      }

      if (ultrasonic.distanceCm <= OBSTACLE_DISTANCE_CM) {
        robot.Stop();
        Serial.print("Obstacle detected at ");
        Serial.print(ultrasonic.distanceCm);
        Serial.println(" cm. Stopping.");
        return;
      }
    }

    uint8_t sl = digitalRead(LINE_LEFT_PIN);
    uint8_t sm = digitalRead(LINE_MIDDLE_PIN);
    uint8_t sr = digitalRead(LINE_RIGHT_PIN);

    unsigned long now = millis();
    bool allWhite = (sl == 0 && sm == 0 && sr == 0);
    bool allBlack = (sl == 1 && sm == 1 && sr == 1);

    if (isCrossing) {
      if (now - crossingStartMs < CROSS_DRIVE_MS) {
        speed_Upper_L = speed_Lower_L = LEFT_SPEED;
        speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
        robot.Advance();
      } else {
        isCrossing = false;
      }
      continue;
    }

    if (allBlack) {
      if (!wasOnFullLine) {
        wasOnFullLine = true;
        isCrossing = true;
        crossingStartMs = now;
      }
      speed_Upper_L = speed_Lower_L = LEFT_SPEED;
      speed_Upper_R = speed_Lower_R = RIGHT_SPEED;
      robot.Advance();
      continue;
    }

    wasOnFullLine = false;

    if (allWhite) {
      speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = NUDGE_SPEED;

      if (recoveryState == 0) {
        recoveryState = 1;
        recoveryStartMs = now;
      }

      if (recoveryState == 1) {
        robot.Advance();
        if (now - recoveryStartMs >= NUDGE_DURATION_MS) {
          recoveryState = 2;
          recoveryStartMs = now;
        }
      } else if (recoveryState == 2) {
        robot.Turn_Left();
        if (now - recoveryStartMs >= HUNT_DURATION_MS) {
          recoveryState = 3;
          recoveryStartMs = now;
        }
      } else {
        robot.Turn_Right();
        if (now - recoveryStartMs >= HUNT_DURATION_MS) {
          recoveryState = 0;
          robot.Stop();
        }
      }
      continue;
    }

    recoveryState = 0;
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
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT5);

  Serial.begin(9600);
  robot.Init();
  servo.attach(SERVO_PIN);
  servo.write(0);
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

  // challenge 3

  if (key == 13) {
    followLineWithDistance();
    // followLineWithTarget(4);
    // followLine();
  }


  // Test for color sensor
  if (key == 12) {
    Serial.println("Reading color...");
    while (IRreceive.getKey() != 70) {
      readColor();
      delay(500);
    }
    Serial.println("Color read stopped.");
  }

  // Button to test out code - gripper
  if (key == 82) {
    static bool gripperOpen = false;
    gripperOpen = !gripperOpen;
    openGripper(gripperOpen);
  }

  // Test for ultrasonic sensor
  if (key == 74) {
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
