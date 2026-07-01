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

const uint8_t  TURNING_SPEED        = 50;
const uint16_t ROTATE180_MS         = 1000; // tune manually — how long TURNING_SPEED gives you 180°
const uint8_t  CORRECTION_SPEED     = 25;   // used by strafeLeft
const uint8_t  LEFT_SPEED           = 40;   // advance speed, left side
const uint8_t  RIGHT_SPEED          = 42;   // advance speed, right side (trim for asymmetry)
const uint8_t  LINE_TURN_SPEED      = 25;   // spin speed for line-follow corrections
const uint8_t  LINE_TICK_MS         = 10;   // follow-loop tick delay
const uint16_t OBSTACLE_DISTANCE_CM = 6;
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


// ─────────────────────────────────────────────────────────────────────────────

// Forward declarations (defined further down)
bool stopRequested();
bool waitOrStop(uint16_t ms);
void stopEverything();

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

void strafeLeft(int ms) {
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = CORRECTION_SPEED;
  robot.L_Move();
  delay(ms);
  robot.Stop();
}

// ─────────────────────────────────────────────────────────────────────────────

// ── Basic 3-sensor line-follow (KS0560 lesson_6 pattern) ─────────────────────
// Advance when middle sensor is on the line and sides are clear.
// Spin toward whichever side sensor picks up the line.
// Count all-black as a full line crossing; stop when count hits targetCount.
void followLineWithTarget(int targetCount) {
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
}

// ─────────────────────────────────────────────────────────────────────────────

// Simple timed 180° right turn. Tune ROTATE180_MS at the top of the file
// until the robot lands facing 180° reliably. Returns false if STOP is
// pressed mid-turn, true otherwise.
bool rotate180() {
  Serial.println("Rotating 180 degrees...");
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TURNING_SPEED;
  robot.Turn_Right();
  bool stopped = waitOrStop(ROTATE180_MS);
  robot.Stop();
  return !stopped;
}

// ── Servo Gripper ─────────────────────────────────────────────────────────

void openGripper(bool state) {
  if (state) {
    servo.write(180);
    Serial.println("Gripper: close");
  } else {
    servo.write(30);
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

void gripAndIdentifyColor(bool gOpen) {
  if (stopRequested()) return;

  Serial.println("Checking TCS3200 color...");
  ColorLabel label = classifyColor();

  delay(1000);
  // servo.write(180);
  openGripper(gOpen);

  if (label == ColorLabel::Blue) {
    Serial.println("Blue detected. Gripper closed at 180.");
  } else if (label == ColorLabel::Red) {
    Serial.println("Red detected. Gripper closed at 180.");
  } else {
    Serial.println("Yellow detected. Gripper closed at 180.");
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
// Same tracking logic as followLineWithTarget; runs until an obstacle is
// detected within OBSTACLE_DISTANCE_CM or STOP is pressed.
void followLineWithDistance() {
  Serial.println("Following line with distance check...");

  UltrasonicReading ultrasonic = {
    ULTRASONIC_IDLE,
    millis() - ULTRASONIC_SAMPLE_MS,
    0,
    999,
    false
  };

  unsigned long lastIRCheckMs      = 0;
  unsigned long lastDistanceReportMs = millis() - 250;

  while (true) {
    unsigned long now = millis();
    if (now - lastIRCheckMs >= 50) {
      lastIRCheckMs = now;
      if (stopRequested()) return;
    }

    updateUltrasonic(ultrasonic);
    if (ultrasonic.sampleReady) {
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

    uint8_t SL = digitalRead(LINE_LEFT_PIN);
    uint8_t SM = digitalRead(LINE_MIDDLE_PIN);
    uint8_t SR = digitalRead(LINE_RIGHT_PIN);

    // Basic 3-sensor tracking. Fallback = Advance so brief sensor blips
    // don't stall the robot; ultrasonic stop is what ends this function.
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
}

bool searchAndCenterLine(uint16_t timeoutMs = 0) {
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
// 1. fast turn left
// 2. fast turn right longer
// 3. fast turn left longer
// 4. creep forward slightly
// repeat
unsigned long stepElapsed = millis() - searchStepStartMs;

speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = LINE_SEARCH_SPEED;

if (searchStep == 0) {
  robot.Turn_Left();

  if (stepElapsed >= 350) {
    searchStep = 1;
    searchStepStartMs = millis();
  }
} 
else if (searchStep == 1) {
  robot.Turn_Right();

  if (stepElapsed >= 800) {
    searchStep = 2;
    searchStepStartMs = millis();
  }
} 
else if (searchStep == 2) {
  robot.Turn_Left();

  if (stepElapsed >= 800) {
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
  servo.write(30);
  // delay(1000);
  Serial.println("Ready. Press IR to start.");
}

void loop() {
  int key = IRreceive.getKey();

  // challenge 1
  if (key == 22) {
    stopAll = false;
    followLineWithTarget(7);
  }

  // challenge 2
  if (key == 25) {
    stopAll = false;
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Following 2 lines...");
    followLineWithTarget(2);
    strafeLeft(1000);
    followLineWithTarget(4);
    if (!rotate180()) return;
    followLineWithTarget(3);
    robot.right_led(true);
    robot.left_led(true);
    Serial.println("Done.");
  }

  // challenge 3
  if (key == 13) {
    stopAll = false;
    followLineWithDistance();
    if (stopAll) return;
    gripAndIdentifyColor(isGripperOpen);
    if (waitOrStop(5000)) return;
    isGripperOpen = !isGripperOpen;
    if (!rotate180()) return;

    if (!searchAndCenterLine()) return;

    followLineWithTarget(7);
    if (stopAll) return;
    openGripper(isGripperOpen);
    if (waitOrStop(5000)) return;
    isGripperOpen = !isGripperOpen;
  }

  if (key == 12) {
    stopAll = false;
    gripAndIdentifyColor(isGripperOpen);
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
    stopAll = true;
    robot.right_led(false);
    robot.left_led(false);
    Serial.println("Stopped.");
  }

}