#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <Servo.h>
#include "ir.h"

IR IRreceive(A3);
mecanumCar robot(3, 2);  // SDA=D3, SCL=D2
Servo servo;

const uint8_t ECHO_PIN = 13;
const uint8_t TRIG_PIN = 12;
const uint8_t SERVO_PIN = 9;

const uint8_t DEFAULT_SPEED = 50;        // 0-255
const uint8_t OBSTACLE_DISTANCE_CM = 40; // Rotate when an object is this close.
const uint16_t TURN_90_MS = 350;         // Tune this for a physical 90 degree turn.
const uint16_t POST_TURN_SETTLE_MS = 300;
const uint16_t LOG_INTERVAL_MS = 250;
const uint16_t DISTANCE_CHECK_MS = 100;
const uint16_t IR_COMMAND_COOLDOWN_MS = 250;
const unsigned long ULTRASONIC_TIMEOUT_US = 12000UL;
const uint8_t START_STOP_KEY = 64;
const uint8_t SERVO_KEY = 22;
const uint8_t SERVO_CLOSED_ANGLE = 0;
const uint8_t SERVO_OPEN_ANGLE = 180;

unsigned long lastLogMs = 0;
unsigned long lastDistanceCheckMs = 0;
unsigned long lastIrCommandMs = 0;
unsigned long turnEndMs = 0;
unsigned long settleEndMs = 0;
bool running = false;
bool servoOpen = false;

enum MotionState {
  STOPPED,
  MOVING_FORWARD,
  TURNING_RIGHT
};

MotionState motionState = STOPPED;

void setSpeed(uint8_t speed) {
  speed_Upper_L = speed_Lower_L = speed;
  speed_Upper_R = speed_Lower_R = speed;
}

uint16_t getDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return 999;
  }

  return duration / 58.2;
}

void logDistance(uint16_t distanceCm, bool obstacleNearby) {
  unsigned long now = millis();
  if (now - lastLogMs < LOG_INTERVAL_MS) {
    return;
  }

  Serial.print("Distance: ");
  if (distanceCm >= 999) {
    Serial.print("out of range");
  } else {
    Serial.print(distanceCm);
    Serial.print(" cm");
  }

  Serial.print(" | ");
  Serial.println(obstacleNearby ? "obstacle: rotating" : "clear: moving forward");
  lastLogMs = now;
}

void stopRobot() {
  if (motionState == STOPPED) {
    return;
  }

  robot.Stop();
  motionState = STOPPED;
}

void moveForward() {
  if (motionState == MOVING_FORWARD) {
    return;
  }

  setSpeed(DEFAULT_SPEED);
  robot.Advance();
  motionState = MOVING_FORWARD;
}

void startRightTurn() {
  if (motionState == TURNING_RIGHT) {
    return;
  }

  robot.Stop();
  delay(20);
  setSpeed(DEFAULT_SPEED);
  robot.Turn_Right();
  motionState = TURNING_RIGHT;
  turnEndMs = millis() + TURN_90_MS;
}

void updateTurn() {
  if (motionState != TURNING_RIGHT) {
    return;
  }

  if ((long)(millis() - turnEndMs) >= 0) {
    robot.Stop();
    motionState = STOPPED;
    settleEndMs = millis() + POST_TURN_SETTLE_MS;
    Serial.println("Turn complete: checking distance before moving");
  }
}

void handleIrCommand() {
  int key = IRreceive.getKey();
  if (key == -1) {
    return;
  }

  unsigned long now = millis();
  if (now - lastIrCommandMs < IR_COMMAND_COOLDOWN_MS) {
    return;
  }
  lastIrCommandMs = now;

  Serial.print("IR key: ");
  Serial.println(key);

  if (key == START_STOP_KEY) {
    running = !running;
    if (running) {
      Serial.println("Autonomous mode: started");
    } else {
      stopRobot();
      Serial.println("Autonomous mode: stopped");
    }
  } else if (key == SERVO_KEY) {
    servoOpen = !servoOpen;
    servo.write(servoOpen ? SERVO_OPEN_ANGLE : SERVO_CLOSED_ANGLE);
    Serial.println(servoOpen ? "Servo: open" : "Servo: closed");
  }
}

void setup() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Serial.begin(9600);
  robot.Init();
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CLOSED_ANGLE);
  setSpeed(DEFAULT_SPEED);
  delay(1000);
}

void loop() {
  handleIrCommand();
  updateTurn();

  if (!running) {
    stopRobot();
    return;
  }

  if (motionState == TURNING_RIGHT || (long)(millis() - settleEndMs) < 0) {
    return;
  }

  if (millis() - lastDistanceCheckMs < DISTANCE_CHECK_MS) {
    return;
  }
  lastDistanceCheckMs = millis();

  uint16_t distanceCm = getDistanceCm();
  bool obstacleNearby = distanceCm <= OBSTACLE_DISTANCE_CM;

  logDistance(distanceCm, obstacleNearby);

  if (obstacleNearby) {
    startRightTurn();
  } else {
    moveForward();
  }
}
