#include <Arduino.h>
#include <MecanumCar_v2.h>

mecanumCar robot(3, 2);  // SDA=D3, SCL=D2

const uint8_t ECHO_PIN = 13;
const uint8_t TRIG_PIN = 12;

const uint8_t DEFAULT_SPEED = 50;        // 0-255
const uint8_t OBSTACLE_DISTANCE_CM = 40; // Rotate when an object is this close.
const uint16_t TURN_90_MS = 350;         // Tune this for a physical 90 degree turn.
const uint16_t LOG_INTERVAL_MS = 250;

unsigned long lastLogMs = 0;

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

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
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

void rotateRight90() {
  setSpeed(DEFAULT_SPEED);
  robot.Stop();
  delay(100);
  robot.Turn_Right();
  delay(TURN_90_MS);
  robot.Stop();
  delay(200);
}

void setup() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  Serial.begin(9600);
  robot.Init();
  setSpeed(DEFAULT_SPEED);
  delay(1000);
}

void loop() {
  uint16_t distanceCm = getDistanceCm();
  bool obstacleNearby = distanceCm <= OBSTACLE_DISTANCE_CM;

  logDistance(distanceCm, obstacleNearby);

  if (obstacleNearby) {
    rotateRight90();
  } else {
    setSpeed(DEFAULT_SPEED);
    robot.Advance();
  }
}
