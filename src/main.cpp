#include <Arduino.h>
#include <MecanumCar_v2.h>

mecanumCar robot(3, 2);  // SDA=D3, SCL=D2

const uint8_t LINE_LEFT_PIN = A0;
const uint8_t LINE_MIDDLE_PIN = A1;
const uint8_t LINE_RIGHT_PIN = A2;

const uint8_t DEFAULT_SPEED = 50;
const uint8_t CORRECTION_SPEED = 60;
const uint16_t LINE_LOG_INTERVAL_MS = 200;

unsigned long lastLineLogMs = 0;

void setSpeed(uint8_t speed) {
  speed_Upper_L = speed_Lower_L = speed;
  speed_Upper_R = speed_Lower_R = speed;
}

void logLineSensors(uint8_t sl, uint8_t sm, uint8_t sr) {
  unsigned long now = millis();
  if (now - lastLineLogMs < LINE_LOG_INTERVAL_MS) {
    return;
  }
  Serial.print("Left:");
  Serial.print(sl);
  Serial.print("  Middle:");
  Serial.print(sm);
  Serial.print("  Right:");
  Serial.println(sr);
  lastLineLogMs = now;
}

void handleLineTracking() {
  uint8_t sl = digitalRead(LINE_LEFT_PIN);
  uint8_t sm = digitalRead(LINE_MIDDLE_PIN);
  uint8_t sr = digitalRead(LINE_RIGHT_PIN);

  logLineSensors(sl, sm, sr);

  if (sm == 1 && sl == 1 && sr == 1) { // All line detected 
    setSpeed(DEFAULT_SPEED);
    robot.Advance();
  } else if (sm == 1 && sl == 0 && sr == 0) { // Mid line only
    setSpeed(DEFAULT_SPEED);
    robot.Advance();
  } else if (sl == 1 && sm == 0 && sr == 0) { // Left only
    setSpeed(CORRECTION_SPEED);
    robot.L_Move();
  } else if (sr == 1 && sl == 0) {
    setSpeed(CORRECTION_SPEED);
    robot.R_Move();
  } else {
    robot.Stop();
  }
}

void setup() {
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_MIDDLE_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);

  Serial.begin(9600);
  robot.Init();
  setSpeed(DEFAULT_SPEED);
  delay(1000);
}

void loop() {
  handleLineTracking();
}
