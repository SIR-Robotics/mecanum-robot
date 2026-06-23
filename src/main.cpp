#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <Servo.h>
#include "ir.h"

extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

mecanumCar robot(3, 2);
Servo servo;
IR IRreceive(A3);

const uint8_t TRIG_PIN = 12;
const uint8_t ECHO_PIN = 13;
const uint8_t SERVO_PIN = 9;

const uint8_t COLOR_S2 = 7;
const uint8_t COLOR_S3 = 6;
const uint8_t COLOR_OUT = 8;

const uint8_t TEST_SPEED = 80;
const uint8_t SERVO_CLOSED = 0;
const uint8_t SERVO_OPEN = 180;
const uint16_t US_TIMEOUT = 12000;

#define SensorLeft    A0
#define SensorMiddle  A1
#define SensorRight   A2

bool testUltrasonic() {
  Serial.print("Ultrasonic... ");

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoStart = 0;
  unsigned long trigTime = micros();

  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - trigTime > US_TIMEOUT) {
      Serial.println("FAIL (no echo start)");
      return false;
    }
  }
  echoStart = micros();

  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - echoStart > US_TIMEOUT) {
      Serial.println("FAIL (no echo end)");
      return false;
    }
  }

  uint16_t distance = (micros() - echoStart) / 58.2;
  Serial.print("OK (");
  Serial.print(distance);
  Serial.println(" cm)");
  return true;
}

int readColor(bool s2, bool s3) {
  digitalWrite(COLOR_S2, s2);
  digitalWrite(COLOR_S3, s3);
  delay(10);
  return pulseIn(COLOR_OUT, LOW, 100000);
}

bool testColorSensor() {
  Serial.print("Color sensor... ");

  int rawR = readColor(LOW, LOW);
  int rawG = readColor(HIGH, HIGH);
  int rawB = readColor(LOW, HIGH);

  if (rawR == 0 || rawG == 0 || rawB == 0) {
    Serial.println("FAIL (readings zero)");
    return false;
  }

  Serial.print("OK (R:");
  Serial.print(rawR);
  Serial.print(" G:");
  Serial.print(rawG);
  Serial.print(" B:");
  Serial.print(rawB);
  Serial.println(")");
  return true;
}

bool testMotor(const char* name, void (mecanumCar::*func)()) {
  Serial.print("Motor ");
  Serial.print(name);
  Serial.print("... ");

  (robot.*func)();
  delay(800);
  robot.Stop();
  delay(100);

  Serial.println("OK");
  return true;
}

bool testMotors() {
  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TEST_SPEED;

  if (!testMotor("spin left", &mecanumCar::Turn_Left)) return false;
  if (!testMotor("spin right", &mecanumCar::Turn_Right)) return false;

  return true;
}

bool testServo() {
  Serial.print("Servo... ");

  servo.write(SERVO_OPEN);
  delay(500);
  servo.write(SERVO_CLOSED);
  delay(500);

  Serial.println("OK");
  return true;
}

bool testIr() {
  Serial.print("IR (line trackers)... ");

  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);

  bool sl = digitalRead(SensorLeft);
  bool sm = digitalRead(SensorMiddle);
  bool sr = digitalRead(SensorRight);

  Serial.print("L:");
  Serial.print(sl);
  Serial.print(" M:");
  Serial.print(sm);
  Serial.print(" R:");
  Serial.print(sr);
  Serial.print(" | IR remote... ");

  int key = IRreceive.getKey();
  if (key != -1) {
    Serial.print("OK (key=");
    Serial.print(key);
    Serial.println(")");
  } else {
    Serial.println("OK (no signal, idle)");
  }
  return true;
}

void blinkDuring(unsigned long durationMs) {
  unsigned long start = millis();
  bool ledOn = false;
  while (millis() - start < durationMs) {
    ledOn = !ledOn;
    robot.right_led(ledOn);
    robot.left_led(ledOn);
    delay(100);
  }
  robot.right_led(0);
  robot.left_led(0);
}

void robotDance() {
  Serial.println("\n=== ALL PASSED - ROBOT DANCE ===");

  for (int i = 0; i < 3; i++) {
    Serial.print("Spin #");
    Serial.println(i + 1);

    robot.Turn_Left();
    blinkDuring(1500);
    robot.Stop();
    blinkDuring(300);

    Serial.println("  Open grip");
    servo.write(SERVO_OPEN);
    blinkDuring(400);

    Serial.println("  Close grip");
    servo.write(SERVO_CLOSED);
    blinkDuring(400);

    servo.write(SERVO_OPEN);
    blinkDuring(400);
    servo.write(SERVO_CLOSED);
    blinkDuring(400);

    robot.Turn_Right();
    blinkDuring(1500);
    robot.Stop();
    blinkDuring(300);

    servo.write(SERVO_OPEN);
    blinkDuring(400);
    servo.write(SERVO_CLOSED);
    blinkDuring(400);
    servo.write(SERVO_OPEN);
    blinkDuring(400);
    servo.write(SERVO_CLOSED);
    blinkDuring(400);
  }

  Serial.println("Dance complete!");
}

void mechanichalShiver() {
  const uint8_t PULSE_MS = 150;

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TEST_SPEED;

  Serial.println("Motor pulse test...");

  robot.Advance();    delay(PULSE_MS);
  robot.Stop();       delay(80);

  robot.Back();       delay(PULSE_MS);
  robot.Stop();       delay(80);

  robot.Turn_Left();  delay(PULSE_MS);
  robot.Stop();       delay(80);

  robot.Turn_Right(); delay(PULSE_MS);
  robot.Stop();       delay(80);

  Serial.println("Motor pulse test OK");
  Serial.print("Flushing IR buffer... ");

  for (int i = 0; i < 10; i++) {
    IRreceive.getKey();
    delay(10);
  }

  Serial.println("OK");
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  Serial.begin(9600);
  Serial.println("=== ROBOT INIT SELF-CHECK ===\n");

  robot.Init();
  delay(1000);

  speed_Upper_L = speed_Lower_L = speed_Upper_R = speed_Lower_R = TEST_SPEED;

  servo.attach(SERVO_PIN);
  servo.write(SERVO_CLOSED);
  delay(500);

  bool allOk = true;

  allOk &= testUltrasonic();
  delay(200);

  allOk &= testColorSensor();
  delay(200);

  allOk &= testMotors();
  delay(200);

  allOk &= testServo();
  delay(200);

  allOk &= testIr();
  delay(200);

  Serial.println();

  if (allOk) {
    Serial.println("=== ALL TESTS PASSED ===");
    // robotDance();
  } else {
    Serial.println("=== INIT FAILED - CHECK WIRING ===");
  }
}

void loop() {
  mechanichalShiver();
}