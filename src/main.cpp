// Color Sensor Reader - TCS3200 (4-pin multiplexer)
// Reads RGB values from a color sensor and outputs normalized values to Serial
// Used for line tracking and color detection in the Mecanum robot

#include <Arduino.h>

#define S2  7
#define S3  6
#define OUT 8
// S0 = GND (fixed LOW)
// S1 = 5V (fixed HIGH)
// Scaling = 2% (LOW, HIGH)

void setup() {
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  Serial.begin(9600);
}

int readColor(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(10);  // Let filter settle
  return pulseIn(OUT, LOW, 100000);  // Pulse duration in µs
}

// Fill these after your calibration
int whiteR = 949, whiteG = 967, whiteB = 881;  // from white surface
int blackR = 5109, blackG = 4806, blackB = 4189;  // measure this yourself

void loop() {
  int rawR = readColor(LOW,  LOW);
  int rawG = readColor(HIGH, HIGH);
  int rawB = readColor(LOW,  HIGH);

  // Map: white surface = 255, black surface = 0
  int r = constrain(map(rawR, blackR, whiteR, 0, 255), 0, 255);
  int g = constrain(map(rawG, blackG, whiteG, 0, 255), 0, 255);
  int b = constrain(map(rawB, blackB, whiteB, 0, 255), 0, 255);

  Serial.print("R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.println(b);

  delay(500);
}