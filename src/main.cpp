/**
 * Mobile Robot Control System
 * Mecanum Car v2 with IR Remote Control + Servo Gripper
 * 
 * Microcontroller: Raspberry Pi Pico
 * Motor Control: via MecanumCar_v2 class
 * Input: IR Remote (RC-like control)
 * Gripper: Servo-controlled
 */

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <Servo.h>
#include "ir.h"

// ─── Hardware Configuration ────────────────────────────────────────────────────
IR IRreceive(A3);              // IR receiver module connected to analog pin A3
mecanumCar robot(3, 2);        // I2C configuration: SDA=pin 3, SCL=pin 2
Servo gripper;                 // Servo for gripper control
const uint8_t SERVO_PIN = 9;   // Servo PWM pin
// ──────────────────────────────────────────────────────────────────────────────

// ─── Calibration & Motor Speed ─────────────────────────────────────────────────
#define DEFAULT_SPEED  150     // Motor PWM speed (0–255, higher = faster)
#define RIGHT_SPEED    150     // Right motors PWM speed
#define LEFT_SPEED     130     // Left motors PWM speed
#define TURN_90_MS     750     // Time (ms) to rotate ~90° at DEFAULT_SPEED
// ──────────────────────────────────────────────────────────────────────────────

// ─── Servo Configuration ──────────────────────────────────────────────────────
#define SERVO_CLOSED_ANGLE  0   // Gripper closed position (degrees)
#define SERVO_OPEN_ANGLE    180 // Gripper open position (degrees)
#define SERVO_TRANSITION_MS 500 // Time for servo to move (ms)
// ──────────────────────────────────────────────────────────────────────────────

// ─── State Management ─────────────────────────────────────────────────────────
bool isLedOn = false;
bool isGripperOpen = false;    // Tracks gripper state
unsigned long lastServoMoveMs = 0;
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Initialization
 * Runs once at startup to configure the robot and communications
 */
void setup() {
  robot.Init();                     // Initialize robot (LEDs off, motors stopped)
  Serial.begin(9600);               // Start serial communication for debugging
  
  // Initialize gripper servo
  gripper.attach(SERVO_PIN);        // Attach servo to pin 9
  gripper.write(SERVO_CLOSED_ANGLE); // Start with gripper closed
  isGripperOpen = false;
  
  delay(2000);                      // Wait for systems to stabilize
  
  Serial.println("Robot initialized: Motors + Gripper ready");
}

/**
 * Toggle Gripper Open/Close
 */
void toggleGripper() {
  unsigned long now = millis();
  
  // Prevent rapid successive commands
  if (now - lastServoMoveMs < SERVO_TRANSITION_MS) {
    return;
  }
  
  lastServoMoveMs = now;
  isGripperOpen = !isGripperOpen;
  
  if (isGripperOpen) {
    gripper.write(SERVO_OPEN_ANGLE);
    Serial.println("Gripper: OPENED");
  } else {
    gripper.write(SERVO_CLOSED_ANGLE);
    Serial.println("Gripper: CLOSED");
  }
}

/**
 * Open Gripper (separate command)
 */
void openGripper() {
  unsigned long now = millis();
  
  if (now - lastServoMoveMs < SERVO_TRANSITION_MS || isGripperOpen) {
    return;
  }
  
  lastServoMoveMs = now;
  isGripperOpen = true;
  gripper.write(SERVO_OPEN_ANGLE);
  Serial.println("Gripper: OPENED");
}

/**
 * Close Gripper (separate command)
 */
void closeGripper() {
  unsigned long now = millis();
  
  if (now - lastServoMoveMs < SERVO_TRANSITION_MS || !isGripperOpen) {
    return;
  }
  
  lastServoMoveMs = now;
  isGripperOpen = false;
  gripper.write(SERVO_CLOSED_ANGLE);
  Serial.println("Gripper: CLOSED");
}

/**
 * Main Loop
 * Continuously monitors IR remote input and executes corresponding motor commands
 * 
 * IR Remote Button Mapping (Standard RC Layout):
 *   ↑ (22)  | ↖ (13)      
 *   ← (12) | OK (64) | → (24)
 *   ↓ (25)      
 *   Additional:
 *   22 = Servo toggle / Open
 *   28 = Close gripper (optional)
 *   90 = Open gripper (optional)
 */
void loop() {
  // Read IR remote button press
  int key = IRreceive.getKey();
  
  // Optional: Log received IR codes for debugging
  if (key != -1) {
    Serial.print("IR Key: ");
    Serial.println(key);
  }

  // Execute motor command based on IR button pressed
  switch (key) {
    // ─── Forward Movement ──────────────────────────────────────────────────
    case 22:  // UP arrow → Move forward
      robot.Motor_Upper_L(1, LEFT_SPEED);   
      robot.Motor_Lower_L(1, LEFT_SPEED);   
      robot.Motor_Upper_R(1, RIGHT_SPEED);   
      robot.Motor_Lower_R(1, RIGHT_SPEED);   
      break;
      
    // ─── Strafe / Side Movement ───────────────────────────────────────────
    case 25:  // DOWN arrow → Strafe right
      robot.R_Move();
      break;
      
    // ─── Diagonal Movement ────────────────────────────────────────────────
    case 13:  // UP-RIGHT diagonal → Move forward-right
      robot.RU_Move();
      break;
      
    // ─── Rotation ──────────────────────────────────────────────────────────
    case 12:  // LEFT arrow → Rotate left (right motors forward, left motors off)
      robot.Motor_Upper_R(0, 0);
      robot.Motor_Lower_R(0, 0);
      robot.Motor_Upper_L(1, DEFAULT_SPEED);
      robot.Motor_Lower_L(1, DEFAULT_SPEED);
      break;
      
    // ─── Circular Motion ──────────────────────────────────────────────────
    case 24:  // RIGHT → Circular motion
      robot.Motor_Upper_L(1, DEFAULT_SPEED);
      robot.Motor_Lower_L(1, DEFAULT_SPEED);
      robot.Motor_Upper_R(0, DEFAULT_SPEED);
      robot.Motor_Lower_R(0, DEFAULT_SPEED);
      break;
      
    case 94:  // Reverse circular motion
      robot.Motor_Lower_L(0, 0);
      robot.Motor_Lower_R(0, 0);
      robot.Motor_Upper_L(1, DEFAULT_SPEED);
      robot.Motor_Upper_R(0, DEFAULT_SPEED);
      break;
      
    // ─── LED Control ──────────────────────────────────────────────────────
    case 8:  // Toggle lights button
      isLedOn = !isLedOn;
      robot.right_led(isLedOn);
      robot.left_led(isLedOn);
      Serial.println(isLedOn ? "LEDs: ON" : "LEDs: OFF");
      break;
      
    // ─── Gripper Control ──────────────────────────────────────────────────
    // Option 1: Toggle with single key
    case 23:  // Option: Use different key for toggle
      toggleGripper();
      break;
      
    // Option 2: Separate open/close commands
    case 28:  // Custom key for CLOSE
      closeGripper();
      break;
      
    case 90:  // Custom key for OPEN
      openGripper();
      break;
      
    // ─── Stop ─────────────────────────────────────────────────────────────
    case 64:  // OK button → Stop
      robot.Stop();
      Serial.println("Robot: STOPPED");
      break;
  }
}