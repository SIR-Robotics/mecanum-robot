#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint16_t PWM_FREQUENCY = 50;
constexpr uint32_t OSCILLATOR_FREQUENCY = 27000000;

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

struct Joint {
  const char *name;
  uint8_t channel;
  int minimumAngle;
  int maximumAngle;
};

const Joint joints[] = {
  {"Base", 5, 0, 180},
  {"Shoulder", 0, 30, 150},
  {"Elbow", 1, 0, 135},
  {"Wrist Pitch", 2, 0, 180},
  {"Wrist Roll", 3, 0, 180},
  {"Gripper", 4, 0, 90},
};

constexpr size_t JOINT_COUNT = sizeof(joints) / sizeof(joints[0]);

// Change these after measuring your own servos.  They are only used by the
// "angle" command; the "channel" command writes the exact PWM count entered.
constexpr uint16_t SERVO_MIN_PWM = 102;
constexpr uint16_t SERVO_MAX_PWM = 512;

void printHelp() {
  Serial.println("\nPCA9685 PWM calibration");
  Serial.println("Enter one command per line:");
  Serial.println("  <channel> <pwm>       Set exact PWM count (example: 5 307)");
  Serial.println("  angle <joint> <deg>   Set a joint by angle (example: angle 0 90)");
  Serial.println("  off                    Turn every PCA9685 output off");
  Serial.println("  help                   Show this help");
  Serial.println("PWM range: 0 to 4095. Joint indexes:");
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    Serial.printf("  %u: %s (channel %u, %d..%d degrees)\n", i, joints[i].name,
                  joints[i].channel, joints[i].minimumAngle, joints[i].maximumAngle);
  }
}

void setChannel(uint8_t channel, uint16_t value) {
  pwm.setPWM(channel, 0, value);
  Serial.printf("Channel %u = PWM %u\n", channel, value);
}

void setJointAngle(uint8_t jointIndex, int angle) {
  const Joint &joint = joints[jointIndex];
  if (angle < joint.minimumAngle || angle > joint.maximumAngle) {
    Serial.printf("%s accepts %d..%d degrees.\n", joint.name, joint.minimumAngle,
                  joint.maximumAngle);
    return;
  }

  uint16_t value = map(angle, 0, 180, SERVO_MIN_PWM, SERVO_MAX_PWM);
  setChannel(joint.channel, value);
  Serial.printf("%s = %d degrees\n", joint.name, angle);
}

void processCommand(char *command) {
  while (*command == ' ' || *command == '\t') ++command;
  if (*command == '\0') return;

  if (!strcasecmp(command, "help")) {
    printHelp();
    return;
  }
  if (!strcasecmp(command, "off")) {
    for (uint8_t channel = 0; channel < 16; ++channel) pwm.setPWM(channel, 0, 0);
    Serial.println("All PCA9685 outputs off.");
    return;
  }

  int channel;
  int value;
  if (sscanf(command, "angle %d %d", &channel, &value) == 2) {
    if (channel < 0 || channel >= static_cast<int>(JOINT_COUNT)) {
      Serial.println("Invalid joint index. Enter help for the joint list.");
      return;
    }
    setJointAngle(channel, value);
    return;
  }

  if (sscanf(command, "%d %d", &channel, &value) == 2) {
    if (channel < 0 || channel > 15 || value < 0 || value > 4095) {
      Serial.println("Channel must be 0..15 and PWM must be 0..4095.");
      return;
    }
    setChannel(channel, value);
    return;
  }

  Serial.println("Invalid command. Enter help for usage.");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  Wire.begin(I2C_SDA, I2C_SCL);

  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_FREQUENCY);
  pwm.setPWMFreq(PWM_FREQUENCY);

  printHelp();
}

void loop() {
  static char command[48];
  static size_t length = 0;

  while (Serial.available()) {
    char character = static_cast<char>(Serial.read());
    if (character == '\r') continue;

    if (character == '\n') {
      command[length] = '\0';
      processCommand(command);
      length = 0;
      continue;
    }

    if (length < sizeof(command) - 1) {
      command[length++] = character;
    } else {
      length = 0;
      Serial.println("Command too long.");
    }
  }
}
