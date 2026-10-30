
/*
 * ============================================================
 * SUMO BOT - ESP32 + REMOTEXY BLE JOYSTICK
 * ============================================================
 * Board        : NodeMCU ESP32 (ESP32-WROOM-32)
 * Controller   : iPhone - RemoteXY
 * Communication: Bluetooth Low Energy
 * Motor Driver : TB6612FNG
 *
 * JOYSTICK:
 *   UP       = Both motors forward
 *   DOWN     = Both motors reverse
 *   LEFT     = Turn left
 *   RIGHT    = Turn right
 *   DIAGONAL = Combined movement
 *   CENTRE   = Stop
 *
 * ============================================================
 */

// ============================================================
// LIBRARIES AND BLUETOOTH CONFIGURATION
// ============================================================

#include <Arduino.h>

#define REMOTEXY_MODE__ESP32CORE_BLE
#define REMOTEXY_BLUETOOTH_NAME "SUMO_BOT"

#include <BLEDevice.h>
#include <RemoteXY.h>
#include <esp_arduino_version.h>

// ============================================================
// REMOTEXY JOYSTICK CONFIGURATION
// Official one-joystick configuration
// ============================================================

#pragma pack(push, 1)

uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255, 2, 0, 0, 0, 22, 0, 19, 0, 0, 0, 0,
  31, 1, 106, 200, 1, 1, 1, 0,
  5, 16, 33, 60, 60, 0, 2, 26, 31
};

struct {

  // Joystick input
  int8_t joystick_01_x;  // -100 to +100
  int8_t joystick_01_y;  // -100 to +100

  // Connection status
  uint8_t connect_flag;

} RemoteXY;

#pragma pack(pop)

// ============================================================
// TB6612FNG PIN CONFIGURATION
// ============================================================

// LEFT MOTOR - CHANNEL A
#define PIN_PWMA 25
#define PIN_AIN1 27
#define PIN_AIN2 26

// RIGHT MOTOR - CHANNEL B
#define PIN_PWMB 33
#define PIN_BIN1 32
#define PIN_BIN2 13

// STANDBY
#define PIN_STBY 14

// ============================================================
// PWM CONFIGURATION
// ============================================================

#define PWM_FREQ 1000
#define PWM_RES  8

// ============================================================
// MOVEMENT TUNING
// ============================================================

#define MAX_SPEED 255
#define DEADZONE  8

// Reverse individual motor direction if necessary
#define LEFT_INVERT  false
#define RIGHT_INVERT false

// RemoteXY joystick screen coordinates:
// Up is normally negative Y, down is positive Y.
#define INVERT_Y true

// ============================================================
// PWM INITIALIZATION
// ESP32 CORE 2.X AND 3.X SUPPORT
// ============================================================

void initializePWM() {

#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

#else

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, 0);

  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, 1);

#endif

}

void writeLeftPWM(int duty) {

  duty = constrain(duty, 0, MAX_SPEED);

#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcWrite(PIN_PWMA, duty);

#else

  ledcWrite(0, duty);

#endif

}

void writeRightPWM(int duty) {

  duty = constrain(duty, 0, MAX_SPEED);

#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcWrite(PIN_PWMB, duty);

#else

  ledcWrite(1, duty);

#endif

}

// ============================================================
// LEFT MOTOR CONTROL
// ============================================================

void setLeftMotor(int speed) {

  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (LEFT_INVERT) {
    speed = -speed;
  }

  if (speed > 0) {

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

  }
  else if (speed < 0) {

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);

  }
  else {

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);

  }

  writeLeftPWM(abs(speed));

}

// ============================================================
// RIGHT MOTOR CONTROL
// ============================================================

void setRightMotor(int speed) {

  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (RIGHT_INVERT) {
    speed = -speed;
  }

  if (speed > 0) {

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

  }
  else if (speed < 0) {

    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);

  }
  else {

    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);

  }

  writeRightPWM(abs(speed));

}

// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors() {

  setLeftMotor(0);
  setRightMotor(0);

}

// ============================================================
// JOYSTICK DEADZONE
// ============================================================

int applyDeadzone(int value) {

  if (abs(value) <= DEADZONE) {
    return 0;
  }

  return value;

}

// ============================================================
// ARCADE DRIVE
// X = STEERING
// Y = THROTTLE
// ============================================================

void driveRobot(int x, int y) {

  // Limit joystick values
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);

  // Correct vertical screen coordinate
  int throttle = INVERT_Y ? -y : y;
  int steering = x;

  // Apply deadzone
  throttle = applyDeadzone(throttle);
  steering = applyDeadzone(steering);

  // Calculate independent motor speeds
  int leftOutput = throttle + steering;
  int rightOutput = throttle - steering;

  // Normalize outputs, preserving their ratio
  int largest = max(abs(leftOutput), abs(rightOutput));

  if (largest > 100) {

    leftOutput = (leftOutput * 100) / largest;
    rightOutput = (rightOutput * 100) / largest;

  }

  // Convert -100...100 into -255...255
  int leftSpeed = (leftOutput * MAX_SPEED) / 100;
  int rightSpeed = (rightOutput * MAX_SPEED) / 100;

  // Independently update both motors
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);

}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("================================");
  Serial.println("       SUMO BOT CONTROLLER      ");
  Serial.println("       ESP32 + REMOTEXY BLE     ");
  Serial.println("================================");

  // Motor direction pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);

  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // Standby
  pinMode(PIN_STBY, OUTPUT);

  // Initialize PWM
  initializePWM();

  // Enable motor driver
  digitalWrite(PIN_STBY, HIGH);

  stopMotors();

  // Initialize RemoteXY BLE
  RemoteXY_Init();

  Serial.println("RemoteXY BLE initialized!");
  Serial.println("Bluetooth name: SUMO_BOT");
  Serial.println("Waiting for iPhone connection...");

}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Process BLE communication continuously
  RemoteXY_Handler();

  // Disconnect safety
  if (!RemoteXY.connect_flag) {

    stopMotors();

    return;

  }

  // Read joystick coordinates
  int x = RemoteXY.joystick_01_x;
  int y = RemoteXY.joystick_01_y;

  // Control robot
  driveRobot(x, y);

  // Serial debugging
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 200) {

    Serial.print("X: ");
    Serial.print(x);

    Serial.print(" | Y: ");
    Serial.print(y);

    Serial.print(" | Connected: ");
    Serial.println(RemoteXY.connect_flag);

    lastPrint = millis();

  }

}
