#include <Arduino.h>
#include <TurtleReceiver.h>

// Motor A (Left) pins
#define ENA 14
#define IN1 27
#define IN2 26

// Motor B (Right) pins
#define ENB 12
#define IN3 25
#define IN4 33

// PWM settings
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// Create controller object
NetController controller;

// Function declarations
// Function declarations
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void setMotorSpeed(int leftSpeed, int rightSpeed);  // Add this line

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Starting...");
  
  // Print MAC address for controller pairing
  Serial.println("Your MAC Address:");
  printMacAddress();
  
  // Setup controller
  controller.controllerSetup();
  
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup PWM for speed control
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  
  stopMotors();
  Serial.println("Ready! Send MAC address to controller team.");
}

void loop() {
  // Get joystick values
  float joyX = controller.getJoy1X();  // Left/Right: -1 to 1
  float joyY = controller.getJoy1Y();  // Forward/Back: -1 to 1
  
  // Convert joystick to motor speeds
  int baseSpeed = 200;  // Max speed
  
  // Calculate left and right motor speeds based on joystick
  int leftSpeed = (joyY + joyX) * baseSpeed;
  int rightSpeed = (joyY - joyX) * baseSpeed;
  
  // Constrain speeds to -255 to 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Apply motor speeds
  setMotorSpeed(leftSpeed, rightSpeed);
  
  delay(20);  // Small delay for stability
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, abs(leftSpeed));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM_CHANNEL_A, abs(leftSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
  }
  
  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(PWM_CHANNEL_B, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(PWM_CHANNEL_B, abs(rightSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(PWM_CHANNEL_B, 0);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}