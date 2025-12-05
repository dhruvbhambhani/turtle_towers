#include <Arduino.h>
#include <TurtleReceiver.h>

// Function declarations
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();

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

void setup() {
  Serial.begin(115200);
  Serial.println("Robot Starting...");
  
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
  Serial.println("Ready!");
}

void loop() {
  // Test sequence - will run when you connect battery
  Serial.println("Forward");
  moveForward(200);
  delay(2000);
  
  Serial.println("Stop");
  stopMotors();
  delay(1000);
  
  Serial.println("Backward");
  moveBackward(200);
  delay(2000);
  
  Serial.println("Stop");
  stopMotors();
  delay(1000);
  
  Serial.println("Turn Left");
  turnLeft(200);
  delay(1000);
  
  Serial.println("Stop");
  stopMotors();
  delay(1000);
  
  Serial.println("Turn Right");
  turnRight(200);
  delay(1000);
  
  Serial.println("Stop");
  stopMotors();
  delay(2000);
}

// Movement functions
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnLeft(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, speed);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}