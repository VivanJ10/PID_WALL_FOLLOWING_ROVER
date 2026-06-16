//  Wall Follower Rover


#include <HCSR04.h>

// All the Imp Pins
// Side Sensor (Left)
const uint8_t TRIG_LEFT  = 9;
const uint8_t ECHO_LEFT  = 10;

// Front Sensor
const uint8_t TRIG_FRONT = 11;
const uint8_t ECHO_FRONT = 12;

// Left motor
const uint8_t ENA = 5;   // PWM
const uint8_t IN1 = 3;
const uint8_t IN2 = 2;

// Right motor
const uint8_t ENB = 6;   // PWM
const uint8_t IN3 = 7;
const uint8_t IN4 = 4;

//  PID constants 
const double Kp = 4.0;
const double Ki = 0.1;
const double Kd = 1.5;

// To configure
const double SETPOINT        = 15.0;  // Target distance from left wall in cm
const double FRONT_THRESHOLD = 20.0;  // Distance to trigger right turn in cm

const int    BASE_SPEED  = 90;   
const int    MAX_SPEED   = 110;
const int    MIN_SPEED   = 75;

// PID State
double integral   = 0.0;
double lastError  = 0.0;
unsigned long lastTime = 0;

UltraSonicDistanceSensor leftSensor(TRIG_LEFT, ECHO_LEFT);
UltraSonicDistanceSensor frontSensor(TRIG_FRONT, ECHO_FRONT);

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  setMotors(BASE_SPEED, BASE_SPEED);
  Serial.println(F("PID Wall Follower + Obstacle Avoidance Ready"));
}

void loop() {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt < 0.02) return;  // 50 Hz update rate
  lastTime = now;

  //  Priority Override: Front Collision Avoidance 
  double frontDist = frontSensor.measureDistanceCm();
  
  // If object is within threshold (and reading didn't time out with a 0)
  if (frontDist > 0 && frontDist <= FRONT_THRESHOLD) {
    Serial.print(F("Obstacle ahead at ")); Serial.print(frontDist, 1); Serial.println(F("cm! Turning Right."));
    
    // Hard right point turn: Left motor forward, Right motor reverse
    setMotors(BASE_SPEED, -BASE_SPEED); 
    
    // Reset PID integral to prevent windup while turning
    integral = 0.0; 
    
    return; // Skip the wall-following math for this loop iteration
  }

  //  Normal Operation: Wall Following (Left Sensor) 
  double leftDist = leftSensor.measureDistanceCm();

  // Validate reading
  if (leftDist <= 0 || leftDist > 80.0) {
    setMotors(BASE_SPEED,BASE_SPEED);
    integral=0.0;
    return; 
  }

  // PID calculation
  double error    = leftDist - SETPOINT;
  integral       += error * dt;
  integral        = constrain(integral, -50.0, 50.0);  // Anti-windup clamp
  double derivative = (error - lastError) / dt;
  lastError       = error;

  double output = Kp * error + Ki * integral + Kd * derivative;

  // Differential Steering Logic
  int leftSpeed  = constrain((int)(BASE_SPEED - output), MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED + output), MIN_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

  // Clean layout for Serial Monitor debugging
  Serial.print(F("Side Dist: "));   Serial.print(leftDist, 1);
  Serial.print(F("cm | Err: "));    Serial.print(error, 1);
  Serial.print(F(" | Out: "));      Serial.print(output, 1);
  Serial.print(F(" | Motor L: "));  Serial.print(leftSpeed);
  Serial.print(F(" R: "));          Serial.println(rightSpeed);
}

// Upgraded setMotors function to handle both forward and reverse
void setMotors(int left, int right) {
  // Left Motor Direction
  if (left >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    left = -left; // Flip back to positive for analogWrite
  }
  
  // Right Motor Direction
  if (right >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    right = -right; // Flip back to positive for analogWrite
  }
  
  analogWrite(ENA, left);
  analogWrite(ENB, right);
}