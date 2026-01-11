// --- PIN DEFINITIONS ---
const int leftLineSensor = 2;
const int rightLineSensor = 3;
const int obstacleSensor = 4;

const int motorA1 = 5;  // Left Motor Forward
const int motorA2 = 6;  // Left Motor Backward
const int motorB1 = 9;  // Right Motor Forward
const int motorB2 = 10; // Right Motor Backward

// --- PID TUNING CONSTANTS ---
float Kp = 110;    // Proportional: How hard to turn when off-center
float Kd = 0;    // Derivative: Dampens the turn to prevent wobbling
float Ki = 0;     // Integral: Usually 0 for 2-sensor setups

// --- MOTOR SPEED SETTINGS ---
int baseSpeed = 120;     // Normal cruising speed (0-255)
int maxSpeed = 200;      // Maximum speed limit
int lastError = 0;
float integral = 0;

void setup() {
  pinMode(leftLineSensor, INPUT);
  pinMode(rightLineSensor, INPUT);
  pinMode(obstacleSensor, INPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  Serial.begin(9600); // For debugging
}

void loop() {
  // 1. OBSTACLE CHECK (Safety First)
  // Most IR obstacle sensors output LOW when an object is detected
  if (digitalRead(obstacleSensor) == LOW) {
    stopMotors();
    return; // Skip the rest of the loop
  }

  // 2. READ LINE SENSORS
  int leftVal = digitalRead(leftLineSensor);
  int rightVal = digitalRead(rightLineSensor);
  int error = 0;

  // Determine Error Value
  // Assuming HIGH = Black Line, LOW = White Floor
  if (leftVal == LOW && rightVal == LOW) {
    error = 0;   // Perfectly centered on white (straddling the line)
  } else if (leftVal == HIGH && rightVal == LOW) {
    error = -1;  // Too far right, move left
  } else if (leftVal == LOW && rightVal == HIGH) {
    error = 1;   // Too far left, move right
  } else if (leftVal == HIGH && rightVal == HIGH) {
    // Both sensors on black - could be a stop line or intersection
    // We'll treat it as centered for now, or you can call stopMotors()
    error = 0; 
  }

  // 3. PID CALCULATION
  // Formula: Correction = (Kp * error) + (Kd * (error - lastError))
  int derivative = error - lastError;
  integral += error;
  int correction = (Kp * error) + (Kd * derivative) + (Ki * integral);
  
  lastError = error;

  // 4. CALCULATE INDIVIDUAL MOTOR SPEEDS
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // 5. DRIVE MOTORS
  drive(leftMotorSpeed, rightMotorSpeed);
}

void drive(int leftSpeed, int rightSpeed) {
  // Constrain speeds to valid PWM range (0-255)
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Left Motor
  analogWrite(motorA1, leftSpeed);
  digitalWrite(motorA2, LOW);

  // Right Motor
  analogWrite(motorB1, rightSpeed);
  digitalWrite(motorB2, LOW);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}