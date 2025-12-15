// Arduino Line Tracing Robot with PID Control
// Based on provided source code, adapted for 4 sensors with PID

// Motor Driver Pin Definitions
#define m1 4  //Right Motor MA1
#define m2 5  //Right Motor MA2
#define m3 2  //Left Motor MB1
#define m4 3  //Left Motor MB2
#define e1 9  //Right Motor Enable Pin EA
#define e2 10 //Left Motor Enable Pin EB

// 4 Channel IR Sensor Connection (without sensor 3)
#define ir1 A0  //Left Most Sensor
#define ir2 A1  //Left Sensor
#define ir4 A3  //Right Sensor
#define ir5 A4  //Right Most Sensor

// PID Constants - Tune these values for your robot
float kp = 1.5;    // Proportional gain
float ki = 0.0;    // Integral gain
float kd = 0.8;    // Derivative gain

// PID Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// Motor speeds
int baseSpeed = 200;     // Base motor speed (0-255)
int maxSpeed = 255;      // Maximum motor speed
int leftSpeed = 0;
int rightSpeed = 0;

// Sensor position weights for PID calculation
// Negative for left sensors, positive for right sensors
float sensorWeights[4] = {-3, -1, 1, 3};

unsigned long lastTime = 0;
const int sampleTime = 50;  // PID calculation interval (ms)

void setup() {
  Serial.begin(9600);
  
  // Motor pins as outputs
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  
  // IR sensor pins as inputs
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  
  // Initialize motors stopped
  stopMotors();
  
  Serial.println("PID Line Tracing Robot Initialized (4 Sensors)");
  Serial.println("0 = Black (line), 1 = White (background)");
  
  lastTime = millis();
}

void loop() {
  // Reading Sensor Values (0 = black line, 1 = white background)
  int s1 = digitalRead(ir1);  //Left Most Sensor
  int s2 = digitalRead(ir2);  //Left Sensor
  int s4 = digitalRead(ir4);  //Right Sensor
  int s5 = digitalRead(ir5);  //Right Most Sensor

  // Debug: Print sensor values (uncomment if needed)
  /*
  Serial.print("Sensors: ");
  Serial.print(s1); Serial.print(" ");
  Serial.print(s2); Serial.print(" ");
  Serial.print(s4); Serial.print(" ");
  Serial.println(s5);
  */

  // Check if any sensor detects black line
  bool lineDetected = (s1 == 0) || (s2 == 0) || (s4 == 0) || (s5 == 0);

  if (lineDetected) {
    // Use PID for smooth line following
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= sampleTime) {
      calculatePID(s1, s2, s4, s5);
      lastTime = currentTime;
    }
    
    // Calculate motor speeds based on PID output
    leftSpeed = constrain(baseSpeed + pidOutput, 0, maxSpeed);
    rightSpeed = constrain(baseSpeed - pidOutput, 0, maxSpeed);
    
    // Move with calculated speeds
    moveWithPIDSpeed(leftSpeed, rightSpeed);
    
    // Debug output
    Serial.print("Line detected | Error: ");
    Serial.print(error);
    Serial.print(" | PID: ");
    Serial.print(pidOutput);
    Serial.print(" | L: ");
    Serial.print(leftSpeed);
    Serial.print(" | R: ");
    Serial.println(rightSpeed);
    
  } else {
    // No line detected - stop motors
    stopMotors();
    // Reset PID variables
    integral = 0;
    lastError = 0;
    Serial.println("No line detected - Motors stopped");
  }
  
  delay(10);  // Small delay for stability
}

// Calculate PID output based on sensor readings
void calculatePID(int s1, int s2, int s4, int s5) {
  // Create sensor array (invert values: 1 for black line, 0 for white)
  int sensor[4] = {1-s1, 1-s2, 1-s4, 1-s5};
  
  // Calculate weighted position of line
  float position = 0;
  int activeSensors = 0;
  
  for (int i = 0; i < 4; i++) {
    if (sensor[i] == 1) {
      position += sensorWeights[i];
      activeSensors++;
    }
  }
  
  // Calculate error (desired position is 0 - center)
  if (activeSensors > 0) {
    error = position / activeSensors;
  } else {
    // No line detected, maintain last error
    error = lastError;
  }
  
  // Calculate integral
  integral += error;
  
  // Limit integral to prevent windup
  integral = constrain(integral, -100, 100);
  
  // Calculate derivative
  derivative = error - lastError;
  
  // Calculate PID output
  pidOutput = (kp * error) + (ki * integral) + (kd * derivative);
  
  // Limit PID output
  pidOutput = constrain(pidOutput, -maxSpeed/2, maxSpeed/2);
  
  // Save error for next iteration
  lastError = error;
}

// Function to move robot with PID-calculated speeds
void moveWithPIDSpeed(int leftSpd, int rightSpd) {
  // Set motor speeds using PWM
  analogWrite(e1, rightSpd);  // Right motor speed
  analogWrite(e2, leftSpd);   // Left motor speed
  
  // Set motor directions (forward)
  digitalWrite(m1, HIGH);  // Right motor forward
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);  // Left motor forward
  digitalWrite(m4, LOW);
}

// Basic movement functions

// Move forward at base speed
void moveForward() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}

// Turn right
void turnRight() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, HIGH);  // Right motor forward
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);   // Left motor stop
  digitalWrite(m4, LOW);
}

// Turn left  
void turnLeft() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, LOW);   // Right motor stop
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);  // Left motor forward
  digitalWrite(m4, LOW);
}

// Sharp right turn
void sharpRight() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, HIGH);  // Right motor forward
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);   // Left motor backward
  digitalWrite(m4, HIGH);
}

// Sharp left turn
void sharpLeft() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, LOW);   // Right motor backward
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);  // Left motor forward
  digitalWrite(m4, LOW);
}

// Stop all motors
void stopMotors() {
  analogWrite(e1, 0);
  analogWrite(e2, 0);
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
  leftSpeed = 0;
  rightSpeed = 0;
}

// Move backward
void moveBackward() {
  analogWrite(e1, baseSpeed);
  analogWrite(e2, baseSpeed);
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);
}
