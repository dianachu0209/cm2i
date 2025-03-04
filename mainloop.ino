// Ultrasonic sensor pins
const int trigPin1 = 26, echoPin1 = 34; //A0,A2
const int trigPin2 = 25, echoPin2 = 39; //A1,A3
const int trigPin3 = 4, echoPin3 = 36; //A5,A4

// Motor control pins, motors 1 and 3 are on the left on hbridge1
const int motor1IN1 = 13;  // IN1 for Motor 1 (H-Bridge 1)
const int motor1IN2 = 12;  // IN2 for Motor 1 (H-Bridge 1)
const int motor2IN1 = 0;  // IN1 for Motor 2 (H-Bridge 1)
const int motor2IN2 = 0;  // IN2 for Motor 2 (H-Bridge 1)

const int motor3IN1 = 33;  // IN1 for Motor 3 (H-Bridge 2)
const int motor3IN2 = 15;  // IN2 for Motor 3 (H-Bridge 2)
const int motor4IN1 = 0;  // IN1 for Motor 4 (H-Bridge 2)
const int motor4IN2 = 0;  // IN2 for Motor 4 (H-Bridge 2)

const int motorSleep1 = 27; // Sleep pin for H-Bridge 1
const int motorSleep2 = 0; // Sleep pin for H-Bridge 2

// PWM pins for motor speed control
const int motor1PWM = 0;  // PWM for Motor 1 (H-Bridge 1)
const int motor2PWM = 0;  // PWM for Motor 2 (H-Bridge 1)
const int motor3PWM = 0;  // PWM for Motor 3 (H-Bridge 2)
const int motor4PWM = 0;  // PWM for Motor 4 (H-Bridge 2)

const int freq = 1000;      // PWM frequency
const int bits = 8;         // PWM resolution

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  // Motor control pins as OUTPUT
  pinMode(motor1IN1, OUTPUT);
  pinMode(motor1IN2, OUTPUT);
  pinMode(motor2IN1, OUTPUT);
  pinMode(motor2IN2, OUTPUT);
  pinMode(motor3IN1, OUTPUT);
  pinMode(motor3IN2, OUTPUT);
  pinMode(motor4IN1, OUTPUT);
  pinMode(motor4IN2, OUTPUT);
  
  // Sleep pins as OUTPUT
  pinMode(motorSleep1, OUTPUT);
  pinMode(motorSleep2, OUTPUT);

  // Enable motors by setting SLP pins HIGH
  digitalWrite(motorSleep1, HIGH);
  digitalWrite(motorSleep2, HIGH);

  // Attach PWM control for motors directly
  ledcAttach(motor1PWM, freq, bits);  // Attach PWM to Motor 1
  ledcAttach(motor2PWM, freq, bits);  // Attach PWM to Motor 2
  ledcAttach(motor3PWM, freq, bits);  // Attach PWM to Motor 3
  ledcAttach(motor4PWM, freq, bits);  // Attach PWM to Motor 4
}

// Function to measure distance
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.0343 / 2;  // Convert to mm
}

// Function to set motor speed
void setMotorSpeed(int motor1, int motor2, int pwmPin, int speed, bool forward) {
  if (forward) {
    digitalWrite(motor1, HIGH);  // Move forward
    digitalWrite(motor2, LOW);
  } else {
    digitalWrite(motor1, LOW);   // Move backward
    digitalWrite(motor2, HIGH);
  }
  ledcWrite(pwmPin, speed);  // Set PWM speed for the motor
}

// Gradual acceleration (ramp up) and deceleration (ramp down) for motors
void gradualPWM(int motorPWM, int targetSpeed, bool rampUp) {
  int currentSpeed = 0;
  int step = (rampUp) ? 5 : -5;  // Speed up (increase) or slow down (decrease)
  
  // Gradually increase or decrease speed to the target
  while ((rampUp && currentSpeed < targetSpeed) || (!rampUp && currentSpeed > targetSpeed)) {
    currentSpeed += step; 
    ledcWrite(motorPWM, currentSpeed);
    delay(20);  // Wait for smooth transition (adjust for your system)
  }
}

// Function to move all motors forward with gradual acceleration
void moveForward(int speed) {
  setMotorSpeed(motor1IN1, motor1IN2, motor1PWM, 0, true);  // Start with 0 speed
  setMotorSpeed(motor2IN1, motor2IN2, motor2PWM, 0, true);  // Start with 0 speed
  setMotorSpeed(motor3IN1, motor3IN2, motor3PWM, 0, true);  // Start with 0 speed
  setMotorSpeed(motor4IN1, motor4IN2, motor4PWM, 0, true);  // Start with 0 speed
  
  gradualPWM(motor1PWM, speed, true);  // Ramp up speed for Motor 1
  gradualPWM(motor2PWM, speed, true);  // Ramp up speed for Motor 2
  gradualPWM(motor3PWM, speed, true);  // Ramp up speed for Motor 3
  gradualPWM(motor4PWM, speed, true);  // Ramp up speed for Motor 4
}

// Function to move all motors backward with gradual acceleration
void moveBackward(int speed) {
  setMotorSpeed(motor1IN1, motor1IN2, motor1PWM, 0, false);  // Start with 0 speed
  setMotorSpeed(motor2IN1, motor2IN2, motor2PWM, 0, false);  // Start with 0 speed
  setMotorSpeed(motor3IN1, motor3IN2, motor3PWM, 0, false);  // Start with 0 speed
  setMotorSpeed(motor4IN1, motor4IN2, motor4PWM, 0, false);  // Start with 0 speed
  
  gradualPWM(motor1PWM, speed, true);  // Ramp up speed for Motor 1
  gradualPWM(motor2PWM, speed, true);  // Ramp up speed for Motor 2
  gradualPWM(motor3PWM, speed, true);  // Ramp up speed for Motor 3
  gradualPWM(motor4PWM, speed, true);  // Ramp up speed for Motor 4
}

// Function to turn left in place with gradual acceleration
void turnLeft(int speed) {
  // Left motors go backward, right motors go forward
  setMotorSpeed(motor1IN1, motor1IN2, motor1PWM, 0, false);  // Motor 1 backward (Left motor backward)
  setMotorSpeed(motor2IN1, motor2IN2, motor2PWM, 0, true);   // Motor 2 forward  (Right motor forward)
  setMotorSpeed(motor3IN1, motor3IN2, motor3PWM, 0, false);  // Motor 3 backward (Left motor backward)
  setMotorSpeed(motor4IN1, motor4IN2, motor4PWM, 0, true);   // Motor 4 forward  (Right motor forward)

  gradualPWM(motor1PWM, speed, true);  // Ramp up speed for Motor 1
  gradualPWM(motor2PWM, speed, true);  // Ramp up speed for Motor 2
  gradualPWM(motor3PWM, speed, true);  // Ramp up speed for Motor 3
  gradualPWM(motor4PWM, speed, true);  // Ramp up speed for Motor 4
}

// Function to turn right in place with gradual acceleration
void turnRight(int speed) {
  // Left motors go forward, right motors go backward
  setMotorSpeed(motor1IN1, motor1IN2, motor1PWM, 0, true);   // Motor 1 forward  (Left motor forward)
  setMotorSpeed(motor2IN1, motor2IN2, motor2PWM, 0, false);  // Motor 2 backward (Right motor backward)
  setMotorSpeed(motor3IN1, motor3IN2, motor3PWM, 0, true);   // Motor 3 forward  (Left motor forward)
  setMotorSpeed(motor4IN1, motor4IN2, motor4PWM, 0, false);  // Motor 4 backward (Right motor backward)

  gradualPWM(motor1PWM, speed, true);  // Ramp up speed for Motor 1
  gradualPWM(motor2PWM, speed, true);  // Ramp up speed for Motor 2
  gradualPWM(motor3PWM, speed, true);  // Ramp up speed for Motor 3
  gradualPWM(motor4PWM, speed, true);  // Ramp up speed for Motor 4
}

void turn180() {
  // Perform a 180-degree turn by spinning the motors in opposite directions
  setMotorSpeed(motor1IN1, motor1IN2, motor1PWM, 0, false);  // Motor 1 backward (Left motor backward)
  setMotorSpeed(motor2IN1, motor2IN2, motor2PWM, 0, true);   // Motor 2 forward  (Right motor forward)
  setMotorSpeed(motor3IN1, motor3IN2, motor3PWM, 0, false);  // Motor 3 backward (Left motor backward)
  setMotorSpeed(motor4IN1, motor4IN2, motor4PWM, 0, true);   // Motor 4 forward  (Right motor forward)

  gradualPWM(motor1PWM, 255, true);  // Ramp up speed for Motor 1
  gradualPWM(motor2PWM, 255, true);  // Ramp up speed for Motor 2
  gradualPWM(motor3PWM, 255, true);  // Ramp up speed for Motor 3
  gradualPWM(motor4PWM, 255, true);  // Ramp up speed for Motor 4

  delay(1000);  // Turn for 1 second (adjust time to match 180-degree turn)
  
  stopMotors();  // Stop after turning 180 degrees
}

// Function to stop all motors with gradual deceleration
void stopMotors() {
  gradualPWM(motor1PWM, 0, false);  // Ramp down speed for Motor 1
  gradualPWM(motor2PWM, 0, false);  // Ramp down speed for Motor 2
  gradualPWM(motor3PWM, 0, false);  // Ramp down speed for Motor 3
  gradualPWM(motor4PWM, 0, false);  // Ramp down speed for Motor 4
}

bool lastTurnLeft = false;  // Track if the last turn was to the left

void loop() {
  float frontDist = measureDistance(trigPin1, echoPin1);  // Front sensor reading
  float leftDist = measureDistance(trigPin2, echoPin2);   // Left sensor reading
  float rightDist = measureDistance(trigPin3, echoPin3);  // Right sensor reading

  // Check if there's an obstacle in front
  if (frontDist < 20) {  // If distance in front is less than 20 cm
    if (leftDist > rightDist) {
      turnLeft(255);  // Turn left if left side is clearer
      lastTurnLeft = true;  // Remember we turned left
    } else {
      turnRight(255);  // Turn right if right side is clearer
      lastTurnLeft = false;  // Remember we turned right
    }
  } else {
    moveForward(255);  // Move forward if no obstacles in front
  }

  // Dead end check: If all directions are blocked, perform a 180-degree turn
  if (frontDist < 20 && leftDist < 20 && rightDist < 20) {
    stopMotors();  // Stop the motors momentarily
    delay(500);    // Wait for half a second to stop before turning

    // Perform a 180-degree turn to avoid retracing steps
    turn180();

    // After the 180-degree turn, decide the next move based on sensor readings
    if (leftDist > rightDist) {
      turnLeft(255);  // If left is clearer, turn left
      lastTurnLeft = true;
    } else {
      turnRight(255);  // If right is clearer, turn right
      lastTurnLeft = false;
    }
  }

  delay(100);  // Short delay for the next iteration
}
