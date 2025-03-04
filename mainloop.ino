// Ultrasonic sensor pins
const int trigPin1 = 4, echoPin1 = 5;
const int trigPin2 = 33, echoPin2 = 15;
const int trigPin3 = 27, echoPin3 = 12;

// Motor control pins using PWM
const int motorLeftPWM = A0;
const int motorRightPWM = A1;
const int motorLeftDir = 32;   // Direction pin
const int motorRightDir = 14;  // Direction pin

// PWM settings
const int freq = 5000;
const int bits = 8;  // 8-bit resolution (0-255)

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(motorLeftDir, OUTPUT);
  pinMode(motorRightDir, OUTPUT);
  
  // Using ledcAttach for both the setup and attaching to pins
  ledcAttach(motorLeftPWM, freq, bits);
  ledcAttach(motorRightPWM, freq, bits);
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

// Function to gradually accelerate motors
void setMotorSpeed(int leftSpeed, int rightSpeed, bool forward) {
  digitalWrite(motorLeftDir, forward);
  digitalWrite(motorRightDir, forward);
  
  for (int speed = 0; speed <= max(leftSpeed, rightSpeed); speed += 10) {
    ledcWrite(motorLeftPWM, min(speed, leftSpeed));
    ledcWrite(motorRightPWM, min(speed, rightSpeed));
    delay(20);
  }
}

// Function to move forward
void moveForward() {
  setMotorSpeed(180, 180, true);
}

// Function to move backward
void moveBackward() {
  setMotorSpeed(180, 180, false);
  delay(600);
}

// Function to turn left smoothly
void turnLeft() {
  setMotorSpeed(100, 180, true);
  delay(600);
}

// Function to turn right smoothly
void turnRight() {
  setMotorSpeed(180, 100, true);
  delay(600);
}

// Function to stop motors smoothly
void stopMotors() {
  for (int speed = 180; speed >= 0; speed -= 10) {
    ledcWrite(motorLeftPWM, speed);
    ledcWrite(motorRightPWM, speed);
    delay(20);
  }
}

// Main loop
void loop() {
  float distanceFront = measureDistance(trigPin1, echoPin1);
  float distanceLeft = measureDistance(trigPin2, echoPin2);
  float distanceRight = measureDistance(trigPin3, echoPin3);

  if (distanceFront < 150 && distanceLeft < 150 && distanceRight < 150) {
    moveBackward();
    delay(800);
    turnRight();
    delay(800);
  } 
  else if (distanceFront < 150) {  
    stopMotors();
    if (distanceLeft > distanceRight) {  
      turnLeft();
    } else {  
      turnRight();
    }
  } else {
    moveForward();
  }

  delay(100);
}
