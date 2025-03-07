// Ultrasonic sensor pins
const int trigPin1 = 26, echoPin1 = 34; //A0,A2
const int trigPin2 = 25, echoPin2 = 39; //A1,A3
const int trigPin3 = 4, echoPin3 = 36; //A5,A4

// Motor control pins, motors 1 and 3 are on the left on hbridge1
const int motor1IN1 = 12;  // IN1 for Motor 1 (H-Bridge 1)
const int motor1IN2 = 13;  // IN2 for Motor 1 (H-Bridge 1)
const int motor2IN1 = 17;  // IN1 for Motor 2 (H-Bridge 1)
const int motor2IN2 = 16;  // IN2 for Motor 2 (H-Bridge 1)

const int motor3IN1 = 33;  // IN1 for Motor 3 (H-Bridge 2)
const int motor3IN2 = 15;  // IN2 for Motor 3 (H-Bridge 2)
const int motor4IN1 = 5;   // IN1 for Motor 4 (H-Bridge 2)
const int motor4IN2 = 18;  // IN2 for Motor 4 (H-Bridge 2)

const int motorSleep1 = 27; // Sleep pin for H-Bridge 1
const int motorSleep2 = 19; // Sleep pin for H-Bridge 2

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

  Serial.begin(115200);
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

// Function to set motor direction (No PWM)
void setMotorDirection(int motor1, int motor2, bool forward) {
  if (forward) {
    digitalWrite(motor1, HIGH);  
    digitalWrite(motor2, LOW);
  } else {
    digitalWrite(motor1, LOW);   
    digitalWrite(motor2, HIGH);
  }
}

// Function to move forward
void moveForward() {
  setMotorDirection(motor1IN1, motor1IN2, true);   // Left motors forward
  setMotorDirection(motor2IN1, motor2IN2, false);  // Right motors backward
  setMotorDirection(motor3IN1, motor3IN2, true);
  setMotorDirection(motor4IN1, motor4IN2, false);
  Serial.println("Moving forward");
}

// Function to move backward
void moveBackward() {
  setMotorDirection(motor1IN1, motor1IN2, false);  // Left motors backward
  setMotorDirection(motor2IN1, motor2IN2, true);   // Right motors forward
  setMotorDirection(motor3IN1, motor3IN2, false);  
  setMotorDirection(motor4IN1, motor4IN2, true);
  Serial.println("Moving backward");
}

// Function to turn left
void turnLeft() {
  setMotorDirection(motor1IN1, motor1IN2, false);
  setMotorDirection(motor2IN1, motor2IN2, false);
  setMotorDirection(motor3IN1, motor3IN2, false);
  setMotorDirection(motor4IN1, motor4IN2, false);
  Serial.println("Turning left");
}

// Function to turn right
void turnRight() {
  setMotorDirection(motor1IN1, motor1IN2, true);  
  setMotorDirection(motor2IN1, motor2IN2, true);
  setMotorDirection(motor3IN1, motor3IN2, true);
  setMotorDirection(motor4IN1, motor4IN2, true);
  Serial.println("Turning right");
}

// Function to perform a 180-degree turn
void turn180() {
  turnLeft();  // Perform a left turn
  delay(1000);  // Adjust timing to complete 180-degree rotation
  stopMotors(); 
  Serial.println("180-degree turn");
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(motor1IN1, LOW);
  digitalWrite(motor1IN2, LOW);
  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, LOW);
  digitalWrite(motor3IN1, LOW);
  digitalWrite(motor3IN2, LOW);
  digitalWrite(motor4IN1, LOW);
  digitalWrite(motor4IN2, LOW);
  Serial.println("Motors stopped");
}

bool lastTurnLeft = false;  // Track if the last turn was to the left

void loop() {
  float frontDist = measureDistance(trigPin1, echoPin1);  // Front sensor reading
  float leftDist = measureDistance(trigPin2, echoPin2);   // Left sensor reading
  float rightDist = measureDistance(trigPin3, echoPin3);  // Right sensor reading

  // Check if there's an obstacle in front
  if (frontDist < 20) {  // If distance in front is less than 20 cm
    if (leftDist > rightDist) {
      turnLeft();  // Turn left if left side is clearer
      lastTurnLeft = true;  
    } else {
      turnRight();  // Turn right if right side is clearer
      lastTurnLeft = false;  
    }
    delay(500);  // Short delay to allow turn
  } else {
    moveForward();  // Move forward if no obstacles in front
  }

  // Dead end check: If all directions are blocked, perform a 180-degree turn
  if (frontDist < 20 && leftDist < 20 && rightDist < 20) {
    stopMotors();  
    delay(500);    

    turn180();

    // After the 180-degree turn, decide the next move based on sensor readings
    if (leftDist > rightDist) {
      turnLeft();
      lastTurnLeft = true;
    } else {
      turnRight();
      lastTurnLeft = false;
    }
  }

  delay(100);  // Short delay for the next iteration
}
