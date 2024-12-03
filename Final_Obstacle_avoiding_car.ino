#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 400  // Increased max distance for a larger car
#define MAX_SPEED 200     // Sets a slower speed for the larger car
#define TURN_SPEED 150    // Speed used during turns

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;

boolean goesForward = false;
int distance = 100;

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  myservo.attach(10);
  myservo.write(115);  // Center the servo
  delay(2000);
}

void loop() {
  // Delay for stability
  delay(40);
  
  distance = readPing();  // Read the distance in front

  // If an obstacle is detected in front
  if (distance <= 22) {  
    moveStop();
    delay(100);
    moveBackward();
    delay(500);  // Increased reverse delay for large car to clear space
    moveStop();
    delay(200);
    
    // Look right, left, and front to find the clearer path
    int distanceR = lookRight();  // Scan right side
    delay(300);
    int distanceL = lookLeft();   // Scan left side
    delay(300);
    int distanceF = readPing();    // Read the distance in front
    delay(100);

    // Debug output: print distances
    Serial.print("Distance Right: ");
    Serial.println(distanceR);
    Serial.print("Distance Left: ");
    Serial.println(distanceL);
    Serial.print("Distance Front: ");
    Serial.println(distanceF);

    // Determine the direction with the minimum distance
    char direction = 'F';  // Default direction is forward

    if (distanceL < distanceR) {
      direction = 'R';  // If left is closer, turn right
    } else if (distanceR < distanceL) {
      direction = 'L';  // If right is closer, turn left
    }

    // Turn in the direction opposite to where the obstacle is closer
    if (direction == 'L') {
      Serial.println("Turning Right (away from Left side obstacle)");
      turnRight();
    } else if (direction == 'R') {
      Serial.println("Turning Left (away from Right side obstacle)");
      turnLeft();
    } else {
      Serial.println("Moving Forward (no clear side detected)");
      moveForward();  // If front is clear, go forward
    }
    
  } else {
    moveForward();  // Continue moving forward if no obstacle is detected
  }
}

int lookRight() {
  myservo.write(40);  // Rotate servo further to the right for wider detection range
  delay(800);  // Increased delay for larger servo motion
  int distance = readPing();
  myservo.write(115);  // Return to center position
  return distance;
}

int lookLeft() {
  myservo.write(180);  // Rotate servo further to the left for wider detection range
  delay(800);  // Increased delay for larger servo motion
  int distance = readPing();
  myservo.write(115);  // Return to center position
  return distance;
}

int readPing() {
  delay(100);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = MAX_DISTANCE;  // If no object detected, set max distance
  }
  return cm;
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Move forward at max speed
void moveForward() {
  if (!goesForward) {
    goesForward = true;
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    // Set all motor speeds to MAX_SPEED for uniform movement
    motor1.setSpeed(MAX_SPEED);
    motor2.setSpeed(MAX_SPEED);
    motor3.setSpeed(MAX_SPEED);
    motor4.setSpeed(MAX_SPEED);
  }
}

// Move backward at max speed
void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  // Set all motor speeds to MAX_SPEED for uniform movement
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);
}

// Turn Right (away from left side obstacle)
void turnRight() {
  // Set all motors to MAX_SPEED for the turn
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  // Motors 1 and 2 move forward, Motors 3 and 4 move backward for the turn
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(3000);  // Adjust the delay based on turn sharpness
  
  moveStop();  // Stop motors after the turn
}

// Turn Left (away from right side obstacle)
void turnLeft() {
  // Set all motors to MAX_SPEED for the turn
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  // Motors 1 and 2 move backward, Motors 3 and 4 move forward for the turn
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(3000);  // Adjust the delay based on turn sharpness
  
  moveStop();  // Stop motors after the turn
}
