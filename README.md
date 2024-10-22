/*
# Intersecting Line Follower Robot

A line follower robot that autonomously navigates along black lines and handles intersections 
using three IR sensors. The robot uses differential drive control and an Arduino-based system 
for efficient line following.

## Hardware Components
- 1 x Arduino UNO
- 3 x IR Line Tracking Sensors
- 1 x L298N Motor Driver Module
- 2 x DC Motors with wheels
- Power Supply (Battery)
- Chassis
- Jumper Wires
- Supporting wheels

## Pin Configuration
Left Motor:
- ENAPIN - Pin 9  (PWM Speed Control)
- N1PIN  - Pin 4  (Direction Control)
- N2PIN  - Pin 6  (Direction Control)

Right Motor:
- ENBPIN - Pin 10 (PWM Speed Control)
- N3PIN  - Pin 7  (Direction Control)
- N4PIN  - Pin 11 (Direction Control)

Sensors:
- LeftSensorPin   - Pin 3
- CenterSensorPin - Pin 13
- RightSensorPin  - Pin 2
*/
## Code
// Pin definitions for Left Motor
#define ENAPIN 9
#define N1PIN 4
#define N2PIN 6

// Pin definitions for Right Motor
#define ENBPIN 10
#define N3PIN 7
#define N4PIN 11

// Pin definitions for IR Sensors
#define LeftSensorPin 3
#define CenterSensorPin 13
#define RightSensorPin 2

// Motor speed settings (PWM values 0-255)
int leftmotorSpeed = 170;  // Adjust this value to balance left motor speed
int rightmotorspeed = 180; // Adjust this value to balance right motor speed

void setup() {
  // Motor pin initialization
  pinMode(ENAPIN, OUTPUT);
  pinMode(N1PIN, OUTPUT);
  pinMode(N2PIN, OUTPUT);
  pinMode(ENBPIN, OUTPUT);
  pinMode(N3PIN, OUTPUT);
  pinMode(N4PIN, OUTPUT);
  
  // Sensor pin initialization
  pinMode(LeftSensorPin, INPUT);
  pinMode(CenterSensorPin, INPUT);
  pinMode(RightSensorPin, INPUT);
  
  // Set initial motor speeds
  analogWrite(ENAPIN, leftmotorSpeed);
  analogWrite(ENBPIN, rightmotorspeed);
}

/*
 * Movement Functions
 * Note: HIGH/LOW combinations determine motor direction
 */

// Move the robot forward
void MoveForward() {
  digitalWrite(N1PIN, HIGH);
  digitalWrite(N2PIN, LOW);
  digitalWrite(N3PIN, HIGH);
  digitalWrite(N4PIN, LOW);
}

// Move the robot backward
void MoveBackward() {
  digitalWrite(N1PIN, LOW);
  digitalWrite(N2PIN, HIGH);
  digitalWrite(N3PIN, LOW);
  digitalWrite(N4PIN, HIGH);
}

// Turn the robot left
void TurnLeft() {
  digitalWrite(N1PIN, LOW);
  digitalWrite(N2PIN, HIGH);
  digitalWrite(N3PIN, HIGH);
  digitalWrite(N4PIN, LOW);
}

// Turn the robot right
void TurnRight() {
  digitalWrite(N1PIN, HIGH);
  digitalWrite(N2PIN, LOW);
  digitalWrite(N3PIN, LOW);
  digitalWrite(N4PIN, HIGH);
}

// Stop the robot
void Stop() {
  digitalWrite(N1PIN, LOW);
  digitalWrite(N2PIN, LOW);
  digitalWrite(N3PIN, LOW);
  digitalWrite(N4PIN, LOW);
}

/*
 * Main Loop
 * Sensor Values:
 * - 0: White surface (off line)
 * - 1: Black surface (on line)
 */
void loop() {
  // Read sensor values
  int LeftSensorValue = digitalRead(LeftSensorPin); 
  int CenterSensorValue = digitalRead(CenterSensorPin);
  int RightSensorValue = digitalRead(RightSensorPin);
  
  // Line following logic
  // Move forward if center sensor detects line
  if(LeftSensorValue == 0 && CenterSensorValue == 1 && RightSensorValue == 0) {
    MoveForward();
  }
  // Turn right if right sensor detects line
  else if(LeftSensorValue == 0 && CenterSensorValue == 1 && RightSensorValue == 1) {
    TurnRight();
  }
  // Turn left if left sensor detects line
  else if(LeftSensorValue == 1 && CenterSensorValue == 1 && RightSensorValue == 0) {
    TurnLeft();
  }
  // Move forward at intersection (all sensors detect line)
  else if(LeftSensorValue == 1 && CenterSensorValue == 1 && RightSensorValue == 1) {
    MoveForward();
  }
  // Stop if no line is detected
  else {
    Stop();
  }
}

/*
## Troubleshooting Guide

1. Robot not following line properly:
   - Check sensor positioning and height from ground
   - Verify sensor connections and pin numbers
   - Adjust motor speeds if turning is uneven

2. Motors not moving:
   - Check battery voltage
   - Verify motor driver connections
   - Check PWM values (leftmotorSpeed and rightmotorspeed)

3. Erratic behavior:
   - Check for loose connections
   - Verify sensor threshold values
   - Check for interference from ambient light

## Performance Adjustments

- Adjust leftmotorSpeed and rightmotorspeed values for:
  * Straight line speed
  * Turning accuracy
  * Overall performance

- Current settings:
  * Left Motor: 170 PWM
  * Right Motor: 180 PWM

## Future Improvements

1. Add PID control for smoother line following
2. Implement variable speed control for turns
3. Add additional sensors for better intersection detection
4. Include obstacle detection capabilities
*/
