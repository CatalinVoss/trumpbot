/**
 * TRUMPBOT MAIN CONTROL
 */

#include <Ultrasonic.h> // From ultrasonic library at http://wiki.tetrasys-design.net/HCSR04Ultrasonic
#include <Timers.h>     // From ME 210
#include <Servo.h>      // Standard arduino library
#include "line_sensor.hpp"

// Pin Layout
#define MOTOR1              5   // Motor Front
#define MOTOR2              6   // Motor Right
#define MOTOR3              9   // Motor Back
#define MOTOR4              10  // Motor Left
#define MOTOR_EN            4   // Motor Enable
#define ULTRASONIC_T        13  // Same trigger for both sensors
#define ULTRASONIC_RIGHT_E  12
#define ULTRASONIC_BACK_E   7
#define SERVO_ARMS          3   // Both arm servos connected here
#define SERVO_LIFTER        11  // Lifter servo

// Speed compensation
#define MOTOR1_SPEED_COMP 1.0
#define MOTOR2_SPEED_COMP 1.0
#define MOTOR3_SPEED_COMP 1.0
#define MOTOR4_SPEED_COMP 1.0

Ultrasonic ultra_right(ULTRASONIC_T,ULTRASONIC_RIGHT_E);
Ultrasonic ultra_back(ULTRASONIC_T,ULTRASONIC_BACK_E);

// Servos
Servo servo_arms;
Servo servo_lifter;

#pragma mark -
#pragma mark Helpers

#define PWM_RANGE 255

// Speed from 0 to 1
// Forward bool
void drive_motor(int motor, float speed, bool forward) {
  switch(motor){ 
    case MOTOR1:
      speed *= MOTOR1_SPEED_COMP;
      break;
    case MOTOR2:
      speed *= MOTOR2_SPEED_COMP;
      break;
    case MOTOR3:
      speed *= MOTOR3_SPEED_COMP;
      break;
    case MOTOR4:
      speed *= MOTOR4_SPEED_COMP;
      break;      
    default:
      break; 
  }
  
  float duty = 0;
  if (forward) {
    duty = 0.0+(1-speed)/2.0;
  } else {
    duty = 1.0-(1-speed)/2.0;
  }
  analogWrite(motor, duty*PWM_RANGE); 
}

void stop_all() {
  drive_motor(MOTOR1, 0, true);
  drive_motor(MOTOR2, 0, true);
  drive_motor(MOTOR3, 0, true);
  drive_motor(MOTOR4, 0, true);
}

#pragma mark -
#pragma mark Init

// Setup code that runs once
void setup() {
  // Start Serial
  Serial.begin(9600);
  Serial.println("Trumping up...");

  // Setup pins
  pinMode(MOTOR1, OUTPUT); 
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);

  // Enable all motors, stopped
  digitalWrite(MOTOR_EN, HIGH);
  stop_all();

  // Init servos
  servo_arms.attach(SERVO_ARMS);
  servo_lifter.attach(SERVO_LIFTER);
}

#pragma mark -
#pragma mark Main

// Main loop code
void loop() {
  // Drive forward
//  drive_motor(MOTOR2, 0.5, false);
//  drive_motor(MOTOR4, 0.5, false  );


  drive_motor(MOTOR1, 0.3, false);
  drive_motor(MOTOR3, 0.2, false  );

//  // Drive backward
//  drive_motor(MOTOR2, 1, false);
//  drive_motor(MOTOR4, 1, false);

//
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}

