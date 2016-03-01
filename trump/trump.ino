/**
 * TRUMPBOT MAIN CONTROL
 */

#include <Ultrasonic.h> // From ultrasonic library at http://wiki.tetrasys-design.net/HCSR04Ultrasonic
#include <NewPing.h>    // From NewPing library at http://playground.arduino.cc/Code/NewPing
#include <Timers.h>     // From ME 210
#include <Servo.h>      // Standard arduino library
#include "line_sensor.hpp"

#define VERBOSE

// Pin Layout
// NOTE: servos MUST be on pins 9, 10, because the servo library shuts those pins off irrespective of where the servos are
//       per https://www.arduino.cc/en/Reference/Servo

// Motors
#define MOTOR1              3   // Motor Front
#define MOTOR2              5   // Motor Right
#define MOTOR3              6   // Motor Back
#define MOTOR4              11  // Motor Left
#define MOTOR_EN            4   // Motor Enable

// Servos
#define SERVO_ARMS          9   // Both arm servos connected here
#define SERVO_LIFTER        10  // Lifter servo

// Ultrasonic Sensors
#define ULTRASONIC_T        13  // Same trigger for both sensors
#define ULTRASONIC_RIGHT_E  12
#define ULTRASONIC_BACK_E   7

// Contact Sensors
#define CONTACT_BACK_L      2
#define CONTACT_BACK_R      A3
#define CONTACT_RIGHT       A4

// Tape sensors
#define TAPE_L              A0
#define TAPE_C              A1
#define TAPE_R              A2

// Speed compensation
#define MOTOR1_SPEED_COMP 1.0
#define MOTOR2_SPEED_COMP 1.0
#define MOTOR3_SPEED_COMP 1.0
#define MOTOR4_SPEED_COMP 1.0

NewPing ultra_right(ULTRASONIC_T,ULTRASONIC_RIGHT_E);
NewPing ultra_back(ULTRASONIC_T, ULTRASONIC_BACK_E);

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

// Servo Position Constants
// in degrees
#define ARMS_REST_POS      0// TODO set this before turning on!
#define ARMS_UNLOAD_POS    0// TODO set this before turning on!
#define LIFTER_REST_POS    0// TODO set this before turning on!
#define LIFTER_UNLOAD_POS  0// TODO set this before turning on!

#define UNLOAD_TIME 100 // in ms

// Resets arms into position to receive chip load
void reset_loader() {
  servo_lifter.write(LIFTER_REST_POS);
  servo_arms.write(ARMS_REST_POS);
  // TODO: move to starting position
}

// Unloads arms (blocking code)
void unload() {
  servo_arms.write(ARMS_UNLOAD_POS); // order is important here... :)
  servo_lifter.write(LIFTER_UNLOAD_POS);
  delay(UNLOAD_TIME);
  reset_loader();
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
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(SERVO_ARMS, OUTPUT);
  pinMode(SERVO_LIFTER, OUTPUT);

  // Enable all motors, stopped
  digitalWrite(MOTOR_EN, HIGH);
  stop_all();

  // Init servos
  servo_arms.attach(SERVO_ARMS);
  servo_lifter.attach(SERVO_LIFTER);
//  reset_loader(); // TODO: comment in
}

#pragma mark -
#pragma mark Main

// Main loop code
void loop() {
  // Drive forward
  drive_motor(MOTOR1, 0.9, false);
  drive_motor(MOTOR4, 0.1, false);

  // TODO: perhaps check these at a lower interval
  int back_dist = ultra_back.ping() / US_ROUNDTRIP_CM;
  delay(10); // This is crucial! Triggering them 1 by 1 otherwise causes interference. The alternative is NOT to share the trigger pin or 
  int right_dist = ultra_right.ping() / US_ROUNDTRIP_CM;

#ifdef VERBOSE
  Serial.print("Right ultrasonic: ");
  Serial.print(right_dist);   
  Serial.println(" cm");
  Serial.print("Back ultrasonic: ");
  Serial.print(back_dist);   
  Serial.println(" cm");
#endif

//  drive_motor(MOTOR1, 0.2, false);
//  drive_motor(MOTOR3, 0.2, false);

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

  // TODO: set loop interal
  delay(50);
}

