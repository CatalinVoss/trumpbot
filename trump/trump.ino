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
#define SERVO_ARM_L         9  
#define SERVO_ARM_R         10
#define SERVO_LIFTER        2   // Lifter servo

// Ultrasonic Sensors
#define ULTRASONIC_T        13  // Same trigger for both sensors
#define ULTRASONIC_RIGHT_E  12
#define ULTRASONIC_BACK_E   7

// Contact Sensors
#define CONTACT_BACK_L      A4
//#define CONTACT_BACK_R      // Not connected
#define CONTACT_RIGHT_B     8
#define CONTACT_RIGHT_F     A3

// Tape sensors
#define TAPE_R              A0
#define TAPE_C              A1
#define TAPE_L              A2

// Speed compensation
#define MOTOR1_SPEED_COMP 1.0
#define MOTOR2_SPEED_COMP 1.0
#define MOTOR3_SPEED_COMP 1.0
#define MOTOR4_SPEED_COMP 1.0

#define SENSOR_D_THRESH 512

#define MAX_DISTANCE 400 // in cm
NewPing ultra_right(ULTRASONIC_T,ULTRASONIC_RIGHT_E, MAX_DISTANCE);
NewPing ultra_back(ULTRASONIC_T, ULTRASONIC_BACK_E, MAX_DISTANCE);

// Servos
// Note that these are controlled differently!
// These servos are now all angle-controlled
Servo servo_arm_l;
Servo servo_arm_r;
Servo servo_lifter;

#pragma mark -
#pragma mark States

enum state {
  starting,
  unloading_chips,
  driving_to_base,
  driving_to_buckets,
  drive_to_corner_back,
  drive_to_corner_right,
  drive_to_center,
  drive_forward
  // TODO: add lots
};

state current_state;

#pragma mark -
#pragma mark Tape sensors

// returns true iff left tape sensor is on black line
bool check_tape_l() {
  return (analogRead(TAPE_L) > SENSOR_D_THRESH);
}

// returns true iff right tape sensor is on black line
bool check_tape_r() {
  return (analogRead(TAPE_R) > SENSOR_D_THRESH);
}

// returns true iff center tape sensor is on black line
bool check_tape_c() {
  return (analogRead(TAPE_C) > SENSOR_D_THRESH);
}

#pragma mark -
#pragma mark Contact

// Contact Sensors
#define CONTACT_BACK_L      A4
//#define CONTACT_BACK_R      // Not connected
#define CONTACT_RIGHT_B     8
#define CONTACT_RIGHT_F     A3

bool check_back_l_contact() {
  return (analogRead(CONTACT_BACK_L) > SENSOR_D_THRESH);
}

bool check_right_b_contact() {
  return digitalRead(CONTACT_RIGHT_B);
}

bool check_right_f_contact() {
  return (analogRead(CONTACT_RIGHT_F) > SENSOR_D_THRESH);
}


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

// Keep turning the robot
void spin() {
  drive_motor(MOTOR1, 0.4, true);
  drive_motor(MOTOR2, 0.4, false);
  drive_motor(MOTOR3, 0.4, false);
  drive_motor(MOTOR4, 0.4, true);
}

// Servo Position Constants
// Angle-controlled
#define LIFTER_REST_POS    45
#define LIFTER_MID_POS     60
#define LIFTER_UNLOAD_POS  130
// Speed-controlled
#define ARM_L_REST_POS     8 // -4 deg hardware adjustment
#define ARM_R_REST_POS     110
#define ARM_L_UNLOAD_POS   106 // -4 deg hardware adjustment
#define ARM_R_UNLOAD_POS   12
#define UNLOAD_TIME 800 // in ms
#define SERVO_DELAY 500

// Resets lifter into position to receive chip load
void reset_loader() {
  servo_arm_l.write(ARM_L_REST_POS);
  servo_arm_r.write(ARM_R_REST_POS);
  servo_lifter.write(LIFTER_REST_POS);
  delay(500); // give the servo time to move to starting position
}

// Unloads arms (blocking code)
void unload() {
  Serial.println("Opening arms");
  for (int i=LIFTER_REST_POS+1; i <= LIFTER_MID_POS; i++) {
      servo_lifter.write(i);
      delay(25);
  }
  
  // Move arms out
  for (int i=1; i <= ARM_L_UNLOAD_POS-ARM_L_REST_POS; i++) {
    servo_arm_l.write(ARM_L_REST_POS+i);
    servo_arm_r.write(ARM_R_REST_POS-i);
    delay(15);
  }
  
  servo_lifter.write(LIFTER_UNLOAD_POS);
  delay(SERVO_DELAY);
  delay(UNLOAD_TIME);
  servo_lifter.write(LIFTER_MID_POS);
  delay(SERVO_DELAY);

  // Move arms in
  for (int i=ARM_L_UNLOAD_POS-ARM_L_REST_POS; i >= 0; i--) {
    servo_arm_l.write(ARM_L_REST_POS+i);
    servo_arm_r.write(ARM_R_REST_POS-i);
    delay(10);
  }
  
  servo_lifter.write(LIFTER_REST_POS);
  delay(SERVO_DELAY);
//  reset_loader();
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
  pinMode(SERVO_ARM_L, OUTPUT);
  pinMode(SERVO_ARM_R, OUTPUT);
  pinMode(SERVO_LIFTER, OUTPUT);
  
  pinMode(CONTACT_RIGHT_B, INPUT);
  pinMode(CONTACT_BACK_L, INPUT);
  pinMode(CONTACT_RIGHT_F, INPUT);
  pinMode(TAPE_L, INPUT);
  pinMode(TAPE_C, INPUT);
  pinMode(TAPE_R, INPUT);

  // Enable all motors, stopped
  digitalWrite(MOTOR_EN, HIGH);
  stop_all();

  // Init servos
  servo_arm_l.attach(SERVO_ARM_L);
  servo_arm_r.attach(SERVO_ARM_R);
  servo_lifter.attach(SERVO_LIFTER);
  reset_loader();

  // Start state machine
  current_state = starting;
  Serial.println("set up done...");
}

#pragma mark -
#pragma mark Main

// TODO: set loop interal
#define MAIN_LOOP_DELAY 50

// Main loop code
void loop() {
  // ===== Measure ultrasonic =====
  int back_dist = ultra_back.ping() / US_ROUNDTRIP_CM;
  // This delay is crucial! Triggering them 1 by 1 otherwise causes interference.
  // The alternative is NOT to share the trigger pin or use the same input PIN for both with a logical OR in between
  delay(10);
  int right_dist = ultra_right.ping() / US_ROUNDTRIP_CM;

  drive_motor(MOTOR2, 0.5, true);
  drive_motor(MOTOR4, 0.5, true);
  delay(3000);

  /*
  // ===== State machine =====
  if (current_state == starting) {
   if (right_dist < 20 && back_dist < 20) {
      // drive into corner
      stop_all();
      current_state = drive_to_corner_back;
      Serial.println("found correct orientation ");
    } else {
      spin();
    }
  } else if (current_state == drive_to_corner_back) {
    if (check_back_l_contact()) {
      delay(200); // drive "a bit more" to make sure we're perfectly lined up -- since the sensor is just on one side
      stop_all();
      Serial.println("found back");
      current_state = drive_to_corner_right;
    } else {
      // Drive backward
      drive_motor(MOTOR2, 0.5, false);
      drive_motor(MOTOR4, 0.5, false);
    }
  } else if (current_state == drive_to_corner_right) {
    if (check_right_b_contact() && check_right_f_contact()) {
      delay(200); // drive "a bit more" to make sure we're perfectly lined up
      stop_all();
      current_state = drive_to_center;
      Serial.println("found right");
    } else {
      // Drive backward
      drive_motor(MOTOR1, 0.5, true);
      drive_motor(MOTOR3, 0.5, true);
    }
  } else if (current_state == drive_to_center) {
    drive_motor(MOTOR1, 0.5, false);
    drive_motor(MOTOR3, 0.5, false);
    delay(5000); // entirely timing based ... uggghhh
    stop_all();
    current_state = drive_forward;
  }
  */

//  Serial.print("tape: ");
//  Serial.println(analogRead(TAPE_L));


  // Print contact sensors
//  Serial.print("analog: ");
//  Serial.print(analogRead(CONTACT_BACK_L));
//  Serial.print(" ");
//  Serial.print("back_l: ");
//  Serial.print(check_back_l_contact());
//  Serial.print(" right_b: ");
//  Serial.print(check_right_b_contact());
//  Serial.print(" right_f: ");
//  Serial.println(check_right_f_contact());

// To test unload:
//  unload();
//  delay(2000);


#ifdef VERBOSE
//  Serial.println(analogRead(TAPE_L)); // this is doing *something*
  
//  Serial.print("Right ultrasonic: ");
//  Serial.print(right_dist);
//  Serial.print(" cm, Back ultrasonic: ");
//  Serial.print(back_dist);
//  Serial.println(" cm");
#endif
  delay(MAIN_LOOP_DELAY);
}

