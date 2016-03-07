/**
   TRUMPBOT MAIN CONTROL
*/

#include <Ultrasonic.h> // From ultrasonic library at http://wiki.tetrasys-design.net/HCSR04Ultrasonic
#include <NewPing.h>    // From NewPing library at http://playground.arduino.cc/Code/NewPing
#include <Timers.h>     // From ME 210
#include <Servo.h>      // Standard arduino library
#include "line_sensor.hpp"
#include <Timers.h>

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
#define TAPE_R_B              A0
#define TAPE_C              A1
#define TAPE_L_B              A2
#define TAPE_R_F              A5
#define TAPE_L_F              A3

// Speed compensation
#define MOTOR1_SPEED_COMP 0.9
#define MOTOR2_SPEED_COMP 1.0
#define MOTOR3_SPEED_COMP 1.0
#define MOTOR4_SPEED_COMP 0.9

#define SENSOR_D_THRESH 512
#define SENSOR_D_THRESH_T1 850
#define SENSOR_D_THRESH_T2 312

#define MAX_DISTANCE 400 // in cm
NewPing ultra_right(ULTRASONIC_T, ULTRASONIC_RIGHT_E, MAX_DISTANCE);
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
  driving_to_corner_back,
  driving_to_corner_right,
  preparing_for_center_drive,
  driving_to_center,
  driving_to_buckets,
  unloading_chips,
  move_left_bucket,
  unloading_chips_l_side,
  move_right_center_line,
  driving_to_loading,
  Loading,
  finished
};

state current_state;

#pragma mark -
#pragma mark Tape sensors

// returns true iff left tape sensor is on black line
bool check_tape_l_f() {
  return (analogRead(TAPE_L_F) > SENSOR_D_THRESH_T1);
}

// returns true iff right tape sensor is on black line
bool check_tape_r_f() {
  return (analogRead(TAPE_R_F) > SENSOR_D_THRESH_T1);
}

// returns true iff left tape sensor is on black line
bool check_tape_l_b() {
  return (analogRead(TAPE_L_B) > SENSOR_D_THRESH_T2);
}

// returns true iff right tape sensor is on black line
bool check_tape_r_b() {
  return (analogRead(TAPE_R_B) > SENSOR_D_THRESH_T2);
}

// returns true iff center tape sensor is on black line
bool check_tape_c() {
  return (analogRead(TAPE_C) > SENSOR_D_THRESH_T1);
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
  switch (motor) {
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
    duty = 0.0 + (1 - speed) / 2.0;
  } else {
    duty = 1.0 - (1 - speed) / 2.0;
  }
  analogWrite(motor, duty * PWM_RANGE);
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
//#define LIFTER_REST_POS    45
#define LIFTER_MID_POS     60
#define LIFTER_UNLOAD_POS  115//120
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
  servo_lifter.write(LIFTER_MID_POS);
  //servo_lifter.write(LIFTER_REST_POS);
  delay(500); // give the servo time to move to starting position
}

// Unloads arms (blocking code)
void unload() {
  Serial.println("Opening arms");
  //  for (int i=LIFTER_REST_POS+1; i <= LIFTER_MID_POS; i++) {
  //      servo_lifter.write(i);
  //      delay(25);
  //  }

  // Move arms out
  for (int i = 1; i <= ARM_L_UNLOAD_POS - ARM_L_REST_POS; i++) {
    servo_arm_l.write(ARM_L_REST_POS + i);
    servo_arm_r.write(ARM_R_REST_POS - i);
    delay(15);
  }

  servo_lifter.write(LIFTER_UNLOAD_POS);
  delay(SERVO_DELAY);
  delay(UNLOAD_TIME);
  servo_lifter.write(LIFTER_MID_POS);
  delay(SERVO_DELAY);

  // Move arms in
  for (int i = ARM_L_UNLOAD_POS - ARM_L_REST_POS; i >= 0; i--) {
    servo_arm_l.write(ARM_L_REST_POS + i);
    servo_arm_r.write(ARM_R_REST_POS - i);
    delay(10);
  }

  servo_lifter.write(LIFTER_MID_POS);
  //servo_lifter.write(LIFTER_REST_POS);
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
  pinMode(TAPE_L_F, INPUT);
  pinMode(TAPE_C, INPUT);
  pinMode(TAPE_R_F, INPUT);
  pinMode(TAPE_L_B, INPUT);
  pinMode(TAPE_R_B, INPUT);

  // Enable all motors, stopped
  digitalWrite(MOTOR_EN, HIGH);
  stop_all();

  // Init servos
  servo_arm_l.attach(SERVO_ARM_L);
  servo_arm_r.attach(SERVO_ARM_R);
  servo_lifter.attach(SERVO_LIFTER);
  reset_loader();

  //starting timer
  TMRArd_InitTimer(2,120000);
  TMRArd_StartTimer(2);

  // Start state machine
  current_state = starting;
  Serial.println("set up done...");
}

#pragma mark -
#pragma mark Main

// TODO: set loop interal
#define MAIN_LOOP_DELAY 50

void driveCorner() {
  // Drive backward
  drive_motor(MOTOR2, 0.5, false);
  drive_motor(MOTOR4, 0.5, false);
  // Drive right
  drive_motor(MOTOR1, 0.5, true);
  drive_motor(MOTOR3, 0.5, true);
}

void driveToBucket(long time) {
  Serial.println("time, we are driving to bucket");
  Serial.println(time);
  while (true) {
    int back_dist = ultra_back.ping() / US_ROUNDTRIP_CM;
    if (!check_tape_l_f() && !check_tape_r_f() && !check_tape_c()) { // else if (check_tape_l() && check_tape_r() && !check_tape_c()) { // left and right on, center off ==> detects t-line
      // Drive forward
      stop_all();
      if(time<105000){
        current_state = unloading_chips;
      }else {
        current_state = move_left_bucket;
        Serial.println("state set to move_left_bucket");
        Serial.println("moving a little back");
        drive_motor(MOTOR2, 0.3, false);
        drive_motor(MOTOR4, 0.3, false);
        delay(400);
        stop_all();
      }
      break;
    } else if (!check_tape_l_f()) { // left off
      // turn in place slightly to right if left tape sensor is off
      drive_motor(MOTOR1, 0.6, true);
      delay(30);
      drive_motor(MOTOR1, 0, true);
    } else if (!check_tape_r_f()) { // right off
      // turn in place slightly to left if left tape sensor is off
      drive_motor(MOTOR1, 0.6, false);
      delay(30);
      drive_motor(MOTOR1, 0.5, true);
    } else if (back_dist > 70) {
      drive_motor(MOTOR2, 0.3, true);
      drive_motor(MOTOR4, 0.3, true);
    } else {
      // Drive forward
      drive_motor(MOTOR2, 0.6, true);
      drive_motor(MOTOR4, 0.6, true);
    }
  }
}


void driveToLoading() {
  Serial.println("in drive to loading");
  while (true) {
    if (!check_tape_l_b() && !check_tape_r_b()){ //(back_dist < 40) { // else if (check_tape_l_b() && check_tape_r_b() && !check_tape_c()) { // left and right on, center off ==> detects t-line
      // Drive forward
      delay(100);
      stop_all();
      current_state = Loading;
      Serial.println("drive to loading stopped b sensor l than r");
      Serial.println(check_tape_l_b() );
      Serial.println(check_tape_r_b());
      break;
    } else if (!check_tape_l_b()) { // left off
      // turn in place slightly to right if left tape sensor is off
      drive_motor(MOTOR3, 0.5, true);
      delay(30);
      drive_motor(MOTOR3, 0, false);
    } else if (!check_tape_r_b()) { // right off
      // turn in place slightly to left if left tape sensor is off
      drive_motor(MOTOR3, 0.5, false);
      delay(30);
      drive_motor(MOTOR3, 0, false);
    } else {
      // Drive forward
      drive_motor(MOTOR2, 0.5, false);
      drive_motor(MOTOR4, 0.5, false);
    }
  }
}

// Main loop code
void loop() {


  // ===== Measure ultrasonic =====
  int back_dist = ultra_back.ping() / US_ROUNDTRIP_CM;
  // This delay is crucial! Triggering them 1 by 1 otherwise causes interference.
  // The alternative is NOT to share the trigger pin or use the same input PIN for both with a logical OR in between
  delay(10);
  int right_dist = ultra_right.ping() / US_ROUNDTRIP_CM;

  long time =TMRArd_GetTime();

  // ===== State machine =====
  if (time>=120000){
    Serial.println("timer up ");
    Serial.println(time);
    current_state = finished;
    stop_all(); 
  }else if (current_state == starting) {
    if (right_dist < 20 && back_dist < 20) {
      // drive into corner
      stop_all();
      current_state = driving_to_corner_back;
      Serial.println("found correct orientation ");
    } else {
      spin();
    }
  } else if (current_state == driving_to_corner_back) {
    if (check_back_l_contact()) {
      delay(250); // drive "a bit more" to make sure we're perfectly lined up -- since the sensor is just on one side
      stop_all();
      Serial.println("found back");
      current_state = driving_to_corner_right;
    } else {
      driveCorner();
      // To isolate the motions, uncomment:
      //      // Drive backward
      //      drive_motor(MOTOR2, 0.5, false);
      //      drive_motor(MOTOR4, 0.5, false);

    }
  } else if (current_state == driving_to_corner_right) {
    if (check_right_f_contact()) {
      delay(250); // drive "a bit more" to make sure we're perfectly lined up
      stop_all();
      current_state = preparing_for_center_drive;
      Serial.println("found right");
    } else {
      Serial.println("driving to corner to find right");
      driveCorner();
      // To isolate the motions, uncomment:
      //      // Drive right
      //      drive_motor(MOTOR1, 0.5, true);
      //      drive_motor(MOTOR3, 0.5, true);
    }
  } else if (current_state == preparing_for_center_drive) {
    // Drive forward
    drive_motor(MOTOR2, 0.6, true);
    drive_motor(MOTOR4, 0.6, true);
    //delay(1300); //delay(2000);
    if (back_dist > 30) { 
      stop_all();
      // Drive right to line up w corner again
      drive_motor(MOTOR1, 0.6, true);
      drive_motor(MOTOR3, 0.6, true);
      delay(250); 
      stop_all();
      delay(100); //delay(500);
      // Drive left
      // gradual speedup first to 0.3 then to 0.5
      drive_motor(MOTOR1, 0.3, false);
      drive_motor(MOTOR3, 0.3, false);
      delay(100); //delay(500);
      current_state = driving_to_center;
    }
  } else if (current_state == driving_to_center) {
    //if (right_dist>91) {
    if (check_tape_l_f()) {
      stop_all();
      current_state = driving_to_buckets;
      drive_motor(MOTOR2, 0.6, true);
      drive_motor(MOTOR4, 0.6, true);
      delay(500);
    } else if (right_dist > 80) {
      drive_motor(MOTOR1, 0.3, false);
      drive_motor(MOTOR3, 0.3, false);
    } else {
      drive_motor(MOTOR1, 0.6, false);
      drive_motor(MOTOR3, 0.6, false);
    }
  } else if (current_state == driving_to_buckets) {
    Serial.println("driving to bucket ");
    driveToBucket(time);
  }else if( current_state == move_left_bucket){
    Serial.println("move to left bucket ");
    //moveLeft();
    drive_motor(MOTOR1, 0.6, false);
    drive_motor(MOTOR3, 0.6, false);
    delay(1400);
    current_state = unloading_chips_l_side;
    stop_all();
  }else if( current_state ==unloading_chips_l_side){
    Serial.println("unload chip left side ");
    unload();
    current_state = move_right_center_line;
    //move back a little so that when drivibg left we will hit center line
    drive_motor(MOTOR2, 0.6, false);
    drive_motor(MOTOR4, 0.6, false);
    delay(400);
    stop_all();
  } else if (current_state == move_right_center_line) {
    Serial.println("move back to center line ");
    //drive left to center line
    if (!check_tape_r_f()){
      drive_motor(MOTOR1, 0.6, true);
      drive_motor(MOTOR3, 0.6, true);
    }else{
      stop_all();
      current_state = driving_to_loading;
    }
  } else if (current_state == unloading_chips) {
    Serial.println("unloading ");
    unload();
    current_state = driving_to_loading;
    drive_motor(MOTOR2, 0.6, false);
    drive_motor(MOTOR4, 0.6, false);
    delay(500);
  } else if (current_state == driving_to_loading) {
    Serial.println("driving to loading ");
    driveToLoading();
  } else if (current_state == Loading) {
    Serial.println("loading ");
    delay(3500);
    current_state = driving_to_buckets;
    drive_motor(MOTOR2, 0.6, true);
    drive_motor(MOTOR4, 0.6, true);
    delay(500);
  }
  
  delay(MAIN_LOOP_DELAY); 

  
//  Serial.println("check_tape_l_b");
//  Serial.println(check_tape_l_b());
//  Serial.println("check_tape_r_b");
//  Serial.println(check_tape_r_b());
//  Serial.println("check_tape_l_f");
//  Serial.println(check_tape_l_f());
//  Serial.println("check_tape_r_f");
//  Serial.println(check_tape_r_f());
//  Serial.println("check_tape_c");
//  Serial.println(check_tape_c());
//  delay(1000);
}

