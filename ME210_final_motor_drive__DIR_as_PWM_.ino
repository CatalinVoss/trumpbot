

/*************************************************************
  File:      ME_210 motor drive
  Contents:  This program controls DC motor(S) with an H bridge, specifically by sending 
              a PWM signal to Direction pins while all motors are always enabled. 
  To use: Download Ultrasonic library (http://wiki.tetrasys-design.net/HCSR04Ultrasonic) 
                    And ME 210 Timers Library

     
  
(Clockwise from the front)
   motor  Channel    Pin        Driver             
  Motor 1  B/D2/M2    5     H bridge driver 1
  Motor 2  A/D1/M1    6     H bridge driver 1
  Motor 3  B/D2/M2    9     H bridge driver 2
  Motor 4  A/D1/M1    10    H bridge driver 2     

  2 parallel motors at 50% duty --> stalled
  then, 2 motors at 100% goes straight

  
.5 0 .5 0 -- moving forwards  
.5 1 .5 1 -- moving backwards. Move straight. Just slowly. 
 
Callibration Notes:

DUTY  Motor   DUTY   Motor     Effect  
 .9     2     1      4       Diagonal motion. 2 still movin slightly faster though since still angled.
 .85    2     1      4       Pretty damn straight 


 ************************************************************/

#include <Timers.h>  //from ME 210 libraries
#include <Ultrasonic.h> 

#define serial Serial 

Ultrasonic ultrasonic(12,13);


int period = 50; // 100kHz frequency gives 10microsecond period 

//pins 
int EN = 4;
int PWM_DIR_pin_1 = 5;   
int PWM_DIR_pin_2 = 6;
int PWM_DIR_pin_3 = 9;
int PWM_DIR_pin_4 = 10;

void setPWM(float d, int pin);
//void callibrate();

void setup(){
 //Declare & Init Pinz
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  pinMode(PWM_DIR_pin_1, OUTPUT); 
  pinMode(PWM_DIR_pin_2, OUTPUT);
  pinMode(PWM_DIR_pin_3, OUTPUT);
  pinMode(PWM_DIR_pin_4, OUTPUT);

  //start OFF
  float duty = .5;
  setPWM(duty, PWM_DIR_pin_1); 
  setPWM(duty, PWM_DIR_pin_2); 
  setPWM(duty, PWM_DIR_pin_3);
  setPWM(duty, PWM_DIR_pin_4);

  //delay(30000);
  TMRArd_InitTimer(0,15000);
  Serial.begin(9600); 
}




void loop(){

  ///ULTRASONIC sensors
  /*
  serial.println(ultrasonic.Ranging(CM));   
  delay(100);
  */

  //MOTORS
  float duty = .5;
  //set up timer for 10s
  if(TMRArd_IsTimerActive(0)== TMRArd_ACTIVE){
    duty = .5; 
    setPWM(duty, PWM_DIR_pin_1);
    
    duty = 0;  
    setPWM(duty, PWM_DIR_pin_2);
    
    duty = .5;     
    setPWM(duty, PWM_DIR_pin_3);
  
    duty = 0; 
    setPWM(duty, PWM_DIR_pin_4);
    
  }
  //after timer
  if(TMRArd_IsTimerActive(0)== TMRArd_NOT_ACTIVE){
    duty = .5;
    setPWM(duty, PWM_DIR_pin_1);
    
    duty = 0;  
    setPWM(duty, PWM_DIR_pin_2);
    
    duty = .5;
    setPWM(duty, PWM_DIR_pin_3);
  
    duty = 0;
    setPWM(duty, PWM_DIR_pin_4);
  } else {
    //OFF
    duty = .5;
    setPWM(duty, PWM_DIR_pin_1);
    setPWM(duty, PWM_DIR_pin_2);
    setPWM(duty, PWM_DIR_pin_3);
    setPWM(duty, PWM_DIR_pin_4);
  }

  
  

 /* BACKWARDS
  duty = 0;
  setPWM(duty, PWM_DIR_pin_1);
  
  duty = 1;
  setPWM(duty, PWM_DIR_pin_2);
  
  duty = 0;
  setPWM(duty, );

  duty = 1;
  setPWM(duty, PWM_DIR_pin_4);
  */



  
  
}
   //set the pwm
  void setPWM(float d, int pin){ 
    float PWM_range = 255;
    int duty = d*PWM_range;
    analogWrite(pin, duty); 
   }

