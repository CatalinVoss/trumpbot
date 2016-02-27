/*************************************************************
  File:      ME_210 motor drive
  Contents:  This program controls DC motor(S) with an H bridge
(Clockwise from the front)
   motor  Channel  Pin        Driver
  Motor 1  B        5     H bridge driver 1
  Motor 2  A        6     H bridge driver 1
  Motor 3  B        9     H bridge driver 2
  Motor 4  A        10    H bridge driver 2

  2 parallel motors at 50% duty --> stalled
  then, 2 motors at 100% goes straight


  To do: some callibration. When moving forwards, 
 
 ************************************************************/


#include <Timers.h>

int half_range = 128;  //really 127.5. Halfway in PWM range.

int period = 50; // 100kHz frequency gives 10microsecond period
int v_pot = 0;
char key;
boolean en = true;

//pins
int led = 13; //for debugging
int EN = 4;
int PWM_DIR_pin_1 = 5;   
int PWM_DIR_pin_2 = 6;
int PWM_DIR_pin_3 = 9;
int PWM_DIR_pin_4 = 10;

void setPWM(float d, int pin);
//void callibrate();

void setup(){
  //adding a delay to start
  
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  
  pinMode(PWM_DIR_pin_1, OUTPUT); 
  pinMode(PWM_DIR_pin_2, OUTPUT);
  pinMode(PWM_DIR_pin_3, OUTPUT);
  pinMode(PWM_DIR_pin_4, OUTPUT);

  float duty = .5;
  setPWM(duty, PWM_DIR_pin_1);
  
  duty = .5;
  setPWM(duty, PWM_DIR_pin_2);
  
  duty = .5;
  setPWM(duty, PWM_DIR_pin_3);

  duty = .5;
  setPWM(duty, PWM_DIR_pin_4);

  delay(5000);
 
  pinMode(led, OUTPUT);
  Serial.begin(9600); 
}



/*Callibration Notes:

DUTY  Motor  DUTY   Motor     Effect  
 .9     2     1      4       Diagonal motion. 2 still movin slightly faster though since still angled.


*/
void loop(){

  //FORWARDS
 
  float duty = .5;
  setPWM(duty, PWM_DIR_pin_1);
  
  duty = .85;  
  setPWM(duty, PWM_DIR_pin_2);
  
  duty = .5;
  setPWM(duty, PWM_DIR_pin_3);

  duty = 1;
  setPWM(duty, PWM_DIR_pin_4);
  

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

