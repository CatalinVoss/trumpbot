/*************************************************************
  File:      ME_210 motor drive
  Contents:  This program controls DC motor(S) with an H bridge
 
 ************************************************************/


#include <Timers.h>


int period = 50; // 100kHz frequency gives 10microsecond period
int v_pot = 0;
char key;
boolean en = true;

//pins
int led = 13; //for debugging
int PWM_DIR_pin_1 = 8;  
int EN_pin_1 = 9; //  

int PWM_DIR_pin_2 = 10;
int EN_pin_2 = 11; 

int ADC_pin = A0;

void setPWM(float d, int pin);
//void callibrate();

void setup(){
  pinMode(ADC_pin, INPUT);
  pinMode(EN_pin_1, OUTPUT);
  pinMode(PWM_DIR_pin_1, OUTPUT);
  digitalWrite(EN_pin_1, en);
  
  pinMode(EN_pin_2, OUTPUT);
  pinMode(PWM_DIR_pin_2, OUTPUT);
 
  digitalWrite(EN_pin_2, en);

  pinMode(led, OUTPUT);
  Serial.begin(9600); 
  //callibrate();
}

void loop(){
  float duty = 200;  //test  
  /*
  for(int i = 0; i < 255; i++){
    duty = i;
     setPWM(duty, PWM_DIR_pin_1);
    delay(5000);
  } 
  */
  //write PWM out 
  //duty = v_pot/adc_range;  
 
  setPWM(duty, PWM_DIR_pin_1);
    
  
}
   //set the pwm
  void setPWM(float d, int pin){ 
    float PWM_range = 255;
    int duty = d*PWM_range;
    analogWrite(pin, duty); //alas cannot use
   }

