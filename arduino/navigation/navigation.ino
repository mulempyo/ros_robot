#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <util/atomic.h>

ros::NodeHandle nh;

#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11
 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

float pwr_left;
float pwr_right;
float left_debt;
float right_debt;
float left_eprev;
float right_eprev;
float left_eintegral;
float right_eintegral;
float kp_left = 2.9;
float ki_left = 0.5;
float kd_left = 0.5;

float kp_right = 3.1;
float ki_right = 2.2;
float kd_right = 1.027;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
long prevT = 0;

int left_e;
int right_e;
volatile int posi=0;
const int enA = 9;
const int in1 = 5;
const int in2 = 6;
const int enB = 10;
const int in3 = 7;
const int in4 = 8;
const int PWM_TURN = 80;
 
void right_wheel_tick() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);
  if(val>0){
    posi++;
  }
  else{
    posi--;
  }
 
  if (val == LOW) {
    Direction_right = false; 
  }
  else {
    Direction_right = true;
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }  
  }
}
 

void left_wheel_tick() {
   
  int val = digitalRead(ENC_IN_LEFT_B);
  if(val>0){
    posi++;
  }
  else{
    posi--;
  }
 
  if (val == LOW) {
    Direction_left = true; 
  }
  else {
    Direction_left = false; 
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }  
  }
}

void teleop(int an1, int an2, int an3, int an4){
  digitalWrite(5,an1);
  digitalWrite(6,an2);
  digitalWrite(7,an3);
  digitalWrite(8,an4);
}
 
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  pwr_left = kp_left*left_e + ki_left*left_eintegral + kd_left*left_debt;
  pwr_right = kp_right*right_e + ki_right*right_eintegral + kd_right*right_debt;

  
    if (cmdVel.linear.x >= 0) { //Straight
      pwr_left = (kp_left*left_e + ki_left*left_eintegral + kd_left*left_debt);
      pwr_right= (kp_right*right_e + ki_right*right_eintegral + kd_right*right_debt);
      analogWrite(9,pwr_left);
      analogWrite(10,pwr_right);
      teleop(1,0,0,1);
    }
       
    else{ //Turn
      if(cmdVel.angular.z > 0){ //left
        analogWrite(9,80);
        analogWrite(10,80);
        teleop(0,1,0,1);
     }
     else { //right
      analogWrite(9,80);
      analogWrite(10,80);
      teleop(1,0,1,0);
    }
   } 
 pwr_left = constrain(pwr_left,0,255);
 pwr_right = constrain(pwr_right,0,255);
}
 
void set_pwm_values() {
   int target = 80;
   
   long currT = micros();
   float deltaT = ((float)(currT - prevT))/(1.0e6);
   prevT = currT;

   
   int pos = 0;
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
   }
   
   left_e = pos-target;
   right_e = pos-target;
   
   left_debt = (left_e-left_eprev)/deltaT;
   right_debt = (right_e-right_eprev)/deltaT;

   left_eintegral = left_eintegral + left_e*deltaT;
   right_eintegral = right_eintegral + right_e*deltaT;
   
   float u_left = kp_left*left_e + kd_left*left_debt + ki_left*left_eintegral;  
   float u_right = kp_right*right_e + kd_right*right_debt + ki_right*right_eintegral;

   left_eprev = left_e;
   right_eprev = right_e;

   pwr_left = fabs(u_left);
   pwr_right = fabs(u_right);
   
   pwr_left = constrain(pwr_left,0,255);
   pwr_right = constrain(pwr_right,0,255);
 
  if ((pwr_left < 0) || (pwr_right < 0)) {
    pwr_left = 0;
    pwr_right = 0;
  }
 
  
  if (pwr_left > 0) { 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwr_left < 0) { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwr_left == 0 && pwr_right == 0 ) { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
 
  if (pwr_right > 0) { 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if(pwr_right < 0) { 
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (pwr_left == 0 && pwr_right == 0) { 
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
 
  analogWrite(enA, pwr_left);
  analogWrite(enB, pwr_right);
}
 

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
  
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
 
  
  analogWrite(enA, 0);
  analogWrite(enB, 0);
 
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}
 
void loop() {
   
  nh.spinOnce();
  currentMillis = millis();
 
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
     
  }
  set_pwm_values();
}
