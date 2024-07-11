/*
Author: Kim Won Jun
Publish: /left_ticks, /right_ticks
Subscribe: /cmd_vel
this code usd when i use navigation package
this code have PID, also it can motor contorl about diff drive`s nine direction
*/

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <util/atomic.h>

ros::NodeHandle nh;

#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11

#define WHEEL_BASE 0.212
#define WHEEL_DIAMETER 0.067 //unit:m
#define LEFT_TICKS_PER_REVOLUTION 1700 //tick publish in 1 cycle
#define RIGHT_TICKS_PER_REVOLUTION 1800 //tick publish in 1 cycle

#define TURN_LEFT_LWHEEL_COMPENSATION 9
#define TURN_LEFT_RWHEEL_COMPENSATION 10
#define TURN_RIGHT_LWHEEL_COMPENSATION 4
#define TURN_RIGHT_RWHEEL_COMPENSATION 3
#define BACK_LWHEEL_COMPENSATION 6
#define BACK_RWHEEL_COMPENSATION 3
#define STRAIGHT_LWHEEL_COMPENSATION 5
#define STRAIGHT_RWHEEL_COMPENSATION 5



const int enA=9;
const int enB=10;
const int in1=5;
const int in2=6;
const int in3=7;
const int in4=8;
 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -2147483648;
const int encoder_maximum = 2147483647;
 

std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

geometry_msgs::Twist cmd;

float pwr_left;
float pwr_right;
float ang_pwr_left;
float ang_pwr_right;
float left_out;
float right_out;
float back_left_out;
float back_right_out;
float lastCmdVelReceived=0;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

volatile int left_posi = 0;
volatile int right_posi = 0;
int left_pos = 0;
int right_pos = 0;

void right_wheel_tick() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    if(val>0){
    right_posi++;
  }
  else{
    right_posi--;
  }
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
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      right_wheel_tick_count.data++; }
      
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      right_wheel_tick_count.data--;  }
      
    }  
  }
}
 

void left_wheel_tick() {
   
  int val = digitalRead(ENC_IN_LEFT_B);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    if(val>0){
    left_posi++;
  }
  else{
    left_posi--;
  }
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
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      left_wheel_tick_count.data++;  }
     
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      left_wheel_tick_count.data--; }
     
    }  
  }
}
float leftSpeed() {
    long previousTime = 0;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    left_pos = left_posi;
   }
    float left_speed = (left_pos / LEFT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER/((float) deltaTime * 1000);  
    previousTime = currentTime;
    return left_speed;
}

float rightSpeed() {
    long previousTime = 0;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;  
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    right_pos = right_posi;
   }
    float right_speed = (right_pos / RIGHT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER/((float) deltaTime * 1000);  
    previousTime = currentTime;
    return right_speed;
}


void teleop(int an1, int an2, int an3, int an4){
  digitalWrite(5,an1);
  digitalWrite(6,an2);
  digitalWrite(7,an3);
  digitalWrite(8,an4);
}

 
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  cmd = cmdVel;
  lastCmdVelReceived = (millis()/1000.0); 
  
  double left_velocity;
  double right_velocity;

  left_velocity = cmd.linear.x - (cmd.angular.z*WHEEL_BASE/2.0); 
  right_velocity = cmd.linear.x + (cmd.angular.z*WHEEL_BASE/2.0);

  if(cmd.angular.z > 0){ //left
   
    right_out = (150*right_velocity+(right_velocity-left_velocity)*130) + (pwr_right-STRAIGHT_RWHEEL_COMPENSATION+ang_pwr_left-TURN_LEFT_RWHEEL_COMPENSATION)/2; 
    left_out = (150*left_velocity-(right_velocity-left_velocity)*130) + (pwr_left+STRAIGHT_LWHEEL_COMPENSATION+ang_pwr_right+TURN_LEFT_LWHEEL_COMPENSATION)/2;
    back_left_out = (150*left_velocity+(right_velocity-left_velocity)*130) + (pwr_left+BACK_LWHEEL_COMPENSATION+ang_pwr_left+TURN_LEFT_LWHEEL_COMPENSATION)/2;
    back_right_out = (150*right_velocity-(right_velocity-left_velocity)*130) + (pwr_right-BACK_RWHEEL_COMPENSATION+ang_pwr_right-TURN_LEFT_RWHEEL_COMPENSATION)/2; 

    if(cmd.linear.x > 0){ //straight and left
      analogWrite(9,left_out);
      analogWrite(10,right_out);
      
    }
    else if(cmd.linear.x < 0){ //back and left
     analogWrite(9,back_left_out);
     analogWrite(10,back_right_out);
      
    }
    else{
      analogWrite(9,pwr_left+TURN_LEFT_LWHEEL_COMPENSATION);//turn left stay in the same place
      analogWrite(10,pwr_right-TURN_LEFT_RWHEEL_COMPENSATION);
       
    }
  }

  else if(cmd.angular.z < 0){ //right
    
    right_out = (150*right_velocity-(left_velocity-right_velocity)*150) + (pwr_right-STRAIGHT_RWHEEL_COMPENSATION+ang_pwr_left-TURN_RIGHT_RWHEEL_COMPENSATION)/2; 
    left_out = (150*left_velocity+(left_velocity-right_velocity)*150) + (pwr_left+STRAIGHT_LWHEEL_COMPENSATION+ang_pwr_right+TURN_RIGHT_LWHEEL_COMPENSATION)/2;
    back_left_out = (150*left_velocity-(left_velocity-right_velocity)*150) + (pwr_left+BACK_LWHEEL_COMPENSATION+ang_pwr_left+TURN_RIGHT_LWHEEL_COMPENSATION)/2;
    back_right_out = (150*right_velocity+(left_velocity-right_velocity)*150) + (pwr_right-BACK_RWHEEL_COMPENSATION+ang_pwr_right-TURN_RIGHT_RWHEEL_COMPENSATION)/2;
    if(cmd.linear.x > 0){ //straight and right
      analogWrite(9,left_out);
      analogWrite(10,right_out);
       
    }
    else if(cmd.linear.x < 0){ //back and right
     analogWrite(9,back_left_out);
     analogWrite(10,back_right_out);
      
    }
    else{
      analogWrite(9,pwr_left+TURN_RIGHT_LWHEEL_COMPENSATION);//turn right stay in the same place
      analogWrite(10,pwr_right-TURN_RIGHT_RWHEEL_COMPENSATION); 
       
    }
  }
  else{
    if(cmd.linear.x > 0){ //straight 
      analogWrite(9,pwr_left);
      analogWrite(10,pwr_right);
       
    }
    else if(cmd.linear.x < 0){ // back
     analogWrite(9,pwr_left+BACK_LWHEEL_COMPENSATION);
     analogWrite(10,pwr_right-BACK_RWHEEL_COMPENSATION);
     
    }
    else{
      lastCmdVelReceived = 0; //stop robot
       
    }
  }

}
void updateVelocity(){
   float target = 0.075; //7.5cm/s
   
   long prevT = 0;
   long currT = millis();
   float deltaT = ((float)(currT - prevT))*1000;
   prevT = currT;
   
   float left_debt;
   float right_debt;
   float left_eprev;
   float right_eprev;
   float left_eintegral;
   float right_eintegral;
   int left_e;
   int right_e;

   float kp_left = 0.2;
   float ki_left = 0.000000000001;
   float kd_left = 0.2;

   float kp_right = 0.1;
   float ki_right = 0.000000000001;
   float kd_right = 0.22;
   
   float ang_kp_left = 0.17;
   float ang_ki_left = 0.0000000000004;
   float ang_kd_left = 0.5;

   float ang_kp_right = 0.05;
   float ang_ki_right = 0.00000000000005;
   float ang_kd_right = 0.08;
   

   left_e = target-leftSpeed();
   right_e = target-rightSpeed();
   
   left_debt = (left_e-left_eprev)/deltaT;
   right_debt = (right_e-right_eprev)/deltaT;

   left_eintegral = left_eintegral + left_e*deltaT;
   right_eintegral = right_eintegral + right_e*deltaT;
   
   float u_left = kp_left*left_e + kd_left*left_debt + ki_left*left_eintegral;  
   float u_right = kp_right*right_e + kd_right*right_debt + ki_right*right_eintegral;

   float ang_u_left = ang_kp_left*left_e + ang_kd_left*left_debt + ang_ki_left*left_eintegral;
   float ang_u_right = ang_kp_right*left_e + ang_kd_right*left_debt + ang_ki_right*left_eintegral;

   left_eprev = left_e;
   right_eprev = right_e;

    pwr_left = fabs(u_left); 
    pwr_right = fabs(u_right);
    ang_pwr_left = fabs(ang_u_left);
    ang_pwr_right = fabs(ang_u_right);
    
    pwr_left = constrain(pwr_left,80,255);
    pwr_right = constrain(pwr_right,80,255); 
    ang_pwr_left = constrain(ang_pwr_left,80,255);
    ang_pwr_right = constrain(ang_pwr_right,80,255);
}

void set_pwm_values() {
   
   if(cmd.angular.z > 0){ //left
    if(cmd.linear.x > 0){ //straight and left
      teleop(1,0,0,1);
    }
    else if(cmd.linear.x < 0){ //back and left
     teleop(0,1,1,0);
    }
    else{
      teleop(0,1,0,1); //turn left stay in the same place
    }
  }

  else if(cmd.angular.z < 0){ //right
    if(cmd.linear.x > 0){ //straight and right
      teleop(1,0,0,1);
    }
    else if(cmd.linear.x < 0){ //back and left
     teleop(0,1,1,0);
    }
    else{
      teleop(1,0,1,0); //turn right stay in the same place
    }
  }

  else{ //cmd.angular.z == 0
    if(cmd.linear.x > 0){ //straight 
      teleop(1,0,0,1);
    }
    else if(cmd.linear.x < 0){ // back
     teleop(0,1,1,0);
    }
    else{
      teleop(0,0,0,0); //stop
    }
  }
  
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
  if((millis()/1000)-lastCmdVelReceived > 1){
    analogWrite(enA,0);
    analogWrite(enB,0);
    teleop(0,0,0,0);
  }
  set_pwm_values();
  updateVelocity();
  leftSpeed();
  rightSpeed();
}
