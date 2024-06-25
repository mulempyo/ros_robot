/*
Author: Kim Won Jun
Publish: /left_ticks, /right_ticks
Subscribe: /cmd_vel
this code use when i using teleop_twist_keyboard,SLAM package
this code have PID, so my robot can go straight,rotate straight
*/
#include <ros.h>
#include <util/atomic.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11

#define WHEEL_DIAMETER 0.067 //unit:m
#define LEFT_TICKS_PER_REVOLUTION 1700 //tick publish in 1 cycle
#define RIGHT_TICKS_PER_REVOLUTION 1800 //tick publish in 1 cycle

#define TURN_LEFT_LWHEEL_COMPENSATION 9
#define TURN_LEFT_RWHEEL_COMPENSATION 10
#define TURN_RIGHT_LWHEEL_COMPENSATION 4
#define TURN_RIGHT_RWHEEL_COMPENSATION 3
#define BACK_LWHEEL_COMPENSATION 6
#define BACK_RWHEEL_COMPENSATION 3
#define STRAIGHT_LWHEEL_COMPENSATION 4
#define STRAIGHT_RWHEEL_COMPENSATION 5

ros::NodeHandle nh;

float pwr_left;
float pwr_right;
float ang_pwr_left;
float ang_pwr_right;
 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -2147483648;
const int encoder_maximum = 2147483647;
volatile int left_posi = 0;
volatile int right_posi = 0;
int left_pos = 0;
int right_pos = 0;

long prevT = 0;
float left_eprev= 0;
float right_eprev = 0;
float left_eintegral = 0;
float right_eintegral = 0;

std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

geometry_msgs::Twist cmd_vel;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

//double left_velocity = (cmd_vel.linear.x - (cmd_vel.angular.z*21.2/2.0))/3.35;
//double right_velocity = (cmd_vel.linear.x + (cmd_vel.angular.z*21.2/2.0))/3.35;
//float left_out;
//float right_out;

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

void teleop(int an1 ,int an2 ,int an3 ,int an4){
  digitalWrite(5, an1);
  digitalWrite(6, an2);
  digitalWrite(7, an3);
  digitalWrite(8, an4);
}

void updateVelocity(){
   
   float target = 0.075; //target velocity pwm:90 -> 60cm/8s -> 7.5 cm/s -> 0.075m/s
   
   float kp_left = 0.2;
   float ki_left = 0.00000000000001;
   float kd_left = 0.2;

   float kp_right = 0.1;
   float ki_right = 0.00000000000001;
   float kd_right = 0.22;

   float ang_kp_left = 0.17;
   float ang_ki_left = 0.0000000000004;
   float ang_kd_left = 0.5;

   float ang_kp_right = 0.05;
   float ang_ki_right = 0.00000000000005;
   float ang_kd_right = 0.08;
   
   long prevT = 0;
   long currT = millis();
   float deltaT = ((float)(currT - prevT))*1000;
   prevT = currT;
   
   int left_e = target-leftSpeed();
   int right_e = target-rightSpeed();

   
   float left_debt = (left_e-left_eprev)/deltaT;
   float right_debt = (right_e-right_eprev)/deltaT;

   left_eintegral = left_eintegral + left_e*deltaT;
   right_eintegral = right_eintegral + right_e*deltaT;
   
   float u_left = kp_left*left_e + kd_left*left_debt + ki_left*left_eintegral;  
   float u_right = kp_right*right_e + kd_right*right_debt + ki_right*right_eintegral;

   float ang_u_left = ang_kp_left*left_e + ang_kd_left*left_debt + ang_ki_left*left_eintegral;
   float ang_u_right = ang_kp_right*right_e + ang_kd_right*right_debt + ang_ki_right*right_eintegral;

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


void subCmdVel(const geometry_msgs::Twist& msg) {
 cmd_vel = msg;
}

   
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", subCmdVel); 


void setup()
{
pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
pinMode(ENC_IN_LEFT_B, INPUT);
pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
pinMode(ENC_IN_RIGHT_B, INPUT);

pinMode(10, OUTPUT); //ENB
pinMode(8, OUTPUT); //IN 4
pinMode(7, OUTPUT); //IN 3 
pinMode(6, OUTPUT); //IN 2
pinMode(5, OUTPUT); //IN 1
pinMode(9, OUTPUT); //ENA 

attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

nh.getHardware()->setBaud(57600);
nh.initNode();
nh.advertise(leftPub);
nh.advertise(rightPub);
nh.subscribe(sub);
}

void loop() {
nh.spinOnce();
currentMillis = millis();
updateVelocity();
leftSpeed();
rightSpeed();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    leftPub.publish(&left_wheel_tick_count );
    rightPub.publish(&right_wheel_tick_count );
 
  }
 
  double a = cmd_vel.linear.x;
  double c = cmd_vel.angular.z; 
  

  if(a > 0 && c == 0){ //straight
  analogWrite(9,pwr_left+STRAIGHT_LWHEEL_COMPENSATION);
  analogWrite(10,pwr_right-STRAIGHT_RWHEEL_COMPENSATION); 
    teleop(1,0,0,1); 
  }
  else if(a < 0 && c == 0){ //back
  analogWrite(9,pwr_left+BACK_LWHEEL_COMPENSATION);
  analogWrite(10,pwr_right-BACK_RWHEEL_COMPENSATION);
    teleop(0,1,1,0);
  }
  else if(a == 0 && c == 0){ //stop
    analogWrite(9,0);
    analogWrite(10,0);
    teleop(0,0,0,0);
  }
  else if(a == 0 && c > 0){ //left
  //right_out = 10*right_velocity + (pwr_right+ang_pwr_left)/2; 
  //left_out = 10*left_velocity + (pwr_left+ang_pwr_right)/2;
  analogWrite(9,ang_pwr_left+TURN_LEFT_LWHEEL_COMPENSATION); 
  analogWrite(10,ang_pwr_right-TURN_LEFT_RWHEEL_COMPENSATION);
    teleop(0,1,0,1);
  }
  else if(a == 0 && c < 0){ //right
  //right_out = right_velocity + (pwr_right+ang_pwr_left)/2; 
  //left_out = left_velocity + (pwr_left+ang_pwr_right)/2;
  
  analogWrite(9,ang_pwr_left+TURN_RIGHT_LWHEEL_COMPENSATION);
  analogWrite(10,ang_pwr_right-TURN_RIGHT_RWHEEL_COMPENSATION);
    teleop(1,0,1,0);
  }

}
