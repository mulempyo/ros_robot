/*
Author: Kim Won Jun
Publish: /left_ticks, /right_ticks
Subscribe: /cmd_vel
this code use when i use teleop_twist_keyboard package
this code can`t go straight, rotate straight
*/



#include <ros.h>
#include <util/atomic.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11
#define MAX_INTEGRAL 150

 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

geometry_msgs::Twist cmd_vel;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

ros::NodeHandle nh;


void right_wheel_tick() {
   
  int val = digitalRead(ENC_IN_RIGHT_B);
 
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


void teleop(int an1 ,int an2 ,int an3 ,int an4){
  analogWrite(9,255);
  analogWrite(10,255);
  digitalWrite(5, an1);
  digitalWrite(6, an2);
  digitalWrite(7, an3);
  digitalWrite(8, an4);
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
nh.advertise(rightPub);
nh.advertise(leftPub);
nh.subscribe(sub);
}

void loop() {
nh.spinOnce();
currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
 
  }
 
  double a = cmd_vel.linear.x;
  double c = cmd_vel.angular.z;

  if(a > 0 && c == 0){
    teleop(1,0,0,1);
  }
  else if(a < 0 && c == 0){
    teleop(0,1,1,0);
  }
  else if(a == 0 && c == 0){
    teleop(0,0,0,0);
  }
  else if(a == 0 && c > 0){
    teleop(1,0,1,0);
  }
  else if(a == 0 && c < 0){
    teleop(0,1,0,1);
  }

}
