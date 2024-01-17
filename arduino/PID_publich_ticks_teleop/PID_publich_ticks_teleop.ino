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
ros::NodeHandle nh;

float pwr_left;
float pwr_right;
 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
volatile int posi = 0;
long prevT = 0;
float eprev= 0;
float aprev = 0;
float eintegral = 0;
float aintegral = 0;
const double TICKS_PER_METER = 1753;
const int DRIFT_MULTIPLIER = 106;
double velLeftWheel = 0;
double velRightWheel = 0;

std_msgs::Float64 left_pwm_msg;
ros::Publisher leftPwm("left_pwm",&left_pwm_msg);
std_msgs::Float64 right_pwm_msg;
ros::Publisher rightPwm("right_pwm",&right_pwm_msg);

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

geometry_msgs::Twist cmd_vel;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;


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

void calc_vel_left_wheel(){
   
  static double prevTime = 0;
  static int prevLeftCount = 0;
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  
  velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
  prevLeftCount = left_wheel_tick_count.data;
  prevTime = (millis()/1000);
 
}

void calc_vel_right_wheel(){
   
  static double prevTime = 0;
  static int prevRightCount = 0;
 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
 
  velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime); 
  prevRightCount = right_wheel_tick_count.data; 
  prevTime = (millis()/1000);
 
}


void teleop(int an1 ,int an2 ,int an3 ,int an4){
  digitalWrite(5, an1);
  digitalWrite(6, an2);
  digitalWrite(7, an3);
  digitalWrite(8, an4);
}
void updateVelocity(){
   int target = 80*sin(prevT/1e6);
   
   float kp_left = 1.25;
   float kd_left = 0.17;
   float ki_left = 0.38;

   float kp_right = 1.18;
   float kd_right = 0.1;
   float ki_right = 0.2;
   
   long currT = micros();
   float deltaT = ((float)(currT - prevT))/(1.0e6);
   prevT = currT;

   static double prevDiff = 0;
   static double prevPrevDiff = 0;
   double currDifference = velLeftWheel - velRightWheel;
   double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
   prevPrevDiff = prevDiff;
   prevDiff = currDifference;
   
   int pos = 0;
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
   }
   
   int e = pos-target;
   
   float debt = (e-eprev)/deltaT;

   eintegral = eintegral + e*deltaT;
   
   float u_left = kp_left*e + kd_left*debt + ki_left*eintegral;
   u_left -= (int)(avgDifference*DRIFT_MULTIPLIER);
   float u_right = kp_right*e + kd_right*debt + ki_right*eintegral;
   u_right += (int)(avgDifference*DRIFT_MULTIPLIER);
   eprev = e;
   
   pwr_left = fabs(u_left);
   pwr_right = fabs(u_right);
      
   pwr_left = constrain(pwr_left,80,150);
   pwr_right = constrain(pwr_right,80,150);
   
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
nh.advertise(leftPwm);
nh.advertise(rightPwm);
nh.subscribe(sub);
}

void loop() {
nh.spinOnce();
currentMillis = millis();

left_pwm_msg.data = pwr_left;
right_pwm_msg.data = pwr_right;
leftPwm.publish(&left_pwm_msg);
rightPwm.publish(&right_pwm_msg);
updateVelocity();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
    calc_vel_right_wheel();
    calc_vel_left_wheel();
 
  }
 
  double a = cmd_vel.linear.x;
  double c = cmd_vel.angular.z;

  if(a > 0 && c == 0){ //straight
  analogWrite(9,pwr_left);
  analogWrite(10,pwr_right);
    teleop(1,0,0,1);
  }
  else if(a < 0 && c == 0){ //back
  analogWrite(9,pwr_left);
  analogWrite(10,pwr_right);
    teleop(0,1,1,0);
  }
  else if(a == 0 && c == 0){ //stop
    analogWrite(9,0);
    analogWrite(10,0);
    teleop(0,0,0,0);
  }
  else if(a == 0 && c > 0){ //left
  analogWrite(9,pwr_left);
  analogWrite(10,pwr_right);
    teleop(0,1,0,1);
  }
  else if(a == 0 && c < 0){ //right
  analogWrite(9,pwr_left);
  analogWrite(10,pwr_right);
    teleop(1,0,1,0);
  }

}
