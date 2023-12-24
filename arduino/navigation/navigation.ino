#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

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
 

const int interval = 200;
long previousMillis = 0;
long currentMillis = 0;


const int enA = 9;
const int in1 = 5;
const int in2 = 6;
const int enB = 10;
const int in3 = 7;
const int in4 = 8;
 

const int PWM_INCREMENT = 1;
const int TICKS_PER_REVOLUTION = 1753; //left:1947
const double WHEEL_RADIUS = 0.033;
const double WHEEL_BASE = 0.17;
const double TICKS_PER_METER = 1753;
const int K_P = 278;
const int b = 52;
const int DRIFT_MULTIPLIER = 120;
const int PWM_TURN = 80;
const int PWM_MIN = 80; 
const int PWM_MAX = 100; 
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
double lastCmdVelReceived = 0;
 
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
 
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  lastCmdVelReceived = (millis()/1000);
  pwmLeftReq = K_P * cmdVel.linear.x + b;
  pwmRightReq = K_P * cmdVel.linear.x + b;
 
  if (cmdVel.angular.z != 0.0) {
    
    if (cmdVel.angular.z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
       
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  
  else {
     
   
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel;
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;
    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }
 
  
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
}
 
void set_pwm_values() {
 
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  
  if (pwmLeftReq > 0) { 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmLeftReq < 0) { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
 
  if (pwmRightReq > 0) { 
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(pwmRightReq < 0) { 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { 
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
 
  
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
  analogWrite(enA, pwmLeftOut);
  analogWrite(enB, pwmRightOut);
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
 
  
  nh.getHardware()->setBaud(128000);
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
 
    calc_vel_right_wheel();
    calc_vel_left_wheel();
     
  }
   
  
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
}
