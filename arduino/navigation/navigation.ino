#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <util/atomic.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::NodeHandle nh;

//#define ENC_IN_LEFT_A 2
//#define ENC_IN_RIGHT_A 3
//#define ENC_IN_LEFT_B 4
//#define ENC_IN_RIGHT_B 11
 
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

geometry_msgs::Twist cmd;
geometry_msgs::PoseStamped desired;
geometry_msgs::PoseWithCovarianceStamped amcl;

float pwr_left;
float pwr_right;
float left_debt;
float right_debt;
float left_eprev;
float right_eprev;
float left_eintegral;
float right_eintegral;
float lastCmdVelReceived=0;

float kp_left = 3.1;
float ki_left = 1.2;
float kd_left = 1.2;

float kp_right = 3.1;
float ki_right = 2.0;
float kd_right = 1.02;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

int left_e;
int right_e;
volatile int posi=0;

bool waypointActive = false;

void right_wheel_tick() {
   
  int val = digitalRead(11);
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
   
  int val = digitalRead(4);
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

void update_pose(const geometry_msgs::PoseWithCovarianceStamped &currentAmclPose)
{
    amcl.pose.pose.position.x = currentAmclPose.pose.pose.position.x;
    amcl.pose.pose.position.y = currentAmclPose.pose.pose.position.y;
    amcl.pose.pose.orientation.z = currentAmclPose.pose.pose.orientation.z;
}

void updateGoal(const geometry_msgs::PoseStamped &desiredPose) {
  desired.pose.position.x = desiredPose.pose.position.x;
  desired.pose.position.y = desiredPose.pose.position.y;
  desired.pose.orientation.z = desiredPose.pose.orientation.z;
  waypointActive = true;
  desired = desiredPose;
}

double getDistanceError() {
  double deltaX = desired.pose.position.x - amcl.pose.pose.position.x;
  double deltaY = desired.pose.position.y - amcl.pose.pose.position.y;
  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

double getAngularError() {
  double deltaX = desired.pose.position.x - amcl.pose.pose.position.x;
  double deltaY = desired.pose.position.y - amcl.pose.pose.position.y;
  double thetaBearing = atan2(deltaY, deltaX);
  double angularError = thetaBearing - amcl.pose.pose.orientation.z;
  angularError = (angularError > PI)  ? angularError - (2 * PI) : angularError;
  angularError = (angularError < -PI) ? angularError + (2 * PI) : angularError;
  return angularError;
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

 if(cmd.angular.z > 0){ //left
    if(cmd.linear.x > 0){ //straight and left
      analogWrite(9,pwr_left);
      analogWrite(10,pwr_right);
    }
    else if(cmd.linear.x < 0){ //back and left
     analogWrite(9,pwr_left);
     analogWrite(10,pwr_right);
    }
    else{
      analogWrite(9,0);//turn left stay in the same place
      analogWrite(10,0);
    }
  }

  else if(cmd.angular.z < 0){ //right
    if(cmd.linear.x > 0){ //straight and right
      analogWrite(9,pwr_left);
      analogWrite(10,pwr_right);
    }
    else if(cmd.linear.x < 0){ //back and right
     analogWrite(9,pwr_left);
     analogWrite(10,pwr_right);
    }
    else{
      analogWrite(9,0);//turn right stay in the same place
      analogWrite(10,0); 
    }
  }
  else{
    if(cmd.linear.x > 0){ //straight 
      analogWrite(9,pwr_left);
      analogWrite(10,pwr_right);
    }
    else if(cmd.linear.x < 0){ // back
     analogWrite(9,pwr_left);
     analogWrite(10,pwr_right);
    }
    else{
      lastCmdVelReceived = 0; //stop
    }
  }
 pwr_left = constrain(pwr_left,0,255);
 pwr_right = constrain(pwr_right,0,255);
}
 
void set_pwm_values() {
   int target = 80;
   long prevT = 0;
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
 
  double linearVel = 0;
  double angularVel = 0;

  static bool angle_met = true;
  static bool location_met = true;

  double final_desired_heading_error = desired.pose.orientation.z - amcl.pose.pose.orientation.z;
  double distanceError = getDistanceError();
  double angularError = getAngularError();

    if(abs(getDistanceError()) >= .05)
        {
        location_met = false;
        }
    else if (abs(getDistanceError()) < .03)
        {
        location_met = true;
        }

     angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;
    if (abs(angularError) > .15)
        {
         angle_met = false;
        }
    else if (abs(angularError) < .1)
        {
         angle_met = true;
        }

     if (waypointActive == true && angle_met == false)
        {
         angularVel = 0.3 * angularError;
         linearVel = 0;
         pwr_left= (linearVel - angularVel) * 255 * u_left;
         pwr_right = (linearVel + angularVel) * 255 * u_right;
        }
    else if (waypointActive == true && abs(getDistanceError()) >= .05 && location_met == false)
        {
         linearVel = 0.5 * getDistanceError();
         angularVel = 0;
         pwr_left = (linearVel - angularVel) * 255 * u_left;
         pwr_right = (linearVel + angularVel) * 255 * u_right;
        }
    else{
        location_met = true;
    }

    if (location_met == true && abs(final_desired_heading_error) < .05) //goal reached!
        {
         waypointActive = false;
         cmd.linear.x == 0;
         cmd.angular.z == 0;
        }

    pwr_left = fabs(u_left); 
    pwr_right = fabs(u_right);    

    pwr_left = constrain(pwr_left,0,255);
    pwr_right = constrain(pwr_right,0,255);    


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
ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> subCurrentPose("amcl_pose",10, &update_pose);
ros::Subscriber<geometry_msgs::PoseStamped> subDesiredPose("waypoint_2d", 1, &updateGoal);
 
void setup() {
  pinMode(2 , INPUT_PULLUP);
  pinMode(4 , INPUT);
  pinMode(3 , INPUT_PULLUP);
  pinMode(11 , INPUT);
 
  attachInterrupt(digitalPinToInterrupt(2), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(3), right_wheel_tick, RISING);
   
  
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
 
  analogWrite(9, 0);
  analogWrite(10, 0);
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.subscribe(subCurrentPose);
  nh.subscribe(subDesiredPose);
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
    analogWrite(9,0);
    analogWrite(10,0);
    teleop(0,0,0,0);
  }
  set_pwm_values();
}
