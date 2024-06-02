#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

#define LEFT_TICKS_PER_REVOLUTION 1700 //tick publish in 1 cycle
#define RIGHT_TICKS_PER_REVOLUTION 1800 //tick publish in 1 cycle
// Create odometry data publishers
ros::Publisher odom_data_pub_quat;

nav_msgs::Odometry odom;

const double PI = 3.141592;

const double WHEEL_DIAMETER = 0.067; // Wheel radius in meters
const double WHEEL_BASE = 0.212; // Center of left tire to center of right tire

double distanceLeft;
double distanceRight;
double dist;
double dth;
double dx;
double dy;
double x=0;
double y=0;
double th=0;
double dt;
double vx=0;
double vth=0;
double left_compensate;
double right_compensate;
 
ros::Time current_time;
ros::Time last_time;

int leftTicks;
int rightTicks;

using namespace std;
 
// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int32& leftCount) {
  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {
         
     leftTicks = (leftCount.data - lastCountL);
 
    if (leftTicks > 2147483000) {
      leftTicks -= 4294967295;
      distanceLeft += (leftTicks/LEFT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER;
    }
    else if (leftTicks < -2147483000) {
      leftTicks += 4294967295;
      distanceLeft += (leftTicks/LEFT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER;
    }
    else{}
    distanceLeft = (leftCount.data/LEFT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER * left_compensate;
 }
  lastCountL = leftCount.data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int32& rightCount) {
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {
 
     rightTicks = rightCount.data - lastCountR;
     
    if (rightTicks > 2147483000) {
      rightTicks -= 4294967295;
      distanceRight += (rightTicks/RIGHT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER;
    }
    else if (rightTicks < -2147483000) {
      rightTicks += 4294967295;
      distanceRight += (rightTicks/RIGHT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER;
    }
    else{}
    distanceRight = (rightCount.data/RIGHT_TICKS_PER_REVOLUTION)*PI*WHEEL_DIAMETER * right_compensate;
  }
  lastCountR = rightCount.data;
}
 
// Update odometry information
void update_odom() { 
  
   current_time = ros::Time::now();
   dt =(current_time-last_time).toSec();

   dist = ((distanceRight + distanceLeft) / 2);
   
   dth = (distanceRight-distanceLeft)/WHEEL_BASE;

   dx = cos(dth)*dist;
   dy = sin(dth)*dist;

   x += dx;
   y += dy;
   th += dth;

   vx = dist/dt;
   vth = dth/dt;

   geometry_msgs::Quaternion odom_quat =tf::createQuaternionMsgFromYaw(th);

   odom.header.stamp = current_time;
   odom.header.frame_id = "odom";
   odom.child_frame_id = "base_footprint";
 
   odom.pose.pose.position.x = x; 
   odom.pose.pose.position.y = y;
   odom.pose.pose.orientation = odom_quat;

   if (dt > 0) {
     odom.twist.twist.linear.x = vx;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = vth;
   } else {
     odom.twist.twist.linear.x = 0;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = 0;
   }
 
   last_time = current_time;
  

   for(int i = 0; i<36; i++) {
     if(i == 0 || i == 7 || i == 14) {
       odom.pose.covariance[i] = 0.01;
      }
      else if (i == 21 || i == 28 || i== 35) {
        odom.pose.covariance[i] = 0.1;
      }
      else {
        odom.pose.covariance[i] = 0;
      }
   } 
 
   odom_data_pub_quat.publish(odom);
}
 
int main(int argc, char **argv) {
   
  // Launch ROS and create a node
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle node;

  ros::param::get("/odom_pub/left_compensate",left_compensate);
  ros::param::get("/odom_pub/right_compensate",right_compensate);

  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right);
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left);
  
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Rate r(30);
   while(ros::ok()){
     update_odom();
     ros::spinOnce();
     r.sleep();
   }
   return 0;
}







 
