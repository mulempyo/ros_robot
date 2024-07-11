/*
Author: Kim Won Jun
Publish: /left_ticks, /right_ticks
Subscribe: /cmd_vel
this code trans PID_publish_ticks_teleop arduino code to OOP
*/




#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

namespace motor_control{

  extern std_msgs::Int32 right_wheel_tick_count;
  extern std_msgs::Int32 left_wheel_tick_count;
  extern boolean Direction_left;
  extern boolean Direction_right;
  extern const int encoder_minimum;
  extern const int encoder_maximum;
  extern volatile int left_posi;
  extern volatile int right_posi;

  class MotorControl{
    private:
    const int enA;
    const int enB;
    const int in1;
    const int in2;
    const int in3;
    const int in4;
    float left_out;
    float right_out;
    float back_left_out;
    float back_right_out;
    float lastCmdVelReceived;
    int left_pos;
    int right_pos;
    float target;
    float left_debt;
    float right_debt;
    float left_eprev;
    float right_eprev;
    float left_eintegral;
    float right_eintegral;
    int left_e;
    int right_e;
    float kp_left;
    float ki_left;
    float kd_left;
    float kp_right;
    float ki_right;
    float kd_right;
    float ang_kp_left;
    float ang_ki_left;
    float ang_kd_left;
    float ang_kp_right;
    float ang_ki_right;
    float ang_kd_right;

    public:
    float pwr_left;
    float pwr_right;
    float ang_pwr_left;
    float ang_pwr_right;
    const int interval;
    long previousMillis;
    long currentMillis;
    MotorControl(
    const int enA_=9,
    const int enB_=10,
    const int in1_=5,
    const int in2_=6,
    const int in3_=7, 
    const int in4_=8,
    float lastCmdVelReceived_= 0,
    const int interval_ = 30,
    long previousMillis_ = 0,
    long currentMillis_ = 0,
    int left_pos_ = 0,
    int right_pos_ = 0,
    float target_ = 0.075,
    float kp_left_ = 0.2,
    float ki_left_ = 0.000000000001,
    float kd_left_ = 0.2,
    float kp_right_ = 0.1,
    float ki_right_ = 0.000000000001,
    float kd_right_ = 0.22,
    float ang_kp_left_ = 0.17,
    float ang_ki_left_ = 0.0000000000004,
    float ang_kd_left_ = 0.5,
    float ang_kp_right_ = 0.05,
    float ang_ki_right_ = 0.00000000000005,
    float ang_kd_right_ = 0.08);

    ~MotorControl();

    float leftSpeed();
    float rightSpeed();
    void teleop(int an1, int an2, int an3, int an4);
    static void calc_pwm_values(const geometry_msgs::Twist& msg);
    void updateVelocity();
    static void left_wheel_tick();
    static void right_wheel_tick();
 };

}

#endif
