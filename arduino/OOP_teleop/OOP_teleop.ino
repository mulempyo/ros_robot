#include "motor_control.h"
#include <Arduino.h>
#include <util/atomic.h>

#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11

#define WHEEL_DIAMETER 0.067 //unit:m
#define LEFT_TICKS_PER_REVOLUTION 1700 //tick publish in 1 cycle
#define RIGHT_TICKS_PER_REVOLUTION 1800 //tick publish in 1 cycle

#define TURN_LEFT_LWHEEL_COMPENSATION 8
#define TURN_LEFT_RWHEEL_COMPENSATION 10
#define TURN_RIGHT_LWHEEL_COMPENSATION 3
#define TURN_RIGHT_RWHEEL_COMPENSATION 1
#define BACK_LWHEEL_COMPENSATION 5
#define BACK_RWHEEL_COMPENSATION 3
#define STRAIGHT_LWHEEL_COMPENSATION 5
#define STRAIGHT_RWHEEL_COMPENSATION 3

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel;

namespace motor_control {
  ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
  ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
  std_msgs::Int32 right_wheel_tick_count;
  std_msgs::Int32 left_wheel_tick_count;
  
  boolean Direction_left = true;
  boolean Direction_right = true;
  const int encoder_minimum = -2147483648;
  const int encoder_maximum = 2147483647;
  volatile int left_posi = 0;
  volatile int right_posi = 0;

  MotorControl::MotorControl(
    const int enA_,
    const int enB_,
    const int in1_,
    const int in2_,
    const int in3_, 
    const int in4_,
    float lastCmdVelReceived_,
    const int interval_,
    long previousMillis_,
    long currentMillis_,
    int left_pos_,
    int right_pos_,
    float target_,
    float kp_left_,
    float ki_left_,
    float kd_left_,
    float kp_right_,
    float ki_right_,
    float kd_right_,
    float ang_kp_left_,
    float ang_ki_left_,
    float ang_kd_left_,
    float ang_kp_right_,
    float ang_ki_right_,
    float ang_kd_right_)
    : enA(enA_), enB(enB_), in1(in1_), in2(in2_), in3(in3_), in4(in4_),
      lastCmdVelReceived(lastCmdVelReceived_), interval(interval_), 
      previousMillis(previousMillis_), currentMillis(currentMillis_), 
      left_pos(left_pos_), right_pos(right_pos_), target(target_), 
      kp_left(kp_left_), ki_left(ki_left_), kd_left(kd_left_), 
      kp_right(kp_right_), ki_right(ki_right_), kd_right(kd_right_), 
      ang_kp_left(ang_kp_left_), ang_ki_left(ang_ki_left_), ang_kd_left(ang_kd_left_), 
      ang_kp_right(ang_kp_right_), ang_ki_right(ang_ki_right_), ang_kd_right(ang_kd_right_) {}     

  

  float MotorControl::leftSpeed() {
    long previousTime = 0;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      left_pos = left_posi;
    }
    float left_speed = (left_pos / LEFT_TICKS_PER_REVOLUTION) * PI * WHEEL_DIAMETER / ((float) deltaTime * 1000);
    previousTime = currentTime;
    return left_speed;
  }

  float MotorControl::rightSpeed() {
    long previousTime = 0;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      right_pos = right_posi;
    }
    float right_speed = (right_pos / RIGHT_TICKS_PER_REVOLUTION) * PI * WHEEL_DIAMETER / ((float) deltaTime * 1000);
    previousTime = currentTime;
    return right_speed;
  }

  void MotorControl::teleop(int an1, int an2, int an3, int an4) {
    digitalWrite(in1, an1);
    digitalWrite(in2, an2);
    digitalWrite(in3, an3);
    digitalWrite(in4, an4);
  }

  void MotorControl::updateVelocity() {
    long prevT = 0;
    long currT = millis();
    float deltaT = ((float)(currT - prevT)) * 1000;
    prevT = currT;

    left_e = target - leftSpeed();
    right_e = target - rightSpeed();

    left_debt = (left_e - left_eprev) / deltaT;
    right_debt = (right_e - right_eprev) / deltaT;

    left_eintegral = left_eintegral + left_e * deltaT;
    right_eintegral = right_eintegral + right_e * deltaT;

    float u_left = kp_left * left_e + kd_left * left_debt + ki_left * left_eintegral;
    float u_right = kp_right * right_e + kd_right * right_debt + ki_right * right_eintegral;

    float ang_u_left = ang_kp_left * left_e + ang_kd_left * left_debt + ang_ki_left * left_eintegral;
    float ang_u_right = ang_kp_right * right_e + ang_kd_right * right_debt + ang_ki_right * right_eintegral;

    left_eprev = left_e;
    right_eprev = right_e;

    pwr_left = fabs(u_left);
    pwr_right = fabs(u_right);
    ang_pwr_left = fabs(ang_u_left);
    ang_pwr_right = fabs(ang_u_right);

    pwr_left = constrain(pwr_left, 80, 255);
    pwr_right = constrain(pwr_right, 80, 255);
    ang_pwr_left = constrain(ang_pwr_left, 80, 255);
    ang_pwr_right = constrain(ang_pwr_right, 80, 255);
  }

  void MotorControl::left_wheel_tick() {
    int val = digitalRead(ENC_IN_LEFT_B);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (val > 0) {
        left_posi++;
      } else {
        left_posi--;
      }
    }
    if (val == LOW) {
      Direction_left = true;
    } else {
      Direction_left = false;
    }

    if (Direction_left) {
      if (left_wheel_tick_count.data == encoder_maximum) {
        left_wheel_tick_count.data = encoder_minimum;
      } else {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          left_wheel_tick_count.data++;
        }
      }
    } else {
      if (left_wheel_tick_count.data == encoder_minimum) {
        left_wheel_tick_count.data = encoder_maximum;
      } else {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          left_wheel_tick_count.data--;
        }
      }
    }
  }

  void MotorControl::right_wheel_tick() {
    int val = digitalRead(ENC_IN_RIGHT_B);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (val > 0) {
        right_posi++;
      } else {
        right_posi--;
      }
    }
    if (val == LOW) {
      Direction_right = false;
    } else {
      Direction_right = true;
    }

    if (Direction_right) {
      if (right_wheel_tick_count.data == encoder_maximum) {
        right_wheel_tick_count.data = encoder_minimum;
      } else {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          right_wheel_tick_count.data++;
        }
      }
    } else {
      if (right_wheel_tick_count.data == encoder_minimum) {
        right_wheel_tick_count.data = encoder_maximum;
      } else {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          right_wheel_tick_count.data--;
        }
      }
    }
  }

   void MotorControl::calc_pwm_values(const geometry_msgs::Twist& msg){
   cmd_vel = msg;
  }

  ros::Subscriber <geometry_msgs::Twist> subCmdVel("cmd_vel",MotorControl::calc_pwm_values);

    MotorControl::~MotorControl() {}
}

void setup() {
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  pinMode(10, OUTPUT); // ENB
  pinMode(8, OUTPUT);  // IN4
  pinMode(7, OUTPUT);  // IN3
  pinMode(6, OUTPUT);  // IN2
  pinMode(5, OUTPUT);  // IN1
  pinMode(9, OUTPUT);  // ENA

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), motor_control::MotorControl::left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), motor_control::MotorControl::right_wheel_tick, RISING);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(motor_control::rightPub);
  nh.advertise(motor_control::leftPub);
  nh.subscribe(motor_control::subCmdVel);
}

void loop() {
  motor_control::MotorControl motorController;
  nh.spinOnce();
  motorController.updateVelocity();
  motorController.leftSpeed();
  motorController.rightSpeed();

  if (millis() - motorController.previousMillis > motorController.interval) {
    motorController.previousMillis = millis();
    motor_control::leftPub.publish(&motor_control::left_wheel_tick_count);
    motor_control::rightPub.publish(&motor_control::right_wheel_tick_count);
  }

  double a = cmd_vel.linear.x;
  double c = cmd_vel.angular.z;

  if (a > 0 && c == 0) { // straight
    analogWrite(9, motorController.pwr_left + STRAIGHT_LWHEEL_COMPENSATION);
    analogWrite(10, motorController.pwr_right - STRAIGHT_RWHEEL_COMPENSATION);
    motorController.teleop(1, 0, 0, 1);
  } else if (a < 0 && c == 0) { // back
    analogWrite(9, motorController.pwr_left + BACK_LWHEEL_COMPENSATION);
    analogWrite(10, motorController.pwr_right - BACK_RWHEEL_COMPENSATION);
    motorController.teleop(0, 1, 1, 0);
  } else if (a == 0 && c == 0) { // stop
    analogWrite(9, 0);
    analogWrite(10, 0);
    motorController.teleop(0, 0, 0, 0);
  } else if (a == 0 && c > 0) { // left
    analogWrite(9, motorController.ang_pwr_left + TURN_LEFT_LWHEEL_COMPENSATION);
    analogWrite(10, motorController.ang_pwr_right - TURN_LEFT_RWHEEL_COMPENSATION);
    motorController.teleop(0, 1, 0, 1);
  } else if (a == 0 && c < 0) { // right
    analogWrite(9, motorController.ang_pwr_left + TURN_RIGHT_LWHEEL_COMPENSATION);
    analogWrite(10, motorController.ang_pwr_right - TURN_RIGHT_RWHEEL_COMPENSATION);
    motorController.teleop(1, 0, 1, 0);
  }
}
