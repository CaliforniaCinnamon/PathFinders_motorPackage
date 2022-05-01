/*
 * @ File: motor_control_prototype_code.cpp
 * @ Version: 0.4.0
 * @ Purpose: subscribe /cmd_vel and contorl velocity of two wheels
 * @ URL: TBD
 * @ License: MIT License
 * @ Maintainer: Won-seok Kim
 **/


#include "ros/ros.h"
#include "wiringPi.h"
#include <softPwm.h>
#include <geometry_msgs/Twist.h> 

#define ENCODER_RATIO        0.0924
#define ROBOT_WIDTH          0.2
#define WHEEL_RADIUS         0.06

#define PIN_ENCODER_A_LEFT   7
#define PIN_ENCODER_B_LEFT   8

#define PIN_ENCODER_A_RIGHT  5
#define PIN_ENCODER_B_RIGHT  6

//**************************** global variables ****************************//
long encoder_position_left = 0;
long encoder_position_right = 0;

double x;
double z;

ros::Publisher pub;

// enum & class def.
enum LR // indicates whether left or right wheel
{
  LEFT,
  RIGHT
};


class Wheel 
{
private:
  // wheel infos
  LR left_or_right;
  double speed_required;
  double current_speed;
  double pwm_output;
  double previous_error_speed;

  // pins
  int PIN_DIR_A;
  int PIN_DIR_B;
  int PIN_PWM;

public:
  long previous_position;

  Wheel (LR _left_or_right, int _PIN_DIR_A, int _PIN_DIR_B, int _PIN_PWM) {
    this->left_or_right = _left_or_right;
    this->PIN_DIR_A = _PIN_DIR_A;
    this->PIN_DIR_B = _PIN_DIR_B;
    this->PIN_PWM = _PIN_PWM;
    previous_position = 0;
    speed_required = 0;
    current_speed = 0;
    pwm_output = 0;
    previous_error_speed = 0;
    
   softPwmCreate(PIN_PWM, 0, 1000);
  }

  void getTargetSpeed();

  void getCurrentSpeed(double delta_time);

  void getPwmOutput(double delta_time, double* gains);

  void controlMotor();

};



// class func. def.

void Wheel::getTargetSpeed()
{
  if (this->left_or_right == LR::LEFT) {
    speed_required = (x - z * ROBOT_WIDTH * 0.5) / WHEEL_RADIUS;
  }
  else if (this->left_or_right == LR::RIGHT) {
    speed_required = (x + z * ROBOT_WIDTH * 0.5) / WHEEL_RADIUS;
  }
  else {
    ROS_ERROR("error!");
  }
  //ROS_INFO_STREAM("ddori : target_speed left=" <<tgt_speed_left << "    rigght=" << tgt_speed_right);
  
  return;
}


void Wheel::getCurrentSpeed(double delta_time)
{
  if (this->left_or_right == LR::LEFT) {
    this->current_speed 
    = static_cast<double>(encoder_position_left - previous_position) * ENCODER_RATIO / delta_time;
  }
  else if (this->left_or_right == LR::RIGHT) {
    this->current_speed 
    = static_cast<double>(encoder_position_right - previous_position) * ENCODER_RATIO / delta_time;
  }
  else {
    ROS_ERROR("error!");
  }

  return;
}

void Wheel::getPwmOutput(double delta_time, double* gains)
{
  double Kp = gains[0];
  double Ki = gains[1];
  double Kd = gains[2];
  double Kc = gains[3];

  ROS_INFO("%f %f %f %f", Kp, Ki, Kd, Kc);

  double error_speed = speed_required - current_speed;
  this->pwm_output
  = Kp * error_speed + Ki * (error_speed - previous_error_speed) * delta_time + Kd * (error_speed - previous_error_speed) * delta_time + Kc;

  previous_error_speed = current_speed;

  return;
}

void Wheel::controlMotor()
{
  //ROS_INFO("pwm output: %f", this->pwm_output);

  if (this->left_or_right == LR::LEFT) 
  {
    if (pwm_output > 0) {
      digitalWrite(PIN_DIR_A, HIGH);
      digitalWrite(PIN_DIR_B, LOW);
    }
    else {
      digitalWrite(PIN_DIR_A, LOW);
      digitalWrite(PIN_DIR_B, HIGH);
    }

    softPwmWrite(PIN_PWM, pwm_output);
  }

  else if (this->left_or_right == LR::RIGHT) 
  {
    if (pwm_output > 0) {
      digitalWrite(PIN_DIR_A, LOW);
      digitalWrite(PIN_DIR_B, HIGH);
    }
    else {
      digitalWrite(PIN_DIR_A, HIGH);
      digitalWrite(PIN_DIR_B, LOW);
    }

    softPwmWrite(PIN_PWM, pwm_output);
  }

  else 
  {
    ROS_ERROR("error!");
  }

  return;
}



// user-defined func.

void leftEncoderA() 
{
  encoder_position_left += (digitalRead(PIN_ENCODER_A_LEFT) == digitalRead(PIN_ENCODER_B_LEFT)) ? 1 : -1;
  //ROS_INFO("encoder_position_left_A, position: %ld", encoder_position_left);
  return;
}

void leftEncoderB() 
{
  encoder_position_left += (digitalRead(PIN_ENCODER_A_LEFT) == digitalRead(PIN_ENCODER_B_LEFT)) ? -1 : 1;
  //ROS_INFO("encoder_position_left_B, position: %ld", encoder_position_left);
  return;
}

void rightEncoderA() 
{
  encoder_position_right += (digitalRead(PIN_ENCODER_A_RIGHT) == digitalRead(PIN_ENCODER_B_RIGHT)) ? 1 : -1;
  //ROS_INFO("encoder_position_right_A, position: %ld", encoder_position_right);
  return;
}

void rightEncoderB() 
{
  encoder_position_right += (digitalRead(PIN_ENCODER_A_RIGHT) == digitalRead(PIN_ENCODER_B_RIGHT)) ? -1 : 1;
  //ROS_INFO("encoder_position_right_B, position: %ld", encoder_position_right);
  return;
}


void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel) 
{
  x = vel->linear.x;
  z = vel->angular.z;
}


// void getGains(double* gains, ros::NodeHandle _nh)
// {
//   if (_nh.hasParam("/Kp")) _nh.getParam("/Kp", gains[0]);
//   else ROS_INFO("error getting Kp!");

//   if (_nh.hasParam("/Ki")) _nh.getParam("/Ki", gains[1]);
//   else ROS_INFO("error getting Ki!");

//   if (_nh.hasParam("/Kd")) _nh.getParam("/Kd", gains[2]);
//   else ROS_INFO("error getting Kd!");

//   if (_nh.hasParam("/Kc")) _nh.getParam("/Kc", gains[0]);
//   else ROS_INFO("error getting Kc!");

//   return;
// }

//***************************************************************************//
//**************************** main func. starts ****************************//
//***************************************************************************//

int main(int argc, char **argv)
{
  // ros setups
  ros::init(argc, argv, "motor_pid_tuning");
  ros::NodeHandle nh;
  //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sub = nh.subscribe("cmd_vel", 100, cmdvelCallback);

  // creating gains to parameter server in case for they don't exist
  nh.setParam("/Kp", 0);
  nh.setParam("/Ki", 0);
  nh.setParam("/Kd", 0);
  nh.setParam("/Kc", 0);

  // gains & params initialization
  double gains[4] = {0, 0, 0, 0}; // Kp, Ki, Kd, Kc respectively

  // wiringPi setups
  wiringPiSetupGpio();

  pinMode(PIN_ENCODER_A_LEFT, INPUT);
  pinMode(PIN_ENCODER_B_LEFT, INPUT);
  pinMode(PIN_ENCODER_A_RIGHT, INPUT);
  pinMode(PIN_ENCODER_B_RIGHT, INPUT);

  wiringPiISR(PIN_ENCODER_A_LEFT, INT_EDGE_BOTH, &leftEncoderA);
  wiringPiISR(PIN_ENCODER_B_LEFT, INT_EDGE_BOTH, &leftEncoderB);
  wiringPiISR(PIN_ENCODER_A_RIGHT, INT_EDGE_BOTH, &rightEncoderA);
  wiringPiISR(PIN_ENCODER_B_RIGHT, INT_EDGE_BOTH, &rightEncoderB);

  // variable & object setups
  const double LOOPTIME = 0.05;
  Wheel wheel_left(LR::LEFT, 23, 24, 25);
  Wheel wheel_right(LR::RIGHT, 27, 17, 22);

  long previous_position_left = encoder_position_left;
  long previous_position_right = encoder_position_right;

  ros::Time previous_time = ros::Time::now();

  // loop starts here
  while(ros::ok())
  {
    double delta_time = (ros::Time::now() - previous_time).toSec();

    //getGains(gains, nh); // get PID gains from parameter server func.

    if (nh.hasParam("/Kp")) nh.getParam("/Kp", gains[0]);
    else ROS_INFO("error getting Kp!");

    if (nh.hasParam("/Ki")) nh.getParam("/Ki", gains[1]);
    else ROS_INFO("error getting Ki!");

    if (nh.hasParam("/Kd")) nh.getParam("/Kd", gains[2]);
    else ROS_INFO("error getting Kd!");

    if (nh.hasParam("/Kc")) nh.getParam("/Kc", gains[3]);
    else ROS_INFO("error getting Kc!");

    if (delta_time > LOOPTIME)
    {
      previous_time = ros::Time::now();     

      wheel_left.getTargetSpeed();
      wheel_right.getTargetSpeed();

      wheel_left.getCurrentSpeed(delta_time);
      wheel_right.getCurrentSpeed(delta_time);

      wheel_left.previous_position = encoder_position_left;
      wheel_right.previous_position = encoder_position_right;

      wheel_left.getPwmOutput(delta_time, gains);
      wheel_right.getPwmOutput(delta_time, gains);

      wheel_left.controlMotor();
      wheel_right.controlMotor();

      ROS_INFO("cmd_vel: %f, %f", x, z);
      //ROS_INFO("left: %lf / %lf", wheel_left.current_speed, wheel_left.pwm_output);
      //ROS_INFO("right: %lf / %lf", wheel_right.current_speed, wheel_right.pwm_output);
    } // end of if

    ros::spinOnce();

  } // end of while

  return 0;
} // end of main