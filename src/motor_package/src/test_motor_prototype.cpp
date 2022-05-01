#include "ros/ros.h"
#include "wiringPi.h"
#include <softPwm.h> 

#define ENCODER_RATIO 0.0924
#define ROBOT_WIDTH 0.2
#define WHEEL_RADIUS 0.06

#define PIN_ENCODER_A_LEFT 7
#define PIN_ENCODER_B_LEFT 8

#define PIN_ENCODER_A_RIGHT 5
#define PIN_ENCODER_B_RIGHT 6


//************************* global vars. ***************************//
long encoder_position_left = 0;
long encoder_position_right = 0;

double linear_velocity_req = 0;
double angular_velocity_req = 0;

double x;
double z;


//****************************** enum & class def. ***************************************//
enum LR 
{
  LEFT,
  RIGHT
};


class Wheel 
{
private:
  // wheel infos
  LR left_or_right;
  double speed_req;
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
    speed_req = 0;
    current_speed = 0;
    pwm_output = 0;
    previous_error_speed = 0;
    
   softPwmCreate(PIN_PWM, 0, 1000);
  }

  void getTargetSpeed();

  void getCurrentSpeed(double delta_time);

  void getPwmOutput(double delta_time);

  void controlMotor();

};

//****************************** class func. def. ***************************************//

void Wheel::getTargetSpeed()
{
  if (this->left_or_right == LR::LEFT) {
    speed_req = (x - z * ROBOT_WIDTH * 0.5) / WHEEL_RADIUS;
    //ROS_INFO_STREAM("ddori : target_speed left=" <<speed_req );
  }
  else if (this->left_or_right == LR::RIGHT) {
    speed_req = (x + z * ROBOT_WIDTH * 0.5) / WHEEL_RADIUS;
    //ROS_INFO_STREAM("ddori : target_speed right=" <<speed_req );
  }
  else {
    ROS_ERROR("error!");
  }
  // ROS_INFO_STREAM("ddori : target_speed left=" <<tgt_speed_left << "    rigght=" << tgt_speed_right);
  
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
    //ROS_ERROR("error!");

  }

  return;
}

void Wheel::getPwmOutput(double delta_time)
{
  // Kp, Ki, Kd, Kc 등 상수 (혹은 파라미터 리시브)

  const double Kp = 4.5;

  double error_speed = speed_req - current_speed; //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  this->pwm_output
  = error_speed * Kp;
  return;
}

void Wheel::controlMotor()
{
  if (this->left_or_right == LR::LEFT) 
  {
    ROS_INFO("left: %lf / %lf", this->current_speed, this->pwm_output);

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
    ROS_INFO("right: %lf / %lf", this->current_speed, this->pwm_output);
    
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
    //ROS_ERROR("error!");
  }

  return;
}



//************************* user-defined func. ***************************//


void leftEncoderA() {
  encoder_position_left += (digitalRead(PIN_ENCODER_A_LEFT) == digitalRead(PIN_ENCODER_B_LEFT)) ? 1 : -1;
  //ROS_INFO("encoder_position_left_A, position: %ld", encoder_position_left);
  return;
}

void leftEncoderB() {
  encoder_position_left += (digitalRead(PIN_ENCODER_A_LEFT) == digitalRead(PIN_ENCODER_B_LEFT)) ? -1 : 1;
  //ROS_INFO("encoder_position_left_B, position: %ld", encoder_position_left);
  return;
}

void rightEncoderA() {
  encoder_position_right += (digitalRead(PIN_ENCODER_A_RIGHT) == digitalRead(PIN_ENCODER_B_RIGHT)) ? 1 : -1;
  //ROS_INFO("encoder_position_right_A, position: %ld", encoder_position_right);
  return;
}

void rightEncoderB() {
  encoder_position_right += (digitalRead(PIN_ENCODER_A_RIGHT) == digitalRead(PIN_ENCODER_B_RIGHT)) ? -1 : 1;
  //ROS_INFO("encoder_position_right_B, position: %ld", encoder_position_right);
  return;
}

void msgCallBack() {

}

//************************* main func. starts ***************************//

int main(int argc, char **argv)
{
  // ros setups
  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;
    // 대충 섭스크라이브 준비하는 코드 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  x = 5;
  z = 0;


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

  while(ros::ok())
  {
    double delta_time = (ros::Time::now() - previous_time).toSec();

    if (delta_time > LOOPTIME)
    {
      previous_time = ros::Time::now();     

      wheel_left.getTargetSpeed();
      wheel_right.getTargetSpeed();

      wheel_left.getCurrentSpeed(delta_time);
      wheel_right.getCurrentSpeed(delta_time);

      wheel_left.previous_position = encoder_position_left;
      wheel_right.previous_position = encoder_position_right;

      wheel_left.getPwmOutput(delta_time);
      wheel_right.getPwmOutput(delta_time);

      wheel_left.controlMotor();
      wheel_right.controlMotor();

      //ROS_INFO("left: %lf / %lf", wheel_left.current_speed, wheel_left.pwm_output);
      //ROS_INFO("right: %lf / %lf", wheel_right.current_speed, wheel_right.pwm_output);
    } // end of if
  } // end of while (루프타임 추가?)

  return 0;
} // end of main
