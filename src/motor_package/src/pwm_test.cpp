#include <wiringPi.h>
#include "ros/ros.h"
#include <softPwm.h>

// global variables
long encoder = 0;

// pin nums
const int wireA = 6; //BCM 17
const int wireB = 5; //BCM 27
const int PIN_PWM_OUTPUT = 22;
const int PIN_DIR_A = 17;
const int PIN_DIR_B = 27;
 
 
void A()
{
    encoder += (digitalRead(wireA) == digitalRead(wireB)) ? 1 : -1;
    //ROS_INFO("A, position: %ld", encoder);
    return;
}
 
void B()
{
    encoder += (digitalRead(wireA) == digitalRead(wireB)) ? -1 : 1;
    //ROS_INFO("B, position: %ld", encoder);
    return;
}
 
int main(int argc, char **argv)
{
  //**************************************************//
  // ros node setups
  ros::init(argc, argv, "test_motor_cpp");
  ros::NodeHandle n;

  // wiringPi setups
  wiringPiSetupGpio();

  pinMode(wireA, INPUT);
  pinMode(wireB, INPUT);
  softPwmCreate(PIN_PWM_OUTPUT, 0, 1000);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  digitalWrite(PIN_DIR_A, HIGH);
  digitalWrite(PIN_DIR_B, LOW);
 
  wiringPiISR(wireA, INT_EDGE_BOTH, &A);
  wiringPiISR(wireB, INT_EDGE_BOTH, &B);

  //**************************************************//

  ROS_INFO("start!");
  ros::Rate r(20);

  double position_rad = 0;
  int pwm_output_value = 130;

  ros::Time before_time;
  ros::Time now_time;

  long before_position = 0;

  double current_speed = 0;

  softPwmWrite(PIN_PWM_OUTPUT, pwm_output_value);

  while(ros::ok()) {
    // now를 얻고, 차이를 계산해서 현재 속도를 얻는다.
    // 목표 속도와 현재 속도의 에러만큼 pwm 출력값을 조정해준다.
    // while 떠나기 전에 before 찍어준다.
    // 다른 코드의 오버헤드를 모방한 sleep 코드

    //now_time = ros::Time::now();
    //current_speed = static_cast<dobule>(encoder - before_position) / (now_time - before_time).toSec();

    double RAD_PER_POSITION = 0.0924;
    position_rad = static_cast<double>(encoder) * RAD_PER_POSITION;

    ROS_INFO("position in radian = %lf", position_rad);
    r.sleep();
  } // end of while

} // end of main