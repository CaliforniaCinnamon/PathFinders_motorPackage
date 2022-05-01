#include <wiringPi.h>
#include "ros/ros.h"

long encoder;

int wireA=23; //BCM 17
int wireB=24; //BCM 27
 
 
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
 
ros::init(argc, argv, "test_motor_cpp");
ros::NodeHandle n;

wiringPiSetupGpio();

pinMode(wireA, INPUT);
pinMode(wireB, INPUT);

ROS_INFO("start!");
 
wiringPiISR(wireA, INT_EDGE_BOTH, &A);
wiringPiISR(wireB, INT_EDGE_BOTH, &B);
 
double position_rad = 0;

while(ros::ok()) { 
  position_rad = encoder * 0.0924;
  ROS_INFO("position in radian = %lf", position_rad);
  delay(100);
} 
}