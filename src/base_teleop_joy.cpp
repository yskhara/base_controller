/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

static constexpr int LeftThumbY = 1;
static constexpr int LeftThumbX = 0;
static constexpr int RightThumbX = 2;

class BaseTeleop
{
public:
	BaseTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


BaseTeleop::BaseTeleop():
  linear_(1),
  angular_(2)
{


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  twist.linear.x = -100.0 * joy->axes[LeftThumbX];
  twist.linear.y = 100.0 * joy->axes[LeftThumbY];
  twist.angular.z = -30.0 * joy->axes[RightThumbX];

  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_teleop_joy");

  BaseTeleop baseTeleop;

  ros::spin();
}




