/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class BaseTeleop
{
public:
	BaseTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
	static int ButtonRB;
};

int BaseTeleop::AxisLeftThumbX = 0;
int BaseTeleop::AxisLeftThumbY = 1;
int BaseTeleop::AxisRightThumbX = 2;
int BaseTeleop::ButtonRB = 4;

BaseTeleop::BaseTeleop()
{


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

	nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
	nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
	nh_.getParam("AxisRightThumbX", AxisRightThumbX);
	nh_.getParam("ButtonRB", ButtonRB);

}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;

	if (joy->buttons[ButtonRB] != 0)
	{
		twist.linear.x = 0.5 * joy->axes[AxisLeftThumbY];
		twist.linear.y = -0.5 * joy->axes[AxisLeftThumbX];
		twist.angular.z = 0.25 * joy->axes[AxisRightThumbX];
	}
	else
	{
		twist.linear.x = 5.0 * joy->axes[AxisLeftThumbY];
		twist.linear.y = -5.0 * joy->axes[AxisLeftThumbX];
		twist.angular.z = 3.0 * joy->axes[AxisRightThumbX];
	}

	vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_teleop_joy");

  BaseTeleop baseTeleop;

  ros::spin();
}




