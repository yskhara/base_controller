/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

// TODO: make separate nodes for 3 wheels, 4 wheels, etc...

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>

class BaseController
{
public:
	BaseController();

private:
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	void CalcWheelSpeed(double actualDt);

	double MaximumAcceleration;
	double MaximumVelocity;

	ros::NodeHandle nh;

	ros::Subscriber cmdVel_sub;
	ros::Publisher motorCmdVel_pub;
	ros::Timer control_tim;

	double targetVelX;
	double targetVelY;
	double targetRotZ;

	//double targetTime;

	double lastTarget[3];// = {0.0, 0.0, 0.0};
	std_msgs::Int16MultiArray motorCmdVel_msg;
};

BaseController::BaseController(void)
{
	this->MaximumAcceleration = 100;
	this->MaximumVelocity = 80;

	cmdVel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &BaseController::CmdVelCallback, this);

	motorCmdVel_pub = nh.advertise<std_msgs::Int16MultiArray>("motor_cmd_vel", 1);

	control_tim = nh.createTimer(ros::Duration(0.02), &BaseController::TimerCallback, this);

	targetVelX = targetVelY = targetRotZ = 0.0;

	lastTarget[0] = 0.0;
	lastTarget[1] = 0.0;
	lastTarget[2] = 0.0;
	motorCmdVel_msg.data.clear();
}

void BaseController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	this->targetVelX = static_cast<double>(msg->linear.x);
	this->targetVelY = static_cast<double>(msg->linear.y);
	this->targetRotZ = static_cast<double>(msg->angular.z);
	//this->targetTime = msg->header.stamp.toSec();
}

void BaseController::TimerCallback(const ros::TimerEvent& event)
{
	CalcWheelSpeed(event.current_real.toSec() - event.last_real.toSec());

	motorCmdVel_pub.publish(motorCmdVel_msg);
}

void BaseController::CalcWheelSpeed(double actualDt)
{
	double t[3];

	t[0] = (targetVelX * 1)					 	+ 										+ targetRotZ;
	t[1] = (targetVelX * cos( 2 * M_PI / 3)) 	+ (targetVelY * sin( 2 * M_PI / 3)) 	+ targetRotZ;
	t[2] = (targetVelX * cos(-2 * M_PI / 3)) 	+ (targetVelY * sin(-2 * M_PI / 3)) 	+ targetRotZ;

	double k[3] = { 1.0, 1.0, 1.0 };

	for(int i = 0; i < 3; i++)
	{
		//if(fabsf(t[i]) > this->MaximumVelocity)
		//{
			k[i] = this->MaximumVelocity / fabs(t[i]);
		//}
	}

	double _k = std::min({k[0], k[1], k[2], 1.0});

	for(int i = 0; i < 3; i++)
	{
		t[i] *= _k;
	}

	// TODO: accel limiting

	float maxVelDelta = this->MaximumAcceleration * actualDt;

	k[0] = 1.0;
	k[1] = 1.0;
	k[2] = 1.0;
	//k = { 1.0f, 1.0f, 1.0f };

	for(int i = 0; i < 3; i++)
	{
		//float diffabs = fabsf(t[i] - lastTarget[i]);

		//if(acc < diffabs && (acc / diffabs) < k[i])
		//{
			k[i] = maxVelDelta / fabs(t[i] - lastTarget[i]);
		//}
	}

	_k = std::min({k[0], k[1], k[2], 1.0});

	for(int i = 0; i < 3; i++)
	{
		t[i] = lastTarget[i] + ((t[i] - lastTarget[i]) * _k);
	}

	this->motorCmdVel_msg.data.clear();
	for(int i = 0; i < 3; i++)
	{
		this->lastTarget[i] = t[i];
		this->motorCmdVel_msg.data.push_back(static_cast<int16_t>(round(t[i])));
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_controller");

	BaseController *baseController = new BaseController();
	ROS_INFO("base_controller node has started.");

	ros::spin();
	ROS_INFO("base_controller node has been terminated.");
}


