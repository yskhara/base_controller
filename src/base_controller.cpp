/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

class BaseController
{
public:
	BaseController();

	//void SetMaximumAcceleration(double acc);
	//void SetMaximumVelocity(double vel);

private:
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	void CalcWheelSpeed(double actualDt);

	double MaximumAcceleration;
	double MaximumVelocity;
	double VelocityCoefficient;
	double RobotRadius;

	bool InvertX = false;
	bool InvertY = false;
	bool InvertZ = false;

	bool LimitVelocity = true;
	bool LimitAcceleration = true;

	ros::NodeHandle nh;

	ros::Subscriber cmdVel_sub;
	ros::Publisher motorCmdVel_pub;
	ros::Timer control_tim;

	double targetVelX;
	double targetVelY;
	double targetRotZ;

	ros::Time targetTime;

	double lastTarget[3];// = {0.0, 0.0, 0.0};
	std_msgs::Float32MultiArray motorCmdVel_msg;

	//static constexpr double WheelDiameter = 0.127; // in metre
	//static constexpr double MachineRadius = 0.500; // in metre
	//static constexpr double VelocityCoefficient = 20.0 / (M_PI * WheelDiameter);

};

BaseController::BaseController(void)
{
	auto _nh = ros::NodeHandle("~");

	_nh.param("motor_max_acc", this->MaximumAcceleration, 0.0);
	_nh.param("motor_max_vel", this->MaximumVelocity, 0.0);
	_nh.param("motor_vel_coeff", this->VelocityCoefficient, 1 / 0.03);	//radian per metre for the wheels, which is equivalent to the inverse of the wheel radius
	_nh.param("robot_radius", this->RobotRadius, 0.500);

	ROS_INFO("motor_max_acc : %f", this->MaximumAcceleration);
	ROS_INFO("motor_max_vel : %f", this->MaximumVelocity);
	ROS_INFO("motor_vel_coeff : %f", this->VelocityCoefficient);
	ROS_INFO("robot_radius : %f", this->RobotRadius);

	if(this->MaximumVelocity < 0)
	{
		this->LimitVelocity = false;
	}

	if(this->MaximumAcceleration < 0)
	{
		this->LimitAcceleration = false;
	}

	_nh.param("invert_x", this->InvertX, false);
	_nh.param("invert_y", this->InvertY, false);
	_nh.param("invert_z", this->InvertZ, false);

	int ctrl_freq;
	_nh.param("ctrl_freq", ctrl_freq, 20);

	cmdVel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &BaseController::CmdVelCallback, this);

	motorCmdVel_pub = nh.advertise<std_msgs::Float32MultiArray>("motor_cmd_vel", 1);

	control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &BaseController::TimerCallback, this);

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
	this->targetTime = ros::Time::now();//msg->header.stamp.toSec();

	if(this->InvertX)
	{
		this->targetVelX *= -1;
	}
	if(this->InvertY)
	{
		this->targetVelY *= -1;
	}
	if(this->InvertZ)
	{
		this->targetRotZ *= -1;
	}
}

void BaseController::TimerCallback(const ros::TimerEvent& event)
{
	/*
	if(event.current_real.toSec() - this->targetTime.toSec() > 0.15)
	{
		this->targetVelX = 0.0;
		this->targetVelY = 0.0;
		this->targetRotZ = 0.0;
	}
	*/

	CalcWheelSpeed(event.current_real.toSec() - event.last_real.toSec());

	motorCmdVel_pub.publish(motorCmdVel_msg);
}

void BaseController::CalcWheelSpeed(double actualDt)
{
	double t[3];

	// To comply with REP-103, use x-front, y-left, z-up coordinate frame.
	// REP-103 says that yaw angle value should INCREASE when you rotate the robot COUNTER-CLOCKWISE.

	t[0] = -(									  (targetVelY * 1)						+ (targetRotZ * RobotRadius)) * VelocityCoefficient;
	t[1] = -((targetVelX * sin( 2 * M_PI / 3))	+ (targetVelY * cos( 2 * M_PI / 3)) 	+ (targetRotZ * RobotRadius)) * VelocityCoefficient;
	t[2] = -((targetVelX * sin(-2 * M_PI / 3))	+ (targetVelY * cos(-2 * M_PI / 3)) 	+ (targetRotZ * RobotRadius)) * VelocityCoefficient;

	double _k = 1.0;

	if(this->LimitVelocity)
	{
		for(int i = 0; i < 3; i++)
		{
			auto _a = fabs(t[i]);
			if(_a * _k > this->MaximumVelocity)
			{
				_k = this->MaximumVelocity / _a;
                //ROS_WARN("An infeasible velocity command detected! You might want to look into it.");
			}
		}

		for(int i = 0; i < 3; i++)
		{
			t[i] *= _k;
		}
	}

	if(this->LimitAcceleration)
	{
		float maxVelDelta = this->MaximumAcceleration * actualDt;

		_k = 1.0;

		for(int i = 0; i < 3; i++)
		{
			double diffabs = fabs(t[i] - lastTarget[i]);
			if(diffabs * _k > maxVelDelta)
			{
				_k = maxVelDelta / diffabs;

				//ROS_WARN("An infeasible acceleration detected! You might want to look into it.");
			}
		}

		for(int i = 0; i < 3; i++)
		{
			t[i] = lastTarget[i] + ((t[i] - lastTarget[i]) * _k);
		}
	}

	this->motorCmdVel_msg.data.clear();
	for(int i = 0; i < 3; i++)
	{
		this->lastTarget[i] = t[i];
		this->motorCmdVel_msg.data.push_back(static_cast<float>(t[i]));
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


