/*
 * shutdown_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

// TODO: dynamically subscribe to various shutdown signal sources specified by parameters

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>


class ShutdownController
{
public:
	ShutdownController();

private:
	void BaseShutdownCallback(const std_msgs::Bool::ConstPtr& msg);
	void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void TimerCallback(const ros::TimerEvent& event);

	ros::NodeHandle nh;

	ros::Subscriber baseShutdown_sub;
	ros::Subscriber joy_sub;

	ros::Publisher shutdown_pub;

	ros::Timer pub_tim;

	std_msgs::Bool shutdown_msg;

	int sd_recovering = -1;

	static int ButtonSelect;
	static int ButtonStart;
};

int ShutdownController::ButtonSelect = 6;
int ShutdownController::ButtonStart = 7;

ShutdownController::ShutdownController(void)
{
	baseShutdown_sub = nh.subscribe<std_msgs::Bool>("base/shutdown", 10, &ShutdownController::BaseShutdownCallback, this);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &ShutdownController::JoyCallback, this);

	shutdown_pub = nh.advertise<std_msgs::Bool>("shutdown", 1);

	pub_tim = nh.createTimer(ros::Duration(0.1), &ShutdownController::TimerCallback, this);

	shutdown_msg.data = true;

	nh.getParam("ButtonSelect", ButtonSelect);
	nh.getParam("ButtonStart", ButtonStart);
}

void ShutdownController::BaseShutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		this->shutdown_msg.data = true;
	}
}

void ShutdownController::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	bool start = joy->buttons[ButtonStart];
	bool select = joy->buttons[ButtonSelect];

	if(start || select)
	{
		if(!this->shutdown_msg.data)
		{
			this->shutdown_msg.data = true;
		}

		if(start && select)
		{
			if(this->shutdown_msg.data && sd_recovering < 0)
			{
				sd_recovering = 10;
			}
			//else if(!this->shutdown_msg.data && sd_)
		}
		else
		{
			if(sd_recovering != 0)
			{
				// if not just recovered
				this->shutdown_msg.data = true;
				sd_recovering = -1;
			}
		}
	}
	else
	{
		/*
		if(sd_recovering > 0)
		{
			// if not just recovered
			this->shutdown_msg.data = true;
			//sd_recovering = -1;
		}
		*/
		sd_recovering = -1;
	}
}

void ShutdownController::TimerCallback(const ros::TimerEvent& event)
{
	if(sd_recovering > 0)
	{
		sd_recovering--;
	}
	else if(sd_recovering == 0)
	{
		this->shutdown_msg.data = false;
		//sd_recovering = -1;
	}

	this->shutdown_pub.publish(this->shutdown_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "shutdown_controller");

	ShutdownController *sdController = new ShutdownController();
	ROS_INFO("shutdown_controller node has started.");

	ros::spin();
	ROS_INFO("shutdown_controller node has been terminated.");
}





