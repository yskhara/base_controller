/*
 * useless_planner.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: yusaku
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class UselessPlanner
{
public:
	UselessPlanner();

	//void SetMaximumAcceleration(double acc);
	//void SetMaximumVelocity(double vel);

private:
	void PathCallback(const nav_msgs::Path::ConstPtr& msg);
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void AbortCallback(const std_msgs::Bool::ConstPtr& msg);
	void LinToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
	void AngToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	//void CalcWheelSpeed(double actualDt);

	double getYawFromQuat(const geometry_msgs::Quaternion& quat)
	{
		tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		return yaw;
	}

	double vel_lim_lin;
	double vel_lim_ang;

	double acc_lim_lin;
	double acc_lim_ang;

	double base_vel_k_lin;
	double base_vel_k_ang;

	double lin_goal_tolerance;
	double ang_goal_tolerance;

	double lin_waypoint_tolerance;

	bool planning = false;

	//double MaximumVelocity;

	ros::NodeHandle nh;

	ros::Subscriber path_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber abort_sub;

	//ros::Subscriber lin_goal_tolerance_sub;
	//ros::Subscriber ang_goal_tolerance_sub;

	//ros::Subscriber lin_waypoint_tolerance_sub;
	//ros::Subscriber ang_waypoint_tolerance_sub;

	ros::Publisher goal_reached_pub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher path_pub;
	ros::Timer control_tim;

	//double targetVelX;
	//double targetVelY;
	//double targetRotZ;

	//double targetTime;

	//double lastTarget[3];// = {0.0, 0.0, 0.0};
	//std_msgs::Int16MultiArray motorCmdVel_msg;
	std_msgs::Bool goal_reached_msg;
	geometry_msgs::Twist cmd_vel_msg;

	geometry_msgs::Pose2D last_target_msg;
	geometry_msgs::Pose2D last_pose_msg;

	//nav_msgs::Path path_msg;
	nav_msgs::Path target_path;
	int target_index = 0;
};

UselessPlanner::UselessPlanner(void)
{
	auto private_nh = ros::NodeHandle("~/UselessPlanner");
	//this->base_max_vel_lin = 1.0;
	//this->base_max_vel_ang = 3.0;

	//this->base_vel_k_lin = 1.0;
	//this->base_vel_k_ang = 1.0;

	//this->lin_goal_tolerance = 0.03;
	//this->ang_goal_tolerance = 0.02;

	private_nh.param("vel_lim_lin", this->vel_lim_lin, 1.0);
	private_nh.param("vel_lim_ang", this->vel_lim_ang, 3.0);

	ROS_INFO("vel_lim_lin : %f", this->vel_lim_lin);
	ROS_INFO("vel_lim_ang : %f", this->vel_lim_ang);

	private_nh.param("acc_lim_lin", this->acc_lim_lin, 1.0);
	private_nh.param("acc_lim_ang", this->acc_lim_ang, 1.0);

	private_nh.param("base_vel_k_lin", this->base_vel_k_lin, 2.0);
	private_nh.param("base_vel_k_ang", this->base_vel_k_ang, 0.5);

	ROS_INFO("base_vel_k_lin : %f", this->base_vel_k_lin);
	ROS_INFO("base_vel_k_ang : %f", this->base_vel_k_ang);

	private_nh.param("goal_tolerance_lin", this->lin_goal_tolerance, 0.05);					// 5 centimetre
	private_nh.param("goal_tolerance_ang", this->ang_goal_tolerance, (M_PI * 5.0 / 180.0));	// \pm 5 deg
	private_nh.param("wp_tolerance_lin", this->lin_waypoint_tolerance, 0.50);			// 50 centimetre

	ROS_INFO("lin_goal_tolerance : %f", this->lin_goal_tolerance);
	ROS_INFO("ang_goal_tolerance : %f", this->ang_goal_tolerance);
	ROS_INFO("lin_waypoint_tolerance : %f", this->lin_waypoint_tolerance);


	path_sub = nh.subscribe<nav_msgs::Path>("target_path", 10, &UselessPlanner::PathCallback, this);
	odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &UselessPlanner::OdomCallback, this);

	abort_sub = nh.subscribe<std_msgs::Bool>("abort", 10, &UselessPlanner::AbortCallback, this);

	//lin_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("lin_tolerance", 10, &UselessPlanner::LinToleranceCallback, this);
	//ang_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("ang_tolerance", 10, &UselessPlanner::AngToleranceCallback, this);

	goal_reached_pub = nh.advertise<std_msgs::Bool>("goal_reached", 1);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

	control_tim = nh.createTimer(ros::Duration(0.02), &UselessPlanner::TimerCallback, this);


	target_path.header.frame_id = "map";
	target_path.header.stamp = ros::Time::now();

	std::vector<geometry_msgs::PoseStamped> _poses(50);
	//_poses.clear();

	double x = 0.0;
	double y = 0.0;
	//geometry_msgs::PoseStamped _pose;

	//_pose.header.frame_id = "map";
	//_pose.header.stamp = ros::Time::now();


	int i = 0;

	for(; i < 20; i++)
	{
		_poses.at(i).header.frame_id = "map";
		_poses.at(i).header.stamp = ros::Time::now();

		_poses.at(i).pose.position.x = x;
		_poses.at(i).pose.position.y = y;
		_poses.at(i).pose.orientation.z = 1;
		_poses.at(i).pose.orientation.w = 0;

		x += 0.1;
	}

	//while(y < 1.0)
	for(; i < 30; i++)
	{
		_poses.at(i).header.frame_id = "map";
		_poses.at(i).header.stamp = ros::Time::now();

		_poses.at(i).pose.position.x = x;
		_poses.at(i).pose.position.y = y;
		_poses.at(i).pose.orientation.z = 1;
		_poses.at(i).pose.orientation.w = 0;

		x += 0.1;
		y += 0.1;
	}

	//while(y < 4.0)
	for(; i < 50; i++)
	{
		_poses.at(i).header.frame_id = "map";
		_poses.at(i).header.stamp = ros::Time::now();

		_poses.at(i).pose.position.x = x;
		_poses.at(i).pose.position.y = y;
		_poses.at(i).pose.orientation.z = 1;
		_poses.at(i).pose.orientation.w = 0;

		y += 0.1;
	}


	target_path.poses = _poses;
	path_pub.publish(target_path);
}

void UselessPlanner::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	// TODO : generate finer path for better path tracking characteristics

	this->target_path = *msg;
	this->target_index = 0;

	this->planning = true;
}

void UselessPlanner::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double yaw = getYawFromQuat(msg->pose.pose.orientation);

	this->last_pose_msg.x = msg->pose.pose.position.x;
	this->last_pose_msg.y = msg->pose.pose.position.y;
	this->last_pose_msg.theta = yaw;
}

void UselessPlanner::AbortCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		this->planning = false;
		this->last_target_msg.x 		= this->last_pose_msg.x;
		this->last_target_msg.y 		= this->last_pose_msg.y;
		this->last_target_msg.theta 	= this->last_pose_msg.theta;

		this->cmd_vel_msg.linear.x = 0;
		this->cmd_vel_msg.linear.y = 0;
		this->cmd_vel_msg.angular.z = 0;

		cmd_vel_pub.publish(cmd_vel_msg);
	}
}

void UselessPlanner::TimerCallback(const ros::TimerEvent& event)
{
	if(!this->planning)
	{
		this->cmd_vel_msg.linear.x = 0;
		this->cmd_vel_msg.linear.y = 0;
		this->cmd_vel_msg.angular.z = 0;

		cmd_vel_pub.publish(cmd_vel_msg);

		return;
	}

	geometry_msgs::PoseStamped target_pose;
	double vel_world_x, vel_world_y, vel_z;

	while(1)
	{
		// 到達判定

		target_pose = this->target_path.poses.at(this->target_index);

		vel_world_x = (target_pose.pose.position.x - this->last_pose_msg.x);
		vel_world_y = (target_pose.pose.position.y - this->last_pose_msg.y);
		vel_z = (getYawFromQuat(target_pose.pose.orientation) - this->last_pose_msg.theta);

		if(vel_z > M_PI)
		{
			vel_z -= (2 * M_PI);
		}

		//if(vel_z < -M_PI)
		//{
		//	vel_z += (2 * M_PI);
		//}

		if(this->target_index < (this->target_path.poses.size() - 1)
				&& (hypot(vel_world_x, vel_world_y) < this->lin_waypoint_tolerance))
		{
			// waypoint reached
			//this->goal_reached_msg.data = true;
			//this->planning = false;
			this->target_index++;
			continue;
		}

		if(this->target_index == (this->target_path.poses.size() - 1)
				&& (hypot(vel_world_x, vel_world_y) < this->lin_goal_tolerance)
				&& (fabs(vel_z) < this->ang_goal_tolerance))
		{
			// goal reached
			this->planning = false;

			this->cmd_vel_msg.linear.x = 0.0;
			this->cmd_vel_msg.linear.y = 0.0;
			this->cmd_vel_msg.angular.z = 0.0;
			cmd_vel_pub.publish(cmd_vel_msg);

			this->goal_reached_msg.data = true;
			this->goal_reached_pub.publish(this->goal_reached_msg);

			return;
		}

		this->goal_reached_msg.data = false;
		//this->goal_reached_pub.publish(this->goal_reached_msg);
		break;
	}

	//

	vel_world_x *= this->base_vel_k_lin;
	vel_world_y *= this->base_vel_k_lin;
	vel_z *= this->base_vel_k_ang;

	double vel_norm = hypot(vel_world_x, vel_world_y);
	if(vel_norm > this->vel_lim_lin)
	{
		double r = this->vel_lim_lin / vel_norm;
		vel_world_x *= r;
		vel_world_y *= r;
	}

	double yaw = this->last_pose_msg.theta;
	double vel_x = (vel_world_x * cos(-yaw)) - (vel_world_y * sin(-yaw));
	double vel_y = (vel_world_x * sin(-yaw)) + (vel_world_y * cos(-yaw));

	this->cmd_vel_msg.linear.x = vel_x;
	this->cmd_vel_msg.linear.y = vel_y;

	double ang_norm = fabs(vel_z);
	if(ang_norm > this->vel_lim_ang)
	{
		double r = this->vel_lim_ang / ang_norm;
		vel_z *= r;
	}

	this->cmd_vel_msg.angular.z = vel_z;//vel_z;

	cmd_vel_pub.publish(cmd_vel_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "useless_planner");

	UselessPlanner *uselessPlanner = new UselessPlanner();
	ROS_INFO("useless_planner node has started.");



	ros::spin();
	ROS_INFO("useless_planner node has been terminated.");
}

