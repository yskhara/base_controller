/*
 * useless_planner.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: yusaku
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>

class UselessPlanner
{
public:
	UselessPlanner();

	//void SetMaximumAcceleration(double acc);
	//void SetMaximumVelocity(double vel);

private:
	void PathCallback(const nav_msgs::Path::ConstPtr& msg);
	void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
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

	double jerk_lim_lin;
	double jerk_lim_ang;

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

	tf::TransformListener _tflistener;

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

	private_nh.param("jerk_lim_lin", this->jerk_lim_lin, 1.0);
	private_nh.param("jerk_lim_ang", this->jerk_lim_ang, 1.0);

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
	//odom_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &UselessPlanner::PoseCallback, this);

	abort_sub = nh.subscribe<std_msgs::Bool>("abort", 10, &UselessPlanner::AbortCallback, this);

	//lin_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("lin_tolerance", 10, &UselessPlanner::LinToleranceCallback, this);
	//ang_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("ang_tolerance", 10, &UselessPlanner::AngToleranceCallback, this);

	goal_reached_pub = nh.advertise<std_msgs::Bool>("goal_reached", 1);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

	control_tim = nh.createTimer(ros::Duration(0.05), &UselessPlanner::TimerCallback, this);


#if 0

	path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

	target_path.header.frame_id = "map";
	target_path.header.stamp = ros::Time::now();

	std::vector<geometry_msgs::PoseStamped> _poses;
	//_poses.clear();

	double x = 0.0;
	double y = 0.0;

	geometry_msgs::PoseStamped _pose;

	//_pose.header.frame_id = "map";
	//_pose.header.stamp = ros::Time::now();


	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();

	_pose.pose.position.x = 0.0;
	_pose.pose.position.y = 0.0;
	_pose.pose.orientation.z = 0;
	_pose.pose.orientation.w = 1;

	_poses.push_back(_pose);


	_pose.header.frame_id = "map";
	_pose.header.stamp = ros::Time::now();

	_pose.pose.position.x = 0.53;
	_pose.pose.position.y = -4.0;
	_pose.pose.orientation.z = 0;
	_pose.pose.orientation.w = 1;

	_poses.push_back(_pose);

	target_path.poses = _poses;
	path_pub.publish(target_path);
#endif
}

void UselessPlanner::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	// TODO : generate finer path for better path tracking characteristics

	this->target_path = *msg;
	this->target_index = 0;

	this->planning = true;
}

void UselessPlanner::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
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
	double vel_world_x, vel_world_y, vel_norm;
	double vel_z;
	//double vel_t, vel_r, theta;
	//double pose_delta_world_x;
	//double pose_delta_world_y;

	double dt = event.current_real.toSec() - event.last_real.toSec();
	double acc_max = this->acc_lim_lin * dt / 1.5;

	tf::StampedTransform base_link;

	try
	{
		this->_tflistener.waitForTransform("/map", "base_link", ros::Time(0), ros::Duration(0.1));
		this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
	}
	catch(...)
	{
		this->cmd_vel_msg.linear.x = 0;
		this->cmd_vel_msg.linear.y = 0;
		this->cmd_vel_msg.angular.z = 0;

		cmd_vel_pub.publish(cmd_vel_msg);

		return;
	}

	this->last_pose_msg.x = base_link.getOrigin().x();
	this->last_pose_msg.y = base_link.getOrigin().y();
	this->last_pose_msg.theta = tf::getYaw(base_link.getRotation());

	while(1)
	{
		// 到達判定

		target_pose = this->target_path.poses.at(this->target_index);

		vel_world_x = (target_pose.pose.position.x - this->last_pose_msg.x);
		vel_world_y = (target_pose.pose.position.y - this->last_pose_msg.y);
		vel_norm = hypot(vel_world_x, vel_world_y);

		vel_z = (getYawFromQuat(target_pose.pose.orientation) - this->last_pose_msg.theta);

		// 旋回方向最適化
		if(vel_z > M_PI)
		{
			vel_z -= (2 * M_PI);
		}

		if(this->target_index < (this->target_path.poses.size() - 1))
		{
			// current target is a waypoint, not the goal

			if(vel_norm < this->lin_waypoint_tolerance)
			{
				// waypoint reached
				this->target_index++;
				continue;
			}
			else
			{
				// waypoint, still tracking
				// at full speed

				//double _r = this->vel_lim_lin / vel_norm;
				//vel_world_x *= _r;
				//vel_world_y *= _r;

				vel_world_x *= this->base_vel_k_lin;
				vel_world_y *= this->base_vel_k_lin;

				//break;
			}
		}
		else if(this->target_index == (this->target_path.poses.size() - 1))
		{
			// current target is the goal, not a waypoint.
			// final approach
			if( 	(vel_norm														< this->lin_goal_tolerance)
				&&	(fabs(vel_z)													< this->ang_goal_tolerance)
				&&	(hypot(this->cmd_vel_msg.linear.x, this->cmd_vel_msg.linear.y)	< this->acc_lim_lin * dt)
				&&	(abs(this->cmd_vel_msg.angular.z)								< this->acc_lim_ang * dt))
			{
				// goal reached

				this->planning = false;

				this->cmd_vel_msg.linear.x = 0.0;
				this->cmd_vel_msg.linear.y = 0.0;
				this->cmd_vel_msg.angular.z = 0.0;
				cmd_vel_pub.publish(cmd_vel_msg);

				this->goal_reached_msg.data = true;
				this->goal_reached_pub.publish(this->goal_reached_msg);
				this->goal_reached_msg.data = false;

				return;
			}
			else
			{
				// goal, final approach

				//double target_vel = sqrt(2 * this->acc_lim_lin * vel_norm) - (acc_max * 3);
				//double _r = fabs(target_vel) / vel_norm;
				vel_world_x *= this->base_vel_k_lin;
				vel_world_y *= this->base_vel_k_lin;
			}
		}

		this->goal_reached_msg.data = false;
		//this->goal_reached_pub.publish(this->goal_reached_msg);
		break;
	}

	// 目標がウェイポイントのとき，並進の速度は制御せず，向きだけ決める．速度は最大．かな？
	// 回転は常に制御したほうがいいと思う．
	vel_z *= this->base_vel_k_ang;
#if 0
	/double vel_z_norm = fabs(vel_z);
	if(vel_z_norm > 0.0000001)
	{
		double _r = fabs(sqrt(2 * this->acc_lim_ang * vel_z_norm) - (this->acc_lim_ang * dt)) / vel_z_norm;
		vel_z *= _r;
	}
#endif

	double vel_z_norm = fabs(vel_z);
	if(vel_z_norm > this->vel_lim_ang)
	{
		double _r = this->vel_lim_ang / vel_z_norm;
		vel_z *= _r;
	}

	double acc_z = vel_z - cmd_vel_msg.angular.z;
	double acc_rot_norm = fabs(acc_z);
	if(acc_rot_norm > (this->acc_lim_ang * dt))
	{
		double _r = (this->acc_lim_ang * dt) / acc_rot_norm;
		acc_z *= _r;
	}

	//pose_delta_world_x -= this->cmd_vel_msg.linear.x * dt;
	//pose_delta_world_y -= this->cmd_vel_msg.linear.y * dt;

	// angle from origin
	//theta = this->last_pose_msg.theta - atan2(-pose_delta_world_x, pose_delta_world_y);
	//theta += this->cmd_vel_msg.angular.z * dt;

	double theta = this->last_pose_msg.theta;
	double vel_x = +(vel_world_x * cos(theta)) + (vel_world_y * sin(theta));
	double vel_y = -(vel_world_x * sin(theta)) + (vel_world_y * cos(theta));

	vel_norm = hypot(vel_x, vel_y);

	if(vel_norm > this->vel_lim_lin)
	{
		double _r = this->vel_lim_lin / vel_norm;
		vel_x *= _r;
		vel_y *= _r;
	}

	/*
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
	 */
	// accel. limit

#define ACCEL_LIMIT

#ifdef ACCEL_LIMIT
	double acc_x = vel_x - cmd_vel_msg.linear.x;
	double acc_y = vel_y - cmd_vel_msg.linear.y;
	double acc_trans_norm = hypot(acc_x, acc_y);
	if(acc_trans_norm > acc_lim_lin * dt)
	{
		double _r = acc_lim_lin * dt / acc_trans_norm;
		acc_x *= _r;
		acc_y *= _r;
	}

	// TODO: JERK limit

	this->cmd_vel_msg.linear.x += acc_x;
	this->cmd_vel_msg.linear.y += acc_y;
#else
	this->cmd_vel_msg.linear.x = vel_x;
	this->cmd_vel_msg.linear.y = vel_y;
#endif

	this->cmd_vel_msg.angular.z += acc_z;

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

