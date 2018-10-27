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

class CrudePlanner
{
public:
    CrudePlanner();

    //void SetMaximumAcceleration(double acc);
    //void SetMaximumVelocity(double vel);

private:
    void TargetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void PathCallback(const nav_msgs::Path::ConstPtr& msg);
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void AbortCallback(const std_msgs::Bool::ConstPtr& msg);
    void LinToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
    void AngToleranceCallback(const std_msgs::Float64::ConstPtr& msg);
    void TimerCallback(const ros::TimerEvent& event);
    //void CalcWheelSpeed(double actualDt);

    inline double getYawFromQuat(const geometry_msgs::Quaternion& quat)
    {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return tf::getYaw(q);
    }

    geometry_msgs::Pose2D pose_to_2d(geometry_msgs::Pose pose)
    {
        geometry_msgs::Pose2D r;
        r.x = pose.position.x;
        r.y = pose.position.y;
        r.theta = getYawFromQuat(pose.orientation);

        return r;
    }

    double vel_lim_lin;
    double vel_lim_ang;

    double accel_lim_lin;
    double accel_lim_ang;
    double deccel_lim_lin;
    double deccel_lim_ang;

    double jerk_lim_lin;
    double jerk_lim_ang;

    double base_vel_k_tan;
    double base_vel_k_nor;
    double base_vel_k_ang;

    double lin_goal_tolerance;
    double ang_goal_tolerance;

    double lin_waypoint_tolerance;

    double control_interval = 1 / 10.0;

    bool planning = false;
    bool moving = false;

    //double MaximumVelocity;

    ros::NodeHandle nh;

    ros::Subscriber path_sub;
    ros::Subscriber target_pose_sub;
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

    //tf::TransformListener _tflistener;

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

CrudePlanner::CrudePlanner(void)
{
    auto private_nh = ros::NodeHandle("~/CrudePlanner");
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

    private_nh.param("acc_lim_lin", this->accel_lim_lin, 1.0);
    private_nh.param("acc_lim_ang", this->accel_lim_ang, 1.0);

    // defaults to the same value as acc_lim
    private_nh.param("dec_lim_lin", this->deccel_lim_lin, this->accel_lim_lin);
    private_nh.param("dec_lim_ang", this->deccel_lim_ang, this->accel_lim_ang);

    private_nh.param("jerk_lim_lin", this->jerk_lim_lin, 1.0);
    private_nh.param("jerk_lim_ang", this->jerk_lim_ang, 1.0);

    private_nh.param("base_vel_k_tan", this->base_vel_k_tan, 1.0);
    private_nh.param("base_vel_k_nor", this->base_vel_k_nor, 2.0);
    private_nh.param("base_vel_k_ang", this->base_vel_k_ang, 0.5);

    ROS_INFO("base_vel_k_tan : %f", this->base_vel_k_tan);
    ROS_INFO("base_vel_k_nor : %f", this->base_vel_k_nor);
    ROS_INFO("base_vel_k_ang : %f", this->base_vel_k_ang);

    private_nh.param("goal_tolerance_lin", this->lin_goal_tolerance, 0.05);					// 5 centimetre
    private_nh.param("goal_tolerance_ang", this->ang_goal_tolerance, (M_PI * 5.0 / 180.0));	// \pm 5 deg
    private_nh.param("wp_tolerance_lin", this->lin_waypoint_tolerance, 0.50);			// 50 centimetre

    ROS_INFO("lin_goal_tolerance : %f", this->lin_goal_tolerance);
    ROS_INFO("ang_goal_tolerance : %f", this->ang_goal_tolerance);
    ROS_INFO("lin_waypoint_tolerance : %f", this->lin_waypoint_tolerance);

    target_pose_sub = nh.subscribe<geometry_msgs::Pose>("target_pose", 1, &CrudePlanner::TargetPoseCallback, this);
    //path_sub = nh.subscribe<nav_msgs::Path>("target_path", 10, &CrudePlanner::PathCallback, this);
    odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("odom_pose", 10, &CrudePlanner::PoseCallback, this);

    //abort_sub = nh.subscribe<std_msgs::Bool>("abort", 10, &CrudePlanner::AbortCallback, this);

    //lin_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("lin_tolerance", 10, &CrudePlanner::LinToleranceCallback, this);
    //ang_goal_tolerance_sub = nh.subscribe<std_msgs::Float64>("ang_tolerance", 10, &CrudePlanner::AngToleranceCallback, this);

    goal_reached_pub = nh.advertise<std_msgs::Bool>("goal_reached", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    path_pub = nh.advertise<nav_msgs::Path>("target_path", 1, true);

    control_tim = nh.createTimer(ros::Duration(control_interval), &CrudePlanner::TimerCallback, this);

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

void CrudePlanner::TargetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    this->last_target_msg.x = msg->position.x;
    this->last_target_msg.y = msg->position.y;
    this->last_target_msg.theta = getYawFromQuat(msg->orientation);

    // total length (linear interpolation at this moment)
    double delta_x = last_target_msg.x - last_pose_msg.x;
    double delta_y = last_target_msg.y - last_pose_msg.y;
    double path_length = hypot(delta_x, delta_y);

    // path_legth exceeding decisionDelta results in a trapezoidal motion profile
    double decision_delta = pow(vel_lim_lin, 2) / accel_lim_lin;

    double accel_until_s = 0.0;
    double cruise_until_s = 0.0;
    double decel_until_s = 0.0;

    double initial_vel = 0.0;
    double cruise_vel = vel_lim_lin;
    double exit_vel = 0.0;

    //const double fine_path_time_interval = 0.01;

    cruise_until_s = (path_length - decision_delta) / vel_lim_lin;

    if (cruise_until_s > 0)
    {
        //trapezoidal profile
        accel_until_s = fabs(vel_lim_lin - initial_vel) / accel_lim_lin;
        decel_until_s = fabs(vel_lim_lin - exit_vel) / accel_lim_lin;
        cruise_vel = vel_lim_lin;
        ROS_INFO("accel_until_s: %f", accel_until_s);
        ROS_INFO("decel_until_s: %f", decel_until_s);
    }
    else
    {
        // triangle profile
        throw new std::exception();
    }

    // sample
    double currentVelocity = 0.0;
    double traveledTotal = 0.0;

    //nav_msgs::Path path;

    // foreach(var segment in segments) {
    //double traveled = 0.0;

    target_path.poses.clear();

    target_path.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";

    while (traveledTotal < path_length)
    {
        pose_msg.pose.position.x = last_pose_msg.x + (delta_x * traveledTotal / path_length);
        pose_msg.pose.position.y = last_pose_msg.y + (delta_y * traveledTotal / path_length);
        target_path.poses.push_back(pose_msg);

        //traveled += currentVelocity;
        traveledTotal += currentVelocity * control_interval;

        if (accel_until_s > 1e-6)
        {
            //currentVelocity = initial_vel + ((cruise_vel - initial_vel) * traveledTotal / accel_until_m);
            currentVelocity += (accel_lim_lin * control_interval);
            accel_until_s -= control_interval;
        }
        else if (cruise_until_s > 1e-6)
        {
            currentVelocity = cruise_vel;
            //currentVelocity = vel_lim_lin;
            cruise_until_s -= control_interval;
        }
        else if (decel_until_s > 1e-6)
        {
            //currentVelocity = exit_vel
            //        + ((exit_vel - cruise_vel) * (decel_until_m - traveledTotal) / (decel_until_m - cruise_until_m));
            currentVelocity -= (accel_lim_lin * control_interval);
            decel_until_s -= control_interval;
        }
        else
        {
            double delta_x = last_target_msg.x - last_pose_msg.x;
            double delta_y = last_target_msg.y - last_pose_msg.y;
            currentVelocity = hypot(delta_x, delta_y) / control_interval;
            break;
        }

        ROS_INFO("current velocity: %f", currentVelocity);
    }

    ROS_INFO("calculated path has %ld poses", target_path.poses.size());

    pose_msg.pose.position.x = last_target_msg.x;
    pose_msg.pose.position.y = last_target_msg.y;
    target_path.poses.push_back(pose_msg);
    // }

    path_pub.publish(target_path);

    target_index = 0;
    moving = true;
    planning = true;
}

void CrudePlanner::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
// TODO : generate finer path for better path tracking characteristics

    //this->target_path = *msg;
    //this->target_index = 0;

    this->planning = true;
}

void CrudePlanner::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double yaw = getYawFromQuat(msg->pose.orientation);

    this->last_pose_msg.x = msg->pose.position.x;
    this->last_pose_msg.y = msg->pose.position.y;
    this->last_pose_msg.theta = yaw;
}

void CrudePlanner::AbortCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        this->planning = false;
        this->last_target_msg.x = this->last_pose_msg.x;
        this->last_target_msg.y = this->last_pose_msg.y;
        this->last_target_msg.theta = this->last_pose_msg.theta;

        this->cmd_vel_msg.linear.x = 0;
        this->cmd_vel_msg.linear.y = 0;
        this->cmd_vel_msg.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel_msg);
    }
}

void CrudePlanner::TimerCallback(const ros::TimerEvent& event)
{
    if (!this->planning)
    {
        this->cmd_vel_msg.linear.x = 0;
        this->cmd_vel_msg.linear.y = 0;
        this->cmd_vel_msg.angular.z = 0;

        cmd_vel_pub.publish(cmd_vel_msg);

        return;
    }

    if(target_path.poses.size() <= target_index)
    {
        this->planning = false;
        target_index = target_path.poses.size() - 1;

        ROS_INFO("reached the End-of-Path");
    }

    geometry_msgs::Pose2D current_target = pose_to_2d(target_path.poses.at(target_index).pose);
    cmd_vel_msg.linear.x = (current_target.x - last_pose_msg.x) / control_interval;
    cmd_vel_msg.linear.y = (current_target.y - last_pose_msg.y) / control_interval;

    target_index++;

    cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "useless_planner");

    CrudePlanner *uselessPlanner = new CrudePlanner();
    ROS_INFO("crude_planner node has started.");

    ros::spin();
    ROS_INFO("crude_planner node has been terminated.");
}

