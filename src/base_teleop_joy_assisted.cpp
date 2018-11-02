/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
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
#include <sensor_msgs/Joy.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>

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

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    tf::TransformListener _tflistener;

    double max_lin;
    double max_lin_turbo;
    double max_ang;
    double max_ang_turbo;

    bool invert_xy = false;
    //double target_theta = -M_PI/2.0;

    static int AxisLeftThumbX;
    static int AxisLeftThumbY;
    static int AxisRightThumbX;
    static int ButtonLB;
    static int ButtonRB;
};

int BaseTeleop::AxisLeftThumbX = 0;
int BaseTeleop::AxisLeftThumbY = 1;
int BaseTeleop::AxisRightThumbX = 2;
int BaseTeleop::ButtonLB = 6;
int BaseTeleop::ButtonRB = 7;

BaseTeleop::BaseTeleop()
{

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("ButtonRB", ButtonRB);

    auto private_nh = ros::NodeHandle("~");
    private_nh.param("invert_xy", invert_xy, false);

    auto _nh = ros::NodeHandle("~");

    _nh.param("max_lin", this->max_lin, 1.0);
    _nh.param("max_lin_turbo", this->max_lin_turbo, this->max_lin);
    _nh.param("max_ang", this->max_ang, M_PI);
    _nh.param("max_ang_turbo", this->max_ang_turbo, this->max_ang);

    ROS_INFO("max_lin: %lf", this->max_lin);
    ROS_INFO("max_lin_turbo: %lf", this->max_lin_turbo);
    ROS_INFO("max_ang: %lf", this->max_ang);
    ROS_INFO("max_ang_turbo: %lf", this->max_ang_turbo);
}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    tf::StampedTransform base_link;
    geometry_msgs::Pose2D last_pose_msg;

    static bool last_lb = false;
    static bool last_rb = false;
    bool _lb = joy->buttons[ButtonLB];
    bool _rb = joy->buttons[ButtonRB];

    try
    {
        this->_tflistener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
        this->_tflistener.lookupTransform("/map", "/base_link", ros::Time(0), base_link);
    }
    catch (...)
    {
        ROS_INFO("failed to look up tf.");
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;

        vel_pub_.publish(twist);

        return;
    }

    last_pose_msg.x = base_link.getOrigin().x();
    last_pose_msg.y = base_link.getOrigin().y();
    last_pose_msg.theta = tf::getYaw(base_link.getRotation());

    // TODO: add invert option for both sides

    double vel_world_x = 0.0;
    double vel_world_y = 0.0;

    if (this->invert_xy)
    {
        vel_world_x = joy->axes[AxisLeftThumbY];
        vel_world_y = joy->axes[AxisLeftThumbX];
    }
    else
    {
        vel_world_x = -joy->axes[AxisLeftThumbY];
        vel_world_y = -joy->axes[AxisLeftThumbX];
    }

    double theta = -last_pose_msg.theta;
    double vel_local_x = +(vel_world_x * cos(theta)) + (vel_world_y * sin(theta));
    double vel_local_y = -(vel_world_x * sin(theta)) + (vel_world_y * cos(theta));

    double vel_z = joy->axes[AxisRightThumbX];

    double vel_norm = hypot(vel_local_x, vel_local_y);
    if (vel_norm > 1.0)
    {
        vel_local_x /= vel_norm;
        vel_local_y /= vel_norm;
    }

    vel_local_x *= this->max_lin;
    vel_local_y *= this->max_lin;
    vel_z *= this->max_ang;

    twist.linear.x = vel_local_x;
    twist.linear.y = vel_local_y;
    twist.angular.z = vel_z;

    vel_pub_.publish(twist);

    last_lb = _lb;
    last_rb = _rb;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_teleop_joy");

    BaseTeleop baseTeleop;

    ros::spin();
}

