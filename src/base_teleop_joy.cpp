/*
 * base_teleop_joy.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>

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

    double max_lin;
    double max_lin_turbo;
    double max_ang;
    double max_ang_turbo;

    double sensitivity_adj_pow_xy;
    double sensitivity_adj_pow_z;

    static int AxisLeftThumbX;
    static int AxisLeftThumbY;
    static int AxisRightThumbX;
    static int ButtonRB;
};

int BaseTeleop::AxisLeftThumbX = 0;
int BaseTeleop::AxisLeftThumbY = 1;
int BaseTeleop::AxisRightThumbX = 2;
int BaseTeleop::ButtonRB = 7;

BaseTeleop::BaseTeleop()
{

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &BaseTeleop::joyCallback, this);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("ButtonRB", ButtonRB);

    auto _nh = ros::NodeHandle("~");

    _nh.param("max_lin", this->max_lin, 1.0);
    _nh.param("max_lin_turbo", this->max_lin_turbo, this->max_lin);
    _nh.param("max_ang", this->max_ang, M_PI);
    _nh.param("max_ang_turbo", this->max_ang_turbo, this->max_ang);
    _nh.param("sensitivity_adj_pow_xy", this->sensitivity_adj_pow_xy, 1.0);
    _nh.param("sensitivity_adj_pow_z", this->sensitivity_adj_pow_z, 1.0);

    ROS_INFO("max_lin: %lf", this->max_lin);
    ROS_INFO("max_lin_turbo: %lf", this->max_lin_turbo);
    ROS_INFO("max_ang: %lf", this->max_ang);
    ROS_INFO("max_ang_turbo: %lf", this->max_ang_turbo);
}

void BaseTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    double vel_x = joy->axes[AxisLeftThumbY];
    double vel_y = joy->axes[AxisLeftThumbX];
    double vel_z = joy->axes[AxisRightThumbX];

    int vel_x_sgn = vel_x < 0 ? -1 : 1;
    int vel_y_sgn = vel_y < 0 ? -1 : 1;
    int vel_z_sgn = vel_z < 0 ? -1 : 1;
    
    vel_x = vel_x_sgn * std::abs(std::pow(vel_x, this->sensitivity_adj_pow_xy));
    vel_y = vel_y_sgn * std::abs(std::pow(vel_y, this->sensitivity_adj_pow_xy));
    vel_z = vel_z_sgn * std::abs(std::pow(vel_z, this->sensitivity_adj_pow_z));

    double vel_norm = hypot(vel_x, vel_y);
    if (vel_norm > 1.0)
    {
        vel_x /= vel_norm;
        vel_y /= vel_norm;
    }

    if (joy->buttons[ButtonRB] != 0)
    {
        vel_x *= this->max_lin_turbo;
        vel_y *= this->max_lin_turbo;
        vel_z *= this->max_ang_turbo;
    }
    else
    {
        vel_x *= this->max_lin;
        vel_y *= this->max_lin;
        vel_z *= this->max_ang;
    }

    twist.linear.x = vel_x;
    twist.linear.y = vel_y;
    twist.angular.z = vel_z;

    vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_teleop_joy");

    BaseTeleop baseTeleop;

    ros::spin();
}

