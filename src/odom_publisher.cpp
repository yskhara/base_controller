/*
 * odom_broadcaster.cpp
 *
 *  Created on: Sep 28, 2018
 *      Author: yusaku
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf/tf.h"

#include <string>

class OdomBroadcaster
{
private:
    ros::NodeHandle nh;

    ros::Subscriber odom_yaw_sub;
    ros::Subscriber odom_x_sub;
    ros::Subscriber odom_y_sub;

#ifdef PUBLISH_ODOM
    nav_msgs::Odometry odom_msg;
    ros::Publisher odom_pub;
#else
    geometry_msgs::PoseStamped pose_msg;
    ros::Publisher pose_pub;
#endif
    ros::Timer control_tim;

    std::string odom_frame;

    double _yaw;
    double _x;
    double _y;

    void odomYawCallback(const std_msgs::Float64::ConstPtr& yaw);
    void odomXCallback(const std_msgs::Float64::ConstPtr& x);
    void odomYCallback(const std_msgs::Float64::ConstPtr& y);

    void TimerCallback(const ros::TimerEvent& event);

public:
    OdomBroadcaster(void);
};

OdomBroadcaster::OdomBroadcaster(void)
{
    auto _nh = ros::NodeHandle("~");

    _yaw = _x = _y = 0.0;

    odom_yaw_sub = nh.subscribe<std_msgs::Float64>("odom/yaw", 10, &OdomBroadcaster::odomYawCallback, this);
    odom_x_sub = nh.subscribe<std_msgs::Float64>("odom/x", 10, &OdomBroadcaster::odomXCallback, this);
    odom_y_sub = nh.subscribe<std_msgs::Float64>("odom/y", 10, &OdomBroadcaster::odomYCallback, this);

#ifdef PUBLISH_ODOM
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_pose", 1);
#else
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("odom_pose", 1);
#endif

    _nh.param<std::string>("odom_frame", odom_frame, "odom");

    int ctrl_freq;
    _nh.param("ctrl_freq", ctrl_freq, 20);

    control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq), &OdomBroadcaster::TimerCallback, this);
}

void OdomBroadcaster::odomYawCallback(const std_msgs::Float64::ConstPtr& yaw)
{
    _yaw = yaw->data;
}

void OdomBroadcaster::odomXCallback(const std_msgs::Float64::ConstPtr& x)
{
    _x = x->data;
}

void OdomBroadcaster::odomYCallback(const std_msgs::Float64::ConstPtr& y)
{
    _y = y->data;
}

void OdomBroadcaster::TimerCallback(const ros::TimerEvent& event)
{
    auto quat = tf::Quaternion();
    quat.setRPY(0, 0, _yaw);

#ifdef PUBLISH_ODOM
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(_yaw);

    odom_msg.pose.pose.position.x = _x;
    odom_msg.pose.pose.position.y = _y;

    odom_msg.header.frame_id = odom_frame;
    odom_msg.header.stamp = ros::Time::now();

    odom_pub.publish(odom_msg);
#else
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_yaw);

    pose_msg.pose.position.x = _x;
    pose_msg.pose.position.y = _y;

    pose_msg.header.frame_id = odom_frame;
    pose_msg.header.stamp = ros::Time::now();

    pose_pub.publish(pose_msg);
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_broadcaster");

    OdomBroadcaster *odomBroadcaster = new OdomBroadcaster();
    ROS_INFO("odom_broadcaster node has started.");

    ros::spin();
    ROS_INFO("odom_broadcaster node has been terminated.");
}

