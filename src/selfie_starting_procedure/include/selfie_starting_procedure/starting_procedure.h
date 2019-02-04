#ifndef STARTING_PROCEDURE_H
#define STARTING_PROCEDURE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class Starting_procedure
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    float dist_tresh;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_dist;

    ros::Publisher pub_drive;
    ros::Publisher pub_start;

    float driven_dist;

public:
    Starting_procedure(const ros::NodeHandle& _pnh, const ros::NodeHandle& _nh);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void distCallback(const std_msgs::Float32::ConstPtr msg);

    void drive();
    void send_reset_flag();
};
#endif // STARTING_PROCEDURE_H
