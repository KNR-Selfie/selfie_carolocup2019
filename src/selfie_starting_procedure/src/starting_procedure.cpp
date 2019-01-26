#include <selfie_starting_procedure/starting_procedure.h>
#include <termios.h>


Starting_procedure::Starting_procedure(const ros::NodeHandle& _pnh, const ros::NodeHandle& _nh):nh(_nh),pnh(_pnh)
{

    sub_odom = nh.subscribe("odom",10,&Starting_procedure::odomCallback,this);
    sub_dist = nh.subscribe("distance",10,&Starting_procedure::distCallback,this);
    sub_button = nh.subscribe("start_button",10,&Starting_procedure::buttonCallback,this);

    pub_drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    pub_start = nh.advertise<std_msgs::Bool>("start",1);

    driven_dist = 0;
    start_flag = -1;


    dist_tresh = 1.5;
    pnh.getParam("dist",dist_tresh);
}

void Starting_procedure::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{

}

void Starting_procedure::distCallback(const std_msgs::Float32::ConstPtr msg)
{
    if(start_flag)
        driven_dist = msg->data;
}

void Starting_procedure::buttonCallback(const std_msgs::Bool::ConstPtr msg)
{
    if(msg->data == true && start_flag == -1)
    {
        start_flag = 0;
        ROS_INFO("button_clicked");
    }

}

bool Starting_procedure::button_clicked()
{
    if(start_flag == 0)
    {
        start_flag = 1;
        return true;
    }

    return false;
}

void Starting_procedure::drive()
{
    ackermann_msgs::AckermannDriveStamped cmd;
    float speed;

    if(driven_dist>dist_tresh)
    {
        std_msgs::Bool start_pub;
        start_pub.data = true;

        pub_start.publish(start_pub);

        speed = 1; //slow down after goin out
    }
    else
    {
        speed = 1.5;
    }

    cmd.drive.speed = speed;
    cmd.drive.steering_angle = 0;
    cmd.drive.steering_angle_velocity = 15;

    pub_drive.publish(cmd);

}
