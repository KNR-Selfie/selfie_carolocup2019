#include <selfie_starting_procedure/starting_procedure.h>
#include <termios.h>


Starting_procedure::Starting_procedure(const ros::NodeHandle& _pnh, const ros::NodeHandle& _nh):nh(_nh),pnh(_pnh)
{
    sub_dist = nh.subscribe("distance",10,&Starting_procedure::distCallback,this);

    pub_drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
    pub_start = nh.advertise<std_msgs::Bool>("start",1);

    driven_dist = 0;

    dist_tresh = 1.5;
    pnh.getParam("dist",dist_tresh);
}

void Starting_procedure::distCallback(const std_msgs::Float32::ConstPtr msg)
{
    driven_dist = msg->data;
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
