#include <selfie_starting_procedure/starting_procedure.h>
#include <termios.h>


Starting_procedure::Starting_procedure(const ros::NodeHandle& _pnh, const ros::NodeHandle& _nh):nh(_nh),pnh(_pnh)
{

    sub_odom = nh.subscribe("odom",10,&Starting_procedure::odomCallback,this);
    sub_dist = nh.subscribe("distance",10,&Starting_procedure::distCallback,this);
    pub_drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);

    driven_dist = 0;
    start_flag = -1;

    dist_tresh = 30;
    pnh.getParam("dist",dist_tresh);
}

void Starting_procedure::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{

}
void Starting_procedure::distCallback(const std_msgs::Float32::ConstPtr msg)
{   ROS_INFO("rec %f",msg->data);
    if(start_flag)
        driven_dist = msg->data;
}


//temp function to simulate button click///////////////////////////
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
////////////////////////////////////////////

bool Starting_procedure::button_clicked() //todo implement this function in STM button callback
{
    if(start_flag == -1)
    {
        int c = getch();

        if (c == 's' )
        {
            start_flag = 0;
            ROS_INFO("button clicked");
            return true;
        }
    }

    return false;

}
void Starting_procedure::send_reset_flag()
{
    ROS_INFO("send reset flag");
}

void Starting_procedure::drive()
{
    start_flag = 1;

    if(driven_dist>dist_tresh)
    {
        send_reset_flag();
    }

    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.drive.speed = 5;
    cmd.drive.steering_angle = 0;
    cmd.drive.steering_angle_velocity = 15;

    pub_drive.publish(cmd);

}
