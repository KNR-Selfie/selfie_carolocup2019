#include "ros/ros.h"
#include "std_msgs/String.h"
#include "selfie_msgs/RoadMarkings.h"
#include <boost/numeric/ublas/lu.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h>

int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<selfie_msgs::RoadMarkings>("road_markings", 1000);

    ros::Rate loop_rate(10);
    geometry_msgs::Point point;

    while (ros::ok())
    {

        selfie_msgs::RoadMarkings msg;

        for(int i =0;i<150;i++)
        {
            point.x = 0.05 * i;
            point.y = 0.01 * (exp(point.x) + (std::rand() %10) - 5);
            point.z = 0;
            msg.center_line.push_back(point);
        }

        for(int i =0;i<150;i++)
        {
            point.x = 0.05 * i;
            point.y = 0.01 * (exp(point.x) + (std::rand() %10) - 5) + 1;
            point.z = 0;
            msg.right_line.push_back(point);
        }

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
