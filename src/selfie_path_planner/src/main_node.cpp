#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "selfie_msgs/RoadMarkings.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include <path_planner/polyfit.hpp>
#include <path_planner/path_planner.h>
#include <string.h>

#define PREVIEW_MODE 1

#if PREVIEW_MODE
    ros::Publisher path_pub;
#endif

ros::Publisher position_offset_pub;
ros::Publisher heading_offset_pub;

poly right_line;
poly left_line;
poly center_line;
poly middle_path;
tangent path_tangent;
tangent zero_line;

void road_markingsCallback(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{
    left_line.get_coeff(msg->left_line);
    right_line.get_coeff(msg->right_line);
    center_line.get_coeff(msg->center_line);
    ROS_INFO("coeff0: %.3f    coeff1: %.3f     coeff2: %.3f ",center_line.coeff[0],center_line.coeff[1],center_line.coeff[2]);

    center_line.x_raw_pts.clear();
    center_line.y_raw_pts.clear();
    right_line.x_raw_pts.clear();
    right_line.y_raw_pts.clear();

    for(float i = 0; i < 0.7; i += 0.05)
    {
        center_line.x_raw_pts.push_back(i);
        center_line.y_raw_pts.push_back(left_line.polyval(i));

        right_line.x_raw_pts.push_back(i);
        right_line.y_raw_pts.push_back(left_line.polyval(i));
    }

    middle_path.fit_middle(center_line,right_line,7);
    path_tangent.calc_coeff(middle_path,1);

    ROS_INFO("POS offset: %.3f     head offset: %.3f", middle_path.get_pos_offset(0,0),path_tangent.get_head_offset(zero_line) );

    position_offset_pub.publish(middle_path.get_pos_offset(0,0));
    heading_offset_pub.publish(path_tangent.get_head_offset(zero_line));

#if PREVIEW_MODE
    nav_msgs::Path calc_path;

//    poly_to_path(left_line,calc_path,true);
    poly_to_path(center_line,calc_path,false);
    poly_to_path(right_line,calc_path,true);
//    poly_to_path(middle_path,calc_path,false);

    //publish to rviz
    calc_path.header.frame_id = "my_frame";
    path_pub.publish(calc_path);
#endif

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimate_position");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("road_markings", 1000, road_markingsCallback);

#if PREVIEW_MODE
    path_pub  = nh.advertise<nav_msgs::Path>("path",1000);
#endif

    position_offset_pub = nh.advertise<std_msgs::Float64>("position_offset",1000);
    heading_offset_pub = nh.advertise<std_msgs::Float64>("heading_offset",1000);

    ros::Rate loop_rate(300);

    zero_line.set_coeff(0,0); //middle of view

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
