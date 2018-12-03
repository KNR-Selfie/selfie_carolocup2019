#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "selfie_msgs/RoadMarkings.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include <path_planner/polyfit.hpp>
#include <path_planner/path_planner.h>
#include <string.h>

#define PREVIEW_MODE 1

#if PREVIEW_MODE
    ros::Publisher cloud_pub;
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
    left_line.get_row_pts(msg->left_line);
    left_line.polyfit(3); //todo parametrize degree

    center_line.get_row_pts(msg->center_line);
    center_line.polyfit(3);

    right_line.get_row_pts(msg->right_line);
    right_line.polyfit(3);

    middle_path.fit_middle(center_line,right_line,7);
    path_tangent.calc_coeff(middle_path,1);

    position_offset_pub.publish(middle_path.get_pos_offset(0,MAT_WIDTH/2));
    heading_offset_pub.publish(path_tangent.get_head_offset(zero_line));

#if PREVIEW_MODE

    sensor_msgs::PointCloud points_preview;
    nav_msgs::Path calc_path;

    left_line.polyval();
    center_line.polyval();
    right_line.polyval();
    middle_path.polyval();

//    poly_to_path(left_line,calc_path);
//    poly_to_path(center_line,calc_path);
//    poly_to_path(right_line,calc_path);
    poly_to_path(middle_path,calc_path);

    RoadMarkings_to_cloud(msg,points_preview);

    //publish to rviz
    points_preview.header.frame_id = "my_frame";
    cloud_pub.publish(points_preview);

    calc_path.header.frame_id = "my_frame";
    path_pub.publish(calc_path);
#endif

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("road_markings", 1000, road_markingsCallback);

#if PREVIEW_MODE
    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1000);
    path_pub  = nh.advertise<nav_msgs::Path>("path",1000);
#endif

    position_offset_pub = nh.advertise<std_msgs::Float64>("position_offset",1000);
    heading_offset_pub = nh.advertise<std_msgs::Float64>("heading_offset",1000);

    ros::Rate loop_rate(300);

    zero_line.set_coeff(0,MAT_WIDTH/2); //middle of view

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
