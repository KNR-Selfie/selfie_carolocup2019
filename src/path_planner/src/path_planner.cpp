#include <path_planner/path_planner.h>
#include "geometry_msgs/PoseStamped.h"
#include "path_planner/path_planner.h"


void RoadMarkings_to_cloud(const selfie_msgs::RoadMarkings::ConstPtr& msg, sensor_msgs::PointCloud& points_preview)
{
    geometry_msgs::Point32 point;

    for(int i = 0;i<msg->center_line.size();i++)
    {
        point.x = MAT_HEIGHT - msg->center_line[i].y;
        point.y = MAT_WIDTH - msg->center_line[i].x;

        points_preview.points.push_back(point);
    }
    for(int i = 0;i<msg->left_line.size();i++)
    {
        point.x = MAT_HEIGHT - msg->left_line[i].y;
        point.y = MAT_WIDTH - msg->left_line[i].x;

        points_preview.points.push_back(point);
    }
    for(int i = 0;i<msg->right_line.size();i++)
    {
        point.x = MAT_HEIGHT - msg->right_line[i].y;
        point.y = MAT_WIDTH - msg->right_line[i].x;

        points_preview.points.push_back(point);
    }
}

void poly_to_path(poly polyline,nav_msgs::Path& path)
{
    geometry_msgs::PoseStamped poseSt;

    if(polyline.y_output_pts.size() == 0)
        return;

    for(int i = 0;i<polyline.y_output_pts.size()-1;i++)
    {
        poseSt.pose.position.x = polyline.x_raw_pts[i];
        poseSt.pose.position.y = polyline.y_output_pts[i];
        path.poses.push_back(poseSt);
    }

}
