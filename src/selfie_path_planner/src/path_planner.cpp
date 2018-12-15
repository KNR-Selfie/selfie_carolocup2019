#include <path_planner/path_planner.h>
#include "path_planner/path_planner.h"

void poly_to_path(poly polyline,nav_msgs::Path& path, bool reversed)
{
    geometry_msgs::PoseStamped poseSt;

    if(polyline.y_output_pts.size() == 0)
        return;

    if(reversed == false)
    {
        for(int i = 0;i<polyline.y_output_pts.size()-1;i++)
        {
            poseSt.pose.position.x = polyline.x_raw_pts[i];
            poseSt.pose.position.y = polyline.y_output_pts[i];
            path.poses.push_back(poseSt);
        }
    }
    else
    {
        for(int i = polyline.y_output_pts.size()-1;i>0;i--)
        {
            poseSt.pose.position.x = polyline.x_raw_pts[i];
            poseSt.pose.position.y = polyline.y_output_pts[i];
            path.poses.push_back(poseSt);
        }
    }

}
