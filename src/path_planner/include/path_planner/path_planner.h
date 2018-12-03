#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "selfie_msgs/RoadMarkings.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Path.h"
#include <path_planner/polyfit.hpp>

#define KUROKESU 1

#if KUROKESU
    #define MAT_HEIGHT 480
    #define MAT_WIDTH 640
#endif

void RoadMarkings_to_cloud(const selfie_msgs::RoadMarkings::ConstPtr& msg, sensor_msgs::PointCloud& points_preview);
void poly_to_path(poly polyline,nav_msgs::Path& path);

#endif // PATH_PLANNER_H
