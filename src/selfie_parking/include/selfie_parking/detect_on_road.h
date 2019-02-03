#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>
#include <selfie_msgs/RoadMarkings.h>

#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/searchAction.h>
#include <vector>
#include <cassert>


#include <selfie_park/parkAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/console.h>

#include "shapes.h"

using namespace std;

class dynamic_detector {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber obstacles_sub;
    ros::Subscriber markings_sub;
    ros::Subscriber scan_sub;
    vector<Box> left_track;

    const float max_range = 1.6;
    const float min_range = 0.3;
    vector<float> left_line;
    vector<float> right_line;
    vector<float> middle_line;

    laser_geometry::LaserProjection projector_;

public:
    dynamic_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    :nh_(nh), pnh_(pnh), max_range(1.6), min_range(0.3)
    {}
    ~dynamic_detector()
    {

    }
    void init()
    {
      //  obstacles_sub = nh_.subscribe("/obstacles", 10, &dynamic_detector::obstacleCB, this);
        markings_sub = nh_.subscribe("/vision/road_markings", 10, &dynamic_detector::markingsCB, this);
        scan_sub = nh_.subscribe("/scan", 10, &dynamic_detector::scanCB, this);
    }

    void obstacleCB(const selfie_msgs::PolygonArray &msg);
    void markingsCB(const selfie_msgs::RoadMarkings &msg);
    void scanCB(const sensor_msgs::LaserScanConstPtr msg);

    bool check_point(geometry_msgs::Point32 &pt);
};