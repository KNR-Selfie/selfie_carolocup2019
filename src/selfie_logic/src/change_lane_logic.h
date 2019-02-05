#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <selfie_msgs/RoadMarkings.h>
#include <selfie_msgs/PolygonArray.h>
#include <geometry_msgs/Point32.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <vector>

struct line_s{
    std::vector<float> center;
    std::vector<float> left;
    std::vector<float> right;
};

enum lane_status_e{
    l_empty,
    l_busy,
};

enum car_lane_e{
    c_left,
    c_right,
};

struct lane_status_s{
    lane_status_e left;
    lane_status_e right;
};

class ChangeLaneLogic{
public:
    line_s line_coef;
    car_lane_e car_lane;
    lane_status_s lane_status;
    uint32_t left_points;
    uint32_t right_points;
    float distance;
    float speed;
    void check_lane_status(geometry_msgs::Polygon polygon);
    uint8_t check_point(float x, float y, float a0_l, float a1_l, float a2_l, float a0_r, float a1_r, float a2_r);

};