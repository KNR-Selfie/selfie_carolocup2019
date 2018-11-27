#include <selfie_perception/obstacles_generator.h>

ObstaclesGenerator::ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
    nh_(nh),
    pnh_(pnh),
    max_range_(1.5),
    min_range_(0.03),
    line_search_max_range_difference_(0.04),
    line_search_max_slope_difference_(2.0),
    line_search_min_slope_difference_(0.05),
    line_search_slope_difference_ratio_(0.06),
    line_search_min_length_(0.015),
    line_min_length_(0.05),
    obstacle_nominal_length_(0.11),
    obstacles_frame_("laser"),
    visualization_frame_("laser"),
    visualize_(true)
{
    obstacles_pub_ = nh_.advertise<selfie_msgs::PolygonArray>("obstacles", 10);
}

ObstaclesGenerator::~ObstaclesGenerator()
{
    obstacle_array_.polygons.clear();
    line_array_.clear();
}

bool ObstaclesGenerator::init()
{
    scan_sub_ = nh_.subscribe("/scan", 10, &ObstaclesGenerator::laserScanCallback, this);
    pnh_.getParam("max_range",max_range_);
    pnh_.getParam("min_range",min_range_);
    pnh_.getParam("line_search_max_range_difference",line_search_max_range_difference_);
    pnh_.getParam("line_search_max_slope_difference",line_search_max_slope_difference_);
    pnh_.getParam("line_search_min_slope_difference",line_search_min_slope_difference_);
    pnh_.getParam("line_search_slope_difference_ratio",line_search_slope_difference_ratio_);
    pnh_.getParam("line_search_min_length",line_search_min_length_);
    pnh_.getParam("line_min_length",line_min_length_);
    pnh_.getParam("obstacle_nominal_length",obstacle_nominal_length_);
    pnh_.getParam("visualize",visualize_);
    pnh_.getParam("obstacles_frame",obstacles_frame_);
    pnh_.getParam("visualization_frame",visualization_frame_);

    if(visualize_)
    {
        visualization_lines_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_lines", 1 );
        visualization_obstacles_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_obstacles", 1 );
    }

    printInfoParams();
    return true;
}

void ObstaclesGenerator::laserScanCallback(const sensor_msgs::LaserScan& msg)
{
    scan_ = msg;
    generateLines();
    if(!line_array_.empty())
    {
        mergeLines();
        deleteSmallLines();
    }
    generateObstacles();
    if (visualize_)
        {
            visualizeLines();
            visualizeObstacles();
        }
}

void ObstaclesGenerator::generateLines()
{
    line_array_.clear();
    Point start_point = getXY(scan_.angle_min, scan_.ranges[0]);
    Point last_fitted_point;
    int points_in_line = 1;
    float act_line_slope_sum = 0;
    float avg_act_line_slope;
    int act_angle_index = 1;
    int start_point_index = 0;
    bool reset_params = false;
    Point act_point;
    float act_line_length = 0;
    float line_slope_difference = line_search_max_slope_difference_;

    for(float act_angle = scan_.angle_min + scan_.angle_increment; act_angle <= scan_.angle_max; act_angle += scan_.angle_increment)
    {
        if(scan_.ranges[act_angle_index] <= max_range_ && scan_.ranges[act_angle_index] >= min_range_)
        {
            act_point = getXY(act_angle, scan_.ranges[act_angle_index]);
            if(std::abs(scan_.ranges[act_angle_index] - scan_.ranges[act_angle_index-1]) <= line_search_max_range_difference_)
            {
                if(points_in_line < 2)
                {
                    act_line_slope_sum += getSlope(start_point, act_point);
                    avg_act_line_slope = act_line_slope_sum;
                    last_fitted_point = act_point;
                }
                else 
                {
                    act_line_length = getDistance(start_point, act_point);
                    if(line_slope_difference > line_search_min_slope_difference_)
                    {
                        line_slope_difference = (line_search_min_slope_difference_ - line_search_max_slope_difference_) / line_search_slope_difference_ratio_ 
                        * act_line_length + line_search_max_slope_difference_;
                        if(line_slope_difference < line_search_min_slope_difference_)
                            line_slope_difference = line_search_min_slope_difference_;
                    }
                    if(std::abs(avg_act_line_slope - getSlope(start_point, act_point)) <= line_slope_difference)
                    {
                        act_line_slope_sum += getSlope(start_point, act_point);
                        avg_act_line_slope = act_line_slope_sum / points_in_line;
                        last_fitted_point = act_point;
                    }
                    else
                        reset_params = true;
                }
            }
            else
                reset_params = true;
        }
        else
            reset_params = true;
        if(reset_params || act_angle == scan_.angle_max)
        {
            if (act_line_length >= line_search_min_length_)
            {
                Line l;
                l.start_point = start_point;
                l.end_point = last_fitted_point;
                l.slope = getSlope(start_point, last_fitted_point);
                l.a = getA(start_point, last_fitted_point);
                l.b = start_point.y - (l.a * start_point.x);
                l.length = act_line_length;
                line_array_.push_back(l);
            }
            do
            {
                act_angle_index ++;
                act_angle += scan_.angle_increment;
            }while(std::isnan(scan_.ranges[act_angle_index]) && act_angle < scan_.angle_max);

            start_point = getXY(act_angle, scan_.ranges[act_angle_index]);
            start_point_index = act_angle_index;
            points_in_line = 0;
            act_line_slope_sum = 0;
            act_line_length = 0;
            line_slope_difference = line_search_max_slope_difference_;
            reset_params = false;
        }
        act_angle_index ++;
        points_in_line ++;
    }
}

Point ObstaclesGenerator::getXY(float &angle, float &range)
{
    Point p;
    p.x = range * cos(angle);
    p.y = range * sin(angle);
    return p;
}

float ObstaclesGenerator::getSlope(Point &p1, Point &p2)
{
        return atan((p2.y - p1.y) / (p2.x - p1.x));
}

float ObstaclesGenerator::getDistance(Point &p1, Point &p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

float ObstaclesGenerator::getA(Point &p1, Point &p2)
{
    return (p2.y - p1.y) / (p2.x - p1.x);
}

void ObstaclesGenerator::visualizeLines()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = visualization_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "line";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

    for(int i = 0; i < line_array_.size(); i++)
    {
        marker_point.x = line_array_[i].start_point.x;
        marker_point.y = line_array_[i].start_point.y;
        marker.points.push_back(marker_point);

        marker_point.x = line_array_[i].end_point.x;
        marker_point.y = line_array_[i].end_point.y;
        marker.points.push_back(marker_point);
    }
    visualization_lines_pub_.publish(marker);
}

void ObstaclesGenerator::printInfoParams()
{
    ROS_INFO("max_range: %.3f",max_range_);
    ROS_INFO("min_range: %.3f",min_range_);
    ROS_INFO("line_min_length: %.3f",line_min_length_);
    ROS_INFO("obstacle_nominal_length: %.3f\n",obstacle_nominal_length_);

    ROS_INFO("line_search_max_range_difference: %.3f",line_search_max_range_difference_);
    ROS_INFO("line_search_max_slope_difference: %.3f",line_search_max_slope_difference_);
    ROS_INFO("line_search_min_slope_difference: %.3f",line_search_min_slope_difference_);
    ROS_INFO("line_search_slope_difference_ratio: %.3f",line_search_slope_difference_ratio_);
    ROS_INFO("line_search_min_length: %.3f\n",line_search_min_length_);

    ROS_INFO("visualize: %d",visualize_);
    ROS_INFO("obstacles_frame: %s",obstacles_frame_.c_str());
    ROS_INFO("visualization_frame: %s\n",visualization_frame_.c_str());
}

void ObstaclesGenerator::mergeLines()
{
    float merge_max_distance = 0.1;
    float merge_max_slope_difference = M_PI / 3.6;
    float distance = 0;
    float slope_diff = 0;
    for(int i = 0; i < line_array_.size() - 1; i++)
    {
        distance = getDistance(line_array_[i].end_point ,line_array_[i + 1].start_point);
        slope_diff = std::abs(line_array_[i].slope - line_array_[i + 1].slope);
        if(distance < merge_max_distance && (slope_diff <= merge_max_slope_difference || slope_diff >= (M_PI - merge_max_slope_difference)))
        {
            line_array_[i].end_point = line_array_[i + 1].end_point;
            line_array_[i].slope = getSlope(line_array_[i].start_point, line_array_[i].end_point);
            line_array_[i].a = getA(line_array_[i].start_point, line_array_[i].end_point);
            line_array_[i].b = line_array_[i].start_point.y - (line_array_[i].a * line_array_[i].start_point.x);
            line_array_[i].length = getDistance(line_array_[i].start_point, line_array_[i].end_point);
            line_array_.erase(line_array_.begin() + i + 1);
            i--;
        }
    }
}

void ObstaclesGenerator::generateObstacles()
{
    obstacle_array_.polygons.clear();
    obstacle_array_.header.stamp = ros::Time::now();
    obstacle_array_.header.frame_id = obstacles_frame_;
    if(!line_array_.empty())
    {
        float distance = 0;
        float max_distance = 0.16;
        float slope_diff = 0;
        geometry_msgs::Point32 p;
        p.z = 0;
        geometry_msgs::Polygon obstacle;
        bool obstacle_generated = false;
        for(int i = 0; i < line_array_.size(); i++)
        {
            obstacle_generated = false;
            if(i != line_array_.size() - 1)
            {
                distance = getDistance(line_array_[i].end_point, line_array_[i + 1].start_point);
                slope_diff = std::abs(line_array_[i].slope - line_array_[i + 1].slope);
                if(distance < max_distance && slope_diff > M_PI / 4.2)
                {
                    p.x = (line_array_[i + 1].b - line_array_[i].b) / (line_array_[i].a - line_array_[i + 1].a);
                    p.y = ((line_array_[i + 1].b * line_array_[i].a) - (line_array_[i].b * line_array_[i + 1].a)) / (line_array_[i].a - line_array_[i + 1].a);
                    obstacle.points.push_back(p);
                    p.x = line_array_[i].start_point.x;
                    p.y = line_array_[i].start_point.y;
                    obstacle.points.push_back(p);
                    float b1 = line_array_[i].start_point.y - line_array_[i + 1].a * line_array_[i].start_point.x;
                    float b2 = line_array_[i + 1].end_point.y - line_array_[i].a * line_array_[i + 1].end_point.x;
                    p.x = (b1 - b2) / (line_array_[i].a - line_array_[i + 1].a);
                    p.y = (b1 * line_array_[i].a - b2 * line_array_[i + 1].a) / (line_array_[i].a - line_array_[i + 1].a);
                    obstacle.points.push_back(p);
                    p.x = line_array_[i + 1].end_point.x;
                    p.y = line_array_[i + 1].end_point.y;
                    obstacle.points.push_back(p);
                    i++;
                    obstacle_generated = true;
                }
            }
            if(!obstacle_generated)
            {
                p.x = line_array_[i].start_point.x;
                p.y = line_array_[i].start_point.y;
                obstacle.points.push_back(p);

                p.x = line_array_[i].end_point.x;
                p.y = line_array_[i].end_point.y;
                obstacle.points.push_back(p);

                float add_x = obstacle_nominal_length_ * sin(line_array_[i].slope);
                float add_y = obstacle_nominal_length_ * cos(line_array_[i].slope);

                float sx = (line_array_[i].end_point.x + line_array_[i].start_point.x) / 2;
                float sy = (line_array_[i].end_point.y + line_array_[i].start_point.y) / 2;

                if(line_array_[i].slope > -1 * M_PI / 4 && line_array_[i].slope < M_PI / 4)
                {
                    if(sy > 0)
                    {
                        add_x *= -1;
                        add_y *= -1;
                    }
                }
                else if((sx > 0 && line_array_[i].slope < 0) || (sx < 0 && line_array_[i].slope > 0))
                {
                    add_x *= -1;
                    add_y *= -1;
                }

                p.x = line_array_[i].end_point.x + add_x;
                p.y = line_array_[i].end_point.y - add_y;
                obstacle.points.push_back(p);

                p.x = line_array_[i].start_point.x + add_x;
                p.y = line_array_[i].start_point.y - add_y;
                obstacle.points.push_back(p);
            }
            obstacle_array_.polygons.push_back(obstacle);
            obstacle.points.clear();
        }
    }
    obstacles_pub_.publish(obstacle_array_);
}

void ObstaclesGenerator::visualizeObstacles()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = visualization_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "line";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.006;
    marker.scale.y = 0.006;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

    int i = 0;
    for(int i = 0; i < obstacle_array_.polygons.size(); i++)
    {

        marker_point.x = obstacle_array_.polygons[i].points[0].x;
        marker_point.y = obstacle_array_.polygons[i].points[0].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[1].x;
        marker_point.y = obstacle_array_.polygons[i].points[1].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[1].x;
        marker_point.y = obstacle_array_.polygons[i].points[1].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[2].x;
        marker_point.y = obstacle_array_.polygons[i].points[2].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[2].x;
        marker_point.y = obstacle_array_.polygons[i].points[2].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[3].x;
        marker_point.y = obstacle_array_.polygons[i].points[3].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[3].x;
        marker_point.y = obstacle_array_.polygons[i].points[3].y;
        marker.points.push_back(marker_point);

        marker_point.x = obstacle_array_.polygons[i].points[0].x;
        marker_point.y = obstacle_array_.polygons[i].points[0].y;
        marker.points.push_back(marker_point);
    }
    visualization_obstacles_pub_.publish(marker);
}

void ObstaclesGenerator::deleteSmallLines()
{
    for(int i = 0; i < line_array_.size(); i++)
    {
        if(line_array_[i].length < line_min_length_)
        {
            line_array_.erase(line_array_.begin() + i);
            i--;
        }
    }
}