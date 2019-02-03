#pragma once
#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

struct Point
{
    float x;
    float y;
};

struct Line
{
    Point start_point;
    Point end_point;
    float slope;
    float b;
    float a;
    float length;
};

class ObstaclesGenerator
{
public:
    ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~ObstaclesGenerator();
    bool init();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher visualization_lines_pub_;
    ros::Publisher visualization_obstacles_pub_;

    std::vector <Line> line_array_;
    selfie_msgs::PolygonArray obstacle_array_;
    sensor_msgs::LaserScan scan_;
    void laserScanCallback(const sensor_msgs::LaserScan& msg);
    void generateLines();
    Point getXY(float &angle, float &range);
    float getSlope(Point &p1, Point &p2);
    float getA(Point &p1, Point &p2);
    float getDistance(Point &p1, Point &p2);
    void visualizeLines();
    void visualizeObstacles();
    void printInfoParams();
    void mergeLines();
    void generateObstacles();

    void deleteSmallLines();

    float max_range_;
    float min_range_;
    float line_search_max_range_difference_;
    float line_search_max_slope_difference_;
    float line_search_min_slope_difference_;
    float line_search_slope_difference_ratio_;
    float line_search_min_length_;
    float line_min_length_;
    float obstacle_nominal_length_;
    bool visualize_;
    float lidar_offset_;
    std::string visualization_frame_;
    std::string obstacles_frame_;
    
};

    
#endif
