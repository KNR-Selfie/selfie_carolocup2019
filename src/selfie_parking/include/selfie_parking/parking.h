#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

class Point
{
public:

    float x;
    float y;

    Point(){}
    Point(float x_, float y_):x(x_), y(y_){}
    Point(geometry_msgs::Point32 point)
    {
      x = point.x;
      y = point.y;
    }
    ~Point(){}
    bool check_position(float min_x, float max_x, float min_y, float max_y)
    {
      if(x < min_x || x > max_x || y < min_y || y >max_y)
        return 0;
      else
        return 1;
    }
    float get_distance(const Point other)
    {
      return ( std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
    }
};

float get_distance(const Point a, const Point b)
{

}

// y= ax+b
struct Line
{
  float a;
  float b;
};

// wspolrzedne lokalne
class Box
{
private:
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    Line left_vertical_line;
    Line bottom_horizontal_line;
public:
    Box(){}
    Box(geometry_msgs::Polygon poly)
    {

    }
};

class Parking{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  std::array<Box, 2> two_boxes;
//  selfie_msgs::PolygonArray obstacle_array_;
//  std::vector<Box> boxes;

public:
  Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Parking();

  bool init();
  void obstacle_callback(const selfie_msgs::PolygonArray &);
  bool find_free_place();
  void visualize();
};
