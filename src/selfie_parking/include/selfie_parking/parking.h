#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/Polygon.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

  using namespace std;

class Point
{
public:

    float x;
    float y;

    Point():x(0), y(0){}
    Point(float x_, float y_):x(x_), y(y_){}
    Point(geometry_msgs::Point32 point)
    {
      x = point.x;
      y = point.y;
    }
    ~Point(){}
    Point operator=(const geometry_msgs::Point32 other)
    {
      this->x = other.x;
      this->y = other.y;
      return *this;
    }
    void reset()
    {
      x = 0;
      y = 0;
    }
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
    float get_distance(geometry_msgs::Point32 other)
    {
      return ( std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
    }
    void print()
    {
      cout << "(x,y) = " << x << ", " << y << ")\n";
    }
};

// y= ax+b
struct Line
{
  float a;
  float b;
};

// wspolrzedne lokalne
class Box
{
public:
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    Line left_vertical_line;
    Line bottom_horizontal_line;
public:
    Box(){}
    Box(const Box &other)
    {
      bottom_left = other.bottom_left;
      bottom_right = other.bottom_right;
      top_left = other.top_left;
      top_right = other.top_right;
      left_vertical_line = other.left_vertical_line;
      bottom_horizontal_line = other.bottom_horizontal_line;
    }
    // this constructor finds and assign correct points to corners of our box
    Box(geometry_msgs::Polygon poly)
    {
      //point (0,0)
      Point zero(0,0);
      vector<float> distances = {zero.get_distance(poly.points[0]), zero.get_distance(poly.points[1]),
                              zero.get_distance(poly.points[2]), zero.get_distance(poly.points[3])};


//        bottom_left = poly.points[std::distance(distances.begin(), min)];

        vector<float>::iterator min = min_element(distances.begin(), distances.end());
        bottom_left = poly.points[std::distance(distances.begin(), min)];
        *min = 1000;
    //    cout <<"distance from the nearest point of the box: " <<  bottom_left.x << endl;

        //pick two closest points
        Point a, b;
        min = min_element(distances.begin(), distances.end());
        a = poly.points[std::distance(distances.begin(), min)];
        *min = 1000;

        min = min_element(distances.begin(), distances.end());
        b = poly.points[std::distance(distances.begin(), min)];
        *min = 1000;

        //pick the one with smaller y coordinate -> it's the left one
        if(a.x > b.x)
          {
            top_left = a;
            bottom_right = b;
          }else{
            top_left = b;
            bottom_right = a;
          }
        min = distances.begin();
        top_right = poly.points[std::distance(distances.begin(), min)];


      }

      Box(Point b_l, Point b_r, Point t_l, Point t_r):
                  bottom_left(b_l), bottom_right(b_r),
                  top_left(t_l), top_right(t_r)
      {
        this->make_lines();
      }
      void reset()
      {
        bottom_left.reset();
        top_left.reset();
        top_right.reset();
        bottom_right.reset();
        left_vertical_line.a = 0;
        left_vertical_line.b = 0;
        bottom_horizontal_line.a = 0;
        bottom_horizontal_line.b = 0;
      }

      void make_lines()
      {
        left_vertical_line.a = (top_left.y - bottom_left.y)/(top_left.x - bottom_left.x);
        left_vertical_line.b = top_left.y - left_vertical_line.a*top_left.x;

        bottom_horizontal_line.a = (bottom_right.y - bottom_left.y)/(bottom_right.x - bottom_left.x);
        bottom_horizontal_line.b = bottom_right.y - bottom_horizontal_line.a*bottom_right.x;
      }
      void print_lines()
      {

      }

      void print()
      {
        cout << "bottom left: ";
        bottom_left.print();
        cout << "bottom_right: ";
        bottom_right.print();
        cout << "top left: ";
        top_left.print();
        cout << "top right: ";
        top_right.print();
        cout<<endl;
      }
      void print_box_dimensions()
      {
        float bottom_edge = bottom_left.get_distance(bottom_right);
        float top_edge = top_left.get_distance(top_right);
        float right_edge = top_right.get_distance(bottom_right);
        float left_edge = top_left.get_distance(bottom_left);

        cout << "bottom_edge: " << bottom_edge <<endl;
        cout << "top_edge: " << top_edge <<endl;
        cout << "right_edge: " << right_edge <<endl;
        cout << "left_edge: " << left_edge <<endl;
      }
};

class Parking{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_lines_pub;
  ros::Publisher position_offset_pub;
  ros::Publisher heading_offset_pub;
  std::vector<Box> boxes_on_the_right_side;
  Box first_free_place;

  float point_min_x = -0.4;
  float point_max_x = 5;

  float point_min_y = -1;
  float point_max_y = 0.2;

public:
  Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Parking();

  bool init();
  void obstacle_callback(const selfie_msgs::PolygonArray &);
  bool find_free_place();
  void generate_offsets();
  void display_bottom_lines();
  void display_free_place();
  void reset();
};
