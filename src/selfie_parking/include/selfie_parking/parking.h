#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

#include "shapes.h"

using namespace std;

enum parking_state{searching, planning, parking};

class Parking{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_lines_pub;
  ros::Publisher visualize_free_place;
  ros::Publisher position_offset_pub;
  ros::Publisher heading_offset_pub;
  ros::Publisher point_pub;
  ros::Publisher parking_state_pub;

  std::vector<Box> boxes_on_the_right_side;
  std::vector<Box> potential_free_places;
  // for now only this is used
  std::vector<Box> for_planning;
  Box first_free_place;
  double distance_to_stop = 0.3;

  parking_state state = searching;
  double planning_scan_counter=0;
  double planning_error_counter = 0;

  // jednostki w metrach
  const float point_min_x = -1;
  const float point_max_x = 5;

  const float point_min_y = -1;
  const float point_max_y = 0.2;

  struct global_box{
    Box box;
    Point pose;
  };

public:
  Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Parking();

  bool init();
  void manager(const selfie_msgs::PolygonArray &);
  void search(const selfie_msgs::PolygonArray &);
  void get_exact_measurements();
  double count_surface_area(Box);
  bool find_free_place();
  double get_dist_from_first_free_place();
  void generate_offsets();
  void display_bottom_lines();
  void display_left_lines();
  void display_free_place();
  void reset();
};
