#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/searchAction.h>


#include <selfie_park/parkAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/console.h>

#include "shapes.h"

using namespace std;

enum parking_state{searching=0,  planning_failed=1, planning=2, parking=3};

class Parking{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher visualize_lines_pub;
  ros::Publisher visualize_free_place;
  ros::Publisher point_pub;
  ros::Publisher parking_state_pub;
  ros::Publisher parking_place_pub;

  actionlib::SimpleActionServer<selfie_msgs::searchAction> search_server_;

  std::vector<Box> boxes_on_the_right_side;
  std::vector<Box> potential_free_places;
  // for now only this is used
  std::vector<Box> for_planning;
  Box first_free_place;
  float distance_to_stop;
  float min_spot_lenght;
  int visualization_type;
  int scans_ignored;
  int scans_taken;
  bool debug_mode;

  bool goal_set;

  selfie_msgs::searchFeedback feedback_msg;
  selfie_msgs::searchResult result;



  parking_state state = searching;
  double planning_scan_counter=0;
  double planning_error_counter = 0;

  // jednostki w metrach
  float point_min_x;
  float point_max_x;

  float point_min_y;
  float point_max_y;

public:
  Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~Parking();

  bool init();
  void manager_init();
  void manager(const selfie_msgs::PolygonArray &);
  void search(const selfie_msgs::PolygonArray &);
  void get_exact_measurements();
  double count_surface_area(Box);
  bool find_free_place();
  double get_dist_from_first_free_place();
  void generate_offsets();
  void publish_place();
  void display_bottom_lines();
  void display_left_lines();
  void display_free_place();
  void reset();
  void send_goal();
  void preemptCB();
  void print_planning_stats();
};
