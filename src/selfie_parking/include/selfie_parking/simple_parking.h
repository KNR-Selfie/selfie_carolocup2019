#pragma once
#include <ros/ros.h>
#include <iostream>
#include <string>
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
#include <ackermann_msgs/AckermannDriveStamped.h>


#include <selfie_park/parkAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/console.h>


class simple_parking {
private:
  float turn_angle;
  float speed;

  float time_go_1;
  float time_turn_2;
  float time_go_3;
  float time_turn_4;
  float time_go_5;

  std::string ackerman_topic_name;

  ros::NodeHandle nh_, pnh_;
  actionlib::SimpleActionServer<selfie_park::parkAction> as_;
  ros::Publisher ackermann_pub;

public:
  simple_parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~simple_parking();

  void goalCB();
  void preemptCB();
  void drive(float speed, float steering_angle, float distance);
};