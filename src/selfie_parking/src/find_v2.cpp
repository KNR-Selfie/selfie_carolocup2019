#include <ros/ros.h>
#include <selfie_msgs/searchAction.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/console.h>
#include "shapes.h"


class detection_v2 {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obstacles_sub;
  ros::Publisher point_pub;

  actionlib::SimpleActionServer<selfie_msgs::searchAction> search_server_;

  Box first_free_place;

  selfie_msgs::searchFeedback feedback_msg;
  selfie_msgs::searchResult result;

  bool goal_set;

  float distance_to_stop;
  float min_spot_lenght;
  int visualization_type;
  int scans_ignored;
  int scans_taken;

  float point_min_x;
  float point_max_x;

  float point_min_y;
  float point_max_y;
  public:
    detection_v2(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    :nh_(nh),
     pnh_(pnh),
     search_server_(nh_, "search",  false)
    {
      search_server_.registerGoalCallback(boost::bind(&detection_v2::manager_init, this));
      search_server_.registerPreemptCallback(boost::bind(&detection_v2::preemptCB, this));
      search_server_.start();
      pnh_.param<float>("/point_min_x", point_min_x, 0);
      pnh_.param<float>("/point_max_x", point_max_x, 2);
      pnh_.param<float>("/point_min_y", point_min_y, -1);
      pnh_.param<float>("/point_max_y", point_max_y, 0.2);
      pnh_.param<float>("/distance_to_stop", distance_to_stop, 0.2);
      pnh_.param<float>("/min_spot_lenght", min_spot_lenght, 0.7);
      pnh_.param<int>("/scans_to_ignore_when_stopped", scans_ignored, 2);
      pnh_.param<int>("/scans_taken", scans_taken, 5);
    }

    void manager_init();
    void preemptCB();
}