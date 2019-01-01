#pragma once

#include <ros/ros.h>
#include <vector>
#include <selfie_msgs/PolygonArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#define ODOM_TO_FRONT 0.26
#define ODOM_TO_BACK 0.04
#define MAX_TURN 0.7
#define CAR_WIDTH 0.2
#define MARGIN 0.05

class Park
{
	private:
		enum Parking_State
		{
			not_parking = 0,
			init_parking_spot = 1,
			go_to_parking_spot = 6,
			going_in = 2,
			parked = 3,
			going_out = 4,
			out = 5
		} parking_state;

		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Publisher ackermann_pub;
		tf::TransformBroadcaster transform_broadcaster;
		tf::TransformListener transform_listener;

		//scan
		message_filters::Subscriber<selfie_msgs::PolygonArray> scan_sub;
		tf::MessageFilter<selfie_msgs::PolygonArray> * tf_filter_scan;
		void scan_callback(const boost::shared_ptr<const selfie_msgs::PolygonArray> & msg_ptr);
		//odom
		message_filters::Subscriber<nav_msgs::Odometry> odomsub;
		tf::MessageFilter<nav_msgs::Odometry> * tf_filter_odom;
		void odom_callback(const boost::shared_ptr<const nav_msgs::Odometry> & msg_ptr);

		//parking spot
		float initial_front_wall;
		float front_wall;
		float back_wall;
		float parking_spot_mid_y;
		float parking_spot_mid_x;
		float traffic_lane_mid_y;
		float parking_spot_width;
		float parking_spot_length;

		void initialize_parking_spot(const geometry_msgs::PolygonStamped &msg);
		
		std::vector<geometry_msgs::PointStamped> convert_polygon_to_points(const geometry_msgs::PolygonStamped &msg);
		tf::Transform odom_to_parking;
		geometry_msgs::PoseStamped actual_pose, actual_back_pose, actual_front_pose;
		double actual_yaw;
		void get_position(const nav_msgs::Odometry &msg);
		void drive(float speed, float steering_angle);
		float front_distance();
		float back_distance();
		bool in_parking_spot();
		bool in_traffic_lane();
		bool is_point_good(const geometry_msgs::PointStamped &pt);

		//move variablres
		float mid_x;
		float mid_y;
		float parking_speed;
		bool front;
		bool right;

		bool get_in();
		bool get_out();
		bool to_parking_spot();

		enum Move_State
		{
			init_move = 0,
			get_straigth = 1,
			get_middles = 2,
			first_phase = 3,
			second_phase = 4,
			end = 5
		} move_state;
		
		//params
		float max_distance_to_wall;
		bool use_scan;
		bool always_get_wall;
		float scan_point_max_distance;
		bool state_msgs;
		float earlier_turn;
		float first_to_second_phase_x_frontwards;
		float first_to_second_phase_x_backwards;
		float minimal_start_parking_x;
		float maximal_start_parking_x;
		void print_params();

	public:
		Park(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
		bool init();
		void broadcast_parking_frame();

};