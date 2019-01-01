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

#define ODOM_TO_FRONT 0.26
#define ODOM_TO_BACK 0.04
#define MAX_TURN 0.7
#define CAR_WIDTH 0.2
#define MARGIN 0.05

class Park
{
	private:
		struct Point
		{
			double x;
			double y;
		};

		enum Parking_State
		{
			not_parking = 0,
			init_parking_spot = 1,
			going_in = 2,
			parked = 3,
			going_out = 4,
			out = 5
		} parking_state;

		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;

		ros::Subscriber odom_sub;
		ros::Subscriber scan_sub;
		//ros::Subscriber parking_spot_sub;
		ros::Publisher ackermann_pub;

		tf::TransformBroadcaster transform_broadcaster;
		tf::TransformListener transform_listener;



		void odom_callback(const nav_msgs::Odometry &msg);
		void scan_callback(const selfie_msgs::PolygonArray &msg);
		//void parking_spot_callback(const geometry_msgs::PolygonStamped &msg);


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
		//parking spot frame
		geometry_msgs::PolygonStamped convert_polygon_to_polygon(const geometry_msgs::PolygonStamped &msg);
		std::vector<geometry_msgs::PointStamped> convert_polygon_to_points( const geometry_msgs::PolygonStamped &msg);
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


		//move vars
		float mid_x;
		float mid_y;
		float parking_speed;
		bool front;
		bool right;

		bool get_in();
		bool get_out();
		enum Move_State
		{
			init_move = 0,
			first_phase = 1,
			second_phase = 2,
			end = 3,
			get_straigth = 4,
			get_middles = 5
		} move_state;
		
		//params
		float max_distance_to_wall;
		bool use_scan;
		bool always_get_wall;
		float scan_point_max_distance;
		bool state_msgs;
		float earlier_turn;





	public:
		Park(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
		bool init();








};