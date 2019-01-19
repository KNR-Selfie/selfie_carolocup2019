#pragma once


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <selfie_park/parkAction.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <vector>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define ODOM_TO_FRONT 0.25
#define ODOM_TO_BACK -0.05
#define ODOM_TO_LASER 0.15
#define CAR_WIDTH 0.22
#define PARKING_SPEED 0.2
#define MAX_TURN 0.5

class ParkService
{
    public:
    ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

    private:
    ros::NodeHandle nh_, pnh_;
    actionlib::SimpleActionServer<selfie_park::parkAction> as_;

    ros::Subscriber odom_sub;
    ros::Publisher ackermann_pub;
    void odom_callback(const nav_msgs::Odometry &msg);
    void goalCB();
    void preemptCB();
    
    struct Position
    {
        float x;
        float y;
        float rot;
        tf::Transform transform;
        float quat_to_rot(const geometry_msgs::Quaternion &quat);
        Position(const nav_msgs::Odometry &msg, float offset = 0);
        Position(float x_ = 0, float y_ = 0, float rot_ = 0);
        Position operator-(const Position &other);
        Position(const tf::Transform &trans);
        Position(const Position &other, float offset = 0);

    } actual_odom_position, actual_parking_position, parking_spot_position, actual_back_odom_position, actual_front_odom_position,
    actual_laser_odom_position, actual_back_parking_position, actual_front_parking_position, actual_laser_parking_position;


    float front_distance();
    float back_distance();
    void drive(float speed, float steering_angle);
    bool in_parking_spot();
    bool in_traffic_lane();
    bool to_parking_spot();
    bool park();
    bool leave();
    void init_parking_spot(const geometry_msgs::Polygon &msg);

    enum Parking_State
	{
		not_parking = 0,
		go_to_parking_spot = 1,
		going_in = 2,
		parked = 3,
		going_out = 4,
	} parking_state;

    float front_wall;
    float back_wall;
    float middle_of_parking_spot_y;
    float middle_of_parking_spot_x;
    float parking_spot_width;
    float parking_spot_length;
    float leaving_target;

    enum Move_State
	{
		init_move = 0,
		get_straigth = 1,
		get_middles = 2,
		first_phase = 3,
		second_phase = 4,
		end = 5
	} move_state;

    float mid_x;
    float mid_y;
    bool right;
    bool front;
    bool parking;

    //params
    std::string odom_topic;
    std::string ackermann_topic;
    float minimal_start_parking_x;
    float maximal_start_parking_x;
    float traffic_lane_marigin;
    float earlier_turn;
    float max_distance_to_wall;
    float first_to_second_phase_x_frontwards;
    float first_to_second_phase_x_backwards;
    bool state_msgs;
};