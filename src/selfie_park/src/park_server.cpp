#include "../include/selfie_park/park_server.h"


ParkService::ParkService(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),
pnh_(pnh),
as_(nh_, "park",  false)
{
  pnh_.param<std::string>("odom_topic", odom_topic,"/vesc/odom");
  pnh_.param<std::string>("ackermann_topic", ackermann_topic,"/sim_drive");
  pnh_.param<float>("minimal_start_parking_x", minimal_start_parking_x, -0.1);
  pnh_.param<float>("maximal_start_parking_x", maximal_start_parking_x, 0.0);
  pnh_.param<float>("traffic_lane_marigin",traffic_lane_marigin, 0.05);
  pnh_.param<float>("earlier_turn", earlier_turn, 0.01);
  pnh_.param<float>("first_to_second_phase_x_frontwards",first_to_second_phase_x_frontwards, 1.0/3.0);
  pnh_.param<float>("first_to_second_phase_x_backwards", first_to_second_phase_x_backwards, 1.0/2.0);
  pnh_.param<bool>("state_msgs",state_msgs, true);
  pnh_.param<float>("max_distance_to_wall", max_distance_to_wall, 0.03);
  move_state = init_move;
  parking_state = not_parking;
  as_.registerGoalCallback(boost::bind(&ParkService::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&ParkService::preemptCB, this));
  as_.start();
  odom_sub = nh_.subscribe(odom_topic, 10, &ParkService::odom_callback, this);
  ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic, 10);
}

void ParkService::odom_callback(const nav_msgs::Odometry &msg)
{
  actual_odom_position = Position(msg);
  actual_back_odom_position = Position(actual_odom_position, ODOM_TO_BACK);
  actual_front_odom_position = Position(actual_odom_position, ODOM_TO_FRONT);
  actual_laser_odom_position = Position(actual_odom_position, ODOM_TO_LASER);
  
  //std::cout<<actual_odom_position.x<<"  "<<actual_odom_position.y<<"  "<<actual_back_odom_position.x<<"  "<<actual_back_odom_position.y<<"  "<<actual_front_odom_position.x<<"  "<<actual_front_odom_position.y<<std::endl;

   if(parking_state>not_parking)
   {
    std::cout<<actual_parking_position.x<<"  "<<actual_parking_position.y<<std::endl;
    actual_parking_position = Position(parking_spot_position.transform.inverse()*actual_odom_position.transform);
    actual_back_parking_position = Position(actual_parking_position, ODOM_TO_BACK);
    actual_front_parking_position = Position(actual_parking_position, ODOM_TO_FRONT);
    actual_laser_parking_position = Position(actual_parking_position, ODOM_TO_LASER);
   }

  switch(parking_state)
	{
		case not_parking:
		if(state_msgs) ROS_INFO("not_parking");
		break;

		case go_to_parking_spot:
		if(to_parking_spot()){parking_state = going_in;}
		if(state_msgs) ROS_INFO("go_to_parking_spot");
		break;

		case going_in:
		if(state_msgs) ROS_INFO("get_in");
		if(park()) parking_state = parked;
		break;

		case parked:
		if(state_msgs) ROS_INFO("PARKED");
		ros::Duration(2).sleep();
		parking_state = going_out;
		break;

		case going_out:
		if(state_msgs) ROS_INFO("get_out");
		if(leave()) parking_state = not_parking;
		break;
	}
}

void ParkService::init_parking_spot(const geometry_msgs::Polygon &msg)
{
  std::vector<tf::Vector3> odom_parking_spot;
  
  for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it<msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    odom_parking_spot.push_back(actual_laser_odom_position.transform*vec);
  }
  tf::Vector3 tl = odom_parking_spot[0];
  tf::Vector3 bl = odom_parking_spot[1];
  tf::Vector3 br = odom_parking_spot[2];
  tf::Vector3 tr = odom_parking_spot[3];
  parking_spot_position = Position(bl.x(), bl.y(), atan2(br.y()-bl.y(), br.x()-bl.x()));
  actual_parking_position = Position(parking_spot_position.transform.inverse()*actual_odom_position.transform);
  std::vector<tf::Vector3> parking_parking_spot;
  for(std::vector<geometry_msgs::Point32>::const_iterator it = msg.points.begin(); it<msg.points.end(); it++)
  {
    tf::Vector3 vec(it->x, it->y, 0);
    parking_parking_spot.push_back(actual_parking_position.transform*vec);
  }
  tl = parking_parking_spot[0];
  bl = parking_parking_spot[1];
  br = parking_parking_spot[2];
  tr = parking_parking_spot[3];
  parking_spot_width = tl.y()<tr.y()?tl.y():tr.y();
  middle_of_parking_spot_y = parking_spot_width/2.0;
  back_wall = tl.x()>bl.x()?tl.x():bl.x();
  front_wall = tr.x()<br.x()?tr.x():br.x();
  middle_of_parking_spot_x = (front_wall - back_wall)/2.0;
  leaving_target = actual_parking_position.y;
}
void ParkService::goalCB()
{
  std::cout<<"got goal"<<std::endl;
  selfie_park::parkGoal goal = *as_.acceptNewGoal();
  init_parking_spot(goal.parking_spot);
  parking_state = go_to_parking_spot;
}

void ParkService::preemptCB()
{
  ROS_INFO("parkService preempted");
  parking_state = not_parking;
  as_.setPreempted();
}

float ParkService::front_distance()
{
  return front_wall - actual_front_parking_position.x - sin(actual_parking_position.rot) * CAR_WIDTH/2.0;
}

float ParkService::back_distance()
{
  return actual_back_parking_position.x - back_wall - sin(actual_parking_position.rot) * CAR_WIDTH/2.0;
}

void ParkService::drive(float speed, float steering_angle)
{
	ackermann_msgs::AckermannDriveStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.drive.speed = speed;
	msg.drive.steering_angle = steering_angle;
	ackermann_pub.publish(msg);
}

//void go()
bool ParkService::in_parking_spot()
{
  float l = cos(actual_parking_position.rot)*CAR_WIDTH/2;
	bool is_in = parking_spot_width > actual_back_parking_position.y + l && actual_back_parking_position.y - l > 0 && parking_spot_width > actual_front_parking_position.y + l && actual_back_parking_position.y - l > 0;
	return is_in;
}
bool ParkService::in_traffic_lane()
{
  if(actual_back_parking_position.y < leaving_target + traffic_lane_marigin && actual_back_parking_position.y > leaving_target - traffic_lane_marigin &&
  actual_front_parking_position.y < leaving_target + traffic_lane_marigin && actual_front_parking_position.y > leaving_target - traffic_lane_marigin)
  {return true;}
  else return false;
}
bool ParkService::to_parking_spot()
{
  selfie_park::parkFeedback feedback;
  feedback.distance = actual_parking_position.y - middle_of_parking_spot_y;
  as_.publishFeedback(feedback);
  if(actual_parking_position.x < back_wall + minimal_start_parking_x) drive(PARKING_SPEED, 0.0);
  else if(actual_parking_position.x > front_wall + maximal_start_parking_x) drive(-PARKING_SPEED, 0.0);
  else return true;

  return false;
}

bool ParkService::park()
{
  selfie_park::parkFeedback feedback;
  feedback.distance = actual_parking_position.y - middle_of_parking_spot_y;
  as_.publishFeedback(feedback);
  
	switch(move_state)
	{
		case init_move:
		if(state_msgs) ROS_INFO("init_move");
		front = front_distance() > back_distance();
		right = actual_parking_position.y > middle_of_parking_spot_y;
		move_state = get_straigth;

		case get_straigth:
		if(state_msgs) ROS_INFO("get_straigth");
		if(front && right)
		{
			if(actual_parking_position.rot > 0) drive(PARKING_SPEED, -MAX_TURN);
			else move_state = get_middles;
		}
		else if(!front && right)
		{
			if(actual_parking_position.rot < 0) drive(-PARKING_SPEED, -MAX_TURN);
			else move_state = get_middles;
		}
		if(front && !right)
		{
			if(actual_parking_position.rot < 0) drive(PARKING_SPEED, MAX_TURN);
			else move_state = get_middles;			
		}
		else if(!front && !right)
		{
			if(actual_parking_position.rot > 0) drive(-PARKING_SPEED, MAX_TURN);
			else move_state = get_middles;
		}
		break;

		case get_middles:
		if(state_msgs) ROS_INFO("get_middles");
		mid_x = (front?actual_front_parking_position.x  + (front_wall - max_distance_to_wall  - actual_front_parking_position.x)*first_to_second_phase_x_frontwards:  back_wall + (actual_back_parking_position.x + max_distance_to_wall - back_wall)*first_to_second_phase_x_backwards);
		mid_y = middle_of_parking_spot_y + (actual_back_parking_position.y - middle_of_parking_spot_y)/2.0;
		move_state = first_phase;
    right = actual_parking_position.y > middle_of_parking_spot_y;
		
		case first_phase:
		if(state_msgs) ROS_INFO("1st phase");
		drive(front?PARKING_SPEED:-PARKING_SPEED, right?-MAX_TURN:MAX_TURN);
		if(front && right)
		{
			if(actual_front_parking_position.x > mid_x - earlier_turn || actual_parking_position.y < mid_y) move_state = second_phase;
		}
		else if(!front && right)
		{
			if(actual_back_parking_position.x < mid_x + earlier_turn || actual_parking_position.y < mid_y) move_state = second_phase;
		}
		if(front && !right)
		{
			if(actual_front_parking_position.x > mid_x  - earlier_turn|| actual_parking_position.y > mid_y) move_state = second_phase;
		}
		else if(!front && !right)
		{
			if(actual_back_parking_position.x < mid_x + earlier_turn || actual_parking_position.y > mid_y) move_state = second_phase;
		}
		break;
		case second_phase:
		if(state_msgs) ROS_INFO("2nd phase");
		drive(front?PARKING_SPEED:-PARKING_SPEED, right?MAX_TURN:-MAX_TURN);
		if(front && right)
		{
			if(actual_parking_position.rot > 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && right)
		{
			if(actual_parking_position.rot < 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		if(front && !right)
		{
			if(actual_parking_position.rot < 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && !right)
		{
			if(actual_parking_position.rot > 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		break;
		case end:
		if(state_msgs) ROS_INFO("end");
		drive(0,0);
			if(in_parking_spot())
			{
				move_state = init_move;
        selfie_park::parkResult result;
        result.done = true;
        as_.setSucceeded(result);
				return true;
			}
			else
			{
				move_state = init_move;
			}
		break;
	
	}
	return false;
}

bool ParkService::leave()
{
  selfie_park::parkFeedback feedback;
  feedback.distance = leaving_target - actual_parking_position.y;
  as_.publishFeedback(feedback);
  
	switch(move_state)
	{
		case init_move:
		if(state_msgs) ROS_INFO("init_move");
		front = front_distance() > back_distance();
		right = actual_parking_position.y > leaving_target;
		move_state = get_straigth;

		case get_straigth:
		if(state_msgs) ROS_INFO("get_straigth");
		if(front && right)
		{
			if(actual_parking_position.rot > 0) drive(PARKING_SPEED, -MAX_TURN);
			else move_state = get_middles;
		}
		else if(!front && right)
		{
			if(actual_parking_position.rot < 0) drive(-PARKING_SPEED, -MAX_TURN);
			else move_state = get_middles;
		}
		if(front && !right)
		{
			if(actual_parking_position.rot < 0) drive(PARKING_SPEED, MAX_TURN);
			else move_state = get_middles;			
		}
		else if(!front && !right)
		{
			if(actual_parking_position.rot > 0) drive(-PARKING_SPEED, MAX_TURN);
			else move_state = get_middles;
		}
		break;

		case get_middles:
		if(state_msgs) ROS_INFO("get_middles");
		mid_x = (front?actual_front_parking_position.x  + (front_wall - max_distance_to_wall  - actual_front_parking_position.x)*first_to_second_phase_x_frontwards:  back_wall + (actual_back_parking_position.x + max_distance_to_wall - back_wall)*first_to_second_phase_x_backwards);
		mid_y = leaving_target + (actual_back_parking_position.y - leaving_target)/2.0;
		move_state = first_phase;
    right = actual_parking_position.y > leaving_target;
		
		case first_phase:
		if(state_msgs) ROS_INFO("1st phase");
		
		drive(front?PARKING_SPEED:-PARKING_SPEED, right?-MAX_TURN:MAX_TURN);
		if(front && right)
		{
			if(actual_front_parking_position.x > mid_x - earlier_turn || actual_parking_position.y < mid_y) move_state = second_phase;
		}
		else if(!front && right)
		{
			if(actual_back_parking_position.x < mid_x + earlier_turn || actual_parking_position.y < mid_y) move_state = second_phase;
		}
		if(front && !right)
		{
			if(actual_front_parking_position.x > mid_x  - earlier_turn|| actual_parking_position.y > mid_y) move_state = second_phase;
		}
		else if(!front && !right)
		{
			if(actual_back_parking_position.x < mid_x + earlier_turn || actual_parking_position.y > mid_y) move_state = second_phase;
		}
		break;
		case second_phase:
		if(state_msgs) ROS_INFO("2nd phase");
		drive(front?PARKING_SPEED:-PARKING_SPEED, right?MAX_TURN:-MAX_TURN);
		if(front && right)
		{
			if(actual_parking_position.rot > 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && right)
		{
			if(actual_parking_position.rot < 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		if(front && !right)
		{
			if(actual_parking_position.rot < 0 || front_distance() < max_distance_to_wall) move_state = end;
		}
		else if(!front && !right)
		{
			if(actual_parking_position.rot > 0 || back_distance() < max_distance_to_wall) move_state = end;
		}
		break;
		case end:
		if(state_msgs) ROS_INFO("end");
		drive(0,0);
			if(in_traffic_lane())
			{
				move_state = init_move;
        selfie_park::parkResult result;
        result.done = true;
        as_.setSucceeded(result);
				return true;
			}
			else
			{
				move_state = init_move;
			}
		break;
	}
	return false;
}

float ParkService::Position::quat_to_rot(const geometry_msgs::Quaternion &quat)
{
  //std::cout<<tf::getYaw(quat)<<std::endl;
  return static_cast<float>(tf::getYaw(quat));
}

ParkService::Position::Position(const nav_msgs::Odometry &msg, float offset)
{
    rot = quat_to_rot(msg.pose.pose.orientation);
    x = msg.pose.pose.position.x + offset*cos(rot);
    y = msg.pose.pose.position.y + offset*sin(rot);
    tf::Vector3 vec(x,y,0);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    transform = tf::Transform(quat, vec);
}
ParkService::Position::Position(float x_, float y_, float rot_):
x(x_),
y(y_),
rot(rot_)
{
    tf::Vector3 vec(x,y,0);
    tf::Quaternion quat = tf::createQuaternionFromYaw(rot);
    transform = tf::Transform(quat, vec);
}
ParkService::Position ParkService::Position::operator-(const Position &other)
{
  return Position(this->x - other.x, this->y - other.y, this->rot - other.rot);
}

ParkService::Position::Position(const tf::Transform &trans)
{
    x = trans.getOrigin().x();
    y = trans.getOrigin().y();
    rot = tf::getYaw(trans.getRotation());
    transform = trans;
}
ParkService::Position::Position(const Position &other, float offset)
{
  rot = other.rot;
  x = other.x + cos(rot) * offset;
  y = other.y + sin(rot) * offset;
  tf::Quaternion quat;
  quat.setRPY(0,0,rot);
  tf::Vector3 vec(x,y,0);
  transform = tf::Transform(quat, vec);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "park_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ParkService park(nh, pnh);
  ros::spin();

  return 0;
}