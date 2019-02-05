#include "change_lane_logic.h"

ChangeLaneLogic cll;

void road_markings_callback(const selfie_msgs::RoadMarkings::ConstPtr& msg);
void obstacles_callback(const selfie_msgs::PolygonArray::ConstPtr& msg);
void distance_callback(const std_msgs::Float32::ConstPtr& msg);
void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void positionCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "selfie_logic");
  ros::NodeHandle n;
  ros::Publisher speed_logic_publisher = n.advertise<std_msgs::Float32>("speed_logic", 100);
  
  ros::Subscriber road_markings_subsciber = n.subscribe("vision/road_markings",1, road_markings_callback);
  ros::Subscriber obstacles_subscriber = n.subscribe("obstacles", 1, obstacles_callback);
  ros::Subscriber distance_subscriver = n.subscribe("distance",1,distance_callback);
  ros::Subscriber ackerman_subscriber = n.subscribe("drive", 1, ackermanCallback);
  ros::Subscriber position_subscriber = n.subscribe("position_offset", 1, positionCallback);
 

  while (ros::ok())
  {
    
    //static std_msgs::Float32 speed_logic_msg;
    //speed_logic_publisher.publish(speed_logic_msg);
    
    ros::spinOnce();
  }
}

//reading road markings callback
void road_markings_callback(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{
  cll.line_coef.left.clear();
  cll.line_coef.right.clear();
  cll.line_coef.center.clear();

  for (int i=0; i<msg->left_line.size(); ++i){
    cll.line_coef.left.push_back(msg->left_line[i]);
    cll.line_coef.right.push_back(msg->right_line[i]);
    cll.line_coef.center.push_back(msg->center_line[i]);
  }
}

//obstacles callback
void obstacles_callback(const selfie_msgs::PolygonArray::ConstPtr& msg)
{
  cll.right_points = 0;
  cll.left_points = 0;
  for(int box_nr = msg->polygons.size()-1;  box_nr >= 0;  --box_nr)
  {
    //ROS_INFO("box_nr: %d",box_nr);

    geometry_msgs::Polygon polygon = msg->polygons[box_nr]; 
    cll.check_lane_status(polygon);
    
  }
  ROS_INFO("Right: %d \t left: %d",cll.right_points, cll.left_points);
}

//distance callback
void distance_callback(const std_msgs::Float32::ConstPtr& msg){
  cll.distance = msg->data;
}

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){
  cll.speed = msg->drive.speed;
}

void positionCallback(const std_msgs::Float64::ConstPtr& msg){
  if (msg-> data>0)
    cll.car_lane = c_right;
  else{
    cll.car_lane = c_left;
  }
}