#include "../../selfie_parking/include/selfie_parking/detect_on_road.h"

void dynamic_detector::obstacleCB(const selfie_msgs::PolygonArray &msg)
{
  
  if(right_line.size() != 0 && middle_line.size() != 0)
  {
    auto it = middle_line.begin();
    ROS_INFO("middle_line: %fx^2 + %fx + %f", *it, *(++it), *(++it));
  }
}


void dynamic_detector::markingsCB(const selfie_msgs::RoadMarkings &msg)
{
   // ROS_INFO("markings_CB");
  for(auto it:msg.left_line)
      left_line.push_back(it);
  for(auto it:msg.right_line)
    middle_line.push_back(it);
  for (auto it:msg.center_line)
    right_line.push_back(it);

 // cout << msg.left_line.size() << "    " << msg.center_line.size() << "   " << msg.right_line.size() << endl;
  //  ROS_INFO("END of markings_CB");
}

void dynamic_detector::scanCB(const sensor_msgs::LaserScanConstPtr msg)
{
//  ROS_INFO("scan CB");
  sensor_msgs::PointCloud cloud;
  projector_.projectLaser(*msg, cloud);

  auto point_vec = cloud.points;
  auto it = point_vec.begin();
  vector<geometry_msgs::Point32> points_on_the_road;
  for(; it != point_vec.end();  ++it)
  {
    if(check_point(*it))
      points_on_the_road.push_back(*it);
  }
  if(!points_on_the_road.empty())
    ROS_WARN("obstacle detected on our track!!");

    
}

bool dynamic_detector::check_point(geometry_msgs::Point32 &pt)
{
    if(pt.x > max_range || pt.x < min_range)
      return false;
//  ROS_INFO("range ok stopien: %f,   %f!", right_line.size(), middle_line.size());
    if(right_line.size() < 3)
      return false;
    if(middle_line.size() < 3)
      return false;
//ROS_INFO("stopien ok!");
    auto i = right_line.begin();
    float a1 = *i;
    float b1 =  *(++i);
    float c1 = *(++i);

    auto i2 = middle_line.begin();
    float a2 = *i2;
    float b2 =  *(++i2);
    float c2 = *(++i2);
//ROS_INFO("przypisywanie OK");
    bool one = false;
    if(pt.y > (a1*pow(pt.x, 2) + b1*pt.x + c1))
      one = true; //punkt jest powyżej prawej lini (po lewej z punktu widzenia auta)
    bool two = false;
    if(pt.y < (a2*pow(pt.x, 2) + b2*pt.x + c2))
      two = true; //punkt jest poniżej środkowej linii
 // ROS_INFO("obliczenia OK");
    if(one == true && two ==true)
      return true;
    else 
      return false;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  dynamic_detector detector(nh, pnh);
  detector.init();
  while (ros::ok())
  {
      ros::spin();
  }

  return 0;
}


/*

void Parking::search(const selfie_msgs::PolygonArray &msg)
{

  //new frame and new vector
    auto begin = std::chrono::high_resolution_clock::now();
    auto end_of_box_creation = std::chrono::high_resolution_clock::now();
    auto end_of_callback = begin;
//
//int box_nr = msg.polygons.size()-1;  box_nr >= 0;  --box_nr
//
  for(auto box_nr=0;  box_nr != msg.polygons.size();  ++box_nr)
  {
    float min_x = point_min_x;
    float max_x = point_max_x;
    float min_y = point_min_y;
    float max_y = point_max_y;
   //  cout << "box_nr:" << box_nr << endl;
    if(visualization_type == 2)
      begin = std::chrono::high_resolution_clock::now();
    geometry_msgs::Polygon polygon = msg.polygons[box_nr];
    bool box_ok = true;
    for(int a = 0;  a < 4;  ++a)
    {
      Point p(polygon.points[a]);
      if( !p.check_position(min_x, max_x, min_y, max_y ))
      {
        box_ok = false;
        break;
      }
    }
    if(box_ok)
    {
     Box temp_box(polygon);
     min_x = temp_box.top_left.x;
     this->boxes_on_the_right_side.push_back(temp_box);
   //  cout << "box_created\n";
    }
      //now we reset, but later
      //TODO: use odometry to find yourself in space
      if(visualization_type == 2)
        end_of_box_creation = std::chrono::high_resolution_clock::now();
  }//box_nr for
  if(visualization_type == 2)
  {
    end_of_callback = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> diff = end_of_box_creation - begin;
    cout << "box_creation: " << diff.count() << " [microseconds]" << endl;
    diff = end_of_callback - begin;
    cout << "whole callback : : " << diff.count()  << " [microseconds]" << endl;
  }
}//obstacle_callback

double inline Parking::get_dist_from_first_free_place()
{ 
  return first_free_place.bottom_left.x;
}

*/
