#include "../../selfie_parking/include/selfie_parking/parking.h"


Parking::Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
                nh_(nh),
                pnh_(pnh)
{}

Parking::~Parking(){}

bool Parking::find_free_place()
{

}

bool Parking::init()
{
  this->obstacles_sub = nh_.subscribe("/obstacles", 10, &Parking::obstacle_callback, this);
}

/*
struct Box
{
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    Line left_vertical_line;
    Line bottom_horizontal_line;
};
*/

void Parking::obstacle_callback(const selfie_msgs::PolygonArray &msg)
{
    // jednostki w metrach
    float point_min_x = 0;
    float point_max_x = 2;

    float point_min_y = -2;
    float point_max_y = 0;

  //two_boxes[0]->bottom_left =
  Box box_1;
  size_t detected_boxes = 0;


  for(size_t box_nr = 0;  box_nr < msg.polygons.size();  ++box_nr)
  {
    geometry_msgs::Polygon polygon = msg.polygons[box_nr];
    bool box_ok = true;
    for(int a = 0;  a < 4;  ++a)
    {
    //  std::cout << "polygon_size: " << polygon.size() << std::endl;
      Point p(polygon.points[a]);
      if( !p.check_position(point_min_x, point_max_x, point_min_y, point_max_y ))
      {
        box_ok = false;
        break;
      }
    }
    if(box_ok)
    {
      ++detected_boxes;
     
      ROS_INFO("box ok");
    }

    /*
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    Line left_vertical_line;
    Line bottom_horizontal_line;
    */
  }//box_nr for
}//obstacle_callback

void Parking::visualize()
{

}
