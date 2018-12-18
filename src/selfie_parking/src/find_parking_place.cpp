#include "../../selfie_parking/include/selfie_parking/parking.h"

Parking::Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
                nh_(nh),
                pnh_(pnh)
{}

Parking::~Parking(){}

bool Parking::find_free_place()
{
  if(boxes_on_the_right_side.empty())
    return 0;
  vector<Box> tmp_vec = boxes_on_the_right_side;
  vector<Box>::iterator it = tmp_vec.begin();
  Point b_l = (*it).bottom_left;
  Point b_r = (*it).top_right;
  ++it;
  Point t_l = (*it).bottom_left;
  Point t_r = (*it).bottom_right;
  Box free_parking_place(b_l, b_r, t_l, t_r);
  first_free_place = free_parking_place;
  return 1;
}

bool Parking::init()
{
  this->obstacles_sub = nh_.subscribe("/obstacles", 10, &Parking::obstacle_callback, this);
  this->visualize_lines_pub = nh_.advertise<visualization_msgs::Marker>( "/visualization_lines", 1 );
  this->position_offset_pub = nh_.advertise<std_msgs::Float64>("/position_offset",200);
  this->heading_offset_pub = nh_.advertise<std_msgs::Float64>("/heading_offset",200);
}

void Parking::obstacle_callback(const selfie_msgs::PolygonArray &msg)
{
    // jednostki w metrach

  size_t detected_boxes = 0;
  //new frame and new vector
  boxes_on_the_right_side.clear();

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
     Box temp_box(polygon);
     point_min_x = temp_box.top_left.x;
     this->boxes_on_the_right_side.push_back(temp_box);
//     cout << "box_created\n";
    }
    else{
      //now we reset, but later
      //TODO: use odometry to find yourself in space
      reset();
    }

  }//box_nr for
//  cout << "out of box creation\n";
    this->generate_offsets();
    this->display_bottom_lines();
    if (this->find_free_place() )
       this->display_free_place();
}//obstacle_callback

void Parking::reset()
{
  boxes_on_the_right_side.clear();
  first_free_place.reset();
  point_min_x = -0.4;
  point_max_x = 5;

  point_min_y = -1;
  point_max_y = 0.2;
}

void Parking::generate_offsets()
{
  if(boxes_on_the_right_side.empty())
    return;

//  cout << "nr_of boxes saved: " << boxes_on_the_right_side.size() <<endl;
  double sum = 0;
  std_msgs::Float64 mean;
  mean.data = 0;
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();

  Point top = (*(--boxes_on_the_right_side.end())).bottom_left;
  Point down = (*iter).bottom_left;

  if(abs(top.y - down.y) < 0.05)
    mean.data = 0;
  else
  {
    mean.data = (top.y - down.y)/(top.x - down.x);
  }
  heading_offset_pub.publish(mean);
  cout << mean.data  <<endl;
}


void Parking::display_bottom_lines()
{
//  cout << "displaying bottom lines\n";
  visualization_msgs::Marker marker;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "lines_from_parking_node";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.color.r = 0.0f;
  marker.color.g = 60.0f;
  marker.color.b = 255.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;


  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  for(size_t b = 0;  b < boxes_on_the_right_side.size();  ++b)
  {
    for(int i = 0; i < 4; i++)
    {
        marker_point.x = boxes_on_the_right_side[b].bottom_left.x;
        marker_point.y = boxes_on_the_right_side[b].bottom_left.y+1;
        marker.points.push_back(marker_point);

        marker_point.x = boxes_on_the_right_side[b].bottom_right.x;
        marker_point.y = boxes_on_the_right_side[b].bottom_right.y-1;
        marker.points.push_back(marker_point);
        visualize_lines_pub.publish(marker);
    }
  }
}

void Parking::display_free_place()
{
  //  cout << "displaying bottom lines\n";
    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "free_parking_place";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 0.0f;
    marker.color.g = 255.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

    for(size_t b = 0;  b < 4;  ++b)
    {
          marker_point.x = first_free_place.bottom_left.x;
          marker_point.y = first_free_place.bottom_left.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.bottom_right.x;
          marker_point.y = first_free_place.bottom_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.top_right.x;
          marker_point.y = first_free_place.top_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.top_left.x;
          marker_point.y = first_free_place.top_left.y;
          marker.points.push_back(marker_point);

          visualize_lines_pub.publish(marker);
    }
}
