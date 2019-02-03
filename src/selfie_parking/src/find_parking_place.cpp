#include "../../selfie_parking/include/selfie_parking/parking.h"


Parking::Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
                nh_(nh),
                pnh_(pnh),
                ac_("park", true)
{
  
  pnh_.param<float>("point_min_x", point_min_x, 0);
  pnh_.param<float>("point_max_x", point_max_x, 2);
  pnh_.param<float>("point_min_y", point_min_y, -1);
  pnh_.param<float>("point_max_y", point_max_y, 0.2);
  pnh_.param<float>("distance_to_stop", distance_to_stop, 0.2);
  pnh_.param<float>("min_spot_lenght", min_spot_lenght, 0.7);
  pnh_.param<int>("scans_to_ignore_when_stopped", scans_ignored, 2);
  pnh_.param<int>("scans_taken", scans_taken, 5);
  // 1->to_rviz  2-> funcion execution time 3->text_only
  pnh_.param<int>("visualization_type", visualization_type, 1);
}

Parking::~Parking(){}


bool Parking::init()
{
  this->obstacles_sub = nh_.subscribe("/obstacles", 10, &Parking::manager, this);
  this->visualize_lines_pub = nh_.advertise<visualization_msgs::Marker>( "/visualization_lines", 1 );
  this->visualize_free_place  = nh_.advertise<visualization_msgs::Marker>( "/free_place", 1 );
  this->point_pub = nh_.advertise<visualization_msgs::Marker>("/box_points", 5);
  this->position_offset_pub = nh_.advertise<std_msgs::Float64>("/position_offset",200);
  this->heading_offset_pub = nh_.advertise<std_msgs::Float64>("/heading_offset",200);
  this->parking_state_pub = nh_.advertise<std_msgs::Int16>("parking_state", 200);
  this->parking_place_pub = nh_.advertise<geometry_msgs::PolygonStamped>("parking_place", 10);
  ROS_INFO("viz type: %d", visualization_type);
}

void Parking::manager(const selfie_msgs::PolygonArray &msg)
{
  reset();
  std_msgs::Int16 state_msg;
  state_msg.data = state;
  parking_state_pub.publish(state_msg);
  if(state == searching)
  {
    planning_error_counter = 0;
    search(msg);
    bool place_found = find_free_place();
 //   first_free_place.visualize(point_pub);
    if(place_found && (get_dist_from_first_free_place()) <= distance_to_stop)
    {
      state = planning;
      planning_scan_counter = 0;
      if(visualization_type  >= 3)
        cout << "state switched to planning\n";
    }
    //display_bottom_lines();
   // display_left_lines();
  }
  else if(state == planning || state == planning_failed)
    {
      // ignore first two scans to give time to stop the car
      if(planning_scan_counter < scans_ignored)
      {
        ++planning_scan_counter;
        return;
      }
      else if(planning_scan_counter < scans_taken + scans_ignored)
      {
        search(msg);
        if(find_free_place())
        {
          for_planning.push_back(first_free_place);
          ++planning_scan_counter;
          if(visualization_type >= 3)
            cout << "ok\n";
        }
      }
      else if(planning_scan_counter == scans_taken)
      {
       // first_free_place.visualize(point_pub);
       // first_free_place.print_box_dimensions();
       if(visualization_type >= 3)
        ROS_DEBUG("getting exact dimensions\n");
       // ros::Duration(0.5).sleep();
       // takes the vector of laser scans and aproximates real parking place dimensions
        get_exact_measurements();
        planning_scan_counter = 0; 
        for_planning.clear();
        if(visualization_type == 1 || visualization_type == 13)    
        {
          display_free_place();
          first_free_place.visualize(point_pub);
        }    
        state = parking;
        send_goal();
        if(visualization_type >= 3)
        {
          first_free_place.print_box_dimensions();
          cout << "state swiched to parking\n";
        }
          
      }
    }
    if(planning_error_counter == 5)
    {
      planning_error_counter = 0;
      planning_scan_counter = 0;
      state = planning_failed;
      first_free_place.reset();
      reset();
    }
}

bool Parking::find_free_place()
{
  if(boxes_on_the_right_side.size() < 2)
    return false;
  float min_space = min_spot_lenght;
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();
  vector<Box>::const_iterator end_iter = boxes_on_the_right_side.cend();
  //for(;  iter != end_iter;  ++iter)
 // {
    double dist = (*iter).top_left.get_distance((*(iter+1)).bottom_left);
    if(dist > min_space)
    {
      Box tmp_box((*iter).top_left, (*iter).top_right, (*(iter+1)).bottom_left, (*(iter+1)).bottom_right);
      first_free_place = tmp_box;
      //first_free_place.print();
      if(visualization_type >= 3)
        tmp_box.visualize(point_pub);
      return true;
    }
    if(state == planning)
    {
      if(visualization_type >= 3)
        cout << "error, place not found!!!!\n";
      ++planning_error_counter;
    }

  //}
  return false;

}

void Parking::get_exact_measurements()
{
  Box real_place;
  auto box_it = for_planning.begin();
  real_place = *box_it;
  double min_surface = count_surface_area(*box_it);
  double min_top_left_x = 1000;
  double min_top_right_x = 1000;
  double max_top_left_y = -1000;
  double max_bottom_left_y = -1000;
  double max_bottom_left_x = -1000;
  double max_bottom_right_x = -1000;
  double max_bottom_right_y = -1000;

  for(;  box_it != for_planning.end();  ++box_it)
  {
    double surface = count_surface_area(*box_it);
    if(surface < min_surface)
    {
      min_surface = surface;
      real_place = *box_it;
    }

    if((*box_it).top_left.x < min_top_left_x)
      min_top_left_x = (*box_it).top_left.x;

    if((*box_it).top_right.x < min_top_right_x)
      min_top_right_x = (*box_it).top_right.x;

    if((*box_it).top_left.y > max_top_left_y)
      max_top_left_y = (*box_it).top_left.y;

    if((*box_it).bottom_left.y > max_bottom_left_y)
      max_bottom_left_y = (*box_it).bottom_left.y;

    if((*box_it).bottom_right.x > max_bottom_right_x)
      max_bottom_right_x = (*box_it).bottom_right.x;

    if((*box_it).bottom_right.y > max_bottom_right_y)
      max_bottom_right_y = (*box_it).bottom_right.y;

    if((*box_it).bottom_left.x > max_bottom_left_x)
      max_bottom_left_x = (*box_it).bottom_left.x;
  }
  real_place.top_left.x = min_top_left_x;
  real_place.top_left.y = max_top_left_y;
  real_place.top_right.x = real_place.top_left.x;
  // na sztywno
  real_place.top_right.y = real_place.top_left.y-0.3;
  real_place.bottom_left.y = max_bottom_left_y;
  real_place.bottom_left.x = max_bottom_left_x;
  real_place.bottom_right.x = real_place.bottom_left.x;
  // na sztywno
  real_place.bottom_right.y = real_place.bottom_left.y-0.3;

  first_free_place = real_place;
  // TODO
}

double Parking::count_surface_area(Box box)
{
  float bottom_edge = box.bottom_left.get_distance(box.bottom_right);
  float top_edge = box.top_left.get_distance(box.top_right);
  float right_edge = box.top_right.get_distance(box.bottom_right);
  float left_edge = box.top_left.get_distance(box.bottom_left);

  return max(bottom_edge, top_edge) * max(left_edge, right_edge);
}

void Parking::send_goal()
{
    selfie_park::parkGoal msg;
    geometry_msgs::Point32 p;
    p.x = first_free_place.bottom_left.x;
    p.y = first_free_place.bottom_left.y;
    msg.parking_spot.points.push_back(p);
    p.x = first_free_place.bottom_right.x;
    p.y = first_free_place.bottom_right.y;
    msg.parking_spot.points.push_back(p);
    p.x = first_free_place.top_right.x;
    p.y = first_free_place.top_right.y;
    msg.parking_spot.points.push_back(p);
    p.x = first_free_place.top_left.x;
    p.y = first_free_place.top_left.y;
    msg.parking_spot.points.push_back(p);
    msg.park = true;
    ac_.sendGoal(msg);
}

void Parking::search(const selfie_msgs::PolygonArray &msg)
{

  //new frame and new vector
    auto begin = std::chrono::high_resolution_clock::now();
    auto end_of_box_creation = std::chrono::high_resolution_clock::now();
    auto end_of_callback = begin;

  for(int box_nr = msg.polygons.size()-1;  box_nr >= 0;  --box_nr)
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
  cout << first_free_place.bottom_left.x << endl;
  return first_free_place.bottom_left.x;
}

void Parking::reset()
{
  boxes_on_the_right_side.clear();
 // first_free_place.reset();
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



void Parking::display_left_lines()
{
 // cout << "displaying left lines\n";
  visualization_msgs::Marker marker;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "left_lines_from_parking_node";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 2;
  marker.lifetime = ros::Duration();

  marker.color.r = 0.0f;
  marker.color.g = 60.0f;
  marker.color.b = 255.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;


  geometry_msgs::Point marker_point;
  marker_point.z = 0;
/*
  for(size_t b = 0;  b < boxes_on_the_right_side.size();  ++b)
  {
    for(int i = 0; i < 4; i++)
    {
        marker_point.x = boxes_on_the_right_side[b].bottom_left.x;
        marker_point.y = boxes_on_the_right_side[b].bottom_left.y;
        marker.points.push_back(marker_point);

        marker_point.x = boxes_on_the_right_side[b].top_left.x;
        marker_point.y = boxes_on_the_right_side[b].top_left.y;
        marker.points.push_back(marker_point);
        visualize_lines_pub.publish(marker);
    }
  }
  */
        marker_point.x = boxes_on_the_right_side[0].top_left.x;
        marker_point.y = boxes_on_the_right_side[0].top_left.y;
        marker.points.push_back(marker_point);

        marker_point.x = boxes_on_the_right_side[1].bottom_left.x;
        marker_point.y = boxes_on_the_right_side[1].bottom_left.y;
        marker.points.push_back(marker_point);
        visualize_lines_pub.publish(marker);


}

void Parking::display_bottom_lines()
{
 // cout << "displaying bottom lines\n";
  visualization_msgs::Marker marker;

  marker.header.frame_id = "laser";
  marker.header.stamp = ros::Time::now();
  marker.ns = "bottom_lines_from_parking_node";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 1;
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

          marker_point.x = first_free_place.bottom_left.x;
          marker_point.y = first_free_place.bottom_left.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.bottom_right.x;
          marker_point.y = first_free_place.bottom_right.y;
          marker.points.push_back(marker_point);

          marker_point.x = first_free_place.bottom_right.x;
          marker_point.y = first_free_place.bottom_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.top_right.x;
          marker_point.y = first_free_place.top_right.y;
          marker.points.push_back(marker_point);

          marker_point.x = first_free_place.top_right.x;
          marker_point.y = first_free_place.top_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.top_left.x;
          marker_point.y = first_free_place.top_left.y;
          marker.points.push_back(marker_point);

          marker_point.x = first_free_place.top_left.x;
          marker_point.y = first_free_place.top_left.y;
          marker.points.push_back(marker_point);
          marker_point.x = first_free_place.bottom_left.x;
          marker_point.y = first_free_place.bottom_left.y;
          marker.points.push_back(marker_point);

          visualize_free_place.publish(marker);

}