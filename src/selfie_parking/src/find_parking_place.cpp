#include "../../selfie_parking/include/selfie_parking/parking.h"


Parking::Parking(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
                nh_(nh),
                pnh_(pnh),
                search_server_(nh_, "search",  false)
{
  search_server_.registerGoalCallback(boost::bind(&Parking::manager_init, this));
  search_server_.registerPreemptCallback(boost::bind(&Parking::preemptCB, this));
  //search_server_.regi
  search_server_.start();
  pnh_.param<float>("point_min_x", point_min_x, 0);
  pnh_.param<float>("point_max_x", point_max_x, 2);
  pnh_.param<float>("point_min_y", point_min_y, -1);
  pnh_.param<float>("point_max_y", point_max_y, 0.2);
  pnh_.param<float>("distance_to_stop", distance_to_stop, 0.2);
  pnh_.param<int>("scans_to_ignore_when_stopped", scans_ignored, 2);
  pnh_.param<int>("scans_taken", scans_taken, 5);
  pnh_.param<bool>("debug_mode", debug_mode, false);
  // 1->to_rviz  2-> funcion execution time 3->text_only
  pnh_.param<int>("visualization_type", visualization_type, 3);
}

Parking::~Parking(){}

void Parking::preemptCB()
{
  ROS_FATAL("preemted");
  search_server_.setPreempted();
  if(!search_server_.isActive())
  {
    reset();
    state = searching;
  }
}

bool Parking::init()
{
  this->obstacles_sub = nh_.subscribe("/obstacles", 10, &Parking::manager, this);
  this->visualize_lines_pub = nh_.advertise<visualization_msgs::Marker>( "/visualization_lines", 1 );
  this->visualize_free_place  = nh_.advertise<visualization_msgs::Marker>( "/free_place", 1 );
  this->point_pub = nh_.advertise<visualization_msgs::Marker>("/box_points", 5);
//  this->parking_state_pub = nh_.advertise<std_msgs::Int16>("parking_state", 200);
  goal_set = false;
  distance_to_stop +=0.2;
 // min_spot_lenght = 0.6;
//  ROS_INFO("viz type: %d", visualization_type);
}

void Parking::manager_init()
{
  ROS_WARN("goal set!!!");
  min_spot_lenght = (*search_server_.acceptNewGoal()).min_spot_lenght;
//  goal_set = true;
}

void Parking::manager(const selfie_msgs::PolygonArray &msg)
{ 
  // to save cpu time just do nothing when new scan comes
 if(!search_server_.isActive())
  {
    ROS_INFO_THROTTLE(1,"server is not active!");
    return;
  }
  
  ROS_INFO("ok");
  if(state == planning_failed)
  {
    /*
    std_msgs::Int16 state_msg;
    state_msg.data = state;
    parking_state_pub.publish(state_msg);
*/
    ROS_WARN("recovery after failed planning");
    reset();
    ros::Duration(1).sleep();
    state = searching;
    return;
  }
  reset();
  /*      //{old} parking state publishing
  std_msgs::Int16 state_msg;
  state_msg.data = state;
  parking_state_pub.publish(state_msg);
  */
  switch (state)
  {
    case searching:
    {
      ROS_INFO("search");
      planning_error_counter = 0;
      search(msg);
      bool place_found = find_free_place();
      if(place_found && debug_mode)
        ROS_INFO("place in range of lidar!!");
      float dist = 9999;
      if(place_found)
      {
        first_free_place.visualize(point_pub);
        dist = get_dist_from_first_free_place();
        ROS_INFO("distance_to_first_place: %f", dist);
      }
      feedback_msg.distance_to_first_place = dist;
      if(place_found && dist <= distance_to_stop)
      {
        feedback_msg.info = "place found, waiting to get exact measurements";
        state = planning;
        display_free_place();
        planning_scan_counter = 0;
        ROS_WARN( "state switched to planning");
      }
      else
        feedback_msg.info = "place too far";
      search_server_.publishFeedback(feedback_msg);
      break;
    }// end searching

    case planning:
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
          print_planning_stats();
          ++planning_scan_counter;
          ROS_INFO("scan ok");
        }
      }// place ok, waiting for the next scan
      else if(planning_scan_counter >= scans_taken)
      {
        ROS_WARN("getting exact dimensions\n");
        // ros::Duration(0.5).sleep();
        // takes the vector of laser scans and aproximates real parking place dimensions
        get_exact_measurements();
        planning_scan_counter = 0; 

        for_planning.clear();
        state = parking;
        first_free_place.print_box_dimensions();
        send_goal();
        ROS_WARN("state swiched to parking");  
      }//end place was ok, send result

      if(planning_error_counter >= scans_taken)
      {
        feedback_msg.info = "error occured, place is too small, move forward!!";
        ROS_FATAL("place appeared to be to small for Selfie");
        feedback_msg.distance_to_first_place = get_dist_from_first_free_place();
        search_server_.publishFeedback(feedback_msg);

        planning_error_counter = 0;
        planning_scan_counter = 0;
        state = planning_failed;
        first_free_place.reset();
        reset();
      }
    }//end planning case
  }//end switch

}

void Parking::print_planning_stats()
{
  vector<float> x_s;
  vector<float> y_s;
  float sum_x = 0;
  float sum_y = 0;
  for(auto it = for_planning.begin();  it != for_planning.end();  ++it)
  {
    sum_x += it->bottom_left.x;
    sum_y += it->top_left.x;
  }
  float avg_x = sum_x/for_planning.size();
  float avg_y = sum_y/for_planning.size();

  ROS_INFO("avg bottom left: ( %f ,  %f )", avg_x, avg_y);
  return;
}

bool Parking::find_free_place()
{
  if(boxes_on_the_right_side.size() < 2)
  {
    ROS_WARN("REJECTED");
    return false;

  }
  float min_space = min_spot_lenght;
  //ROS_INFO("min space: %f", min_space);
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();
  vector<Box>::const_iterator end_iter = boxes_on_the_right_side.cend();
  //for(;  iter != end_iter;  ++iter)
 // {
    double dist = (*iter).top_left.get_distance((*(iter+1)).bottom_left);
   // ROS_INFO("dist: %f", dist);
    if(dist > min_space)
    {
      Box tmp_box((*iter).top_left, (*iter).top_right, (*(iter+1)).bottom_left, (*(iter+1)).bottom_right);
      first_free_place = tmp_box;

      return true;
    }
    if(state == planning)
    {
      if(visualization_type >= 3)
        ROS_WARN("error, place too small!!!!");
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
  /*
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
*/

    geometry_msgs::Point32 p;
    p.x = first_free_place.bottom_left.x;
    p.y = first_free_place.bottom_left.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.bottom_right.x;
    p.y = first_free_place.bottom_right.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.top_right.x;
    p.y = first_free_place.top_right.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.top_left.x;
    p.y = first_free_place.top_left.y;
    result.parking_spot.points.push_back(p);

    reset();
    state = searching;

    search_server_.setSucceeded(result);
}



void Parking::search(const selfie_msgs::PolygonArray &msg)
{


//
//int box_nr = msg.polygons.size()-1;  box_nr >= 0;  --box_nr
//auto box_nr=0;  box_nr != msg.polygons.size();  ++box_nr
//
  for(int box_nr = msg.polygons.size()-1;  box_nr >= 0;  --box_nr)
  {
    float min_x = point_min_x;
    float max_x = point_max_x;
    float min_y = point_min_y;
    float max_y = point_max_y;
   //  cout << "box_nr:" << box_nr << endl;

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
     ROS_INFO("box_created");
    }
      //now we reset, but later

  }//box_nr for
}//obstacle_callback

double inline Parking::get_dist_from_first_free_place()
{ 
  return first_free_place.bottom_left.x;
}

void Parking::reset()
{
  boxes_on_the_right_side.clear();
  first_free_place.reset();
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
