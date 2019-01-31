#include "../../selfie_parking/include/selfie_parking/parking.h"

using namespace std;

class crossing_handler {
public:
  void load_obstacles(const selfie_msgs::PolygonArray &msg);

private:
  vector<int> top_road_zone;
  vector<int> bottom_road_zone;
};


void crossing_handler::load_obstacles(const selfie_msgs::PolygonArray &msg)
{
  for(auto box_it = msg.polygons.begin();  box_it != msg.polygons.end();  ++box_it)
  {
    geometry_msgs::Polygon polygon = *box_it;
    Box temp_box(polygon);
     
  }//box_nr for
}//obstacle_callback
