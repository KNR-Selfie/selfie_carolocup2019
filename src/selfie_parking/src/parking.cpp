#include <iostream>
#include <vector>


#include "../../selfie_parking/include/selfie_parking/parking.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parking");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Parking parking_node(nh, pnh);
  parking_node.init();
  while (ros::ok())
  {
      ros::spin();
  }

  return 0;
}
