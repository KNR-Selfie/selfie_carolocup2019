#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

#include <actionlib/server/simple_action_server.h>
#include <selfie_msgs/parkingAction.h>


#include <selfie_park/parkAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/console.h>

#include "shapes.h"


