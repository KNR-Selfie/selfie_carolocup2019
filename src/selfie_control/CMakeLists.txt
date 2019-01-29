cmake_minimum_required(VERSION 2.8.3)
project(selfie_control)

find_package(catkin REQUIRED COMPONENTS
  genmsg actionlib_msgs
  dynamic_reconfigure
  ackermann_msgs
  actionlib_msgs
  actionlib
  genmsg
  rospy
  roscpp
  std_msgs
)

add_action_files(
  DIRECTORY action
  FILES ChangeLane.action
)

 generate_messages(
   DEPENDENCIES actionlib_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/Speed.cfg
  cfg/HeadingCoeff.cfg
)

catkin_package(CATKIN_DEPENDS actionlib_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(change_lane src/change_lane.cpp src/ChangeLane.cpp)
target_link_libraries(change_lane ${catkin_LIBRARIES})
add_dependencies(change_lane ${selfie_control_EXPORTED_TARGETS})

add_executable(demo_change_lane src/demo_change_lane.cpp)
target_link_libraries(demo_change_lane ${catkin_LIBRARIES})
add_dependencies(demo_change_lane ${selfie_control_EXPORTED_TARGETS})

add_executable(demo_position src/demo_position.cpp)
target_link_libraries(demo_position ${catkin_LIBRARIES})
add_dependencies(demo_position ${selfie_control_EXPORTED_TARGETS})
