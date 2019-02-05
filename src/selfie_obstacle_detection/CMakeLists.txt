cmake_minimum_required(VERSION 2.8.3)
project(selfie_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS nodelet roscpp sensor_msgs
  LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS})

# Nodelet library
add_library(${PROJECT_NAME} src/nodelets/remove_background.cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
