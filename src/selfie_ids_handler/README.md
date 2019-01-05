# IDS CAMERA HANDLER

`selfie_ids_handler` package provides a node to handle image processing from ids camera. It gets image from camera and convert it to /image_ids ROS message. It uses opencv_bridge and image_transport packages to sending frame to message, but ids is getting and giving image not using ROS package.

## Usage
```
. devel/setup.bash
rosrun selfie_ids_handler selfie_ids_handler
```
## Library needed
In this node some external libraries are used:
- OpenCV 3.3.1
- EYE_LIBRARY (installed inside /bin)

## Topics
### Published topics
- `/image_ids` ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Imgage.html))
  - Message with camera image
