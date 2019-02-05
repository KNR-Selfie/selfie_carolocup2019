# Selfie logic

`selfie_logic` package provides a node that takes care of changing lane procedure

## `change_lane_logic`

### Subscribed topics

`vision/road_markings` (selfie_msgs/msgs/oadMarkings.msg)
Road markings from selfie_detection

`obstacles` (selfie_msgs/msgs/PolygonArray.msg)
Polygon Array contains vertexes of every rectangular obstacle in local XY

`distance` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Distance from encoder from STM32 bridge

### Published topics

`change_lane_status` ([std_msgs/UInt16](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
Status of changing lane:
0 - on right and nothing on right lane
1 - changing lane finished
2 - on left lane and obstacle on right
3 - waiting for distance to pass
4 - change lane back finished


