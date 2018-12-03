# Path Planner

## Usage
```
. devel/setup.bash
rosrun path_planner path_planner_node
```
## Topics
### Subscribed topics
- `road_markings` ([selfie_msgs/RoadMarkings](https://github.com/KNR-Selfie/selfie_carolocup2019/blob/develop/src/selfie_msgs/msg/RoadMarkings.msg))

### Published topics
- `position_offset` ([std_msgs/Float64](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html))
- `heading_offset` ([std_msgs/Float64](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html))

extra `#define PREVIEW_MODE 1` 
- `cloud`
- `path`


