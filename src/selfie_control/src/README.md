##Info:
change lane node inside selfie_control package
Parameters: ~lane_width (float), ~error_margin (float)

## Subscribed and published topics
Subscribed topics:
position_offset (std_msgs/Float64)

Published topics:
target_offset (std_msgs/Float64)
left_turn_indicator (std_msgs/Bool)
right_turn_indicator (std_msgs/Bool)

## Running

### Demo of changing lane action

1) Run demo offset publisher that will generate offset_position based of the target position from module
rosrun selfie_control demo_position

2) Run main module that will start server for change_lane action. Give lane width and error margin as argument
rosrun selfie_control change_lane _lane_width:=1 _error_margin:=0.1

3) Start action of changing when you want to do so. Give an argument that tells which lane do you want.
rosrun selfie_control demo_change_lane _lane:=l (or _lane:=r)
