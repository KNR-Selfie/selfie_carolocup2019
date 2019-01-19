# STM32 Bridge

`selfie_stm32_bridge` package provides a node with the same name responsible for communication with on-board STM32 microcontroller that handles IMU, encoders and vehicle control. The node communicates with ROS using standarized message types, as described below.

## `selfie_stm32_bridge`

### Subscribed topics

`drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
Steering commands to be applied.

`left_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
Status of left turn indicator.

`right_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
Status of right turn indicator.

### Published topics

`imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
Data stream from IMU.

`speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Linear velocity magnitude at the center of rear axle, as calculated from encoder data (in m/s).

`start_button` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
Status of the start button (0 - no start, 1 - start).