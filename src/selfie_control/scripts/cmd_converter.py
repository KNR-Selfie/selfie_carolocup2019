#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

UPDATE_RATE = 50

steering_angle = 0.0
target_speed = 0.0

def steering_angle_callback(msg):
    global steering_angle
    steering_angle = msg.data

def target_speed_callback(msg):
    global target_speed
    target_speed = msg.data

if __name__ == '__main__':
    rospy.init_node('cmd_coverter')

    cmd_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)

    rospy.Subscriber('steering_angle', Float64, steering_angle_callback)
    rospy.Subscriber('target_speed', Float64, target_speed_callback)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown():
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = target_speed
        cmd_pub.publish(cmd)
        rate.sleep()
