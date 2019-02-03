#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int16

SPEED = 0.4
UPDATE_RATE = 50

def state_callback(msg):
    if(msg.data < 2):
        #rospy.loginfo_throttle(1, "searching for a place")
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = SPEED
        drive_msg.drive.steering_angle_velocity = 5
    elif(msg.data == 2):
        drive_msg.drive.speed = 0
        drive_msg.drive.steering_angle = 0
        #rospy.loginfo_throttle(1, 'planning')
    else:
        rospy.loginfo_throttle(1, 'parking')
        rospy.signal_shutdown('parking_server took control over')



if __name__ == '__main__':
    rospy.init_node('parking_drive_manager')
    global drive_msg
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = 0
    drive_msg.drive.speed = SPEED
    drive_msg.drive.steering_angle_velocity = 5

    global drive_pub
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    parking_state_sub = rospy.Subscriber('parking_state', Int16, state_callback, queue_size=10)
    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown():
        drive_pub.publish(drive_msg)
        rate.sleep()
