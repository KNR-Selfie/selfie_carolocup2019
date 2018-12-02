#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from dynamic_reconfigure.client import Client

UPDATE_RATE = 50

def speed_callback(msg):
    if msg.data < 1e-9: return

    kp = nominal_kp
    ki = min(nominal_ki / msg.data * nominal_speed, max_ki)
    kd = min(nominal_kd / msg.data * nominal_speed, max_kd)

    pid_client.update_configuration({
        'Kp': kp,
        'Ki': ki,
        'Kd': kd
    })

    rospy.loginfo(kd)

if __name__ == '__main__':
    rospy.init_node('selfie_pid_tuner')

    pid_node = rospy.get_param('~pid_node')

    global nominal_kp, nominal_ki, nominal_kd
    nominal_kp = rospy.get_param('~nominal_kp')
    nominal_ki = rospy.get_param('~nominal_ki')
    nominal_kd = rospy.get_param('~nominal_kd')

    global max_ki, max_kd
    max_ki = rospy.get_param('~max_ki', 50.0)
    max_kd = rospy.get_param('~max_kd', 50.0)

    global nominal_speed
    nominal_speed = rospy.get_param('~nominal_speed', 1.0)

    global pid_client
    pid_client = Client(pid_node)

    speed_sub = rospy.Subscriber('/speed', Float32, speed_callback, queue_size=1)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown(): rate.sleep()
