#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('forward_force_talker', anonymous=True)
pub = rospy.Publisher('Motors_Force_Forward', Float64MultiArray, queue_size=10)

while not rospy.is_shutdown():
    force = float(input('Forward Force: '))
    pub.publish(Float64MultiArray(data=[force]*2))

