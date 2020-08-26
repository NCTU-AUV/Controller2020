#! /usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray

def listener():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('lefttop_point', Float32MultiArray, callback)
    rospy.spin()

def callback(data):
    print(data.data)

if __name__ == '__main__':
    listener()
