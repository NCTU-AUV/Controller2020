#!/usr/bin/env python3

# license removed for brevity
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

def talker(pub):
    rate = rospy.Rate(10) # 10hz
    
    force = [0.4 for _ in range(8)]
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # force = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        
        rospy.loginfo(Float64MultiArray(data=force))
        pub.publish(Float64MultiArray(data=force))
        rate.sleep()

def stop_motor(pub):
    force = [0 for _ in range(8)]
    for i in range(3):
        pub.publish(Float64MultiArray(data=force))
        time.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)
    try:
        talker(pub)
    except rospy.ROSInterruptException:
        stop_motor(pub)