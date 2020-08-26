#! /usr/bin/python3
import random
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub_p = rospy.Publisher('Motors_Force', Float32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    
    while not rospy.is_shutdown():
        array = []
        for i in range(8):
            array.append(random.random()*100)
        array[1] = float(input('val: '))
        
        left_top = Float32MultiArray(data=array)
            #也可以采用下面的形式赋值
            #left_top = Float32MultiArray()
            #left_top.data = [521,1314]
            #left_top.label = 'love'
        rospy.loginfo(left_top)
        pub_p.publish(left_top)
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print('bye')
