#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class
import math

from control_pkg.srv import PidControl, PidControlResponse
import rospy

#Coefficient of PID
#roll
kp1 = 1
order_p1 = 4
ki1 = 0 #0.01
order_i1 = 0
kd1 = 0 #0.01
order_d1 = 0

#pitch
kp2 = 1
order_p2 = 4
ki2 = 0 #0.01
order_i2 = 0
kd2 = 0 #0.01
order_d2 = 0

pid = pid_class.PID(math.pow(kp1, order_p1), math.pow(ki1, order_i1), math.pow(kd1, order_d1), math.pow(kp2, order_p2)
                    , math.pow(ki2, order_i2), math.pow(kd2, order_d2))
limit = 100
K_roll = 1
K_pitch = 1

motor = [0.0]*8

#       0-3 up/down
#       4-5 thrust
#       6-7 direction
#               
#      u       <-        u
#      0        7        3
#       -----------------
#       |               |
#    4  |               |  5
#    f  |               |  f
#       |               |
#       |               |
#       -----------------
#      1        6        2
#      u       ->        u
        

def handle_pid_control(req):
    kp1 = req.p1
    order_p1 = req.po1
    ki1 = req.i1
    order_i1 = req.io1
    kd1 = req.d1
    order_d1 = req.do1
    kp2 = req.p2
    order_p2 = req.po2
    ki2 = req.i2
    order_i2 = req.io2
    kd2 = req.d2
    order_d2 = req.do2

    print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f]"%(kp1, order_p1, ki1, order_i1, kd1, order_d1
        , kp2, order_p2, ki2, order_i2, kd2, order_d2))

    pid.setAllCoeff( [math.pow(kp1, order_p1), math.pow(ki1, order_i1), math.pow(kd1, order_d1), math.pow(kp2, order_p2)
                    , math.pow(ki2, order_i2), math.pow(kd2, order_d2)] )

    return PidControlResponse(True)

def pid_control_server():
    # rospy.init_node('pid_control_server')
    s = rospy.Service('pid_control', PidControl, handle_pid_control)
    print("Ready to get control msg.")
    #rospy.spin()

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
    
    #D term = w
    pid.setDTerm_roll(data.data[0])
    pid.setDTerm_pitch(data.data[2])
    
    feedback = pid.update_RollandPitch(data.data[0], data.data[2])
    #print(feedback)
    update_motor(feedback)

    talker()

def listener():
    rospy.Subscriber("IMU/Attitude", Float64MultiArray, callback)
    rospy.spin()

def update_motor(feedback):
    global motor
    global limit
    global K_roll
    global K_pitch
    value_roll = feedback[0] * K_roll
    value_pitch = feedback[1] * K_pitch

    value = [-value_roll - value_pitch, -value_roll + value_pitch, value_roll + value_pitch, value_roll - value_pitch]

    for i in range(4):
        if motor[i] + value[i] < -limit:
            motor[i] = -limit
        elif motor[i] + value[i] > limit:
            motor[i] = limit
        else:
            motor[i] += value[i]

def talker():
    rospy.loginfo(motor)
    pub.publish(Float64MultiArray(data = motor))

if __name__ == '__main__':
    rospy.init_node('pid', anonymous=True)
    pid_control_server()
    pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)
    listener()
