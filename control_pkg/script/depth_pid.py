#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class
import math

from control_pkg.srv import PidControl, PidControlResponse
import rospy

#Coefficient of PID
#depth_left
kp_l = 1
order_p_l = 4
ki_l = 0 #0.01
order_i_l = 0
kd_l = 0 #0.01
order_d_l = 0
K_l = 1

#depth_right
kp_r = 1
order_p_r = 4
ki_r = 0 #0.01
order_i_r = 0
kd_r = 0 #0.01
order_d_r = 0
K_r = 1

depth_pid_l = pid_class.PID(math.pow(kp_l, order_p_l), math.pow(ki_l, order_i_l), math.pow(kd_l, order_d_l), K_l)
depth_pid_r = pid_class.PID(math.pow(kp_r, order_p_r), math.pow(ki_r, order_i_r), math.pow(kd_r, order_d_r), K_r)

#motor_limit
limit = 100

#set depth
depth = 40

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
#    f  |depth0   depth1|  f
#       |               |
#       |               |
#       -----------------
#      1        6        2
#      u       ->        u
        

def handle_pid_control(req):
    #set coefficient of pid for left
    kp_l = req.p_l
    order_p_l = req.po_l
    ki_l = req.i_l
    order_i_l = req.io_l
    kd_l = req.dl
    order_d_l = req.do_l

    #set coefficient of pid for right
    kp_r = req.p_r
    order_p_r = req.po_r
    ki_r = req.i_r
    order_i_r = req.io_r
    kd_r = req.d_r
    order_d_r = req.do_r

    print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f]"%(kp_l, order_p_l, ki_l, order_i_l, kd_l, order_d_l,
    kp_r, order_p_r, ki_r, order_i_r, kd_r, order_d_r))

    depth_pid_l.setAllCoeff( [math.pow(kp_l, order_p_l), math.pow(ki_l, order_i_l), math.pow(kd_l, order_d_l)] )
    depth_pid_r.setAllCoeff( [math.pow(kp_r, order_p_r), math.pow(ki_r, order_i_r), math.pow(kd_r, order_d_r)] ) 

    return PidControlResponse(True)

def pid_control_server():
    # rospy.init_node('pid_control_server')
    s = rospy.Service('depth_pid_control', PidControl, handle_pid_control)
    print("Ready to get control msg.")
    #rospy.spin()

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
    feedback = [depth_pid_l.update_Depth(depth - data.data[0]), depth_pid_r.update_Depth(depth - data.data[1])]
    #print(feedback)
    update_motor(feedback)

    talker()

def listener():
    rospy.Subscriber("IMU/Depth", Float64MultiArray, callback)
    rospy.spin()

def update_motor(force):
    global motor
    global limit

    value_depth_l = force[0]
    value_depth_r = force[1]

    value = [value_depth_l, value_depth_l, value_depth_r, value_depth_r]

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
    rospy.init_node('depth_pid', anonymous=True)
    pid_control_server()
    pub = rospy.Publisher('Motors_Force_Depth', Float64MultiArray, queue_size=10)
    listener()
