#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class
import math

from control_pkg.srv import PidControl, PidControlResponse
import rospy

#Coefficient of PID
#roll
kp_r = 1
order_p_r = -2
ki_r = 0 #0.01
order_i_r = 0
kd_r = 0.01 #0.01
order_d_r = 0
K_roll = 100

#pitch
kp_p = 1
order_p_p = -2
ki_p = 0 #0.01
order_i_p = 0
kd_p = 0.01 #0.01
order_d_p = 0
K_pitch = 100

roll_pid = pid_class.PID(kp_r, ki_r, kd_r, K_roll)
pitch_pid = pid_class.PID(kp_p, ki_p, kd_p, K_pitch)

upper_bound = 10000
lower_bound = 50

motor = [0.0]*4

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
    kp_r = req.p_r
    order_p_r = req.po_r
    ki_r = req.i_r
    order_i_r = req.io_r
    kd_r = req.d_r
    order_d_r = req.do_r
    kp_p = req.p2_p
    order_p_p = req.po_p
    ki_p = req.i_p
    order_i_p = req.io_p
    kd_p = req.d_p
    order_d_p = req.do_p

    print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f]"%(kp_r, order_p_r, ki_r, order_i_r, kd_r, order_d_r,
    kp_p, order_p_p, ki_p, order_i_p, kd_p, order_d_p))

    roll_pid.setAllCoeff( [math.pow(kp_r, order_p_r), math.pow(ki_r, order_i_r), math.pow(kd_r, order_d_r)] )
    pitch_pid.setAllCoeff( [math.pow(kp_p, order_p_p), math.pow(ki_p, order_i_p), math.pow(kd_p, order_d_p)] )

    return PidControlResponse(True)

def pid_control_server():
    # rospy.init_node('pid_control_server')
    s = rospy.Service('attitude_pid_control', PidControl, handle_pid_control)
    #print("Ready to get control msg.")
    #rospy.spin()

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
    
    #D term = w
    roll_pid.setDTerm(data.data[0])
    pitch_pid.setDTerm(data.data[1])

    feedback = [roll_pid.update_RollorPitch(data.data[0]), pitch_pid.update_RollorPitch(data.data[1])]
    #print(feedback)
    update_motor(feedback)

    talker()

def listener():
    rospy.Subscriber("IMU/Attitude", Float64MultiArray, callback)
    rospy.spin()

def update_motor(force):
    global motor
    global limit
    value_roll = force[0]
    value_pitch = force[1]

    value = [-value_roll - value_pitch, -value_roll + value_pitch, value_roll + value_pitch, value_roll - value_pitch]

    for i in range(4):
        if value[i] < -upper_bound:
            motor[i] = -upper_bound
        elif value[i] > upper_bound:
            motor[i] = upper_bound
        elif value[i] > -lower_bound and value[i] < lower_bound:
            motor[i] = 0
        else:
            motor[i] = -value[i]

def talker():
    rospy.loginfo(motor)
    pub.publish(Float64MultiArray(data = motor))

if __name__ == '__main__':
    rospy.init_node('attitude_pid', anonymous=True)
    pid_control_server()
    roll_pid.setSetPoint(-5)
    pitch_pid.setSetPoint(20)
    pub = rospy.Publisher('Motors_Force_Attitude', Float64MultiArray, queue_size=10)
    listener()
