#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import pid_class

from control_pkg.srv import PidControl, PidControlResponse
import rospy

#Coefficient of PID
#roll
kp1 = 1e4
ki1 = 0 #0.01
kd1 = 0 #0.01

#pitch
kp2 = 1e4
ki2 = 0 #0.01
kd2 = 0 #0.01
pid = pid_class.PID(kp1, ki1, kd1, kp2, ki2, kd2)

motor = [0.0]*8

#       0-3 up/down
#       4-5 thrust
#       6-7 direction
#               
#      u       <-        3
#      0        7        D
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
    ki1 = req.i1
    kd1 = req.d1
    kp2 = req.p2
    ki2 = req.i2
    kd2 = req.d2
    print("Get control msg [%f %f %f %f %f %f]"%(kp1, ki1, kd1, kp2, ki2, kd2))
    pid.setAllCoeff([kp1, ki1, kd1, kp2, ki2, kd2])
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
    if feedback[0] > 0:
        for i in range(3):
            if motor[i] < 5:
                motor[i] += 0.1
        if motor[3] > -5:        
            motor[3] += -0.1

    elif feedback[0] < 0:
        for i in range(3):
            if motor[i] > -5:
                motor[i] += -0.1
        if motor[3] < 5:  
            motor[3] += 0.1

    if feedback[1] > 0:
        for i in range(1, 4):
            if motor[i] < 5:
                motor[i] += 0.1
        if motor[0] > -5:
            motor[0] += -0.1
    elif feedback[1] < 0:
        if motor[0] > -5:
            motor[0] += -0.1
        if motor[1] > -5:
            motor[1] += -0.1
        if motor[2] < 5:
            motor[2] += 0.1
        if motor[3] > -5:
            motor[3] += -0.1

def talker():
    rospy.loginfo(motor)
    pub.publish(Float64MultiArray(data = motor))

if __name__ == '__main__':
    rospy.init_node('pid', anonymous=True)
    pid_control_server()
    pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)
    listener()