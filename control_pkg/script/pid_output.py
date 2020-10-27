#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import pid_class

from control_pkg.srv import PidControl, PidControlResponse
import rospy

kp1 = 0.01
ki1 = 0.01
kd1 = 0.01
kp2 = 0.01
ki2 = 0.01
kd2 = 0.01
pid_roll = pid_class.PID(kp1, ki1, kd1)
pid_pitch = pid_class.PID(kp2, ki2, kd2)

motor = [0.0]*8  # 1-4 A-D

#       1-4 up/down
#       A-D direction
#               
#     <-        u       <-
#      A        1        D
#       -----------------
#       |               |
#    u  |               |  u
#    2  |               |  4
#       |               |
#       |               |
#       -----------------
#      B        3        C
#     <-        u       <-
        

def handle_pid_control(req):
    kp1 = req.p1
    ki1 = req.i1
    kd1 = req.d1
    kp2 = req.p2
    ki2 = req.i2
    kd2 = req.d2
    print("Get control msg [%f %f %f %f %f %f]"%(kp1, ki1, kd1, kp2, ki2, kd2))
    return PidControlResponse(True)

def pid_control_server():
    # rospy.init_node('pid_control_server')
    s = rospy.Service('pid_control', PidControl, handle_pid_control)
    print("Ready to get control msg.")
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
    feedback_roll = pid_roll.update(data.data[0])
    feedback_pitch = pid_pitch.update(data.data[2])

    print(feedback_roll)
    print(feedback_pitch)
    
    update_motor_roll(feedback_roll)
    update_motor_pitch(feedback_pitch)
    talker()

def listener():
    rospy.Subscriber("IMU/Attitude", Float64MultiArray, callback)
    rospy.spin()

def update_motor_roll(roll_value):
    global motor
    if roll_value > 0:
        for i in range(3):
            if motor[i] < 5:
                motor[i] += 1
        if motor[3] > -5:        
            motor[3] += -1

    elif roll_value < 0:
        for i in range(3):
            if motor[i] > -5:
                motor[i] += -1
        if motor[3] < 5:  
            motor[3] += 1

def update_motor_pitch(pitch_value):
    global motor
    if pitch_value > 0:
        for i in range(1, 4):
            if motor[i] < 5:
                motor[i] += 1
        if motor[0] > -5:
            motor[0] += -1
    elif pitch_value < 0:
        if motor[0] > -5:
            motor[0] += -1
        if motor[1] > -5:
            motor[1] += -1
        if motor[2] < 5:
            motor[2] += 1
        if motor[3] > -5:
            motor[3] += -1

def talker():
    rospy.loginfo(Float64MultiArray(data = motor))
    pub.publish(Float64MultiArray(data = motor))

if __name__ == '__main__':
    rospy.init_node('pid', anonymous=True)
    pid_control_server()
    pub = rospy.Publisher('motor_data', Float64MultiArray, queue_size=10)
    listener()