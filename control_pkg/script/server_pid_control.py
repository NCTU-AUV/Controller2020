#!/usr/bin/env python3

from control_pkg.srv import PidControl, PidControlResponse
import rospy

def handle_pid_control(req):
    print("Get control msg [%f %f %f %f %f %f]"%(req.p1, req.i1, req.d1, req.p2, req.i2, req.d2))
    return PidControlResponse(True)

def pid_control_server():
    rospy.init_node('pid_control_server')
    s = rospy.Service('pid_control', PidControl, handle_pid_control)
    print("Ready to get control msg.")
    rospy.spin()

if __name__ == "__main__":
    pid_control_server()