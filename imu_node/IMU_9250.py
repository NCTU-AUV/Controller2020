#! /usr/bin/env python3

import serial
import rospy
import threading
from std_msgs.msg import Float32MultiArray
import time
import random

class IMUAttitude:
    def __init__(self):
        self.imu_port = '/dev/ttyACM0'       
        self.arduino = serial.Serial(self.imu_port, 9600, timeout=1)
        if not self.arduino.is_open:
            self.arduino.open()

        rospy.on_shutdown(self.shutdown)
        rospy.init_node('IMU_9250', anonymous=True)
        self.pub = rospy.Publisher('IMU/Attitude', Float32MultiArray, queue_size=10)

        self.imu_t = threading.Thread(target=self.get_data, daemon=True)
        self.imu_t.start()

        rospy.spin()
    
    def get_data(self):
        # while True:
        while self.arduino.is_open:
            # attitude = [random.random()*10 for _ in range(2)]
            try:
                raw_data = self.arduino.readline()
                attitude = [float(val) for val in raw_data.split()]
                # print(raw_data)
                # time.sleep(0.5)
            except Exception:
                print('oops')
            
            print(attitude)
            self.pub.publish(Float32MultiArray(data=attitude))
            rospy.loginfo(attitude)

    def shutdown(self):
        print('\nbye')

def main():
    IMUAttitude()

if __name__ == '__main__':
    main()
