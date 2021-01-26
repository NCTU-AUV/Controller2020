#! /usr/bin/env python3

import serial
import glob
import rospy
import threading
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import time
import random
from tf.transformations import euler_from_quaternion

class IMUAttitude:
    def __init__(self):
        self.arduino_port = glob.glob('/dev/ttyACM*')[0]       
        
        self.arduino = serial.Serial(self.arduino_port, 115200, timeout=1)
        if not self.arduino.is_open:
            self.arduino.open()

        rospy.on_shutdown(self.shutdown)
        rospy.init_node('IMU_9250', anonymous=True)

        #For PID
        self.arr_pub = rospy.Publisher('IMU/Attitude', Float64MultiArray, queue_size=10) #[roll, pitch, yaw]

        #For visualize
        self.imu_visualize_data = Imu()
        self.imu_visualize_pub = rospy.Publisher('IMU/Visualize', Imu, queue_size=100)

        self.imu_t = threading.Thread(target=self.get_data, daemon=True)
        self.imu_t.start()

        rospy.spin()
    
    def get_data(self):
        while self.arduino.is_open:
            try:
                raw_data = self.arduino.readline()
                #print('arduino raw data: ')
                #print(raw_data)
                data = unpack('ffffffffffc', raw_data) 
                attitude = euler_from_quaternion(float(data[0:4]))

            except Exception as e:
                print('oops')
                print(e)
                # time.sleep(0.5)
                #print(attitude)
            
            #roll, pitch, yaw
            rospy.loginfo(attitude)
            self.arr_pub.publish(Float64MultiArray(data=attitude))
            
            #imu.orientation
            self.imu_visualize_data.orientation = [float(data[0:4])]
            self.imu_visualize_data.orientation_covariance[0] = -1

            #imu.angular_velocity
            self.imu_visualize_data.angular_velocity = [float(data[4:7])]
            self.imu_visualize_data.angular_velocity_covariance[0] = -1
            
            #imu.linear_acceleration
            self.imu_visualize_data.linear_acceleration = [float(data[7:10])]
            self.imu_visualize_data.linear_acceleration_covariance[0] = -1

            self.imu_visualize_pub.publish(self.imu_visualize_data)

            rospy.sleep(0.01) #100 hz


    def shutdown(self):
        self.arduino.close()
        print('\nbye')

def main():
    IMUAttitude()

if __name__ == '__main__':
    main()
