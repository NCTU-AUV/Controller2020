#!/usr/bin/env python3

import serial
import glob
from struct import *
# import rospy
# from std_msgs.msg import String
# from std_msgs.msg import Float32
# from sensor_msgs.msg import Imu

port = glob.glob('/dev/ttyACM*')[0]

arduino = serial.Serial(port, 9600, timeout=1)
if not arduino.is_open:
    arduino.open()

a = 1
# for i in range(30):
#     data_raw = arduino.readline()
    
while(arduino.is_open):
    a += 1
    data_raw = arduino.readline()
    # data = tuple()
    try:
        data = unpack('ffffffffffffc', data_raw)
        print(a, data)
    except Exception as e:
        print('length: '+ str(len(data_raw)))
        print(e)
    '''
    readline() until asscii 10
    Serial.write(10)
    '''
    # print(a, data_raw)
    

