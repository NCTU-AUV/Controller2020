#! /usr/bin/env python3

'''
The listener controlling motors with pca9685
I/P: array of 8 motor forces (kgf)
O/P: controll motors

PCA9685 I2C
http://blog.ittraining.com.tw/2015/07/raspberry-pi2-python-i2c-16-pwm-pca9685.html

'''

from smbus2 import SMBus
import time
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32

pca9685_addr = 0x40
bus = SMBus(1)
FORCE = (-4.069476226666667, -4.05057656, -4.023361039999999, -3.96439408, -3.9008912, -3.8691397599999995, -3.82378056, -3.8011009600000003, -3.75120584, -3.7149184799999997, -3.6559515200000003, -3.58791272, -3.5198739199999998, -3.45183512, -3.3974040800000003, -3.3112216, -3.25225464, -3.20235952, -3.1071052, -3.0390664, -2.9937072, -2.9392761600000004, -2.8621655199999996, -2.82134224, -2.77144712, -2.70794424, -2.6580491200000003, -2.5809384800000004, -2.54918704, -2.50836376, -2.45393272, -2.381358, -2.35414248, -2.27703184, -2.2362085599999997, -2.19538528, -2.11827464, -2.08198728, -2.02302032, -1.97766112, -1.94137376, -1.8597271999999998, -1.814368, -1.76447288, -1.7009699999999999, -1.66921856, -1.6102516, -1.5558205600000001, -1.51499728, -1.4877817599999998, -1.4378866399999999, -1.39706336, -1.34716824, -1.30180904, -1.25644984, -1.2020188, -1.16119552, -1.12037224, -1.1022285600000001, -1.05233344, -1.020582, -0.9797587200000001, -0.94347136, -0.90264808, -0.87089664, -0.82100152, -0.7847141599999999, -0.7438908799999999, -0.71667536, -0.680388, -0.6486365599999999, -0.6168851200000001, -0.58059776, -0.53977448, -0.5125589599999999, -0.48080752000000004, -0.44452016, -0.41730464, -0.3855532, -0.35380176, -0.32205032, -0.2948348, -0.25854743999999996, -0.23586784, -0.20865232, -0.1814368, -0.14968536000000002, -0.12700576000000002, -0.10432616, -0.08618248, -0.0680388, -0.04989512, -
         0.03628736, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.040823279999999997, 0.05443104000000001, 0.07711064000000001, 0.09979024, 0.12700576, 0.15422128, 0.18143679999999998, 0.21772416, 0.24947559999999996, 0.29029888, 0.32205032, 0.35833768, 0.39916096, 0.43544832, 0.47173568, 0.5125589599999999, 0.5579181599999999, 0.60327736, 0.63956472, 0.680388, 0.72121128, 0.77564232, 0.8164656, 0.87089664, 0.907184, 0.9480072799999999, 0.9933664799999999, 1.0341897599999998, 1.09769264, 1.14305184, 1.18387512, 1.23830616, 1.27912944, 1.3290245600000001, 1.38799152, 1.4378866399999999, 1.48324584, 1.53767688, 1.587572, 1.6510748800000001, 1.69189816, 1.7599369599999999, 1.8234398400000003, 1.8778708800000001, 1.9323019200000002, 1.9912688800000002, 2.04569992, 2.1228105600000005, 2.1772416000000003, 2.21806488, 2.2770318400000003, 2.381358, 2.43125312, 2.5174356, 2.5764025600000005, 2.6535132000000003, 2.7306238400000002, 2.75783936, 2.8394859200000004, 2.88938104, 2.97556352, 3.05267416, 3.1071052000000003, 3.17060808, 3.21596728, 3.3021497600000003, 3.3747244800000002, 3.42008368, 3.51080208, 3.6060564, 3.6831670400000003, 3.73759808, 3.81924464, 3.88728344, 3.95532224, 4.0596484, 4.14583088, 4.254692960000001, 4.30458808, 4.38169872, 4.5087044800000005, 4.52684816, 4.649318, 4.70828496, 4.7853956, 4.83529072, 4.93054504, 5.0121916, 5.07569448, 5.14373328, 5.17548472, 5.22084392, 5.245035493333333)
CONTROLL = (386.28, 387.0992, 387.9184, 388.7376, 389.5568, 390.376, 391.1952, 392.0144, 392.8336, 393.6528, 394.472, 395.2912, 396.1104, 396.9296, 397.74879999999996, 398.568, 399.3872, 400.20640000000003, 401.0256, 401.84479999999996, 402.664, 403.4832, 404.30240000000003, 405.1216, 405.94079999999997, 406.76, 407.5792, 408.39840000000004, 409.2176, 410.03679999999997, 410.856, 411.6752, 412.49440000000004, 413.3136, 414.1328, 414.952, 415.7712, 416.5904, 417.4096, 418.2288, 419.048, 419.8672, 420.6864, 421.5056, 422.3248, 423.144, 423.9632, 424.7824, 425.6016, 426.4208, 427.24, 428.0592, 428.8784, 429.6976, 430.5168, 431.336, 432.1552, 432.9744, 433.7936, 434.6128, 435.432, 436.2512, 437.0704, 437.8896, 438.7088, 439.528, 440.3472, 441.1664, 441.9856, 442.8048, 443.624, 444.4432, 445.2624, 446.0816, 446.9008, 447.72, 448.5392, 449.3584, 450.1776, 450.9968, 451.816, 452.6352, 453.4544, 454.2736, 455.0928, 455.912, 456.7312, 457.5504, 458.3696, 459.1888, 460.008, 460.8272, 461.6464, 462.4656, 463.2848, 464.104, 464.9232, 465.7424, 466.5616, 467.3808, 468.2, 469.0192, 469.8384, 470.6576, 471.4768, 472.296, 473.1152, 473.9344, 474.7536, 475.5728, 476.392, 477.2112, 478.0304, 478.8496, 479.6688, 480.488, 481.3072, 482.1264, 482.9456, 483.7648, 484.584, 485.4032, 486.2224, 487.0416, 487.8608, 488.68, 489.4992, 490.3184, 491.1376, 491.9568, 492.776, 493.5952,
            494.4144, 495.2336, 496.0528, 496.872, 497.6912, 498.5104, 499.3296, 500.1488, 500.968, 501.7872,
            502.6064, 503.4256, 504.2448, 505.064, 505.8832, 506.7024, 507.5216, 508.3408, 509.16, 509.9792, 510.7984, 511.6176, 512.4368, 513.256, 514.0752, 514.8944, 515.7136, 516.5328, 517.352, 518.1712, 518.9904, 519.8096, 520.6288, 521.448, 522.2672, 523.0864, 523.9056, 524.7248, 525.544, 526.3632, 527.1823999999999, 528.0016, 528.8208, 529.64, 530.4592, 531.2783999999999, 532.0976, 532.9168, 533.736, 534.5552, 535.3743999999999, 536.1936000000001, 537.0128, 537.832, 538.6512, 539.4703999999999, 540.2896000000001, 541.1088, 541.928, 542.7472, 543.5663999999999, 544.3856000000001, 545.2048, 546.024, 546.8432, 547.6623999999999, 548.4816000000001, 549.3008, 550.12)


def set_PWM_ON(addr, ch, value):
    low_byte_val = value & 0x00FF
    high_byte_val = (value & 0x0F00) >> 8
    reg_low_byte = 0x06 + 4 * ch
    bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
    bus.write_i2c_block_data(addr, reg_low_byte+1, [high_byte_val])


def set_PWM_OFF(addr, ch, value):
    low_byte_val = value & 0x00FF
    high_byte_val = (value & 0x0F00) >> 8
    reg_low_byte = 0x08 + 4 * ch
    bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
    bus.write_i2c_block_data(addr, reg_low_byte + 1, [high_byte_val])


def set_motor(motor, val):
    set_PWM_OFF(pca9685_addr, motor, val)


def set_sleep(addr):
    reg_mode1 = 0x00
    sleep_bit = 0x01 << 4
    old_mode1_val = bus.read_byte_data(addr, reg_mode1)
    print(type(old_mode1_val))
    bus.write_i2c_block_data(addr, reg_mode1, [old_mode1_val | sleep_bit])


def unset_sleep(addr):
    reg_mode1 = 0x00
    sleep_bit = 0x01 << 4
    old_mode1_val = bus.read_byte_data(addr, reg_mode1)
    bus.write_i2c_block_data(addr, reg_mode1, [old_mode1_val & ~(sleep_bit)])


def init(freq=71):
    freq_raw = round(25000000 / (4096 * freq)) - 1
    print(freq_raw)

    # Set PWM frequency
    set_sleep(pca9685_addr)
    # freq_raw = 85 (freq approximatly 71)
    # will make steps equaling width with the datasheet of T200
    bus.write_byte_data(pca9685_addr, 0xFE, freq_raw)
    unset_sleep(pca9685_addr)

    for i in range(16):
        set_PWM_ON(pca9685_addr, i, 0)
    # for i in range(16):
    #     set_motor(i, 330)    # send start signal
    set_motor(1, 468)
    print('freq signal: ' + str(bus.read_byte_data(pca9685_addr, 0xFE)))


class MotorController:
    def __init__(self):
        listener = rospy.Subscriber(
            'Motors_Force', Float32MultiArray, self.callback)

    def callback(self, data):
        print(type(data.data))
        cmd = np.interp(data.data, FORCE, CONTROLL)
        cmd = [self.fuse(val) for val in cmd]
        #cmd = self.fuse(cmd)
        print('final', cmd[1])
        set_motor(1, cmd[1])
        rospy.loginfo(data)
    
    def fuse(self, val):
        if 461<=val<=474:
            return 468
        if val<400:
            return 400
        if val>530:
            return 530
        return int(val) 

def shutdown():
        for i in range(16):
            set_motor(i, 0)
        print('\nstop')

def main(args):
    init()
    MotorController()
    rospy.init_node('Motor_Controller', anonymous=True)
    rospy.on_shutdown(shutdown)
 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('bye')

if __name__ == '__main__':
    main(sys.argv)
