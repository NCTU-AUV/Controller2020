#! /usr/bin/env python3

'''
The listener controlling motors with pca9685
I/P: array of 8 motor forces (kgf)
O/P: controll motors

PCA9685 I2C
http://blog.ittraining.com.tw/2015/07/raspberry-pi2-python-i2c-16-pwm-pca9685.html

Datasheet of T200
https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
'''

from smbus2 import SMBus
import time
import sys
import numpy as np

class TestPCA9685:
    pca9685_addr = 0x40
    bus = SMBus(1)    

    def __init__(self) -> None:
        self.f = int(input('input freq: '))
        self.motor_init(self.f)
   
    def motor_init(self, freq=224):
            # Set PWM frequency
            self.freq_raw = round(25e6 / (4096 * freq)) - 1
            print(f'freq: {freq}')
            print(f'freq_raw: {self.freq_raw}')
            '''
            freq_raw = 85 (freq approximatly 71)
            will make steps equaling width with the datasheet of T200
            If want to change the frequency, need to set pca9685 to sleep mode
            '''
            self.set_sleep(self.pca9685_addr)
            self.bus.write_byte_data(self.pca9685_addr, 0xFE, self.freq_raw)
            self.unset_sleep(self.pca9685_addr)

            
            for i in range(16):
                self.set_PWM_ON(self.pca9685_addr, i, 0)
            for i in range(16):
                self.set_motor(i, 1500)    # send start signal
            #self.set_motor(1, 468)
            
            ''' Check the desired frequnency is written to register '''
            # print('freq signal: ' + str(self.bus.read_byte_data(self.pca9685_addr, 0xFE)))

    @classmethod
    def set_motor(cls, motor, val):
        cls.set_PWM_OFF(cls.pca9685_addr, motor, val)
    
    '''
    function from Ref
    '''
    @classmethod
    def set_PWM_ON(cls, addr, ch, value):
        low_byte_val = value & 0x00FF
        high_byte_val = (value & 0x0F00) >> 8
        reg_low_byte = 0x06 + 4 * ch
        cls.bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
        cls.bus.write_i2c_block_data(addr, reg_low_byte+1, [high_byte_val])

    @classmethod
    def set_PWM_OFF(cls, addr, ch, value):
        low_byte_val = value & 0x00FF
        high_byte_val = (value & 0x0F00) >> 8
        reg_low_byte = 0x08 + 4 * ch
        cls.bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
        cls.bus.write_i2c_block_data(addr, reg_low_byte + 1, [high_byte_val])

    @classmethod
    def set_sleep(cls, addr):
        reg_mode1 = 0x00
        sleep_bit = 0x01 << 4
        old_mode1_val = cls.bus.read_byte_data(addr, reg_mode1)
        cls.bus.write_i2c_block_data(
            addr, reg_mode1, [old_mode1_val | sleep_bit])

    @classmethod
    def unset_sleep(cls, addr):
        reg_mode1 = 0x00
        sleep_bit = 0x01 << 4
        old_mode1_val = cls.bus.read_byte_data(addr, reg_mode1)
        cls.bus.write_i2c_block_data(
            addr, reg_mode1, [old_mode1_val & ~(sleep_bit)])


if __name__ == '__main__':
    test = TestPCA9685()

    try:
        while 1:
            cmd = int(input('input cmd val: '))
            print(f'it shoud be {cmd/test.f/4096*1e6} us')
            for i in range(8):
                test.set_motor(i, cmd)
    except KeyboardInterrupt:
        print('bye') 
