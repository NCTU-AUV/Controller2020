'''
PCA9685 I2C
http://blog.ittraining.com.tw/2015/07/raspberry-pi2-python-i2c-16-pwm-pca9685.html


'''

from smbus2 import SMBus
import time

pca9685_addr = 0x40
sus = SMBus(1)

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

def init(freq = 50):
    freq = int(25000000 / (4096 * freq - 1.0))
    #print(freq)
    #freq = int(freq)
    
    # Set PWM frequency
    # 121 -> 50Hz
    # 117 -> 52Hz
    bus.write_i2c_block_data(pca9685_addr, 0xFE, [freq])
    # Set to wakeup mode
    bus.write_i2c_block_data(pca9685_addr, 0x00, [0x01])
    for i in range(16):
        set_PWM_ON(pca9685_addr, i, 0)

def main():
    init()
    
    #set_PWM_OFF(pca9685_addr, 1, 330)
    #time.sleep(1)
    #set_PWM_OFF(pca9685_addr, 1, 340)
    try:
        while True:
            motor = int(input('motor: '))
            val = int(input('val: '))
            set_motor(motor, val)
            '''
            for i in range(4096):
                set_PWM_OFF(pca9685_addr, 0, i)
                print(i)
                time.sleep(0.5)
            '''
    except KeyboardInterrupt:
        for i in range(16):
            set_motor(i, 0)
        print('\nstop')

if __name__ == '__main__':
    main()
