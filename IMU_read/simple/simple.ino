// https://github.com/hideakitai/MPU9250


#include "MPU9250.h"

MPU9250 mpu;

struct Imu
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, yaw, pitch;
};


void setup()
{
    Serial.begin(9600);

    Wire.begin();

    delay(2000);
    mpu.setup();
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 90) // 0.1s
    {
        mpu.update();
        // mpu.print();
        
        // Direct use write() to sedn buffer data
        Imu data = {mpu.getAcc(0), mpu.getAcc(1), mpu.getAcc(2),
                    mpu.getGyro(0), mpu.getGyro(1), mpu.getGyro(2),
                    mpu.getMag(0), mpu.getMag(1), mpu.getMag(2),
                    mpu.getRoll(), mpu.getYaw(), mpu.getPitch()};
        Serial.write((byte*)&data, sizeof(data));
        Serial.write(10);

        // Could use print() & println() to sedn over String
        /*
        Serial.print(mpu.getAcc(0));
        Serial.print(" ");
        Serial.print(mpu.getAcc(1));
        Serial.println();
        */

        prev_ms = millis();
    }
}
