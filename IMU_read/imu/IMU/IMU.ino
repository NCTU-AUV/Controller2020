#include "MPU9250.h"
#include <Adafruit_AHRS_Madgwick.h>

MPU9250 IMU(Wire,0x68);
Adafruit_Madgwick filter;
int state, hz = 100;
float ax, ay, az, gx, gy, gz, mx, my, mz;
float q0, q1, q2, q3;
float StartTime;

void setup() {
  Serial.begin(115200);
  state = IMU.begin();
  if(state < 0) {
    while(1) {
      Serial.println(state);
      delay(1000);
      }
  }
  filter.begin(hz);
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(1000/hz-1); 
  IMU.setAccelCalX(0.032, 1.005); IMU.setAccelCalY(0.032, 1.011); IMU.setAccelCalZ(0.597, 0.988);
  IMU.setGyroBiasX_rads(0.003); IMU.setGyroBiasY_rads(-0.070); IMU.setGyroBiasZ_rads(0.018);
  IMU.setMagCalX(24.840, 1.027); IMU.setMagCalY(14.153, 1.051); IMU.setMagCalZ(2.125, 0.931);
  StartTime = millis()+1000/hz;
}

void loop() {
  IMU.readSensor();
  ax = IMU.getAccelX_mss(); ay = IMU.getAccelY_mss(); az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads()/M_PI*180; gy = IMU.getGyroY_rads()/M_PI*180; gz = IMU.getGyroZ_rads()/M_PI*180;
  mx = IMU.getMagX_uT(); my = IMU.getMagY_uT(); mz = IMU.getMagZ_uT();
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  filter.getQuaternion(&q0, &q1, &q2, &q3);
  if(millis() > StartTime) {
    Serial.print(q0); Serial.print(", "); Serial.print(q1); Serial.print(", "); Serial.print(q2); Serial.print(", "); Serial.println(q3);
    StartTime += 1000/hz;
  }
}
