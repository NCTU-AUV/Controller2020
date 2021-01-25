#include "MPU9250.h"

MPU9250 IMU(Wire,0x68);
int state, hz = 100;
float gxb, gyb, gzb;

void setup() {
  Serial.begin(115200);
  state = IMU.begin();
  if(state < 0) {
    while(1) {
      Serial.println(state);
      delay(1000);
      }
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(1000/hz-1);
}

void loop() {
  state = IMU.calibrateGyro();
  gxb = IMU.getGyroBiasX_rads();
  gyb = IMU.getGyroBiasY_rads();
  gzb = IMU.getGyroBiasZ_rads();
  Serial.print(gxb, 4); Serial.print(", ");
  Serial.print(gyb, 4); Serial.print(", ");
  Serial.println(gzb, 4);
  delay(10000);
}
