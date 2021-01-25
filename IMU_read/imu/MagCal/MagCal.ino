#include "MPU9250.h"

MPU9250 IMU(Wire,0x68);
int state, hz = 100;
float hxb, hyb, hzb;
float hxs, hys, hzs;

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
  Serial.println("Start");
  state = IMU.calibrateMag();
  Serial.println("End");
  hxb = IMU.getMagBiasX_uT(); hxs = IMU.getMagScaleFactorX();
  hyb = IMU.getMagBiasY_uT(); hys = IMU.getMagScaleFactorY();
  hzb = IMU.getMagBiasZ_uT(); hzs = IMU.getMagScaleFactorZ();
  Serial.print(hxb, 4); Serial.print(", "); Serial.print(hxs, 4); Serial.print(", ...");
  Serial.print(hyb, 4); Serial.print(", "); Serial.print(hys, 4); Serial.print(", ...");
  Serial.print(hzb, 4); Serial.print(", "); Serial.println(hzs, 4);
  delay(10000);
}
