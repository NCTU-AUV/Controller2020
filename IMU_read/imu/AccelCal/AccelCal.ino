#include "MPU9250.h"

MPU9250 IMU(Wire,0x68);
int state, hz = 100;
float axb, ayb, azb;
float axs, ays, azs;

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
  for(int i = 1; i <= 6; i++) {
    Serial.print(i);
    state = IMU.calibrateAccel();
    Serial.println(state);
    delay(10000);
  }
  axb = IMU.getAccelBiasX_mss(); axs = IMU.getAccelScaleFactorX();
  ayb = IMU.getAccelBiasY_mss(); ays = IMU.getAccelScaleFactorY();
  azb = IMU.getAccelBiasZ_mss(); azs = IMU.getAccelScaleFactorZ();
  Serial.print(axb, 4); Serial.print(", "); Serial.print(axs, 4); Serial.print(", ...");
  Serial.print(ayb, 4); Serial.print(", "); Serial.print(ays, 4); Serial.print(", ...");
  Serial.print(azb, 4); Serial.print(", "); Serial.println(azs, 4);
  delay(10000);
}
