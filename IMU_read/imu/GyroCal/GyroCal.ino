#include "MPU9250.h" //讀IMU值的library

MPU9250 IMU(Wire,0x68);
int state, hz = 100; //state用來確認IMU的狀態，hz為取IMU資料的頻率

//gyroscope的校正只有bias
float gxb, gyb, gzb; 

void setup() { //此部分和IMU的code相同
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
  state = IMU.calibrateGyro(); //收集資料以測量bias
  
  gxb = IMU.getGyroBiasX_rads(); //X軸的bias
  gyb = IMU.getGyroBiasY_rads(); //Y軸的bias
  gzb = IMU.getGyroBiasZ_rads(); //Z軸的bias
  
  //將值顯示出來
  Serial.print(gxb, 4); Serial.print(", ");
  Serial.print(gyb, 4); Serial.print(", ");
  Serial.println(gzb, 4);
  
  delay(10000);
}
