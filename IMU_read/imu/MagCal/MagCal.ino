#include "MPU9250.h" //讀IMU值的library

MPU9250 IMU(Wire,0x68);
int state, hz = 100; //state用來確認IMU的狀態，hz為取IMU資料的頻率

//magnetometer校正分為兩部分，bias和scale
float hxb, hyb, hzb; //bias
float hxs, hys, hzs; //scale

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
  //開始測量bias和scale
  Serial.println("Start");
  state = IMU.calibrateMag();
  Serial.println(state); // 如果測量結果成功則會顯示1
  
  //X軸的bias和scale
  hxb = IMU.getMagBiasX_uT(); 
  hxs = IMU.getMagScaleFactorX();
  
  //Y軸的bias和scale
  hyb = IMU.getMagBiasY_uT(); 
  hys = IMU.getMagScaleFactorY();
  
  //Z軸的bias和scale
  hzb = IMU.getMagBiasZ_uT(); 
  hzs = IMU.getMagScaleFactorZ();
  
  //將值顯示出來
  Serial.print(hxb, 4); Serial.print(", "); Serial.print(hxs, 4); Serial.print(", ...");
  Serial.print(hyb, 4); Serial.print(", "); Serial.print(hys, 4); Serial.print(", ...");
  Serial.print(hzb, 4); Serial.print(", "); Serial.println(hzs, 4);
  
  delay(10000);
}
