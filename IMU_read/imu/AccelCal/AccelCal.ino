#include "MPU9250.h" //讀IMU值的library

MPU9250 IMU(Wire,0x68);
int state, hz = 100; //state用來確認IMU的狀態，hz為取IMU資料的頻率

//accelerometer校正分為兩部分，bias和scale
float axb, ayb, azb; //bias
float axs, ays, azs; //scale

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
  for(int i = 1; i <= 6; i++) { //收集六面資料
    Serial.print(i);
    state = IMU.calibrateAccel(); //收集資料以測量bias和scale
    Serial.println(state); //如果此面收集資料成功則螢幕會顯示1
    delay(10000); //每一面為10秒
  }
  
  //X軸的bias和scale
  axb = IMU.getAccelBiasX_mss(); 
  axs = IMU.getAccelScaleFactorX();
  
  //Y軸的bias和scale
  ayb = IMU.getAccelBiasY_mss(); 
  ays = IMU.getAccelScaleFactorY();
  
  //Z軸的bias和scale
  azb = IMU.getAccelBiasZ_mss(); 
  azs = IMU.getAccelScaleFactorZ();
  
  //將值顯示出來
  Serial.print(axb, 4); Serial.print(", "); Serial.print(axs, 4); Serial.print(", ...");
  Serial.print(ayb, 4); Serial.print(", "); Serial.print(ays, 4); Serial.print(", ...");
  Serial.print(azb, 4); Serial.print(", "); Serial.println(azs, 4);
  
  delay(10000);
}
