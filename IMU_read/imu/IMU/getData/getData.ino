#include "MPU9250.h" //讀IMU值的library
#include <Adafruit_AHRS_Madgwick.h> //filter的library

MPU9250 IMU(Wire, 0x68);
Adafruit_Madgwick filter;
int state, hz = 100; //state用來確認IMU的狀態，hz為取IMU資料的頻率

struct Imu{
//  float q0, q1, q2, q3;
//  float gx, gy, gz;
//  float ax, ay, az;
  float r, p, y;
};

float ax, ay, az; //a: Accelerometer
float gx, gy, gz; //g: Gyroscope
float mx, my, mz; //m: Magnetometer
float q0, q1, q2, q3; //q: Quaternion
float StartTime; //控制輸出時間

void setup() {
  Serial.begin(115200);
  state = IMU.begin();
  if(state < 0) {
    while(1) {
      Serial.println(state);
      delay(1000);
      }
  }
  filter.begin(hz); //設定filter的頻率
  
  //以下要更改可以參考reference
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G); //設定Accelerometer的值的範圍
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS); //設定Gyroscope的值的範圍
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); //設定Digital Low Pass Filter的bandwidth
  
  IMU.setSrd(1000/hz-1); //hz(output rate) = 1000 / (1+SRD)
  
  //IMU校正的部分，此部分因應不同的IMU而改
  IMU.setAccelCalX(0.0587, 1.0036); 
  IMU.setAccelCalY(0.0587, 1.0003); 
  IMU.setAccelCalZ(0.5999, 0.9899);
  
  IMU.setGyroBiasX_rads(0.0152); 
  IMU.setGyroBiasY_rads(-0.0659); 
  IMU.setGyroBiasZ_rads(0.0227);
  
  IMU.setMagCalX(17.8386, 1.6019); 
  IMU.setMagCalY(16.1988, 0.9611); 
  IMU.setMagCalZ(20.0220, 0.7489);
  
  StartTime = millis()+1000/hz; //固定輸出的時間
}

void loop() {
  IMU.readSensor(); //讀IMU測到的值
  
  //將取出的值已m/(s*s)的單位存起來
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss(); 
  az = IMU.getAccelZ_mss();
  
  //將取出的值已degree的單位存起來
  gx = IMU.getGyroX_rads(); 
  gy = IMU.getGyroY_rads(); 
  gz = IMU.getGyroZ_rads();
  
  //將取出的值已uT的單位存起來
  mx = IMU.getMagX_uT(); 
  my = IMU.getMagY_uT(); 
  mz = IMU.getMagZ_uT();
  
  //將值送入filter內
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); 
  
  //將濾過的值以Quaternion的表示法存起來
  filter.getQuaternion(&q0, &q1, &q2, &q3);

  
  float r, p, y;
  r = filter.getRoll();
  p = filter.getPitch();
  y = filter.getYaw();
  
  
  //當到達需要輸出的時間後就輸出
  if(millis() > StartTime) {
    Imu data = {/*q0, q1, q2, q3, gx, gy, gz, ax, ay, az,*/ r, p, y};
    
//    Serial.print(r);
//    Serial.print(' ');
//    Serial.print(p);
//    Serial.print(' ');
//    Serial.print(y);
//    Serial.println(' ');
    
    
    Serial.write((byte*)&data, sizeof(data));
    Serial.write(10);
    
    StartTime += 1000/hz;
  }
}