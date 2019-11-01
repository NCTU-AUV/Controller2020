#include <DueTimer.h>
#include "MPU9250.h"
#include <avr/dtostrf.h>

int myLed = 13;
bool ledOn = false;
MPU9250 mpu;


char c[6]="";
char data[80]="123";




void setup()
{
    Serial.begin(115200);
      SerialUSB.begin(0);
    while(!SerialUSB);

    Wire.begin();

    delay(2000);
    mpu.setup();
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
       
        //mpu.print();
        mpu.update();
       
        strcpy(data,"");
  
        strcat(data, "Roll ");
        dtostrf(mpu.getRoll(),2,2,c);
        strcat(data, c);
        
        strcat(data, " Pitch ");
        dtostrf(mpu.getPitch(),2,2,c);
        strcat(data, c);
      
        strcat(data, " Yaw ");
        dtostrf(mpu.getYaw(),2,2,c);
        strcat(data, c);
        strcat(data, "\r\n");
        SerialUSB.write(data,80);
        prev_ms = millis();
    }
}
