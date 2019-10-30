#include <avr/dtostrf.h>
void setup() {
    pinMode(13,OUTPUT);

  SerialUSB.begin(0);
  while(!SerialUSB);

}
float Temp = 0;
float humidity  =0;
float voltage[4]={0};

void loop() {
  char  data[80] ={0};
  Temp = 35.12+(float)random(10);
  humidity = 50.12+(float)random(10);
  for(int i=0;i<4; i++){
    voltage[i] = 4.2+(float)random(10)/100.;
  }
  char c[6];
  dtostrf(Temp,2,2,c);
  strcat(data, " Temp ");
  strcat(data, c);
  dtostrf(humidity,2,2,c);
  strcat(data, " humiditay ");
  strcat(data, c);
  strcat(data, " voltage ");
  for(int i = 0 ;i<4;i++){
    dtostrf(voltage[i],1,2,c);
    strcat(data, c);
    strcat(data, " ");
  }
  SerialUSB.write(data,60);

  delay(1);
  strcpy(data,"");
}
