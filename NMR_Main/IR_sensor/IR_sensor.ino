#include "IRSensor.h"



// IR 센서 핀 번호 (예: A0 ~ A5)
IRSensor ir(32,33,34,35,36,39);

void setup() {Serial.begin(9600);}

void loop() {
  int sensorValues[6];
  ir.readValues(sensorValues);

  // 값 출력
  for (int i = 0; i < 6; i++) {
    Serial.print("IR[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(sensorValues[i]);
    Serial.print("  ");
  
  if(sensorValues[i]<1000)
  {
    tone(16,1000,100);
    Serial.print("장애물 감지!");  
  }  
  else{

  }
  }
  Serial.println();






  delay(100); // 0.5초마다 갱신
}