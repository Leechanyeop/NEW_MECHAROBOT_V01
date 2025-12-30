#include "IRSensor.h"



// IR 센서 핀 번호 (예: A0 ~ A5)
IRSensor ir(32,33,34,35,36,39);

void setup() {Serial.begin(9600);}

void loop() {
  
  }
  Serial.println();






  delay(100); // 0.5초마다 갱신
}