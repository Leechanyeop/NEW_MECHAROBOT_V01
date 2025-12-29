#include "IRSensor.h"

// IR 센서 핀 번호 (예: A0 ~ A5)
IRSensor ir(A0, A1, A2, A3, A4, A5);

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
  }
  Serial.println();

  delay(500); // 0.5초마다 갱신
}