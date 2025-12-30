#include "IR_Array.h"
//#include "C:\#PROJECT\NEW_MECHROBOT_V.01\2.Sensor_recog\MEGA2560 line_trace\Line_trace\IR_Array.h"

// IR 센서 핀 설정 (예: 디지털 핀 51~47)
IR_Array ir(51,50,49,48,47);  // 6개 핀 전달

void setup() {Serial.begin(9600);}

void loop() {
  int sensorValues[5];
  ir.readValues(sensorValues);

  // 값 출력
  for (int i = 0; i < 5; i++) {
    Serial.print("IR_ARRAY[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(sensorValues[i]);
    Serial.print("  ");
  } Serial.println();
  delay(500); // 0.5초마다 갱신
}
