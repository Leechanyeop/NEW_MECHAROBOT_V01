#include <ESP32Servo.h>

Servo myServo;
void setup() {
  Serial.begin(115200);
  myServo.attach(13);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int x = data.substring(0, commaIndex).toInt();
      // 화면 폭 240 기준 → 각도 변환
      int angle = map(x, 0, 240, 0, 180);
      myServo.write(angle);
    }
  }
}