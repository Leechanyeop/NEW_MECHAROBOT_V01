#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

void setup() {
  Serial.begin(115200);
  // Serial2: RX=16, TX=17 (DFR1154 연결)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // 서보 연결 (GPIO26)
  myServo.attach(26);   // 필요시 두 번째 인자(minPulse, maxPulse)로 펄스 범위 조정 가능
  myServo.write(90);    // 초기 중앙 위치
  Serial.println("ESP32Servo ready on pin 26. Waiting for QR commands...");
}

void loop() {
  if (Serial2.available()) {
    String qr = Serial2.readStringUntil('\n');
    qr.trim(); // \r, 공백 제거
    Serial.println("QR: " + qr);

    if (qr.equalsIgnoreCase("ON")) {
      // ON -> 좌로 90도 (180도)
      myServo.write(180);
      Serial.println("Action: ON -> Servo to 180 deg (left 90)");
    }
    else if (qr.equalsIgnoreCase("OFF")) {
      // OFF -> 우로 90도 (0도)
      myServo.write(0);
      Serial.println("Action: OFF -> Servo to 0 deg (right 90)");
    }
    else {
      Serial.println("Unknown QR command");
    }
  }
}