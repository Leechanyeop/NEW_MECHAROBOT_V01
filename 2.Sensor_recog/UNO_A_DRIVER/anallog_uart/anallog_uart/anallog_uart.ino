#include <SoftwareSerial.h>

// A0 → RX, A1 → TX
SoftwareSerial mySerial(A0, A1);

void setup() {
  Serial.begin(9600);       // 기본 USB 시리얼
  mySerial.begin(9600);     // 다른 보드와 통신용
}

void loop() {
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.print("받은 데이터: ");
    Serial.println(c);
  }

  if (Serial.available()) {
    char c = Serial.read();
    mySerial.write(c); // 다른 보드로 전송
  }
}