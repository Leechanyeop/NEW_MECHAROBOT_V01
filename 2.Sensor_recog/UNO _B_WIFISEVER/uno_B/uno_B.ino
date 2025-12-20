#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);   // PC 디버깅용
  mySerial.begin(9600); // Uno A와 통신용
}

void loop() {
  if (mySerial.available() > 0) {
    int value = mySerial.parseInt(); // 숫자값 수신
    Serial.print("Received sensor value: ");
    Serial.println(value);
    // 여기서 value를 활용해 모터 제어 등 가능
  }
}