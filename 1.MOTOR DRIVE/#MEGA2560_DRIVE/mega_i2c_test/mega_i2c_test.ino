#include <Wire.h>

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
}

void requestEvent() {
  Wire.write("Hi ESP32!");
}

void setup() {
  Serial.begin(9600);
  Wire.begin(8); // Slave 주소 8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  // Slave는 이벤트 기반으로 동작
}