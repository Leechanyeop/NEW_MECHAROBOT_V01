#include <Wire.h>

void setup() {
  Wire.begin(21, 22); // ESP32 기본 SDA=21, SCL=22
  Serial.begin(115200);
}

void loop() {
  // Mega 2560에 데이터 전송
  Wire.beginTransmission(8); // Slave 주소 8
  Wire.write((const uint8_t*)"Hello Mega", strlen("Hello Mega"));
  Wire.endTransmission();

  delay(1000);

  // Mega 2560으로부터 데이터 요청
  Wire.requestFrom(8, 16);
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
  delay(1000);
}