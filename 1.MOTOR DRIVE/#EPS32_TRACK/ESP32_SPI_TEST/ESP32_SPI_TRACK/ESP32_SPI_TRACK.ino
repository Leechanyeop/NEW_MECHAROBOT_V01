#include <SPI.h>

#define CS_PIN 5

void setup() {
  Serial.begin(115200);
  SPI.begin(18, 19, 23, CS_PIN); // SCLK=18, MISO=19, MOSI=23, CS=5
  pinMode(CS_PIN, OUTPUT);
}

void loop() {
  // 예: LED 켜라는 명령 (0x01)
  digitalWrite(CS_PIN, LOW);
  byte response = SPI.transfer(0x01);
  digitalWrite(CS_PIN, HIGH);

  Serial.print("Mega 응답: ");
  Serial.println(response, HEX);

  delay(2000);

  // 예: LED 끄라는 명령 (0x02)
  digitalWrite(CS_PIN, LOW);
  response = SPI.transfer(0x02);
  digitalWrite(CS_PIN, HIGH);

  Serial.print("Mega 응답: ");
  Serial.println(response, HEX);

  delay(2000);
}