#include <SPI.h>

const int CS_PIN = 5;               // ESP32 CS 핀 (필요시 변경)
SPIClass *vspi = &SPI;              // VSPI 사용

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz, MSB-first, MODE0

void setup() {
  Serial.begin(115200);
  delay(100);

  // VSPI: SCK=18, MISO=19, MOSI=23, CS=CS_PIN
  vspi->begin(18, 19, 23, CS_PIN);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // idle HIGH

  Serial.println("ESP32 SPI Master Ready");
  Serial.println("Type '1' to send ON, '2' to send OFF");
}

uint8_t sendCmd(uint8_t cmd) {1

  uint8_t resp = 0xFF;

  vspi->beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);      // CS를 LOW로 유지한 채로 연속 전송

  vspi->transfer(cmd);            // 1) 명령 전송
  vspi->transfer(0x00);           // 2) 첫 더미: 슬레이브가 응답 준비
  resp = vspi->transfer(0x00);    // 3) 두 번째 더미에서 실제 응답 읽기

  digitalWrite(CS_PIN, HIGH);     // 전송 종료
  vspi->endTransaction();

  return resp;
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      uint8_t r = sendCmd(0x01);
      Serial.print("Sent ON, response: 0x");
      Serial.println(r, HEX);
    } else if (c == '2') {
      uint8_t r = sendCmd(0x02);
      Serial.print("Sent OFF, response: 0x");
      Serial.println(r, HEX);
    }
  }

  // 자동 주기 전송 예시 (주석 해제해서 사용)
  // static unsigned long last = 0;
  // if (millis() - last > 5000) {
  //   last = millis();
  //   uint8_t r = sendCmd(0x01); // 5초마다 ON 전송
  //   Serial.println(r, HEX);
  // }
}