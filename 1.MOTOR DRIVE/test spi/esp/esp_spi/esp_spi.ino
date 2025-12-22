#include <SPI.h>
// ESP32 쪽 마스터
#define CS_PIN 5

void setup() {
  Serial.begin(115200);

  SPI.begin(18, 19, 23, CS_PIN); // SCK=18 MISO=19 MOSI=23 SS=5
  pinMode(CS_PIN, OUTPUT);       // MASTER 설정
  digitalWrite(CS_PIN, HIGH);    // 중요: 기본 HIGH (SPDR값을 전달하기전에 초기화)

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1MHz 권장
}

// 바이트 더미
byte sendCmd(byte cmd) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(cmd);          // 1) 명령 전송 (이때 읽히는 값은 무시)
  //SPI.transfer(0x00);             // 첫 더미보냄 (Mega가 응답 준비)
  byte resp = SPI.transfer(0x00); // 2) 두번쨰 더미에서 응답 읽기 1바이트
  //byte resp2 =SPI.transfer16(0x00);// 2바이트로 받기
  digitalWrite(CS_PIN, HIGH);
  return resp;
}

void loop() {
  //on 명령
  byte r1 = sendCmd(0x01);
  Serial.print("Mega 응답(ON) : 0x");
  Serial.println(r1, HEX);
  delay(1000);
  //off 명령
  byte r2 = sendCmd(0x02);
  Serial.print("Mega 응답(OFF): 0x");
  Serial.println(r2, HEX);
  delay(1000);
}
