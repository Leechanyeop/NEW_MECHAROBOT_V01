/**
 * @file SPI_TEST.ino
 * @brief Arduino UNO WiFi Rev2 SPI Master for controlling Mega 2560
 * 
 * [배선 주의사항 - UNO WiFi Rev2]
 * 이 보드는 디지털 11,12,13번에 SPI 핀이 없습니다.
 * 반드시 ICSP 헤더(6핀)를 사용해야 합니다.
 * 
 * UNO WiFi (ICSP)    Mega 2560
 * ----------------   ---------
 * ICSP-3 (SCK)   ->  Pin 52 (SCK)
 * ICSP-4 (MOSI)  ->  Pin 51 (MOSI)
 * ICSP-1 (MISO)  <-  Pin 50 (MISO)
 * Pin 8  (SS)    ->  Pin 53 (SS)
 * GND            --  GND
 */

#include <SPI.h>

// SS(Slave Select) 핀 설정
// UNO WiFi Rev2는 SS를 임의의 디지털 핀으로 설정 가능 (보통 10 or 8)
#define SS_PIN 8

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // 시리얼 연결 대기

  // SPI 핀 초기화
  SPI.begin();
  
  // SS 핀 출력 설정 및 High(비활성) 상태 유지
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  Serial.println(F("=== UNO WiFi SPI Master Control ==="));
  Serial.println(F("Enter commands to send to Mega:"));
  Serial.println(F("  w: Forward"));
  Serial.println(F("  s: Backward"));
  Serial.println(F("  a: Left"));
  Serial.println(F("  d: Right"));
  Serial.println(F("  x: Stop"));
  Serial.println(F("  l: Line Trace"));
  Serial.println(F("  p: PID Trace"));
  Serial.println(F("==================================="));
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // 개행 문자(\r, \n)는 무시하고 유효한 문자만 전송
    if (cmd != '\r' && cmd != '\n') {
      sendCommand(cmd);
    }
  }
}

/**
 * @brief Mega로 SPI 명령 패킷 전송
 * 프로토콜: '<' + 명령어 + '>' (3바이트)
 */
void sendCommand(char cmd) {
  // 트랜잭션 시작 (속도 50kHz ~ 100kHz 권장 - 안정성 위함)
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  
  // Slave Select Low (활성화)
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(100); // 안정화 대기

  // 패킷 전송
  SPI.transfer('<');        // 시작 바이트
  delayMicroseconds(50);
  char ack = SPI.transfer(cmd); // 명령어 전송 (동시에 Slave의 이전 데이터를 받음)
  delayMicroseconds(50);
  SPI.transfer('>');        // 종료 바이트
  
  delayMicroseconds(100);
  // Slave Select High (비활성화)
  digitalWrite(SS_PIN, HIGH);
  
  SPI.endTransaction();

  // 결과 출력
  Serial.print(F("[Sent] "));
  Serial.print(cmd);
  Serial.print(F(" | [Ack] 0x"));
  Serial.println(ack, HEX);
}
