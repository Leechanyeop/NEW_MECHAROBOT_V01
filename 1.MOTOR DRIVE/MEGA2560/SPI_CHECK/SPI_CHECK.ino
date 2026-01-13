/**
 * @file SPI_CHECK.ino
 * @brief Mega 2560 SPI Slave Receiver Test
 * 
 * 기능:
 * 1. SPI 슬레이브 모드로 동작
 * 2. UNO WiFi에서 보내는 데이터를 수신하여 시리얼 모니터에 출력
 * 3. 데이터 수신 시 17번 LED 깜빡임
 * 
 * [배선 확인]
 * Mega 52 (SCK)  <--- UNO WiFi ICSP-3
 * Mega 51 (MOSI) <--- UNO WiFi ICSP-4
 * Mega 53 (SS)   <--- UNO WiFi 8
 * GND            ---- GND
 */

#include <SPI.h>

volatile char receivedData = 0;
volatile bool newData = false;

// 디버그 LED 핀 (Mega 보드 17번 사용 가정, 없으면 13번 LED_BUILTIN 사용 가능)
#define LED_PIN 17 

void setup() {
  Serial.begin(115200);
  
  // LED 설정
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // ========== SPI Slave 설정 ==========
  // SS(53) 핀은 반드시 INPUT이어야 슬레이브로 동작함
  pinMode(53, INPUT);
  
  // MISO(50) 핀은 데이터를 보내야 하므로 OUTPUT
  pinMode(50, OUTPUT);

  // SPI 통신 활성화 (SPE) + 인터럽트 활성화 (SPIE)
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

  Serial.println(F("\n=== MEGA 2560 SPI MONITOR START ==="));
  Serial.println(F("Waiting for data from UNO WiFi..."));
}

// SPI 인터럽트 루틴
ISR(SPI_STC_vect) {
  char c = SPDR;        // 데이터 레지스터에서 값 읽기
  receivedData = c;     // 전역 변수에 저장
  newData = true;       // 플래그 설정
  
  // LED 토글 (데이터 들어올 때마다 깜빡임)
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void loop() {
  if (newData) {
    char c = receivedData;
    newData = false; // 플래그 초기화

    // 시리얼 모니터에 출력
    // 제어 문자나 공백 등이 올 수 있으므로 HEX 값도 같이 출력
    Serial.print(F("Recv: '"));
    Serial.print(c);
    Serial.print(F("' (Hex: 0x"));
    if (c < 0x10) Serial.print('0');
    Serial.print(c, HEX);
    Serial.println(F(")"));
  }
}
