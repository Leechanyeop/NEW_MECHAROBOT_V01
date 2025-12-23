#include <SPI.h>

volatile byte command;
volatile byte response;

ISR(SPI_STC_vect) {
  command = SPDR; // ESP32가 보낸 명령 읽기

  // 명령에 따라 동작 수행
  if (command == 0x01) {
    digitalWrite(LED_BUILTIN, HIGH); // LED 켜기
    Serial.println("on");
    response = 0xA1;                 // 응답 코드
  } else if (command == 0x02) {
    digitalWrite(LED_BUILTIN, LOW);  // LED 끄기
    Serial.println("off");
    response = 0xA2;                 // 응답 코드
  } else {
    response = 0xFF;                 // 알 수 없는 명령
  }

  SPDR = response; // 응답 전송
}

void setup() {
  pinMode(MISO, OUTPUT);       // Slave 출력
  pinMode(LED_BUILTIN, OUTPUT);
  SPCR |= _BV(SPE);            // SPI 활성화, Slave 모드
  SPI.attachInterrupt();       // 인터럽트 활성화
  Serial.begin(9600);
  Serial.print("ESp32_통신준비 완료");
}

void loop() {
  // 수신된 명령을 모니터에 출력
  if (command != 0) {
    Serial.print("ESP32 명령: ");
    Serial.println(command, HEX);
    command = 0; // 초기화
  }
}