#include <SPI.h>

// 메가 slave쪽
const uint8_t LED_PIN = 9;    // 실제 LED 연결 핀
const uint8_t SS_PIN  = 53;   // Mega의 SS

volatile uint8_t lastCmd = 0;
volatile uint8_t nextResp = 0xFF;
volatile bool cmdFlag = false;
volatile bool ledState = false; // true = 켜짐, false = 꺼짐

void setup() {
  Serial.begin(115200);           // 디버그는 loop에서만
  pinMode(LED_BUILTIN, OUTPUT;
  pinMode(LED_PIN, OUTPUT);

  pinMode(MISO, OUTPUT);          // Slave 출력
  pinMode(SS_PIN, INPUT_PULLUP);  // SS(53) 중요: 입력 유지

  // 클래식 AVR용 SPI 슬레이브 활성화
  SPCR |= _BV(SPE);               // SPI Enable (Slave)
  SPI.attachInterrupt();          // ISR enable

  SPDR = 0xFF;                    // 첫 응답 초기화
  Serial.println("MEGA SPI Slave Ready");
  delay(1000);
}

// ISR은 매우 짧게 유지
ISR(SPI_STC_vect) {
  uint8_t c = SPDR; // 마스터가 보낸 바이트 읽기

  // 명령 바이트(0x01 또는 0x02)일 때만 상태 변경
  if (c == 0x01) {
    ledState = true;    // 켜기 명령 수신
    nextResp = 0xA1;
  } else if (c == 0x02) {
    ledState = false;   // 끄기 명령 수신
    nextResp = 0xA2;
  } else {
    // 더미 바이트나 기타 값은 상태 변경하지 않음
    nextResp = 0xFF;
  }

  lastCmd = c;     // 디버깅/로깅용으로 보관
  cmdFlag = true;  // loop에서 처리하도록 플래그 설정

  SPDR = nextResp; // 다음 transfer에서 나갈 값 준비
}

void loop() {
  if (cmdFlag) {
    // ISR에서 설정한 플래그 읽고 처리
    noInterrupts();
    uint8_t cmd = lastCmd;
    bool state = ledState;
    cmdFlag = false;
    interrupts();

    Serial.print("CMD: 0x");
    Serial.println(cmd, HEX);

    // 상태에 따라 LED 유지
    if (state) {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }
  }

  // 필요시 다른 작업 수행
}