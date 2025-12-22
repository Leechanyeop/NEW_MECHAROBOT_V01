#include <SPI.h>
// 메가 slave쪽
#define LED 2 // 실

volatile byte lastCmd = 0; // lED 출력
volatile byte nextResp = 0x00; // ESP 에서 값받고 응답할 바이트
volatile bool cmdFlag = false;

void setup() {
  Serial.begin(115200);           // 디버그는 loop에서만
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED, OUTPUT); 
  pinMode(MISO, OUTPUT);          // Slave 출력
  pinMode(53, INPUT_PULLUP);      // SS(53) 중요: 입력 유지

  SPCR |= _BV(SPE);               // SPI Enable (Slave)
  SPI.attachInterrupt();          // ISR enable
 
  SPDR = 0x00;                    // 첫 응답 초기화
  Serial.println("MEGA SPI Slave Ready");
  delay(1000);
}

// 고속으로 데이터 처리
ISR(SPI_STC_vect) {
  byte c = SPDR;                  // 받은 바이트
  lastCmd = c;
  cmdFlag = true;

  // 다음 바이트에서 Master가 읽을 응답 준비
  if (c == 0x01)      nextResp = 0xA1;// esp on 명령
  else if (c == 0x02) nextResp = 0xA2;// esp off 명령
  else                nextResp = 0xFF;// 예외값

  SPDR = nextResp;                // 다음 transfer에서 나갈 값
}

void loop() {
  if (cmdFlag) {
    cmdFlag = false;

    Serial.print("CMD: 0x");
    Serial.println(lastCmd, HEX);

    if (lastCmd == 0x01) digitalWrite(LED_BUILTIN, HIGH);digitalWrite(LED, HIGH);
    if (lastCmd == 0x02) digitalWrite(LED_BUILTIN, LOW);digitalWrite(LED, LOW);


  }
}
