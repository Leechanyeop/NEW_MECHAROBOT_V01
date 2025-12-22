// Mega2560 SPI Slave - control pin 9
#include <SPI.h>

const uint8_t LED_PIN = 9;
const uint8_t SS_PIN  = 53; // Mega SS

volatile uint8_t lastCmd = 0;
volatile uint8_t nextResp = 0xFF;
volatile bool cmdFlag = false;
volatile bool ledState = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MISO, OUTPUT);
  pinMode(SS_PIN, INPUT_PULLUP);

  // 클래식 AVR SPI 슬레이브 활성화
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();

  SPDR = 0xFF;
  Serial.println("Mega SPI Slave Ready");
}

ISR(SPI_STC_vect) {
  uint8_t c = SPDR;

  if (c == 0x01) {
    ledState = true;
    nextResp = 0xA1;
  } else if (c == 0x02) {
    ledState = false;
    nextResp = 0xA2;
  } else {
    // 더미 바이트 등은 상태 변경하지 않음
    nextResp = 0xFF;
  }

  lastCmd = c;
  cmdFlag = true;
  SPDR = nextResp;
}

void loop() {
  if (cmdFlag) {
    noInterrupts();
    uint8_t cmd = lastCmd;
    bool state = ledState;
    cmdFlag = false;
    interrupts();

    Serial.print("CMD: 0x");
    Serial.println(cmd, HEX);

    if (state) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}