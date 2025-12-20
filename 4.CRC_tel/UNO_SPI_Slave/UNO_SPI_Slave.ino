// UNO_SPI_Slave.ino
#include <SPI.h>

volatile bool frameReady = false;
volatile uint8_t buf[64];
volatile uint8_t idx = 0;

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j=0;j<8;j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

ISR(SPI_STC_vect) {
  uint8_t c = SPDR; // 수신된 바이트
  buf[idx++] = c;
  // 즉시 응답 바이트 준비(기본 0x00)
  SPDR = 0x00;
  // 간단한 프레임 종료 조건: 최소 길이 넘고 CRC 위치 도달하면 표시
  if (idx >= 6) {
    // Start byte check
    if (buf[0] != 0x55) { idx = 0; return; }
    uint8_t len = buf[3];
    if (idx >= 4 + len + 2) {
      frameReady = true;
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);  // SPI Enable (Slave)
  SPCR |= _BV(SPIE); // SPI Interrupt Enable
  idx = 0;
}

void loop() {
  if (frameReady) {
    noInterrupts();
    uint8_t localBuf[64];
    uint8_t localLen = idx;
    memcpy(localBuf, (const void*)buf, localLen);
    idx = 0;
    frameReady = false;
    interrupts();

    // Validate
    if (localBuf[0] != 0x55) { Serial.println("Bad start"); continue; }
    uint8_t cmd = localBuf[1];
    uint8_t seq = localBuf[2];
    uint8_t len = localBuf[3];
    uint16_t recvCrc = (localBuf[4+len] << 8) | localBuf[4+len+1];
    uint16_t calc = crc16(localBuf, 4 + len);
    if (recvCrc != calc) { Serial.println("CRC fail"); continue; }

    // 명령 처리 예: SET_SERVO (cmd 0x10), payload: [servoId, angle]
    if (cmd == 0x10 && len >= 2) {
      uint8_t id = localBuf[4];
      uint8_t angle = localBuf[5];
      Serial.print("Set servo "); Serial.print(id); Serial.print(" to "); Serial.println(angle);
      // 실제 서보 제어 코드 삽입
    }

    // ACK 준비 및 전송: SPI는 마스터가 클럭을 제공할 때만 전송되므로
    // 간단히 상태를 시리얼로 출력하고 SPDR에 다음 바이트를 채워둠
    // (ESP32가 추가로 클럭을 돌려 ACK를 읽어가도록 설계해야 함)
    // 여기서는 SPDR에 ACK 바이트를 미리 채움
    // 실제 ACK 프레임 전송은 마스터가 추가로 클럭을 제공할 때 읽혀짐
    // 예: SPDR = 0xA0; // 첫 응답 바이트
  }
}