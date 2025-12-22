// Mega2560 I2C Slave
#include <Wire.h>

const uint8_t SLAVE_ADDR = 0x08;
const int LED_PIN = 9;

volatile uint8_t lastCmd = 0;
volatile uint8_t respByte = 0xFF;
volatile bool newCmd = false;

void onReceive(int numBytes) {
  if (numBytes >= 1) {
    lastCmd = Wire.read(); // 명령 바이트 읽기
    newCmd = true;
    // 명령에 따라 응답 준비
    if (lastCmd == 0x01) respByte = 0xA1;
    else if (lastCmd == 0x02) respByte = 0xA2;
    else respByte = 0xFF;
  }
  // 만약 마스터가 추가 바이트를 보냈다면 읽어 버림
  while (Wire.available()) Wire.read();
}

void onRequest() {
  // 마스터가 요청하면 준비된 응답을 보냄
  Wire.write(respByte);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(SLAVE_ADDR);        // I2C 슬레이브 시작
  Wire.onReceive(onReceive);    // 수신 핸들러 등록
  Wire.onRequest(onRequest);    // 요청 핸들러 등록
  Serial.begin(115200);
  Serial.println("Mega I2C Slave Ready");
}

void loop() {
  if (newCmd) {
    newCmd = false;
    Serial.print("Received CMD: 0x");
    Serial.println(lastCmd, HEX);

    if (lastCmd == 0x01) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (lastCmd == 0x02) {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
    // 응답(respByte)은 onRequest에서 전송됨
  }
}