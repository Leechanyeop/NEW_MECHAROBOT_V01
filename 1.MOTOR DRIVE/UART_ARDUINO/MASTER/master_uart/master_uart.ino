// Uno I2C Master
#include <Wire.h>

const uint8_t SLAVE_ADDR = 0x08;

void setup() {
  Wire.begin(); // 마스터로 시작
  Serial.begin(115200);
  Serial.println("Uno I2C Master Ready");
  delay(200);
}

uint8_t sendCmdAndGetResp(uint8_t cmd) {
  uint8_t resp = 0xFF;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmd);               // 명령 전송
  Wire.endTransmission();        // 전송 완료

  delay(50);                     // 슬레이브가 처리할 시간(필요시 조정)

  Wire.requestFrom(SLAVE_ADDR, (uint8_t)1); // 응답 1바이트 요청
  if (Wire.available()) {
    resp = Wire.read();
  }
  return resp;
}

void loop() {
  uint8_t r1 = sendCmdAndGetResp(0x01); // ON
  Serial.print("Mega 응답(ON) : 0x");
  Serial.println(r1, HEX);
  delay(1000);

  uint8_t r2 = sendCmdAndGetResp(0x02); // OFF
  Serial.print("Mega 응답(OFF): 0x");
  Serial.println(r2, HEX);
  delay(1000);
}