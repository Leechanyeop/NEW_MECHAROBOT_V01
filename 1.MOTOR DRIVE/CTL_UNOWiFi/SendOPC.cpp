#include "SendOPC.h"

// 변수 정의
IPAddress serverIP(192, 168, 0, 19); 
uint16_t serverPort = 49320;               

WiFiClient opcWifiClient;
ModbusTCPClient modbusTCPClient(opcWifiClient);
int32_t registerOffset = 0;

// 실제 사용할 레지스터 주소 계산
uint32_t reg(uint32_t orig) {
  return orig + registerOffset;
}

// 레지스터 읽기(단일) - 실패 시 0xFFFF 반환
uint16_t readRegisterSafe(uint32_t address) {
  int val = modbusTCPClient.holdingRegisterRead(address);
  if (val < 0) {
    return 0xFFFF;
  }
  return (uint16_t)val;
}

// 레지스터 쓰기 + 검증 - 성공 true/실패 false
bool writeRegisterVerify(uint32_t address, uint16_t value) {
  bool ok = modbusTCPClient.holdingRegisterWrite(address, value);
  if (!ok) {
    Serial.print("[WRITE FAIL] addr=");
    Serial.print(address);
    Serial.print(" value=");
    Serial.println(value);
    return false;
  }
  
  delay(10);
  uint16_t readBack = readRegisterSafe(address);
  if (readBack != value) {
    Serial.print("[VERIFY FAIL] addr=");
    Serial.print(address);
    Serial.print(" wrote=");
    Serial.print(value);
    Serial.print(" read=");
    Serial.println(readBack);
    return false;
  }
  
  Serial.print("[WRITE OK] addr=");
  Serial.print(address);
  Serial.print(" value=");
  Serial.println(value);
  return true;
}

// float -> 2 레지스터 쓰기 (40001=low, 40002=high)
bool writeFloatToRegisters(uint32_t startOrig, float value) {
  union {
    float f;
    uint16_t regs[2];
  } data;
  data.f = value;
  
  uint32_t a0 = reg(startOrig);
  uint32_t a1 = reg(startOrig + 1);
  bool ok0 = writeRegisterVerify(a0, data.regs[0]);
  bool ok1 = writeRegisterVerify(a1, data.regs[1]);
  return ok0 && ok1;
}

// 레지스터 오프셋 자동 감지
void detectRegisterOffset() {
  Serial.println("Detecting register addressing mode...");
  
  // 시도 1: 원래 주소(40001) 그대로
  uint16_t r1 = readRegisterSafe(REG_POS_X_LOW);
  uint16_t r2 = readRegisterSafe(REG_POS_X_HIGH);
  if (r1 != 0xFFFF || r2 != 0xFFFF) {
    registerOffset = 0;
    Serial.println("Using 1-based addressing (no offset).");
    return;
  }
  
  // 시도 2: 0-based (subtract 40001)
  int32_t altOffset = -40001;
  uint16_t r1b = readRegisterSafe(REG_POS_X_LOW + altOffset);
  uint16_t r2b = readRegisterSafe(REG_POS_X_HIGH + altOffset);
  if (r1b != 0xFFFF || r2b != 0xFFFF) {
    registerOffset = altOffset;
    Serial.println("Using 0-based addressing (offset -40001).");
    return;
  }
  
  // 실패: 기본값 유지
  registerOffset = 0;
  Serial.println("Address detection failed; defaulting to 1-based.");
}