#include <WiFiNINA.h>
#include <ArduinoModbus.h>

// WiFi 설정
const char* ssid     = "hhme";
const char* password = "hme*12345";

// ModbusTCP 서버 정보
IPAddress serverIP(192, 168, 0, 19); // 서버 IP
int serverPort = 49320;               // ModbusTCP 포트

WiFiClient wifiClient;
ModbusTCPClient modbusTCPClient(wifiClient);

// 레지스터 맵 (사용할 레지스터만 정의)
const uint16_t REG_POS_X_LOW   = 40001;  // Position X (Low 16bit)
const uint16_t REG_POS_X_HIGH  = 40002;  // Position X (High 16bit)
const uint16_t REG_SPEED       = 40007;  // Speed (mm/s)
const uint16_t REG_STATUS      = 40008;  // Status (0=IDLE, 1=RUNNING, 2=ARRIVED)

// 자동 보정된 오프셋 (0 또는 -40001)
int32_t registerOffset = 0;

// 헬퍼: 실제 사용할 레지스터 주소 계산
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.print("WiFi 연결 중...");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nWiFi 연결 실패 (타임아웃). 재시작 필요.");
      while (1) delay(1000);
    }
  }
  Serial.println(" 연결 완료!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());

  Serial.print("ModbusTCP 서버 연결 시도...");
  if (!modbusTCPClient.begin(serverIP, serverPort)) {
    Serial.println("실패");
    while (1) delay(1000);
  }
  Serial.println("성공");

  // 자동 오프셋 감지
  detectRegisterOffset();
}

void loop() {
  // 시뮬레이션 값들
  static float posX = 0.0;
  static int16_t speed = 0;
  static int16_t status = 0;  // 0=IDLE, 1=RUNNING, 2=ARRIVED

  // 값 변경 시뮬레이션
  posX += 0.12;
  if (posX > 10.0) posX = 0.0;
  speed = (speed + 7) % 300;
  status = (speed > 0) ? 1 : 0;

  // 40001, 40002: Position.X 쓰기 (float -> 2 registers)
  bool okPos = writeFloatToRegisters(REG_POS_X_LOW, posX);

  // 40007: Speed 쓰기
  bool okSpeed = writeRegisterVerify(reg(REG_SPEED), (uint16_t)speed);

  // 40008: Status 쓰기
  bool okStatus = writeRegisterVerify(reg(REG_STATUS), (uint16_t)status);

  // 결과 요약 로깅
  Serial.print("[SUMMARY] PosX:");
  Serial.print(okPos ? "OK " : "FAIL ");
  Serial.print("Speed:");
  Serial.print(okSpeed ? "OK " : "FAIL ");
  Serial.print("Status:");
  Serial.println(okStatus ? "OK" : "FAIL");
  
  Serial.print("  -> posX=");
  Serial.print(posX);
  Serial.print(" speed=");
  Serial.print(speed);
  Serial.print(" status=");
  Serial.println(status);

  delay(2000);
}