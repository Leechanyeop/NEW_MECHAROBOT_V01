/*
 * Modbus TCP Client for ESP32
 * 임의의 Modbus TCP 서버와 통신하기 위한 클라이언트 코드
 * 
 * 지원 기능:
 * - Coil 읽기/쓰기 (Function Code 01, 05, 15)
 * - Discrete Input 읽기 (Function Code 02)
 * - Holding Register 읽기/쓰기 (Function Code 03, 06, 16)
 * - Input Register 읽기 (Function Code 04)
 */

#include <WiFi.h>

// ===================== 네트워크 설정 =====================
const char* WIFI_SSID     = "codelab";        // WiFi SSID
const char* WIFI_PASSWORD = "20380800";       // WiFi 비밀번호

// Modbus 서버 설정 (연결할 서버 정보)
const char* MODBUS_SERVER_IP = "192.168.0.100";  // Modbus 서버 IP
const uint16_t MODBUS_PORT   = 502;              // Modbus TCP 기본 포트

// ===================== Modbus 프로토콜 상수 =====================
#define MODBUS_UNIT_ID           1    // 슬레이브 ID

// Function Codes
#define FC_READ_COILS            0x01
#define FC_READ_DISCRETE_INPUTS  0x02
#define FC_READ_HOLDING_REGS     0x03
#define FC_READ_INPUT_REGS       0x04
#define FC_WRITE_SINGLE_COIL     0x05
#define FC_WRITE_SINGLE_REG      0x06
#define FC_WRITE_MULTIPLE_COILS  0x0F
#define FC_WRITE_MULTIPLE_REGS   0x10

// ===================== 전역 변수 =====================
WiFiClient modbusClient;
uint16_t transactionID = 0;

// 수신 버퍼
uint8_t rxBuffer[256];
uint8_t txBuffer[256];

// ===================== Modbus TCP 함수 =====================

/**
 * Modbus TCP 서버에 연결
 */
bool modbusConnect() {
  if (modbusClient.connected()) {
    return true;
  }
  
  Serial.printf("Connecting to Modbus server %s:%d...\n", MODBUS_SERVER_IP, MODBUS_PORT);
  
  if (modbusClient.connect(MODBUS_SERVER_IP, MODBUS_PORT)) {
    Serial.println("Modbus server connected!");
    return true;
  } else {
    Serial.println("Modbus connection failed!");
    return false;
  }
}

/**
 * Modbus TCP 연결 해제
 */
void modbusDisconnect() {
  if (modbusClient.connected()) {
    modbusClient.stop();
    Serial.println("Modbus disconnected");
  }
}

/**
 * MBAP 헤더 생성 (Modbus Application Protocol Header)
 * @param pduLength PDU(Protocol Data Unit) 길이
 * @return 헤더 총 길이 (7 bytes)
 */
int buildMBAPHeader(uint8_t* buffer, uint16_t pduLength) {
  transactionID++;
  
  // Transaction Identifier (2 bytes)
  buffer[0] = (transactionID >> 8) & 0xFF;
  buffer[1] = transactionID & 0xFF;
  
  // Protocol Identifier (2 bytes) - 항상 0x0000 for Modbus
  buffer[2] = 0x00;
  buffer[3] = 0x00;
  
  // Length (2 bytes) - Unit ID + PDU 길이
  uint16_t length = 1 + pduLength;
  buffer[4] = (length >> 8) & 0xFF;
  buffer[5] = length & 0xFF;
  
  // Unit Identifier (1 byte)
  buffer[6] = MODBUS_UNIT_ID;
  
  return 7;
}

/**
 * Modbus 요청 전송 및 응답 수신
 */
int modbusTransaction(uint8_t* request, int requestLen, uint8_t* response, int maxResponseLen) {
  if (!modbusClient.connected()) {
    if (!modbusConnect()) {
      return -1;
    }
  }
  
  // 요청 전송
  modbusClient.write(request, requestLen);
  modbusClient.flush();
  
  // 응답 대기 (타임아웃 3초)
  unsigned long startTime = millis();
  while (modbusClient.available() < 8) {
    if (millis() - startTime > 3000) {
      Serial.println("Modbus response timeout");
      return -2;
    }
    delay(10);
  }
  
  // 응답 수신
  int received = 0;
  startTime = millis();
  while (modbusClient.available() && received < maxResponseLen) {
    response[received++] = modbusClient.read();
    if (millis() - startTime > 1000) break;
    delay(1);
  }
  
  return received;
}

/**
 * Holding Registers 읽기 (Function Code 03)
 * @param startAddr 시작 레지스터 주소
 * @param quantity 읽을 레지스터 개수
 * @param values 결과를 저장할 배열
 * @return 성공 시 읽은 레지스터 수, 실패 시 음수
 */
int readHoldingRegisters(uint16_t startAddr, uint16_t quantity, uint16_t* values) {
  // PDU 구성
  uint8_t pdu[5];
  pdu[0] = FC_READ_HOLDING_REGS;
  pdu[1] = (startAddr >> 8) & 0xFF;
  pdu[2] = startAddr & 0xFF;
  pdu[3] = (quantity >> 8) & 0xFF;
  pdu[4] = quantity & 0xFF;
  
  // 전체 프레임 구성
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  // 전송 및 수신
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 9) {
    Serial.printf("Invalid response length: %d\n", recvLen);
    return -1;
  }
  
  // 에러 응답 체크
  if (rxBuffer[7] & 0x80) {
    Serial.printf("Modbus error: 0x%02X\n", rxBuffer[8]);
    return -rxBuffer[8];
  }
  
  // 데이터 파싱
  int byteCount = rxBuffer[8];
  int regCount = byteCount / 2;
  
  for (int i = 0; i < regCount && i < quantity; i++) {
    values[i] = (rxBuffer[9 + i*2] << 8) | rxBuffer[9 + i*2 + 1];
  }
  
  return regCount;
}

/**
 * Input Registers 읽기 (Function Code 04)
 */
int readInputRegisters(uint16_t startAddr, uint16_t quantity, uint16_t* values) {
  uint8_t pdu[5];
  pdu[0] = FC_READ_INPUT_REGS;
  pdu[1] = (startAddr >> 8) & 0xFF;
  pdu[2] = startAddr & 0xFF;
  pdu[3] = (quantity >> 8) & 0xFF;
  pdu[4] = quantity & 0xFF;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 9) return -1;
  if (rxBuffer[7] & 0x80) return -rxBuffer[8];
  
  int byteCount = rxBuffer[8];
  int regCount = byteCount / 2;
  
  for (int i = 0; i < regCount && i < quantity; i++) {
    values[i] = (rxBuffer[9 + i*2] << 8) | rxBuffer[9 + i*2 + 1];
  }
  
  return regCount;
}

/**
 * Coils 읽기 (Function Code 01)
 */
int readCoils(uint16_t startAddr, uint16_t quantity, uint8_t* values) {
  uint8_t pdu[5];
  pdu[0] = FC_READ_COILS;
  pdu[1] = (startAddr >> 8) & 0xFF;
  pdu[2] = startAddr & 0xFF;
  pdu[3] = (quantity >> 8) & 0xFF;
  pdu[4] = quantity & 0xFF;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 9) return -1;
  if (rxBuffer[7] & 0x80) return -rxBuffer[8];
  
  int byteCount = rxBuffer[8];
  memcpy(values, rxBuffer + 9, byteCount);
  
  return quantity;
}

/**
 * Discrete Inputs 읽기 (Function Code 02)
 */
int readDiscreteInputs(uint16_t startAddr, uint16_t quantity, uint8_t* values) {
  uint8_t pdu[5];
  pdu[0] = FC_READ_DISCRETE_INPUTS;
  pdu[1] = (startAddr >> 8) & 0xFF;
  pdu[2] = startAddr & 0xFF;
  pdu[3] = (quantity >> 8) & 0xFF;
  pdu[4] = quantity & 0xFF;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 9) return -1;
  if (rxBuffer[7] & 0x80) return -rxBuffer[8];
  
  int byteCount = rxBuffer[8];
  memcpy(values, rxBuffer + 9, byteCount);
  
  return quantity;
}

/**
 * 단일 Holding Register 쓰기 (Function Code 06)
 */
bool writeSingleRegister(uint16_t regAddr, uint16_t value) {
  uint8_t pdu[5];
  pdu[0] = FC_WRITE_SINGLE_REG;
  pdu[1] = (regAddr >> 8) & 0xFF;
  pdu[2] = regAddr & 0xFF;
  pdu[3] = (value >> 8) & 0xFF;
  pdu[4] = value & 0xFF;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 12) return false;
  if (rxBuffer[7] & 0x80) {
    Serial.printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

/**
 * 단일 Coil 쓰기 (Function Code 05)
 */
bool writeSingleCoil(uint16_t coilAddr, bool value) {
  uint8_t pdu[5];
  pdu[0] = FC_WRITE_SINGLE_COIL;
  pdu[1] = (coilAddr >> 8) & 0xFF;
  pdu[2] = coilAddr & 0xFF;
  pdu[3] = value ? 0xFF : 0x00;
  pdu[4] = 0x00;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 12) return false;
  if (rxBuffer[7] & 0x80) {
    Serial.printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

/**
 * 다중 Holding Registers 쓰기 (Function Code 16)
 */
bool writeMultipleRegisters(uint16_t startAddr, uint16_t quantity, uint16_t* values) {
  if (quantity > 123) return false;  // 최대 123개 레지스터
  
  uint8_t pdu[256];
  int pduLen = 0;
  
  pdu[pduLen++] = FC_WRITE_MULTIPLE_REGS;
  pdu[pduLen++] = (startAddr >> 8) & 0xFF;
  pdu[pduLen++] = startAddr & 0xFF;
  pdu[pduLen++] = (quantity >> 8) & 0xFF;
  pdu[pduLen++] = quantity & 0xFF;
  pdu[pduLen++] = quantity * 2;  // Byte count
  
  for (int i = 0; i < quantity; i++) {
    pdu[pduLen++] = (values[i] >> 8) & 0xFF;
    pdu[pduLen++] = values[i] & 0xFF;
  }
  
  int headerLen = buildMBAPHeader(txBuffer, pduLen);
  memcpy(txBuffer + headerLen, pdu, pduLen);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + pduLen, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 12) return false;
  if (rxBuffer[7] & 0x80) {
    Serial.printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

/**
 * 다중 Coils 쓰기 (Function Code 15)
 */
bool writeMultipleCoils(uint16_t startAddr, uint16_t quantity, uint8_t* values) {
  int byteCount = (quantity + 7) / 8;
  if (byteCount > 246) return false;
  
  uint8_t pdu[256];
  int pduLen = 0;
  
  pdu[pduLen++] = FC_WRITE_MULTIPLE_COILS;
  pdu[pduLen++] = (startAddr >> 8) & 0xFF;
  pdu[pduLen++] = startAddr & 0xFF;
  pdu[pduLen++] = (quantity >> 8) & 0xFF;
  pdu[pduLen++] = quantity & 0xFF;
  pdu[pduLen++] = byteCount;
  
  memcpy(pdu + pduLen, values, byteCount);
  pduLen += byteCount;
  
  int headerLen = buildMBAPHeader(txBuffer, pduLen);
  memcpy(txBuffer + headerLen, pdu, pduLen);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + pduLen, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 12) return false;
  if (rxBuffer[7] & 0x80) {
    Serial.printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

// ===================== 유틸리티 함수 =====================

/**
 * 레지스터 값 출력
 */
void printRegisters(const char* label, uint16_t* values, int count) {
  Serial.printf("%s: ", label);
  for (int i = 0; i < count; i++) {
    Serial.printf("[%d]=0x%04X(%d) ", i, values[i], values[i]);
  }
  Serial.println();
}

/**
 * Coil 값 출력
 */
void printCoils(const char* label, uint8_t* values, int count) {
  Serial.printf("%s: ", label);
  for (int i = 0; i < count; i++) {
    int byteIdx = i / 8;
    int bitIdx = i % 8;
    bool state = (values[byteIdx] >> bitIdx) & 0x01;
    Serial.printf("[%d]=%d ", i, state);
  }
  Serial.println();
}

// ===================== 메인 프로그램 =====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("   ESP32 Modbus TCP Client");
  Serial.println("=================================\n");
  
  // WiFi 연결
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n\n", WiFi.RSSI());
  } else {
    Serial.println("\nWiFi Connection Failed!");
    Serial.println("Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
  
  // Modbus 서버 연결
  modbusConnect();
}

void loop() {
  static unsigned long lastPollTime = 0;
  const unsigned long POLL_INTERVAL = 2000;  // 2초마다 폴링
  
  // WiFi 연결 확인
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.reconnect();
    delay(5000);
    return;
  }
  
  // 주기적으로 Modbus 서버 폴링
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();
    
    Serial.println("--- Modbus Poll ---");
    
    // 예제 1: Holding Registers 읽기 (주소 0부터 10개)
    uint16_t holdingRegs[10];
    int result = readHoldingRegisters(0, 10, holdingRegs);
    if (result > 0) {
      printRegisters("Holding Registers", holdingRegs, result);
    } else {
      Serial.printf("Failed to read holding registers: %d\n", result);
    }
    
    // 예제 2: Input Registers 읽기 (주소 0부터 5개)
    uint16_t inputRegs[5];
    result = readInputRegisters(0, 5, inputRegs);
    if (result > 0) {
      printRegisters("Input Registers", inputRegs, result);
    } else {
      Serial.printf("Failed to read input registers: %d\n", result);
    }
    
    // 예제 3: Coils 읽기 (주소 0부터 16개)
    uint8_t coils[2];
    result = readCoils(0, 16, coils);
    if (result > 0) {
      printCoils("Coils", coils, 16);
    } else {
      Serial.printf("Failed to read coils: %d\n", result);
    }
    
    // 예제 4: 단일 레지스터 쓰기 (주소 100에 값 12345 쓰기)
    if (writeSingleRegister(100, 12345)) {
      Serial.println("Write single register: OK");
    } else {
      Serial.println("Write single register: FAILED");
    }
    
    // 예제 5: 단일 Coil 쓰기 (주소 50에 ON 쓰기)
    if (writeSingleCoil(50, true)) {
      Serial.println("Write single coil: OK");
    } else {
      Serial.println("Write single coil: FAILED");
    }
    
    Serial.println();
  }
  
  // 시리얼 명령 처리 (수동 테스트용)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "help") {
      Serial.println("Commands:");
      Serial.println("  rhr <addr> <count> - Read Holding Registers");
      Serial.println("  rir <addr> <count> - Read Input Registers");
      Serial.println("  rc  <addr> <count> - Read Coils");
      Serial.println("  wsr <addr> <value> - Write Single Register");
      Serial.println("  wsc <addr> <0|1>   - Write Single Coil");
      Serial.println("  connect            - Connect to server");
      Serial.println("  disconnect         - Disconnect from server");
      Serial.println("  status             - Show connection status");
    }
    else if (cmd.startsWith("rhr ")) {
      int addr, count;
      sscanf(cmd.c_str() + 4, "%d %d", &addr, &count);
      uint16_t regs[125];
      count = min(count, 125);
      int result = readHoldingRegisters(addr, count, regs);
      if (result > 0) {
        printRegisters("Holding Registers", regs, result);
      } else {
        Serial.printf("Error: %d\n", result);
      }
    }
    else if (cmd.startsWith("rir ")) {
      int addr, count;
      sscanf(cmd.c_str() + 4, "%d %d", &addr, &count);
      uint16_t regs[125];
      count = min(count, 125);
      int result = readInputRegisters(addr, count, regs);
      if (result > 0) {
        printRegisters("Input Registers", regs, result);
      } else {
        Serial.printf("Error: %d\n", result);
      }
    }
    else if (cmd.startsWith("rc ")) {
      int addr, count;
      sscanf(cmd.c_str() + 3, "%d %d", &addr, &count);
      uint8_t coils[250];
      count = min(count, 2000);
      int result = readCoils(addr, count, coils);
      if (result > 0) {
        printCoils("Coils", coils, count);
      } else {
        Serial.printf("Error: %d\n", result);
      }
    }
    else if (cmd.startsWith("wsr ")) {
      int addr, value;
      sscanf(cmd.c_str() + 4, "%d %d", &addr, &value);
      if (writeSingleRegister(addr, value)) {
        Serial.println("OK");
      } else {
        Serial.println("FAILED");
      }
    }
    else if (cmd.startsWith("wsc ")) {
      int addr, value;
      sscanf(cmd.c_str() + 4, "%d %d", &addr, &value);
      if (writeSingleCoil(addr, value != 0)) {
        Serial.println("OK");
      } else {
        Serial.println("FAILED");
      }
    }
    else if (cmd == "connect") {
      modbusConnect();
    }
    else if (cmd == "disconnect") {
      modbusDisconnect();
    }
    else if (cmd == "status") {
      Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
      Serial.printf("Modbus: %s\n", modbusClient.connected() ? "Connected" : "Disconnected");
      Serial.printf("Server: %s:%d\n", MODBUS_SERVER_IP, MODBUS_PORT);
    }
  }
  
  delay(10);
}
