/*
 * Modbus TCP Client for UNO WiFi Rev2
 * 임의의 Modbus TCP 서버와 통신하기 위한 클라이언트 코드
 */

#include <SPI.h>
#include <WiFiNINA.h>

// ===================== 네트워크 설정 =====================
const char* WIFI_SSID     = "hhme";        // WiFi SSID
const char* WIFI_PASSWORD = "hme*12345";       // WiFi 비밀번호

// Modbus 서버 설정 (연결할 서버 정보)
const char* MODBUS_SERVER_IP = "192.168.0.19";  // Modbus 서버 IP
const uint16_t MODBUS_PORT   = 49320;              // Modbus TCP 기본 포트

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

// ===================== 헬퍼 함수 =====================
void serial_printf(const char* format, ...) {
  char buf[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  Serial.print(buf);
}

void (*resetFunc)(void) = 0; // 소프트웨어 리셋 함수

// ===================== Modbus TCP 함수 =====================

/**
 * Modbus TCP 서버에 연결
 */
bool modbusConnect() {
  if (modbusClient.connected()) {
    return true;
  }
  
  serial_printf("Connecting to Modbus server %s:%u...\n", MODBUS_SERVER_IP, MODBUS_PORT);
  
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
 * MBAP 헤더 생성
 */
int buildMBAPHeader(uint8_t* buffer, uint16_t pduLength) {
  transactionID++;
  buffer[0] = (transactionID >> 8) & 0xFF;
  buffer[1] = transactionID & 0xFF;
  buffer[2] = 0x00;
  buffer[3] = 0x00;
  uint16_t length = 1 + pduLength;
  buffer[4] = (length >> 8) & 0xFF;
  buffer[5] = length & 0xFF;
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
  
  modbusClient.write(request, requestLen);
  modbusClient.flush();
  
  unsigned long startTime = millis();
  while (modbusClient.available() < 8) {
    if (millis() - startTime > 3000) {
      Serial.println("Modbus response timeout");
      return -2;
    }
    delay(10);
  }
  
  int received = 0;
  startTime = millis();
  while (modbusClient.available() && received < maxResponseLen) {
    response[received++] = modbusClient.read();
    if (millis() - startTime > 1000) break;
    delay(1);
  }

  // [디버딩] 수신 데이터 16진수로 찍어보기
  if (received > 0) {
    Serial.print("RX (Hex): ");
    for (int i = 0; i < received; i++) {
        if (response[i] < 0x10) Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
  }
  
  return received;
}

/**
 * Holding Registers 읽기
 */
int readHoldingRegisters(uint16_t startAddr, uint16_t quantity, uint16_t* values) {
  uint8_t pdu[5];
  pdu[0] = FC_READ_HOLDING_REGS;
  pdu[1] = (startAddr >> 8) & 0xFF;
  pdu[2] = startAddr & 0xFF;
  pdu[3] = (quantity >> 8) & 0xFF;
  pdu[4] = quantity & 0xFF;
  
  int headerLen = buildMBAPHeader(txBuffer, 5);
  memcpy(txBuffer + headerLen, pdu, 5);
  
  int recvLen = modbusTransaction(txBuffer, headerLen + 5, rxBuffer, sizeof(rxBuffer));
  
  if (recvLen < 9) {
    serial_printf("Invalid response length: %d\n", recvLen);
    return -1;
  }
  
  if (rxBuffer[7] & 0x80) {
    serial_printf("Modbus error: 0x%02X\n", rxBuffer[8]);
    return -rxBuffer[8];
  }
  
  int byteCount = rxBuffer[8];
  int regCount = byteCount / 2;
  
  for (int i = 0; i < regCount && i < quantity; i++) {
    values[i] = (rxBuffer[9 + i*2] << 8) | rxBuffer[9 + i*2 + 1];
  }
  
  return regCount;
}

/**
 * Input Registers 읽기
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
 * Coils 읽기
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
 * 단일 Holding Register 쓰기
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
    serial_printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

/**
 * 단일 Coil 쓰기
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
    serial_printf("Modbus write error: 0x%02X\n", rxBuffer[8]);
    return false;
  }
  
  return true;
}

// ===================== 유틸리티 함수 =====================

void printRegisters(const char* label, uint16_t* values, int count) {
  serial_printf("%s: ", label);
  for (int i = 0; i < count; i++) {
    serial_printf("[%d]=0x%04X(%d) ", i, values[i], values[i]);
  }
  Serial.println();
}

void printCoils(const char* label, uint8_t* values, int count) {
  serial_printf("%s: ", label);
  for (int i = 0; i < count; i++) {
    int byteIdx = i / 8;
    int bitIdx = i % 8;
    bool state = (values[byteIdx] >> bitIdx) & 0x01;
    serial_printf("[%d]=%d ", i, state);
  }
  Serial.println();
}

// ===================== 메인 프로그램 =====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("   UNO WiFi Modbus TCP Client");
  Serial.println("=================================\n");
  
  // WiFi 연결
  serial_printf("Connecting to WiFi: %s\n", WIFI_SSID);
  
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    
    // IP 주소 출력 (AVR 방식)
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm\n");
  } else {
    Serial.println("\nWiFi Connection Failed!");
    Serial.println("Restarting in 5 seconds...");
    delay(5000);
    resetFunc(); // 소프트웨어 리셋
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
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Reconnect
    delay(5000);
    return;
  }
  
  // 주기적으로 Modbus 서버 폴링
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();
    
    Serial.println("--- Modbus Poll ---");
    
    // 예제 1: Holding Registers 읽기
    uint16_t holdingRegs[10];
    int result = readHoldingRegisters(0, 10, holdingRegs);
    if (result > 0) {
      printRegisters("Holding Registers", holdingRegs, result);
    } else {
      serial_printf("Failed to read holding registers: %d\n", result);
    }
    
    Serial.println();
  }
  
  // 시리얼 명령 처리 (수동 테스트용)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "connect") {
      modbusConnect();
    }
    else if (cmd == "status") {
      Serial.print("WiFi: ");
      Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    }
  }
  
  delay(10);
}
