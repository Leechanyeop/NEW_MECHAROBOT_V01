/*
 * Modbus TCP Server for Arduino UNO WiFi
 * 아두이노 우노 와이파이용 Modbus TCP 서버
 * 
 * 하드웨어: Arduino UNO WiFi Rev2 / Arduino Nano 33 IoT
 * 라이브러리: WiFiNINA
 */

#include <SPI.h>
#include <WiFiNINA.h>

// ===================== WiFi 설정 =====================
const char* ssid     = "hhme";
const char* password = "hme*12345";

// Modbus TCP 포트
const uint16_t MODBUS_PORT = 502;

// ===================== Modbus 레지스터 설정 =====================
// Holding Register 주소 (0-based, 40001 -> 0)
const int REG_POS_X_LOW  = 0;   // 40001: Position X (Low 16bit)
const int REG_POS_X_HIGH = 1;   // 40002: Position X (High 16bit)
const int REG_SPEED      = 6;   // 40007: Speed (mm/s)
const int REG_STATUS     = 7;   // 40008: Status

const int HOLDING_REG_COUNT = 10;

// 레지스터 저장소
uint16_t holdingRegisters[HOLDING_REG_COUNT];

// ===================== 전역 변수 =====================
WiFiServer server(MODBUS_PORT);
WiFiClient client;

// Modbus 버퍼
uint8_t rxBuffer[64];
uint8_t txBuffer[64];

// ===================== Modbus TCP 처리 함수 =====================
void processModbusRequest() {
  client = server.available();
  
  if (client) {
    if (client.connected() && client.available()) {
      int len = 0;
      
      // MBAP 헤더 + PDU 읽기
      while (client.available() && len < sizeof(rxBuffer)) {
        rxBuffer[len++] = client.read();
      }
      
      if (len >= 12) {
        // MBAP 헤더 파싱
        uint16_t transactionId = (rxBuffer[0] << 8) | rxBuffer[1];
        uint16_t protocolId = (rxBuffer[2] << 8) | rxBuffer[3];
        uint16_t length = (rxBuffer[4] << 8) | rxBuffer[5];
        uint8_t unitId = rxBuffer[6];
        uint8_t functionCode = rxBuffer[7];
        
        if (protocolId != 0x0000) return;
        
        int responseLen = 0;
        
        switch (functionCode) {
          case 0x03:  // Read Holding Registers
          {
            uint16_t startAddr = (rxBuffer[8] << 8) | rxBuffer[9];
            uint16_t quantity = (rxBuffer[10] << 8) | rxBuffer[11];
            
            if (startAddr + quantity <= HOLDING_REG_COUNT) {
              txBuffer[0] = rxBuffer[0];
              txBuffer[1] = rxBuffer[1];
              txBuffer[2] = 0x00;
              txBuffer[3] = 0x00;
              uint16_t respLength = 3 + quantity * 2;
              txBuffer[4] = (respLength >> 8) & 0xFF;
              txBuffer[5] = respLength & 0xFF;
              txBuffer[6] = unitId;
              txBuffer[7] = 0x03;
              txBuffer[8] = quantity * 2;
              
              responseLen = 9;
              for (int i = 0; i < quantity; i++) {
                txBuffer[responseLen++] = (holdingRegisters[startAddr + i] >> 8) & 0xFF;
                txBuffer[responseLen++] = holdingRegisters[startAddr + i] & 0xFF;
              }
              
              client.write(txBuffer, responseLen);
            }
            break;
          }
          
          case 0x06:  // Write Single Register
          {
            uint16_t regAddr = (rxBuffer[8] << 8) | rxBuffer[9];
            uint16_t value = (rxBuffer[10] << 8) | rxBuffer[11];
            
            if (regAddr < HOLDING_REG_COUNT) {
              holdingRegisters[regAddr] = value;
              client.write(rxBuffer, len);
              
              Serial.print("[WRITE] Reg ");
              Serial.print(regAddr);
              Serial.print(" = ");
              Serial.println(value);
            }
            break;
          }
          
          case 0x10:  // Write Multiple Registers
          {
            uint16_t startAddr = (rxBuffer[8] << 8) | rxBuffer[9];
            uint16_t quantity = (rxBuffer[10] << 8) | rxBuffer[11];
            
            if (startAddr + quantity <= HOLDING_REG_COUNT) {
              for (int i = 0; i < quantity; i++) {
                holdingRegisters[startAddr + i] = 
                  (rxBuffer[13 + i*2] << 8) | rxBuffer[13 + i*2 + 1];
              }
              
              txBuffer[0] = rxBuffer[0];
              txBuffer[1] = rxBuffer[1];
              txBuffer[2] = 0x00;
              txBuffer[3] = 0x00;
              txBuffer[4] = 0x00;
              txBuffer[5] = 0x06;
              txBuffer[6] = unitId;
              txBuffer[7] = 0x10;
              txBuffer[8] = rxBuffer[8];
              txBuffer[9] = rxBuffer[9];
              txBuffer[10] = rxBuffer[10];
              txBuffer[11] = rxBuffer[11];
              
              client.write(txBuffer, 12);
            }
            break;
          }
        }
      }
    }
  }
}

// ===================== 시뮬레이션 데이터 업데이트 =====================
void updateSimulationData() {
  static unsigned long lastUpdate = 0;
  static float posX = 0.0;
  static uint16_t speed = 0;
  static uint16_t status = 0;
  
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();
    
    posX += 0.12;
    if (posX > 10.0) posX = 0.0;
    speed = (speed + 7) % 300;
    status = (speed > 0) ? 1 : 0;
    
    union {
      float f;
      uint16_t regs[2];
    } data;
    data.f = posX;
    
    holdingRegisters[REG_POS_X_LOW] = data.regs[0];
    holdingRegisters[REG_POS_X_HIGH] = data.regs[1];
    holdingRegisters[REG_SPEED] = speed;
    holdingRegisters[REG_STATUS] = status;
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println("\n=================================");
  Serial.println("  Arduino UNO WiFi Modbus Server");
  Serial.println("=================================\n");
  
  // WiFi 모듈 체크
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not found!");
    while (true);
  }
  
  // WiFi 연결
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    WiFi.begin(ssid, password);
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    while (true);
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  
  // Modbus TCP 서버 시작
  server.begin();
  Serial.print("Modbus TCP Server started on port ");
  Serial.println(MODBUS_PORT);
  
  // 레지스터 초기화
  memset(holdingRegisters, 0, sizeof(holdingRegisters));
  
  Serial.println("\nReady!\n");
}

// ===================== Loop =====================
void loop() {
  processModbusRequest();
  updateSimulationData();
  
  // 상태 출력 (5초마다)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    
    Serial.print("Regs: PosX_L=");
    Serial.print(holdingRegisters[REG_POS_X_LOW]);
    Serial.print(" PosX_H=");
    Serial.print(holdingRegisters[REG_POS_X_HIGH]);
    Serial.print(" Speed=");
    Serial.print(holdingRegisters[REG_SPEED]);
    Serial.print(" Status=");
    Serial.println(holdingRegisters[REG_STATUS]);
  }
}