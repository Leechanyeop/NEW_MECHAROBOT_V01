#include <WiFi.h>

/**
 * @file modbusTCP.ino (ESP32 Version)
 * @brief Standalone Modbus TCP Client for ESP32
 * Sends Robot Pose (X, Y, Theta) as Floats to registers 40001-40006
 */

// ===================== 네트워크 설정 =====================
const char* ssid     = "hhme";        
const char* password = "hme*12345";   

// Modbus 서버 설정 (PC 등)
const char* serverIP = "192.168.0.19"; 
const uint16_t serverPort = 502;      

// ===================== Modbus 설정 =====================
#define UNIT_ID 0x01
#define FC_WRITE_MULTIPLE_REGS 0x10

WiFiClient client;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000; // 1초마다 전송

// 가상의 데이터 (실제 프로젝트에서는 엔코더/오도메트리 값으로 대체)
float posX = 1.25f;
float posY = 0.75f;
float posTheta = 90.0f;

// ===================== Modbus TCP 패킷 빌더 =====================
void sendModbusPose(float x, float y, float t) {
  if (!client.connected()) return;

  uint8_t frame[25];
  uint16_t transactionID = random(0, 0xFFFF);

  // 1. Transaction ID
  frame[0] = (transactionID >> 8) & 0xFF;
  frame[1] = transactionID & 0xFF;
  // 2. Protocol ID (0 = Modbus)
  frame[2] = 0x00; frame[3] = 0x00;
  // 3. Length (UnitID(1) + FC(1) + Addr(2) + Qty(2) + BC(1) + Data(12) = 19바이트)
  frame[4] = 0x00; frame[5] = 19;
  // 4. Unit ID
  frame[6] = UNIT_ID;
  // 5. Function Code (16 = Write Multiple Registers)
  frame[7] = FC_WRITE_MULTIPLE_REGS;
  // 6. Starting Address (0x0000 = 40001)
  frame[8] = 0x00; frame[9] = 0x00;
  // 7. Quantity of Registers (3 Floats = 6 Registers)
  frame[10] = 0x00; frame[11] = 0x06;
  // 8. Byte Count (6 regs * 2 bytes = 12)
  frame[12] = 12;

  // 9. Data (Float to bytes - Big Endian / IEEE 754)
  auto floatToBytes = [&](float val, int start) {
      uint32_t data;
      memcpy(&data, &val, 4);
      frame[start] = (data >> 24) & 0xFF;
      frame[start+1] = (data >> 16) & 0xFF;
      frame[start+2] = (data >> 8) & 0xFF;
      frame[start+3] = data & 0xFF;
  };

  floatToBytes(x, 13);
  floatToBytes(y, 17);
  floatToBytes(t, 21);

  client.write(frame, 25);
  Serial.printf("[MODBUS] Sent POSE - X:%.2f, Y:%.2f, T:%.2f\n", x, y, t);
}

// ===================== 프로그램 메인 =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("  ESP32 Modbus TCP Client (NEW)");
  Serial.println("=================================\n");
  
  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi: "); Serial.println(ssid);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("Local IP: "); Serial.println(WiFi.localIP());
  Serial.print("Target Server: "); Serial.print(serverIP);
  Serial.print(":"); Serial.println(serverPort);
}

void loop() {
  uint32_t now = millis();

  // 1. WiFi 연결 확인
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected. Reconnecting...");
    WiFi.begin(ssid, password);
    delay(5000);
    return;
  }

  // 2. Modbus 서버 연결
  if (!client.connected()) {
    static uint32_t lastRetry = 0;
    if (now - lastRetry > 3000) {
      lastRetry = now;
      client.stop();
      if (client.connect(serverIP, serverPort)) {
        client.setNoDelay(true);
        Serial.println("[SYSTEM] Modbus Server Connected!");
      } else {
        Serial.println("[SYSTEM] Connection Failed... Retrying.");
      }
    }
    return;
  }

  // 3. 주기적 데이터 전송 (테스트용 가상 데이터 증가)
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    
    // 주행 시뮬레이션: 데이터 조금씩 변경
    posX += 0.01f;
    posY += 0.005f;
    
    sendModbusPose(posX, posY, posTheta);
  }

  // 서버로부터 오는 응답(ACK) 버리기
  while (client.available() > 0) {
    client.read();
  }

  delay(1);
}
