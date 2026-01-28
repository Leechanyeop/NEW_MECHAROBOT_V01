#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

// 1. 네트워크 및 서버 설정
const char* ssid     = "hhme";     // WiFi 이름
const char* password = "hme*12345"; // WiFi 비밀번호

const char* MODBUS_SERVER_IP = "192.168.0.19"; // 외부 Modbus 서버 IP (전송지)
const uint16_t MODBUS_PORT   = 49320;          // 외부 Modbus 포트

// 2. Modbus 객체 및 전송 주기 설정
ModbusIP mb;
unsigned long lastSendTime = 0;
const uint16_t sendInterval = 1000; // 1초마다 전송

// 전역 변수
bool state = false;
uint16_t from  = 0;
uint16_t to    = 0;

// 3. Modbus 레지스터 주소 (Offset)
const uint16_t REG_POS_START = 0;   // 40001 (x, y, theta 시작)
const uint16_t REG_STAT_START = 3;  // 40004 (state, from, to 시작)

// 통신 결과 확인을 위한 콜백 함수
bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    Serial.printf("Modbus Write Error: 0x%02X\n", event);
  } else {
    Serial.println("Modbus Write Success!");
  }
  return true;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway:    ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Target Server: ");
  Serial.print(MODBUS_SERVER_IP);
  Serial.print(":");
  Serial.println(MODBUS_PORT);
  
  mb.server(); // ESP32 서버 시작 (기본 포트: 502)
  Serial.println("Modbus TCP Server Listening on Port 502");
  mb.client(); // 외부 서버 전송용 클라이언트 시작

  // 외부로부터 받을 레지스터 (0:state, 1:from, 2:to)
  mb.addHreg(0, 0, 3); 
}

void loop() {
  mb.task();

  // 외부(서버)에서 ESP32로 보내준 값 실시간 덮어쓰기
  state = mb.Hreg(0);
  from  = mb.Hreg(1);
  to    = mb.Hreg(2);

  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();

    if (mb.isConnected(MODBUS_SERVER_IP)) {
      Serial.println("Server Connected. Sending data...");
      if (state) {
        // [상태 1] x, y, theta 업데이트 후 전체 6개 데이터 전송
        uint16_t x     = random(0, 1000);
        uint16_t y     = random(0, 1000);
        uint16_t theta = random(0, 360);
        uint16_t fullData[] = {x, y, theta, (uint16_t)state, from, to};
        
        // 40001번지부터 6개 쓰기 (cbWrite 콜백 추가)
        mb.writeHreg(MODBUS_SERVER_IP, REG_POS_START, fullData, 6, cbWrite, MODBUS_PORT);
        Serial.printf("[FULL SEND] x:%d, y:%d, th:%d, St:%d, Fr:%d, To:%d\n", x, y, theta, state, from, to);
      } 
      else {
        // [상태 0 또는 기타] 기본 데이터 {state, from, to} 3개만 항시 전송
        uint16_t statusData[] = {(uint16_t)state, from, to};
        
        // 40004번지부터 3개 쓰기 (cbWrite 콜백 추가)
        mb.writeHreg(MODBUS_SERVER_IP, REG_STAT_START, statusData, 3, cbWrite, MODBUS_PORT);
        Serial.printf("[STAT SEND] State:%d, From:%d, To:%d\n", state, from, to);
      }
    } else {
      mb.connect(MODBUS_SERVER_IP, MODBUS_PORT);
    }
  }
}