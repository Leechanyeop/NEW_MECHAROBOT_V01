/**
 * @file SPI_TEST.ino
 * @brief UNO WiFi Rev2: WiFi to SPI Bridge for Mega 2560 Control
 * 
 * 기능:
 * 1. WiFi 연결 (TCP Server 포트 8888)
 * 2. WiFi로 수신된 문자('w','a','s','d' 등)를 SPI를 통해 Mega 2560으로 전송
 * 
 * [배선 주의사항 - UNO WiFi Rev2]
 * ICSP-3 (SCK)   ->  Mega 52
 * ICSP-4 (MOSI)  ->  Mega 51
 * ICSP-1 (MISO)  <-  Mega 50
 * Pin 8  (SS)    ->  Mega 53
 * GND            --  GND
 */

#include <SPI.h>
#include <WiFiNINA.h>

// ============== WiFi 설정 ==============
char ssid[] = "codelab";       // 와이파이 이름
char pass[] = "20380800";   // 와이파이 비밀번호
int status = WL_IDLE_STATUS;
WiFiServer server(8888);         // 포트 8888에서 명령 대기

// ============== SPI 설정 ==============
#define SS_PIN 8

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } 

  // 1. SPI 초기화
  SPI.begin();
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  Serial.println(F("=== UNO WiFi SPI Bridge (WiFi -> Mega) ==="));

  // 2. WiFi 연결
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("[Error] Communication with WiFi module failed!"));
    while (true);
  }

  while (status != WL_CONNECTED) {
    Serial.print(F("[WiFi] Connecting to: "));
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  server.begin();
  Serial.println(F("[WiFi] Connected!"));
  Serial.print(F("[WiFi] Server running at IP: "));
  Serial.println(WiFi.localIP());
  Serial.println(F("[Info] Send 'w', 'a', 's', 'd' via WiFi to control Mega."));
}

void loop() {
  // WiFi 클라이언트 연결 확인
  WiFiClient client = server.available();

  if (client) {
    // 클라이언트가 연결되면
    if (client.connected()) {
      Serial.println(F("[Client] Connected"));
      
      while (client.connected()) {
        if (client.available()) {
          // 1. WiFi로 데이터 수신
          char cmd = client.read();
          
          // 2. 유효한 명령어만 SPI로 전송 (개행문자 제외)
          if (cmd != '\r' && cmd != '\n') {
             // 화면에 출력
             Serial.print(F("[WiFi Recv] "));
             Serial.print(cmd);
             
             // Mega로 전송
             sendCommand(cmd);
             
             // 클라이언트에게 에코(응답) 전송
             client.print("ACK: ");
             client.println(cmd);
          }
        }
      }
      Serial.println(F("[Client] Disconnected"));
      client.stop();
    }
  }
}

/**
 * @brief Mega로 SPI 명령 패킷 전송
 */
void sendCommand(char cmd) {
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(100);

  SPI.transfer('<');        // Header
  delayMicroseconds(50);
  char ack = SPI.transfer(cmd); // Command
  delayMicroseconds(50);
  SPI.transfer('>');        // Footer
  
  delayMicroseconds(100);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();

  // 시리얼 디버깅
  Serial.print(F(" -> [SPI Sent] "));
  Serial.print(cmd);
  Serial.print(F(" (Slave Ack: 0x"));
  Serial.print(ack, HEX);
  Serial.println(F(")"));
}
