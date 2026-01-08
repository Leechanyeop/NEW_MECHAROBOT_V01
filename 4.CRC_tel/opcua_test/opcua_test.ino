#include <WiFiNINA.h>

// WiFi/서버 설정 (기존 값 사용)
const char* ssid     = "hhme";
const char* password = "hme*12345";
IPAddress serverIP(192, 168, 0, 19);
uint16_t serverPort = 49320;

// TCP 연결 테스트 함수 (전역에 정의)
bool testTcpConnect(IPAddress ip, uint16_t port, uint16_t timeoutMs = 3000) {
  WiFiClient client;
  unsigned long start = millis();
  if (client.connect(ip, port)) {
    client.stop();
    return true;
  }
  while (millis() - start < timeoutMs) {
    if (client.connect(ip, port)) {
      client.stop();
      return true;
    }
    delay(100);
  }
  return false;
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
      Serial.println("\nWiFi 연결 실패 (타임아웃).");
      while (1) delay(1000);
    }
  }
  Serial.println(" 연결 완료!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());

  // TCP 연결 테스트 호출
  Serial.print("TCP connect test to ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  bool tcpOk = testTcpConnect(serverIP, serverPort, 3000);
  Serial.print("TCP connect result: ");
  Serial.println(tcpOk ? "OK" : "FAIL");

  // 필요하면 여기서 modbus 연결 시도 (tcpOk 체크 후)
  // if (tcpOk) { modbusTCPClient.begin(serverIP, serverPort); ... }
}

void loop() {
  // 메인 루프

   // TCP 연결 테스트 호출
  Serial.print("TCP connect test to ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  bool tcpOk = testTcpConnect(serverIP, serverPort, 3000);
  Serial.print("TCP connect result: ");
  Serial.println(tcpOk ? "OK" : "FAIL");
}