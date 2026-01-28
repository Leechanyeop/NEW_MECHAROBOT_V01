#include <WiFi.h>

const char* ssid     = "hhme";
const char* password = "hme*12345";

const char* host = "192.168.0.19";
const uint16_t port = 4210;

WiFiClient client;
unsigned long lastAttempt = 0;
unsigned long reconnectInterval = 2000; // 초기 재접속 간격
unsigned long lastPing = 0;
const unsigned long pingInterval = 10000; // 10초마다 핑

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 20000) { // 20초 이상 연결 안되면 재시도 루프 탈출
      Serial.println("\nWiFi connect timeout, rebooting...");
      ESP.restart();
    }
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  connectToServer();
}

void connectToServer() {
  if (client.connected()) return;
  Serial.print("Trying to connect to ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  // 기존 소켓이 남아있다면 정리
  if (client) client.stop();

  if (client.connect(host, port)) {
    Serial.println("Connected to server!");
    client.println("Hello ESP32!");
    reconnectInterval = 2000; // 성공하면 간격 초기화
  } else {
    Serial.println("Connection failed.");
  }
  lastAttempt = millis();
}

void loop() {
  // Wi-Fi 연결 상태 체크
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, trying to reconnect WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(2000);
    return;
  }

  // 서버로부터 데이터 수신
  if (client.connected() && client.available()) {
    String msg = client.readStringUntil('\n');
    msg.trim();
    if (msg.length()) {
      Serial.print("Received from server: ");
      Serial.println(msg);
    }
  }

  // 주기적으로 핑(에코) 전송해서 연결 유지 확인
  if (client.connected() && millis() - lastPing > pingInterval) {
    client.println("PING");
    client.println("Hello ESP32!");
    lastPing = millis();
  }

  // 연결 끊김 감지 및 재접속 로직
  if (!client.connected()) {
    Serial.println("Disconnected. Trying to reconnect...");
    // 소켓 정리
    client.stop();

    // 지수 백오프(최대 1분)
    if (millis() - lastAttempt >= reconnectInterval) {
      connectToServer();
      // 실패 시 재접속 간격 증가
      if (!client.connected()) {
        reconnectInterval = min(reconnectInterval * 2, 60000UL);
      } else {
        reconnectInterval = 2000; // 성공 시 초기화
      }
      lastAttempt = millis();
    }
  }

  delay(10);
}