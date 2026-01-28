#include <WiFi.h>

// Wi-Fi 접속 정보
const char* ssid     = "hhme";
const char* password = "hme*12345";

// 서버 IP와 포트 (PC에서 실행되는 TCP 서버)
const char* host = "192.168.0.19";  // 서버 IP
const uint16_t port = 4210;         // 서버 포트

WiFiClient client;

void setup() {
  Serial.begin(115200);

  WiFi.setPowerSave(WIFI_PS_NONE); 

  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // 서버 접속 시도
  if (client.connect(host, port)) {
    Serial.println("Connected to server!");
    client.println("Hello ESP32!");   // 서버로 메시지 전송
  } else {
    Serial.println("Connection failed.");
  }
}

void loop() {
  // 서버로부터 데이터 수신
  if (client.connected() && client.available()) {
    String msg = client.readStringUntil('\n');
    Serial.print("Received from server: ");
    Serial.println(msg);
  }

  // 연결이 끊어졌을 경우 재접속 시도
  if (!client.connected()) {
    Serial.println("Disconnected. Trying to reconnect...");
    if (client.connect(host, port)) {
      Serial.println("Reconnected to server!");
      client.println("Hello ESP32!");
    }
    delay(5000);
  }
}