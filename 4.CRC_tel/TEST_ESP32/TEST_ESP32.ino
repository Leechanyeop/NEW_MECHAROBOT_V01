#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "SK_WiFiGIGA11A8_2.4G";
const char* password = "1907023462";

WiFiUDP udp;
const char* udpAddress = "192.168.0.49"; // Ubuntu PC IP 주소
const int udpPort = 8888;                 // 수신 포트

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

   // 현재 ESP32가 받은 IP 주소 출력
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

}

void loop() {
  udp.beginPacket(udpAddress, udpPort);
  udp.print("Hello from ESP32!");
  udp.endPacket();
  delay(1000);
}