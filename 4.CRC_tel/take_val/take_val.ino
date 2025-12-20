#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "SK_WiFiGIGA11A8_2.4G";
const char* password = "1907023462";

WiFiUDP udp;
unsigned int localPort = 8888;  // PC와 동일한 포트

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
}

void loop() {
  char incomingPacket[255];
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    Serial.printf("받은 문자열: %s\n", incomingPacket);
  }
}