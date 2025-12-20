#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WiFiUDP udp;
unsigned int localPort = 1234;
Servo myServo;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  udp.begin(localPort);

  myServo.attach(13);  // 서보 핀
}

void loop() {
  char incomingPacket[255];
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = '\0';

    int angle = atoi(incomingPacket);  // 문자열 → 숫자 변환
    Serial.printf("받은 각도: %d\n", angle);

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
    }
  }
}