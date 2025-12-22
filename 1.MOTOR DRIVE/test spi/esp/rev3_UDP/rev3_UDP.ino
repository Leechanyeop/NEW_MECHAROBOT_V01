// ESP32 UDP -> SPI Master
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

const char* ssid     = "codelab";
const char* password = "20380800";

const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

const int CS_PIN = 5;               // 변경 가능
SPIClass *vspi = &SPI;
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // 1MHz

void setup() {
  Serial.begin(115200);
  delay(100);

  // WiFi 연결 (STA)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // UDP 시작
  udp.begin(UDP_PORT);
  Serial.print("UDP server started on port ");
  Serial.println(UDP_PORT);

  // SPI 초기화 (VSPI)
  vspi->begin(18, 19, 23, CS_PIN);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  Serial.println("SPI ready (VSPI)");
}

// SPI로 명령 전송: cmd(0x01 또는 0x02) -> 응답 바이트 반환
uint8_t sendCmdSPI(uint8_t cmd) {
  uint8_t resp = 0xFF;
  vspi->beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);

  vspi->transfer(cmd);            // 명령
  vspi->transfer(0x00);           // 더미
  resp = vspi->transfer(0x00);    // 응답 읽기

  digitalWrite(CS_PIN, HIGH);
  vspi->endTransaction();
  return resp;
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // 수신 데이터 읽기
    char incoming[64];
    int len = udp.read(incoming, sizeof(incoming)-1);
    if (len < 0) len = 0;
    incoming[len] = 0;
    Serial.print("UDP recv: ");
    Serial.println(incoming);

    // 텍스트 명령 처리 (대소문자 무시)
    String s = String(incoming);
    s.trim();
    s.toUpperCase();

    uint8_t cmd = 0;
    if (s == "ON" || (len == 1 && (uint8_t)incoming[0] == 0x01)) cmd = 0x01;
    else if (s == "OFF" || (len == 1 && (uint8_t)incoming[0] == 0x02)) cmd = 0x02;

    if (cmd != 0) {
      uint8_t r = sendCmdSPI(cmd);
      Serial.print("Sent SPI cmd 0x");
      Serial.print(cmd, HEX);
      Serial.print(", resp 0x");
      Serial.println(r, HEX);

      // 응답을 보낸 UDP 송신자에게 회신(선택)
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.printf("ACK 0x%02X\n", r);
      udp.endPacket();
    } else {
      Serial.println("Unknown command");
    }
  }

  // 기타 주기 작업
  delay(10);
}