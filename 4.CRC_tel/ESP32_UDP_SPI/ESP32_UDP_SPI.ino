// ESP32_UDP_SPI.ino
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>

const char* ssid = "codelab";
const char* password = "20380800";
WiFiUDP udp;
const unsigned int localPort = 8888;

#define SS_PIN 5
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23

uint8_t udpBuf[256];

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<len;i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j=0;j<8;j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.printf("UDP listening on %d\n", localPort);

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SS_PIN);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
}

bool validateUdpFrame(uint8_t *buf, int len, uint8_t &msgType, uint8_t &seq, uint8_t *&payload, uint8_t &payloadLen) {
  if (len < 6) return false;
  if (buf[0] != 0xAA) return false;
  if (buf[1] != 0x01) return false;
  msgType = buf[2];
  seq = buf[3];
  payloadLen = buf[4];
  if (5 + payloadLen + 2 != len) return false;
  payload = &buf[5];
  uint16_t recvCrc = (buf[5+payloadLen] << 8) | buf[5+payloadLen+1];
  uint16_t calc = crc16(buf, 5 + payloadLen);
  return recvCrc == calc;
}

bool spiTransferAndWaitAck(uint8_t *frame, size_t frameLen, uint8_t seq) {
  // CS LOW, transfer frame, then read ACK (simple synchronous exchange)
  digitalWrite(SS_PIN, LOW);
  for (size_t i=0;i<frameLen;i++) SPI.transfer(frame[i]);
  // read ACK frame header (assume fixed small ACK: Start(0x55), Cmd(0xA0), Seq, Len, payload, CRC)
  uint8_t ackBuf[8];
  for (int i=0;i<6;i++) ackBuf[i] = SPI.transfer(0x00);
  digitalWrite(SS_PIN, HIGH);

  // minimal validation
  if (ackBuf[0] != 0x55) return false;
  if (ackBuf[1] != 0xA0) return false;
  if (ackBuf[2] != seq) return false;
  // payload check omitted for brevity
  return true;
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(udpBuf, sizeof(udpBuf));
    uint8_t msgType, seq, *payload; uint8_t payloadLen;
    if (!validateUdpFrame(udpBuf, len, msgType, seq, payload, payloadLen)) {
      Serial.println("Invalid UDP frame");
      return;
    }
    Serial.printf("UDP MsgType %02X Seq %d Len %d\n", msgType, seq, payloadLen);

    // Build SPI frame from payload (example: Cmd = payload[0] or map msgType->Cmd)
    uint8_t spiFrame[64];
    size_t spiLen = 0;
    spiFrame[spiLen++] = 0x55;                // Start
    spiFrame[spiLen++] = payload[0];          // Cmd (예: payload 첫바이트)
    spiFrame[spiLen++] = seq;                 // Seq
    spiFrame[spiLen++] = payloadLen - 1;      // Len (payload에서 첫바이트 제외)
    for (int i=1;i<payloadLen;i++) spiFrame[spiLen++] = payload[i];
    uint16_t crc = crc16(spiFrame, spiLen);
    spiFrame[spiLen++] = (crc >> 8) & 0xFF;
    spiFrame[spiLen++] = crc & 0xFF;

    // ACK/재전송 루틴
    bool ok = false;
    for (int attempt=0; attempt<3 && !ok; attempt++) {
      ok = spiTransferAndWaitAck(spiFrame, spiLen, seq);
      if (!ok) delay(150);
    }

    // (선택) PC에 상태 응답 전송
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    if (ok) udp.print("ACK");
    else udp.print("NACK");
    udp.endPacket();
  }
}