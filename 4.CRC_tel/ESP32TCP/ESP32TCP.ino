#include <WiFi.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include "IRSensor.h"

const char* ssid = "codelab";
const char* password = "20380800";
const uint16_t TCP_PORT = 4210;

IRSensor ir(32,33,34,35,36,39);

Servo servo1, servo2;
int servo1Angle = 90, servo2Angle = 90;

WiFiServer server(TCP_PORT);
WiFiClient client;

unsigned long lastIrRead = 0;
const unsigned long IR_READ_INTERVAL = 200;
uint16_t cachedIR[6] = {0};

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  server.begin();
  server.setNoDelay(true);

  servo1.attach(26, 500, 2500);
  servo2.attach(27, 500, 2500);
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
}

void loop() {
  unsigned long now = millis();

  // 1) IR 센서 읽기 (주기적)
  if (now - lastIrRead >= IR_READ_INTERVAL) {
    int tmp[6];
    ir.readValues(tmp);
    for (int i=0;i<6;i++) cachedIR[i] = (uint16_t)tmp[i];
    lastIrRead = now;
  }

  // 2) 클라이언트 연결 확인
  if (!client || !client.connected()) {
    if (server.hasClient()) {
      client = server.available();
      Serial.println("Client connected: " + client.remoteIP().toString());
    }
  }

  // 3) 클라이언트 데이터 처리
  if (client && client.connected() && client.available()) {
    char buf[64];
    int r = client.readBytes(buf, sizeof(buf)-1);
    buf[r] = 0;
    String s = String(buf);
    s.trim();

    if (s.startsWith("SV1:")) {
      servo1Angle = s.substring(4).toInt();
      servo1.write(servo1Angle);
      client.printf("OK SV1=%d SV2=%d\n", servo1Angle, servo2Angle);
    } else if (s.startsWith("SV2:")) {
      servo2Angle = s.substring(4).toInt();
      servo2.write(servo2Angle);
      client.printf("OK SV1=%d SV2=%d\n", servo1Angle, servo2Angle);
    } else {
      uint8_t cmd = (uint8_t)buf[0];
      if (cmd == 'I') {
        uint8_t out[12];
        for (int i=0;i<6;i++) {
          out[i*2]   = cachedIR[i] & 0xFF;
          out[i*2+1] = (cachedIR[i] >> 8) & 0xFF;
        }
        client.write(out, sizeof(out));
      } else {
        // SPI 명령 처리 예시
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(5, LOW);
        delayMicroseconds(5);
        uint8_t resp = SPI.transfer(cmd);
        delayMicroseconds(5);
        digitalWrite(5, HIGH);
        SPI.endTransaction();
        client.write(&resp, 1);
      }
    }
  }

  // 4) 연결 끊김 처리
  if (client && !client.connected()) {
    Serial.println("Client disconnected");
    client.stop();
  }

  delay(5); // CPU 쉬게 하기
}