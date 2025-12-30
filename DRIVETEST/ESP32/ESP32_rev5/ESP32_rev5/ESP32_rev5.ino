// ESP32_UDP_to_SPI_Master_withServo_fixed_clean2.ino
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <ESP32Servo.h> // Library Manager에서 설치 필요
#include "IRSensor.h"

// WiFi 설정
const char* ssid = "codelab";
const char* password = "20380800";
const unsigned int localUdpPort = 4210;
IPAddress remoteIp ="192.168.0.38";
uint16_t remotePort =4211;
// SPI CS
const int CS_PIN = 5; // ESP32 -> Mega SS

// IR 센서 핀 번호
IRSensor ir(32,33,34,35,36,39);

// UDP
WiFiUDP udp;
char incomingPacket[64];

// Servo
Servo servo1;
Servo servo2;
const int SERVO1_PIN = 26;
const int SERVO2_PIN = 27;
int servo1Angle = 90;
int servo2Angle = 90;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Buzzer (LEDC)
const int BUZZER_PIN = 16;
const int BUZZER_CH = 0;
const int BUZZER_FREQ = 2000; // 기본 주파수

void setup() {
  Serial.begin(115200);

  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // UDP 시작
  udp.begin(localUdpPort);
  Serial.printf("UDP listening on %u\n", localUdpPort);

  // SPI 초기화
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  // Servo 초기화
  servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
  Serial.println("Servos attached on pins 26 and 27");

  // Buzzer 초기화 (ledcSetup 유지)
  //ledcSetup(BUZZER_CH, BUZZER_FREQ, 8); // 채널, 주파수, 해상도
  //ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  //ledcWriteTone(BUZZER_CH, 0); // 초기에는 끔





}

void setServoAngleSimple(int servoNum, int angle) {
  angle = constrain(angle, 0, 180);
  if (servoNum == 1) {
    servo1Angle = angle;
    servo1.write(servo1Angle);
    Serial.printf("Servo1 -> %d\n", servo1Angle);
  } else if (servoNum == 2) {
    servo2Angle = angle;
    servo2.write(servo2Angle);
    Serial.printf("Servo2 -> %d\n", servo2Angle);
  }
}

void IRREAD() {
  int sensorValues[6];
  ir.readValues(sensorValues);

  for (int i = 0; i < 6; i++) {
    Serial.print("IR[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(sensorValues[i]);
    Serial.print("  ");

    if (sensorValues[i] < 1000) {
      // ledc로 부저 울림
      ledcWriteTone(BUZZER_CH, 1000);
      delay(100);
      ledcWriteTone(BUZZER_CH, 0);
      Serial.print("장애물 감지!");
    }
    Serial.println();
  }
}

bool requestSensors(uint8_t &leftVal, uint8_t &rightVal) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI.transfer('R');
  delayMicroseconds(5);
  leftVal = SPI.transfer(0x00);
  rightVal = SPI.transfer(0x00);
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return true;
}

bool handleServoCommand(const char* pkt, int len) {
  String s = String(pkt).substring(0, len);
  s.trim();
  if (s.startsWith("SV1:")) {
    int angle = s.substring(4).toInt();
    setServoAngleSimple(1, angle);
    return true;
  } else if (s.startsWith("SV2:")) {
    int angle = s.substring(4).toInt();
    setServoAngleSimple(2, angle);
    return true;
  }
  return false;
}

void IRREAD_UDP(IPAddress remoteIp, unsigned int remotePort) {
  int sensorValues[6];
  ir.readValues(sensorValues);

  // 6개 값을 바이트 배열로 전송 (각각 0~255 범위로 압축하거나 2바이트로 보낼 수 있음)
  uint8_t out[12]; // 6개 * 2바이트 (uint16_t)
  for (int i = 0; i < 6; i++) {
    out[i*2]   = sensorValues[i] & 0xFF;
    out[i*2+1] = (sensorValues[i] >> 8) & 0xFF;
  }

  udp.beginPacket(remoteIp, remotePort);
  udp.write(out, sizeof(out));
  udp.endPacket();
}

unsigned long lastIrRead = 0;
const unsigned long IR_READ_INTERVAL = 200; // ms

void loop() {
  unsigned long now = millis();
  if (now - lastIrRead >= IR_READ_INTERVAL) {
    IRREAD();
    lastIrRead = now;
  }

  int packetSize = udp.parsePacket();
  if (packetSize == 0) {
    delay(10);
    return;
  }

  int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
  if (len > 0) incomingPacket[len] = 0;
  Serial.printf("UDP recv %d bytes: %s\n", len, incomingPacket);

  if (handleServoCommand(incomingPacket, len)) {
    IPAddress remoteIp = udp.remoteIP();
    unsigned int remotePort = udp.remotePort();
    if (remotePort != 0) {
      char resp[48];
      int rlen = snprintf(resp, sizeof(resp), "OK SV1=%d SV2=%d", servo1Angle, servo2Angle);
      udp.beginPacket(remoteIp, remotePort);
      udp.write((uint8_t*)resp, rlen);
      udp.endPacket();
    }
    delay(10);
    return;
  }

  byte cmd = incomingPacket[0];
  if (cmd == 'I') {
  int sensorValues[6];
  ir.readValues(sensorValues);
  uint8_t out[12];
  for (int i = 0; i < 6; i++) {
    out[i*2]   = sensorValues[i] & 0xFF;
    out[i*2+1] = (sensorValues[i] >> 8) & 0xFF;
  }
  udp.beginPacket(remoteIp, remotePort);
  udp.write(out, 12);
  udp.endPacket();
  return;
}

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  byte resp = SPI.transfer(cmd);
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.printf("Sent SPI cmd 0x%02X, got resp 0x%02X\n", cmd, resp);

  IPAddress remoteIp = udp.remoteIP();
  unsigned int remotePort = udp.remotePort();
  if (remotePort != 0) {
    udp.beginPacket(remoteIp, remotePort);
    udp.write(&resp, 1);
    udp.endPacket();
  }

  delay(10);
}