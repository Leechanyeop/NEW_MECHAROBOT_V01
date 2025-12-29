// ESP32_UDP_to_SPI_Master_withServo_fixed_clean2.ino

#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <ESP32Servo.h> // Library Manager에서 설치 필요
#include "motor/MotorDrive.h"
#include "C:\#PROJECT\NEW_MECHROBOT_V.01\2.Sensor_recog\ESP32_IR\IR_sensor.h"
#include "motion/MotionTrack.h"
#include "comm/CRCtel.h"
#include "vision/VisionCam.h"
#include "qr/QRvisible.h"

// WiFi 설정
const char* ssid = "SK_WiFiGIGA11A8_2.4G";
const char* password = "1907023462";
const unsigned int localUdpPort = 4210;

// SPI CS
const int CS_PIN = 5; // ESP32 -> Mega SS

// UDP
WiFiUDP udp;
char incomingPacket[64];

// Servo (ESP32Servo 사용)
Servo servo1;
Servo servo2;
const int SERVO1_PIN = 26;
const int SERVO2_PIN = 27;
int servo1Angle = 90;
int servo2Angle = 90;

// 서보 펄스 범위 (마이크로초)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

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
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Servo 초기화: attach(pin, minPulse, maxPulse)
  // ESP32Servo의 attach(pin, minUs, maxUs) 사용 (min/max in microseconds)
  servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
  Serial.println("Servos attached on pins 26 and 27 (attach with min/max pulse)");
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

// Mega에 'R' 전송 후 2바이트(왼,오) 읽음
bool requestSensors(uint8_t &leftVal, uint8_t &rightVal) {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  SPI.transfer('R');
  delayMicroseconds(5);
  leftVal = SPI.transfer(0x00);
  rightVal = SPI.transfer(0x00);
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  return true;
}

// "SV1:90" 또는 "SV2:45" 형태 처리
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

bool

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize == 0) {
    delay(10);
    return;
  }

  int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
  if (len > 0) incomingPacket[len] = 0;
  Serial.printf("UDP recv %d bytes: %s\n", len, incomingPacket);

  // 서보 명령 처리
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

  // 센서 요청 처리 ('R')
  byte cmd = incomingPacket[0];
  if (cmd == 'R') {
    uint8_t L, R;
    requestSensors(L, R);
    Serial.printf("Sensors L=%u R=%u\n", L, R);
    IPAddress remoteIp = udp.remoteIP();
    unsigned int remotePort = udp.remotePort();
    if (remotePort != 0) {
      uint8_t out[2] = { L, R };
      udp.beginPacket(remoteIp, remotePort);
      udp.write(out, 2);
      udp.endPacket();
    }
    delay(10);
    return;
  }

  // 일반 명령을 SPI로 전송
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  byte resp = SPI.transfer(cmd);
  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);

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