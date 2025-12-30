// ESP32_FreeRTOS_TCP_SPI_IR_Servo.ino
#include <WiFi.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include "IRSensor.h"

// WiFi
const char* ssid = "codelab";
const char* password = "20380800";
const uint16_t TCP_PORT = 4210;

// SPI CS
const int CS_PIN = 5;

// IR sensor
IRSensor ir(32,33,34,35,36,39);

// Servo
Servo servo1, servo2;
const int SERVO1_PIN = 26;
const int SERVO2_PIN = 27;
int servo1Angle = 90, servo2Angle = 90;

// Task / Queue
TaskHandle_t irTaskHandle = NULL;
TaskHandle_t tcpTaskHandle = NULL;
TaskHandle_t spiTaskHandle = NULL;

QueueHandle_t irQueue = NULL;      // uint16_t[6] 배열 포인터 또는 구조체
QueueHandle_t spiReqQueue = NULL;  // SPI 요청 구조체
QueueHandle_t spiRespQueue = NULL; // SPI 응답 구조체

// Timing
const unsigned long IR_READ_INTERVAL = 200; // ms
const unsigned long RESPONSE_INTERVAL = 80; // ms

// 구조체 정의
typedef struct {
  uint16_t vals[6];
  unsigned long ts;
} IRPacket;

typedef struct {
  uint8_t cmd;
  // 추가 필드(파라미터) 필요 시 확장
} SPIReq;

typedef struct {
  uint8_t resp;
} SPIResp;

// 함수 선언
void irTask(void* pvParameters);
void tcpTask(void* pvParameters);
void spiTask(void* pvParameters);
void setServoAngleSimple(int servoNum, int angle);

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

  // SPI init
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();

  // Servo init
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);
  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  // Queue 생성
  irQueue = xQueueCreate(4, sizeof(IRPacket));      // 캐시용, 깊이 4
  spiReqQueue = xQueueCreate(8, sizeof(SPIReq));
  spiRespQueue = xQueueCreate(8, sizeof(SPIResp));

  // 태스크 생성 (스택/우선순위는 필요에 따라 조정)
  xTaskCreatePinnedToCore(irTask, "IR Task", 4096, NULL, 1, &irTaskHandle, 1);
  xTaskCreatePinnedToCore(tcpTask, "TCP Task", 8192, NULL, 2, &tcpTaskHandle, 1);
  xTaskCreatePinnedToCore(spiTask, "SPI Task", 4096, NULL, 2, &spiTaskHandle, 1);

  Serial.println("Tasks created");
}

void loop() {
  // FreeRTOS 태스크 사용 시 loop는 비워두거나 낮은 우선순위 작업만 수행
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ---------------- IR Task ---------------- */
void irTask(void* pvParameters) {
  IRPacket pkt;
  unsigned long lastRead = 0;
  for (;;) {
    unsigned long now = millis();
    if (now - lastRead >= IR_READ_INTERVAL) {
      int tmp[6];
      ir.readValues(tmp);
      for (int i=0;i<6;i++) pkt.vals[i] = (uint16_t)tmp[i];
      pkt.ts = now;
      // 큐에 최신값 넣기: 큐가 가득하면 오래된 값 덮어쓰기(대기 없이)
      xQueueOverwrite(irQueue, &pkt); // 큐 깊이 1처럼 동작(최신값 유지)
      lastRead = now;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 짧은 슬립
  }
}

/* ---------------- SPI Task ---------------- */
void spiTask(void* pvParameters) {
  SPIReq req;
  SPIResp resp;
  for (;;) {
    if (xQueueReceive(spiReqQueue, &req, portMAX_DELAY) == pdTRUE) {
      // SPI 전송 (단일 바이트 예)
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(CS_PIN, LOW);
      delayMicroseconds(5);
      uint8_t r = SPI.transfer(req.cmd);
      delayMicroseconds(5);
      digitalWrite(CS_PIN, HIGH);
      SPI.endTransaction();

      resp.resp = r;
      xQueueSend(spiRespQueue, &resp, 0); // 응답 큐에 넣기
    }
  }
}

/* ---------------- TCP Task ----------------
   - 단일 클라이언트 처리 예
   - 텍스트 명령(SV1:, SV2:) 처리
   - 'I' 요청 시 irQueue에서 최신값 읽어 전송 (리틀엔디언 uint16_t x6)
*/
void tcpTask(void* pvParameters) {
  WiFiServer server(TCP_PORT);
  server.begin();
  server.setNoDelay(true);
  WiFiClient client;
  Serial.printf("TCP server started on %u\n", TCP_PORT);

  IRPacket latest;
  while (true) {
    // 클라이언트 연결 수락
    if (!client || !client.connected()) {
      if (server.hasClient()) {
        WiFiClient newClient = server.available();
        if (newClient) {
          if (client && client.connected()) client.stop();
          client = newClient;
          client.setNoDelay(true);
          Serial.println("Client connected: " + client.remoteIP().toString());
        }
      }
    }

    // 클라이언트 데이터 처리
    if (client && client.connected() && client.available()) {
      // 읽기 (최대 128바이트)
      char buf[128];
      int r = client.readBytes(buf, min((int)sizeof(buf)-1, client.available()));
      if (r <= 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
      buf[r] = 0;
      Serial.printf("TCP recv %d bytes: %s\n", r, buf);

      // 텍스트 명령 처리
      String s = String(buf);
      s.trim();
      if (s.startsWith("SV1:")) {
        int angle = s.substring(4).toInt();
        setServoAngleSimple(1, angle);
        char resp[64];
        int len = snprintf(resp, sizeof(resp), "OK SV1=%d SV2=%d\n", servo1Angle, servo2Angle);
        client.write((uint8_t*)resp, len);
        continue;
      } else if (s.startsWith("SV2:")) {
        int angle = s.substring(4).toInt();
        setServoAngleSimple(2, angle);
        char resp[64];
        int len = snprintf(resp, sizeof(resp), "OK SV1=%d SV2=%d\n", servo1Angle, servo2Angle);
        client.write((uint8_t*)resp, len);
        continue;
      }

      // 바이너리/단일 바이트 명령 처리: 첫 바이트로 판단
      uint8_t cmd = (uint8_t)buf[0];
      if (cmd == 'I') {
        // 최신 IR 값 읽기 (큐에서 복사)
        if (xQueuePeek(irQueue, &latest, 0) == pdTRUE) {
          // 전송 빈도 제한
          static unsigned long lastSent = 0;
          unsigned long now = millis();
          if (now - lastSent >= RESPONSE_INTERVAL) {
            uint8_t out[12];
            for (int i=0;i<6;i++) {
              uint16_t v = latest.vals[i];
              out[i*2]   = v & 0xFF;        // LSB
              out[i*2+1] = (v >> 8) & 0xFF; // MSB
            }
            client.write(out, sizeof(out));
            lastSent = now;
            Serial.println("Sent IR (12 bytes)");
          } else {
            // 너무 빠르면 생략(또는 빈 응답)
          }
        } else {
          // 큐에 값이 없으면 0으로 채워서 전송
          uint8_t out[12] = {0};
          client.write(out, sizeof(out));
        }
        continue;
      }

      // SPI 명령을 태스크에 위임하고 응답 대기 (옵션)
      SPIReq req; req.cmd = cmd;
      if (xQueueSend(spiReqQueue, &req, 0) == pdTRUE) {
        SPIResp resp;
        if (xQueueReceive(spiRespQueue, &resp, pdMS_TO_TICKS(100))) {
          client.write(&resp.resp, 1);
        } else {
          uint8_t zero = 0;
          client.write(&zero, 1);
        }
      } else {
        uint8_t zero = 0;
        client.write(&zero, 1);
      }
    }

    // 연결 끊김 처리
    if (client && !client.connected()) {
      Serial.println("Client disconnected");
      client.stop();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setServoAngleSimple(int servoNum, int angle) {
  angle = constrain(angle, 0, 180);
  if (servoNum == 1) {
    servo1Angle = angle;
    servo1.write(servo1Angle);
    Serial.printf("Servo1 -> %d\n", servo1Angle);
  } else {
    servo2Angle = angle;
    servo2.write(servo2Angle);
    Serial.printf("Servo2 -> %d\n", servo2Angle);
  }
}