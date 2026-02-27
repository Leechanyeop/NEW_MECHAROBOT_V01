#include <SPI.h>
#include <WiFi.h>

/**
 * @file ESP32_Master.ino
 * @brief ESP32 Master: Send framed commands to Mega Slave via SPI and heartbeat
 * pose via WiFi/SPI
 */

// ============== WiFi Settings ==============
const char *ssid = "codelab";
const char *password = "20380800";
WiFiClient client;
const char *host = "192.168.0.85"; // PC IP Address
const uint16_t port = 5000;

// ============== SPI Settings ==============
const int chipSelectPin = 5;

// ============== Encoder Pins ==============
const int enc1A = 32;
const int enc1B = 33;
const int enc2A = 25;
const int enc2B = 26;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // ms
const float PI_F = 3.14159265358979323846f;

// ============== Target Mode Variables (Global) ==============
// 0:None, 1:5x300mm, 2:Turn90, 3:Forward300, 4:Backward300
// int targetMode = 0;
// int targetStep = 0;
// float targetStartTheta = 0.0f;
// float targetStartX = 0.0f;
// volatile float travelDist = 0.0f;
int targetMode = 0;

// ============== SPI Utility ==============
void sendCommandString(const char *frame) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  size_t len = strlen(frame);
  for (size_t i = 0; i < len; ++i) {
    SPI.transfer((uint8_t)frame[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  Serial.print("[SPI] Sent: ");
  Serial.println(frame);
}

// ============== Poll and Process Data from Mega ==============
void processMegaResponse() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);

  char responseBuf[32];
  bool dataReceived = false;

  // 첫 바이트 확인
  uint8_t firstByte = SPI.transfer(0x00);
  if (firstByte != 0x00 && (uint8_t)firstByte != 0xFF) {
    responseBuf[0] = (char)firstByte;
    dataReceived = true;

    // 나머지 바이트 읽기
    for (int i = 1; i < 31; i++) {
      uint8_t b = SPI.transfer(0x00);
      if (b == 0x00 || b == 0xFF) {
        responseBuf[i] = '\0';
        break;
      }
      responseBuf[i] = (char)b;
      if (i == 30)
        responseBuf[31] = '\0';
    }
  }

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  if (dataReceived) {
    String resp = String(responseBuf);
    if (resp.startsWith("C:")) {
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("[SERVER CMD] Station Arrival Detected!"));
      Serial.print(F("[RECVD] "));
      Serial.println(resp);

      // 파싱: C:1,T:1234
      int commaIdx = resp.indexOf(',');
      if (commaIdx > 0) {
        String countStr = resp.substring(2, commaIdx);
        String timeStr = resp.substring(commaIdx + 3);

        Serial.printf(" > Station Count: %s\n", countStr.c_str());
        Serial.printf(" > Travel Time: %.2f sec\n", timeStr.toInt() / 1000.0);

        // WiFi 서버로도 전송
        if (client.connected()) {
          client.printf("STATION:%s,TIME:%s\n", countStr.c_str(),
                        timeStr.c_str());
        }
      }
      Serial.println(F("========================================"));
    } else if (resp.length() > 0) {
      Serial.print("[MEGA] Raw: ");
      Serial.println(resp);
    }
  }
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.printf("\nConnecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();

  Serial.println("\n=== ESP32 SPI Master Ready ===");
}

// ============== Loop ==============
void loop() {
  uint32_t now = millis();

  // WiFi & Server Reconnection
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      static uint32_t lastServerTry = 0;
      if (now - lastServerTry > 5000) {
        lastServerTry = now;
        Serial.printf("Connecting to Server %s:%d ... ", host, port);
        if (client.connect(host, port))
          Serial.println("Connected!");
        else
          Serial.println("Failed.");
      }
    }
  } else {
    static uint32_t lastTry = 0;
    if (now - lastTry > 5000) {
      WiFi.begin(ssid, password);
      lastTry = now;
    }
  }

  // Handle incoming data (WiFi or Serial)
  char cmdChar = 0;
  if (client.connected() && client.available() > 0)
    cmdChar = client.read();
  else if (Serial.available() > 0)
    cmdChar = Serial.read();

  if (cmdChar != 0 && cmdChar != '\n' && cmdChar != '\r') {
    Serial.printf("[IN] Cmd: %c\n", cmdChar);

    if (cmdChar == '1') {
      Serial.println("[CMD] Sequence Start: PID Line Trace to X=2500mm");
      targetMode = 1;
      // targetStartX = posX; // 필요한 경우 상대 거리 계산용
      sendCommandString("<p>"); // PID 모드 시작 명령 전송
    } else if (cmdChar == '2') {
      Serial.println("[CMD] Auto Station Mode: PID + Stop on White + Conveyor");
      targetMode = 2;
      sendCommandString("<2>"); // 전용 자동 모드 명령 전송
    } else if (cmdChar == 'w') {
      sendCommandString("<w>");
    } else if (cmdChar == 's') {
      sendCommandString("<s>");
    } else if (cmdChar == 'm') {
      Serial.println("[CMD] Motor Diagnostic: Test Left then Right");
      sendCommandString("<m>");
    } else if (cmdChar == 18 || cmdChar == 'R') { // 18 is CTRL+R
      // resetOdometry();
    } else {
      targetMode = 0;
      char buf[16];
      snprintf(buf, sizeof(buf), "<%c>", cmdChar);
      sendCommandString(buf);
    }
  }

  // updateOdometry();

  // ============== Target Mode Logic ==============
  /*
    // ============== Target Mode Logic ==============
    if (targetMode == 1) {
      if (posX >= 2500.0f) {
        ...
      }
    }
    ...
  */

  // Heartbeat & Debug
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    // sendHeartbeat();

    // long e1, e2;
    // safeReadEncoders(e1, e2);
    // Serial.printf("Enc: %ld, %ld | Pos: X:%.1f, Y:%.1f, T:%.2f | Dist:
    // %.1f\n",
    //               e1, e2, posX, posY, posTheta, travelDist);
    // Serial.println("[HEARTBEAT] SPI Master Active");

    // Mega로부터 데이터 폴링 및 처리
    processMegaResponse();
  }

  yield();
}
