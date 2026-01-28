#include <WiFi.h>
#include <SPI.h>

/**
 * @file ESP32_Master.ino
 * @brief ESP32 Master: Send framed commands to Mega Slave via SPI and heartbeat pose via WiFi/SPI
 */

// ============== WiFi Settings ==============
const char* ssid = "codelab";
const char* password = "20380800";
WiFiClient client;
const char* host = "192.168.0.36"; // PC IP Address
const uint16_t port = 5000;

// ============== SPI Settings ==============
const int chipSelectPin = 5;

// ============== Encoder Pins ==============
const int enc1A = 32;
const int enc1B = 33;
const int enc2A = 25;
const int enc2B = 26;

// ============== Global Variables ==============
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // ms

// ============== Robot Physical Constants ==============
const float WHEEL_DIAMETER_MM = 67.0f;
const float GEAR_RATIO = 30.0f;
const int ENCODER_CPR = 11;
const float TICKS_PER_REV = (float)ENCODER_CPR * 4.0f * GEAR_RATIO; // 1320
const float WHEEL_BASE_MM = 200.0f;

float posX = 0.0f, posY = 0.0f, posTheta = 0.0f;
long lastEnc1 = 0, lastEnc2 = 0;

const float PI_F = 3.14159265358979323846f;

// ============== Target Mode Variables (Global) ==============
// 0:None, 1:5x300mm, 2:Turn90, 3:Forward300, 4:Backward300
int targetMode = 0; 
int targetStep = 0; 
float targetStartTheta = 0.0f;
float targetStartX = 0.0f;
volatile float travelDist = 0.0f; 

// ============== Encoder ISRs ==============
void IRAM_ATTR isr1A() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) encoder1Count++;
  else encoder1Count--;
}
void IRAM_ATTR isr1B() {
  if (digitalRead(enc1A) != digitalRead(enc1B)) encoder1Count++;
  else encoder1Count--;
}
void IRAM_ATTR isr2A() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) encoder2Count++;
  else encoder2Count--;
}
void IRAM_ATTR isr2B() {
  if (digitalRead(enc2A) != digitalRead(enc2B)) encoder2Count++;
  else encoder2Count--;
}

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

// ============== Odometry ==============
void safeReadEncoders(long &e1, long &e2) {
  noInterrupts();
  e1 = encoder1Count;
  e2 = encoder2Count;
  interrupts();
}

void updateOdometry() {
  long curE1, curE2;
  safeReadEncoders(curE1, curE2);

  long dE1 = curE1 - lastEnc1;
  long dE2 = curE2 - lastEnc2;
  lastEnc1 = curE1;
  lastEnc2 = curE2;

  if (TICKS_PER_REV == 0.0f || WHEEL_BASE_MM == 0.0f) return;

  float dD1 = ((float)dE1 / TICKS_PER_REV) * (PI_F * WHEEL_DIAMETER_MM);
  float dD2 = ((float)dE2 / TICKS_PER_REV) * (PI_F * WHEEL_DIAMETER_MM);
  float dS  = (dD1+dD2)*0.5f;
  float dT  = (dD2 - dD1) / WHEEL_BASE_MM; // 표준 공식 (라디안 단위)

  // 누적 이동 거리 (전진 시 증가, 후진 시 감소)
  if (targetMode != 0) {
      travelDist += dS;
  }

  float midTheta = posTheta + dT * 0.5f;
  posX += dS * cosf(midTheta);
  posY += dS * sinf(midTheta);
  posTheta += dT;

  while (posTheta > PI_F) posTheta -= 2.0f * PI_F;
  while (posTheta < -PI_F) posTheta += 2.0f * PI_F;
}

void resetOdometry() {
  noInterrupts();
  encoder1Count = 0;
  encoder2Count = 0;
  interrupts();
  
  lastEnc1 = 0;
  lastEnc2 = 0;
  posX = 0.0f;
  posY = 0.0f;
  posTheta = 0.0f;
  travelDist = 0.0f;
  targetMode = 0;
  Serial.println("[RESET] Odometry and Encoders cleared to 0.");
}

void safeReadPose(float &x, float &y, float &t) {
  noInterrupts();
  x = posX; y = posY; t = posTheta;
  interrupts();
}

void sendHeartbeat() {
  float x, y, th;
  safeReadPose(x, y, th);
  char buf[80];
  int n = snprintf(buf, sizeof(buf), "<P,X:%.2f,Y:%.2f,T:%.3f>", x, y, th);
  if (n > 0 && n < (int)sizeof(buf)) {
    if (client && client.connected()) {
      client.print(buf);
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
    delay(500); Serial.print(".");
  }

  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), isr1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2B), isr2B, CHANGE);

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
        if (client.connect(host, port)) Serial.println("Connected!");
        else Serial.println("Failed.");
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
  if (client.connected() && client.available() > 0) cmdChar = client.read();
  else if (Serial.available() > 0) cmdChar = Serial.read();

  if (cmdChar != 0 && cmdChar != '\n' && cmdChar != '\r') {
    Serial.printf("[IN] Cmd: %c\n", cmdChar);
    
    if (cmdChar == '1') {
      Serial.println("[CMD] Sequence Start: PID Line Trace to X=2500mm");
      targetMode = 1;
      // targetStartX = posX; // 필요한 경우 상대 거리 계산용
      sendCommandString("<p>"); // PID 모드 시작 명령 전송
    } 
    else if (cmdChar == '2') {
      Serial.println("[CMD] Sequence Start: PID Line Trace to X=5000mm (Step 2)");
      targetMode = 2;
      sendCommandString("<p>"); // PID 모드 시작 명령 전송
    }
    else if (cmdChar == 'w') {
      targetMode = 3;
      travelDist = 0.0f;
      sendCommandString("<w>");
    } 
    else if (cmdChar == 's') {
      targetMode = 4;
      travelDist = 0.0f;
      sendCommandString("<s>");
    }
    else if (cmdChar == 'x') {
      targetMode = 0;
      sendCommandString("<x>");
    }
    else if (cmdChar == 18 || cmdChar == 'R') { // 18 is CTRL+R
      resetOdometry();
    }
    else {
      targetMode = 0;
      char buf[16];
      snprintf(buf, sizeof(buf), "<%c>", cmdChar);
      sendCommandString(buf);
    }
  }

  updateOdometry();

  // ============== Target Mode Logic ==============
  if (targetMode == 1) {
    // PID 라인트레이싱 X좌표 2500mm 도달 모드
    if (posX >= 2500.0f) {
      Serial.println("[Target 1] X:2500 Reached. Stopping & Operating Conveyor.");
      sendCommandString("<x>"); // 정지 명령 전송
      delay(200);               // 물리적 정지를 위한 짧은 대기
      sendCommandString("<j>"); // 컨베이어(j) 가동 명령 전송
      targetMode = 0;
    }
  } 
  else if (targetMode == 2) {
    // PID 라인트레이싱 X좌표 5000mm 도달 모드 (Step 2)
    if (posX >= 5000.0f) {
      Serial.println("[Target 2] X:5000 Reached. Stopping & Operating Conveyor.");
      sendCommandString("<x>"); // 정지 명령 전송
      delay(200);               // 물리적 정지를 위한 짧은 대기
      sendCommandString("<j>"); // 컨베이어(j) 가동 명령 전송
      targetMode = 0;
    }
  }
  else if (targetMode == 3) {
    if (travelDist >= 100.0f) {
      sendCommandString("<x>");
      targetMode = 0;
    }
  } 
  else if (targetMode == 4) {
    if (travelDist <= -100.0f) {
      sendCommandString("<x>");
      targetMode = 0;
    }
  }

  // Heartbeat & Debug
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    sendHeartbeat();
    
    long e1, e2;
    safeReadEncoders(e1, e2);
    Serial.printf("Enc: %ld, %ld | Pos: X:%.1f, Y:%.1f, T:%.2f | Dist: %.1f\n", e1, e2, posX, posY, posTheta, travelDist);
  }

  yield();
}
