#include <WiFi.h>
#include <SPI.h>

/**
 * ESP32 Master: Send framed commands to Mega Slave via SPI and heartbeat pose via WiFi/SPI
 */

// WiFi
const char* ssid = "codelab";//와이파이 이름
const char* password = "20380800";//pw
WiFiClient client; //클라이언트
const char* host = "192.168.0.70"; // PC IP Address (Update based on Get-NetIPAddress output)
const uint16_t port = 5000; // 포트번호

// SPI
const int chipSelectPin = 5; // chip select pin

// Encoder pins
const int enc1A = 36; // encoder 1 A pin
const int enc1B = 39; // encoder 1 B pin
const int enc2A = 34; // encoder 2 A pin
const int enc2B = 35; // encoder 2 B pin

// Globals
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 500; // ms

// Robot constants
const float WHEEL_DIAMETER_MM = 65.0f;
const float GEAR_RATIO = 30.0f;
const int ENCODER_CPR = 11;
const float TICKS_PER_REV = (float)ENCODER_CPR * 2.0f * GEAR_RATIO; // 660
const float WHEEL_BASE_MM = 160.0f;

float posX = 0.0f, posY = 0.0f, posTheta = 0.0f;
long lastEnc1 = 0, lastEnc2 = 0;

const float PI_F = 3.14159265358979323846f;

// ISR
// ============== Interrupt Service Routines (ISR) with Debounce ==============
// 50us (0.05ms) 이내의 중복 신호는 노이즈로 간주하고 무시 (최대 약 20kHz 대역폭)
volatile unsigned long lastIsrTime1 = 0;
volatile unsigned long lastIsrTime2 = 0;
const unsigned long DEBOUNCE_US = 50;

void IRAM_ATTR isr1A() {
  unsigned long now = micros();
  // 이전 인터럽트로부터 일정 시간이 지나지 않았으면 무시
  if (now - lastIsrTime1 < DEBOUNCE_US) return;
  lastIsrTime1 = now;

  int a = digitalRead(enc1A);
  int b = digitalRead(enc1B);
  if (a == b) encoder1Count++;
  else encoder1Count--;
}

void IRAM_ATTR isr2A() {
  unsigned long now = micros();
  if (now - lastIsrTime2 < DEBOUNCE_US) return;
  lastIsrTime2 = now;

  int a = digitalRead(enc2A);
  int b = digitalRead(enc2B);
  if (a == b) encoder2Count++;
  else encoder2Count--;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // WiFi connect (with timeout)
  Serial.printf("\nConnecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println("\nWiFi connect timeout. Continuing without WiFi.");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    
    // Connect to TCP Server
    Serial.print("Connecting to TCP Server: ");
    Serial.print(host);
    Serial.print(":");
    Serial.println(port);
    
    if (client.connect(host, port)) {
      Serial.println("TCP Connected!");
    } else {
      Serial.println("TCP Connection Failed!");
    }
  }

  // Encoder pins
  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);

  // SPI
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();

  Serial.println("\n=== ESP32 SPI Master Ready ===");
  Serial.println("Target: Mega 2560 Slave");
  Serial.println("Control via Serial Monitor (Input char to send)");
}

// SPI send (String)
void sendCommandString(const String &frame) {
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  for (size_t i = 0; i < frame.length(); ++i) {
    SPI.transfer((uint8_t)frame[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  Serial.print("[SPI] Sent: ");
  Serial.println(frame);
}

// SPI send (C-string)
void sendCommandString(const char *frame) {
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
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

// Safe encoder read
void safeReadEncoders(long &e1, long &e2) {
  noInterrupts();
  e1 = encoder1Count;
  e2 = encoder2Count;
  interrupts();
}

// Update odometry using safe encoder copy
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
  float dS  = (dD1 + dD2) * 0.5f;
  float dT  = (dD2 - dD1) / WHEEL_BASE_MM;

  float midTheta = posTheta + dT * 0.5f;
  posX += dS * cosf(midTheta);
  posY += dS * sinf(midTheta);
  posTheta += dT;

  while (posTheta > PI_F) posTheta -= 2.0f * PI_F;
  while (posTheta < -PI_F) posTheta += 2.0f * PI_F;
}

// Safe pose read
void safeReadPose(float &x, float &y, float &t) {
  noInterrupts();
  x = posX;
  y = posY;
  t = posTheta;
  interrupts();
}

// Heartbeat: send pose via SPI and TCP (if connected)
void sendHeartbeat() {
  float x, y, th;
  safeReadPose(x, y, th);

  char buf[80];
  int n = snprintf(buf, sizeof(buf), "<P,X:%.2f,Y:%.2f,T:%.3f>", x, y, th);
  if (n > 0 && n < (int)sizeof(buf)) {
    // SPI
    sendCommandString(buf);
    // TCP
    if (client && client.connected()) {
      client.print(buf);
      client.print("\n");
      Serial.println("[WiFi] Sent heartbeat to server");
    } else {
      Serial.println("[WiFi] Not connected to server, skipping send");
    }
  } else {
    Serial.println("[ERROR] Heartbeat frame truncated or snprintf failed.");
  }
}

// ... (Global variables)
bool isTargetMode = false;

// ... (existing functions)

void loop() {
  uint32_t now = millis();

  // ... (WiFi/TCP logic)

  // Target Mode Logic (Check posX)
  if (isTargetMode) {
      if (posX >= 1000.0) {
          Serial.println("[Target] Reached X=1000mm! Stopping.");
          sendCommandString("<x>"); // Stop command
          isTargetMode = false;
      }
  }

  // Receive from TCP Server -> Send to Mega (SPI)
  if (client.connected() && client.available() > 0) {
    String tcpLine = client.readStringUntil('\n'); // Read line from server
    tcpLine.trim(); // Remove whitespace
    if (tcpLine.length() > 0) {
      Serial.print("[TCP] Recv: ");
      Serial.println(tcpLine);
      
      if (tcpLine == "1") {
          Serial.println("[CMD] Start Move to X=1000");
          // Reset X (Optional? Or just rely on current X?)
          // Usually we want to move +1000 from current, or absolute 1000?
          // Let's assume absolute X=1000 destination.
          // Or reset Pose to 0? Let's reset Pose for simplicity if needed.
          // For now, just send 'w' to start moving and enable target check.
          
          // Reset Pose (옵션: 1번 누를 때마다 0부터 다시 시작하려면 주석 해제)
          noInterrupts();
          posX = 0.0; posY = 0.0; posTheta = 0.0;
          encoder1Count = 0; encoder2Count = 0;
          lastEnc1 = 0; lastEnc2 = 0;
          interrupts();

          sendCommandString("<w>"); // Forward command
          isTargetMode = true;
      } 
      else if (tcpLine.length() == 1) {
          char buf[16];
          snprintf(buf, sizeof(buf), "<%c>", tcpLine[0]);
          sendCommandString(buf); 
          isTargetMode = false; // Manual command cancels target mode
      } else {
          sendCommandString(tcpLine);
          isTargetMode = false;
      }
      lastSendTime = now; 
    }
  }

  // Serial input (Manual) : Same logic for '1' via Serial
  if (Serial.available() > 0) {
    char userCmd = Serial.read();
    if (userCmd != '\n' && userCmd != '\r') {
      Serial.print("[USER] Manual Input: ");
      Serial.println(userCmd);
      
      if (userCmd == '1') {
          Serial.println("[CMD] Start Move to X=1000");
          noInterrupts();
          posX = 0.0; posY = 0.0; posTheta = 0.0;
          encoder1Count = 0; encoder2Count = 0;
          lastEnc1 = 0; lastEnc2 = 0;
          interrupts();
          sendCommandString("<w>");
          isTargetMode = true;
      } else {
          char buf[16];
          snprintf(buf, sizeof(buf), "<%c>", userCmd);
          sendCommandString(buf);
          lastSendTime = now;
          isTargetMode = false;
      }
    }
  }

  // Update odometry
  updateOdometry();
// ...

  // Heartbeat
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    sendHeartbeat();

    long e1, e2;
    safeReadEncoders(e1, e2);
    Serial.print("Encoder counts - ENC1: ");
    Serial.print(e1);
    Serial.print(", ENC2: ");
    Serial.println(e2);
  }

  yield();
}