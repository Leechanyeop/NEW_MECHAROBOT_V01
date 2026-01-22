#include <WiFi.h>
#include <SPI.h>

/**
 * @file ESP32_Master.ino
 * @brief ESP32 Master: Send command '1' to Mega Slave via SPI
 * 
 * Wiring (ESP32 -> Mega 2560):
 * - SS (GPIO 5)   -> Pin 53 (SS)
 * - MOSI (GPIO 23) -> Pin 51 (MOSI)
 * - MISO (GPIO 19) -> Pin 50 (MISO)
 * - SCK (GPIO 18)  -> Pin 52 (SCK)
 * - GND           -> GND (Essential!)
 */

// ============== WiFi Settings ==============
const char* ssid = "hhme";            
const char* password = "hme*12345";   

// ============== SPI Settings ==============
const int chipSelectPin = 5;

// ============== Encoder Pins ==============
const int enc1A = 36;
const int enc1B = 39;
const int enc2A = 34;
const int enc2B = 35;

// ============== Global Variables ==============
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 500; // Send command every 500ms (to keep watchdog alive)


WiFiClient client = server.available();

// ============== Interrupt Service Routines (ISR) ==============
void IRAM_ATTR isr1A() {
  int a = digitalRead(enc1A);
  int b = digitalRead(enc1B);
  if (a == b) encoder1Count++;
  else encoder1Count--;
}

void IRAM_ATTR isr2A() {
  int a = digitalRead(enc2A);
  int b = digitalRead(enc2B);
  if (a == b) encoder2Count++;
  else encoder2Count--;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. WiFi Setup
  Serial.printf("\nConnecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  // 2. Encoder Pin Setup
  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);

  // 3. SPI Master Setup
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();
  // SPI settings for Mega: 1MHz, MSBFIRST, SPI_MODE0
  
  Serial.println("\n=== ESP32 SPI Master Ready ===");
  Serial.println("Target: Mega 2560 Slave");
  Serial.println("Control via Serial Monitor (Input char to send)");
}

void sendCommand(char cmd) {
  // 슬레이브(Mega)와의 안정적인 통신을 위해 Transaction과 Delay를 추가합니다.
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0)); 
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer('<');
  delayMicroseconds(30); // 메가가 인터럽트를 처리할 시간을 줍니다. (20us -> 30us로 상향)
  SPI.transfer(cmd);
  delayMicroseconds(30);
  SPI.transfer('>');
  
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  
  Serial.print("[SPI] Sent: <");
  Serial.print(cmd);
  Serial.println(">");
}

void loop() {
  uint32_t now = millis();

  if (client) { // 클라이언트가 접속해 있다면
      if (client.connected() && client.available()) { // 데이터가 들어왔다면
        char wifiCmd = client.read(); // 데이터 읽기
        
        // 줄바꿈 문자 무시 및 명령 전송
        if (wifiCmd != '\n' && wifiCmd != '\r') {
          Serial.print("[WiFi] Remote Input: ");
          Serial.println(wifiCmd);
          sendCommand(wifiCmd); // <--- 기존 명령 함수 재사용!
          lastSendTime = now;   // 하트비트 타이머 리셋
        }
      }
    }

  // 1. 시리얼 모니터 수동 명령 입력
  if (Serial.available() > 0) {
    char userCmd = Serial.read();
    if (userCmd != '\n' && userCmd != '\r') {
      Serial.print("[USER] Manual Input: ");
      Serial.println(userCmd);
      sendCommand(userCmd);
      lastSendTime = now;
    }
  }

  // 2. 하트비트 전송 (워치독 방지)
  // 500ms 간격으로 마지막 명령 또는 'x'를 보냅니다.
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    
    // 워치독을 깨우기 위한 heartbeat 전송
    sendCommand('x'); // 기본적으로 정지 신호를 보내 하트비트 유지

    // 엔코더 디버그 출력
    Serial.print("Encoder counts - ENC1: ");
    Serial.print(encoder1Count);
    Serial.print(", ENC2: ");
    Serial.println(encoder2Count);
  }

  delay(1);
}
