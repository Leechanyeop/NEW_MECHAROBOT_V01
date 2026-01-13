/**
 * @file CTL_ESP32.ino
 * @brief ESP32 SPI Master - Arduino Mega Motor Controller + 4 Encoders
 * @version 1.1
 * 
 * ESP32가 SPI 마스터로 동작하여 Arduino Mega(Slave)에
 * 모터 제어 명령을 전송하고, 4개의 엔코더 값을 직접 읽습니다.
 * 
 * SPI 연결:
 *   ESP32     Arduino Mega
 *   -----     ------------
 *   GPIO18 -> SCK  (52)
 *   GPIO23 -> MOSI (51)
 *   GPIO19 <- MISO (50)
 *   GPIO5  -> SS   (53)
 *   GND    -- GND
 * 
 * 엔코더 연결:
 *   Encoder 1: GPIO16(A), GPIO17(B) - 왼쪽 앞
 *   Encoder 2: GPIO12(A), GPIO13(B) - 오른쪽 앞
 *   Encoder 3: GPIO14(A), GPIO25(B) - 왼쪽 뒤
 *   Encoder 4: GPIO26(A), GPIO27(B) - 오른쪽 뒤
 */

#include <SPI.h>

// ============== SPI 핀 설정 (ESP32 기본 VSPI) ==============
#define SPI_SCK   18
#define SPI_MOSI  23
#define SPI_MISO  19
#define SPI_SS    5

// ============== 엔코더 핀 설정 (ESP32) ==============
#define ENCODER_1A 16   // 엔코더 1 - A상
#define ENCODER_1B 17   // 엔코더 1 - B상
#define ENCODER_2A 12   // 엔코더 2 - A상
#define ENCODER_2B 13   // 엔코더 2 - B상
#define ENCODER_3A 14   // 엔코더 3 - A상
#define ENCODER_3B 25   // 엔코더 3 - B상
#define ENCODER_4A 26   // 엔코더 4 - A상
#define ENCODER_4B 27   // 엔코더 4 - B상

// ============== 명령어 정의 ==============
#define CMD_FORWARD     'w'
#define CMD_BACKWARD    's'
#define CMD_LEFT        'a'
#define CMD_RIGHT       'd'
#define CMD_STOP        'x'
#define CMD_SPEED_UP    '+'
#define CMD_SPEED_DOWN  '-'
#define CMD_LINE_TRACE  'l'
#define CMD_LINE_PID    'p'

// ============== 전역 변수 ==============
SPIClass *vspi = NULL;
char currentCommand = CMD_STOP;

// 엔코더 카운트 (volatile - 인터럽트에서 사용)
volatile long encoderCount[4] = {0, 0, 0, 0};

// 엔코더 핀 배열
const int encoderPinsA[4] = {ENCODER_1A, ENCODER_2A, ENCODER_3A, ENCODER_4A};
const int encoderPinsB[4] = {ENCODER_1B, ENCODER_2B, ENCODER_3B, ENCODER_4B};

// ============== 함수 선언 ==============
void sendCommand(char cmd);
void processSerialCommand();
void printEncoderValues();
void resetEncoders();

// ============== 엔코더 인터럽트 핸들러 ==============
void IRAM_ATTR encoder1ISR() {
  if (digitalRead(ENCODER_1A) == digitalRead(ENCODER_1B)) {
    encoderCount[0]++;
  } else {
    encoderCount[0]--;
  }
}

void IRAM_ATTR encoder2ISR() {
  if (digitalRead(ENCODER_2A) == digitalRead(ENCODER_2B)) {
    encoderCount[1]++;
  } else {
    encoderCount[1]--;
  }
}

void IRAM_ATTR encoder3ISR() {
  if (digitalRead(ENCODER_3A) == digitalRead(ENCODER_3B)) {
    encoderCount[2]++;
  } else {
    encoderCount[2]--;
  }
}

void IRAM_ATTR encoder4ISR() {
  if (digitalRead(ENCODER_4A) == digitalRead(ENCODER_4B)) {
    encoderCount[3]++;
  } else {
    encoderCount[3]--;
  }
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP32 SPI Master + 4 Encoders ===");
  Serial.println("Commands:");
  Serial.println("  w: Forward    s: Backward");
  Serial.println("  a: Left       d: Right");
  Serial.println("  x: Stop");
  Serial.println("  +: Speed Up   -: Speed Down");
  Serial.println("  l: Line Trace (Simple)");
  Serial.println("  p: Line Trace (PID)");
  Serial.println("  e: Print Encoders");
  Serial.println("  r: Reset Encoders");
  Serial.println("======================================");
  
  // SPI 초기화 (VSPI)
  vspi = new SPIClass(VSPI);
  vspi->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_SS);
  
  // SS 핀 설정
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  
  // 엔코더 핀 설정 및 인터럽트 연결
  for (int i = 0; i < 4; i++) {
    pinMode(encoderPinsA[i], INPUT_PULLUP);
    pinMode(encoderPinsB[i], INPUT_PULLUP);
  }
  
  // 각 엔코더에 인터럽트 연결 (A상 CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3A), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_4A), encoder4ISR, CHANGE);
  
  Serial.println("[ESP32] SPI Master + Encoders Ready!");
}

// ============== Main Loop ==============
void loop() {
  // 시리얼 명령 처리
  processSerialCommand();
  
  // 엔코더 값 주기적 출력 (100ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    printEncoderValues();
    lastPrint = millis();
  }
}

// ============== SPI 명령 전송 ==============
void sendCommand(char cmd) {
  vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_SS, LOW);
  delayMicroseconds(10);
  
  vspi->transfer(cmd);
  
  delayMicroseconds(10);
  digitalWrite(SPI_SS, HIGH);
  vspi->endTransaction();
  
  currentCommand = cmd;
  
  Serial.print("[ESP32] Sent: ");
  Serial.println(cmd);
}

// ============== 엔코더 값 출력 ==============
void printEncoderValues() {
  Serial.printf("ENC[1]:%6ld  [2]:%6ld  [3]:%6ld  [4]:%6ld\n",
                encoderCount[0], encoderCount[1], 
                encoderCount[2], encoderCount[3]);
}

// ============== 엔코더 리셋 ==============
void resetEncoders() {
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    encoderCount[i] = 0;
  }
  interrupts();
  Serial.println("[ESP32] Encoders Reset!");
}

// ============== 개별 엔코더 값 읽기 ==============
long getEncoderCount(int index) {
  if (index < 0 || index >= 4) return 0;
  noInterrupts();
  long val = encoderCount[index];
  interrupts();
  return val;
}

// ============== 시리얼 명령 처리 ==============
void processSerialCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'w': case 'W':
        sendCommand(CMD_FORWARD);
        Serial.println(">> Forward");
        break;
        
      case 's': case 'S':
        sendCommand(CMD_BACKWARD);
        Serial.println(">> Backward");
        break;
        
      case 'a': case 'A':
        sendCommand(CMD_LEFT);
        Serial.println(">> Turn Left");
        break;
        
      case 'd': case 'D':
        sendCommand(CMD_RIGHT);
        Serial.println(">> Turn Right");
        break;
        
      case 'x': case 'X':
        sendCommand(CMD_STOP);
        Serial.println(">> Stop");
        break;
        
      case '+': case '=':
        sendCommand(CMD_SPEED_UP);
        Serial.println(">> Speed Up");
        break;
        
      case '-': case '_':
        sendCommand(CMD_SPEED_DOWN);
        Serial.println(">> Speed Down");
        break;
        
      case 'l': case 'L':
        sendCommand(CMD_LINE_TRACE);
        Serial.println(">> Line Trace (Simple)");
        break;
        
      case 'p': case 'P':
        sendCommand(CMD_LINE_PID);
        Serial.println(">> Line Trace (PID)");
        break;
        
      case 'e': case 'E':  // 엔코더 값 출력
        Serial.println("\n=== Encoder Values ===");
        Serial.printf("Encoder 1 (Front-L): %ld\n", encoderCount[0]);
        Serial.printf("Encoder 2 (Front-R): %ld\n", encoderCount[1]);
        Serial.printf("Encoder 3 (Rear-L):  %ld\n", encoderCount[2]);
        Serial.printf("Encoder 4 (Rear-R):  %ld\n", encoderCount[3]);
        Serial.println("======================\n");
        break;
        
      case 'r': case 'R':  // 엔코더 리셋
        resetEncoders();
        break;
        
      case '?':
        Serial.println("\n=== Commands ===");
        Serial.println("w/s: Forward/Backward");
        Serial.println("a/d: Left/Right");
        Serial.println("x: Stop");
        Serial.println("+/-: Speed Up/Down");
        Serial.println("l/p: LineTrace/PID");
        Serial.println("e: Print Encoders");
        Serial.println("r: Reset Encoders");
        Serial.println("================\n");
        break;
        
      default:
        break;
    }
  }
}

// ============== 자동 주행 예시 ==============
void autoRun() {
  sendCommand(CMD_FORWARD);
  delay(2000);
  
  sendCommand(CMD_LEFT);
  delay(500);
  
  sendCommand(CMD_FORWARD);
  delay(2000);
  
  sendCommand(CMD_RIGHT);
  delay(500);
  
  sendCommand(CMD_STOP);
  delay(1000);
}

// ============== 연속 라인트레이싱 ==============
void continuousLineTrace() {
  while (true) {
    sendCommand(CMD_LINE_PID);
    delay(50);
    
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        sendCommand(CMD_STOP);
        break;
      }
    }
  }
}
