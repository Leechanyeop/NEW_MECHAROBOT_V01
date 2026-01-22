/**
 * @file CTL_UNOWiFi.ino
 * @brief UNO WiFi Rev2 Main Controller
 * 
 * 기능:
 * 1. WiFi TCP 서버 (8888번) -> PC에서 'w' 등을 보내면 로봇 제어
 * 2. SPI 마스터 -> Mega 2560으로 명령 전송
 * 3. 엔코더 값 항시 출력 (시리얼 모니터)
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoModbus.h>

// ============== WiFi 설정 ==============
char ssid[] = "hhme";       // 와이파이 이름
char pass[] = "hme*12345";  // 와이파이 비밀번호
int status = WL_IDLE_STATUS;

WiFiServer server(8888);        // 8888번 포트 TCP 서버 (CMD 제어용)
ModbusTCPServer modbusServer;   // Modbus TCP 서버 (포트 502) - 필요한 경우 사용

// ============== 기본 설정 ==============
#define DEFAULT_SPEED 50

// ============== SPI 핀 설정 (Arduino UNO WiFi Rev2) ==============
#define SPI_SS    8 // Slave Select (표준 SPI SS 핀)

// ============== 엔코더 핀 설정 (UNO 인터럽트 핀 2, 3) ==============
#define ENCODER_1A 2   // 엔코더 1 - A상 (인터럽트 0)
#define ENCODER_1B 4   // 엔코더 1 - B상
#define ENCODER_2A 3   // 엔코더 2 - A상 (인터럽트 1)
#define ENCODER_2B 5   // 엔코더 2 - B상

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
char currentCommand = CMD_STOP;
volatile long encoderCount[2] = {0, 0}; // 엔코더 카운트

// ============== 함수 선언 ==============
void sendCommand(char cmd);
void processSerialCommand();
void processWiFiCommand();
void printEncoderValues();
void resetEncoders();

// ============== 엔코더 인터럽트 핸들러 ==============
void encoder1ISR() {
  if (digitalRead(ENCODER_1A) == digitalRead(ENCODER_1B)) {
    encoderCount[0]++;
  } else {
    encoderCount[0]--;
  }
}

void encoder2ISR() {
  if (digitalRead(ENCODER_2A) == digitalRead(ENCODER_2B)) {
    encoderCount[1]++;
  } else {
    encoderCount[1]--;
  }
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  
  // 1. SPI 초기화 (표준 SPI 라이브러리)
  SPI.begin();
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  
  // 2. 엔코더 설정
  pinMode(ENCODER_1A, INPUT_PULLUP);
  pinMode(ENCODER_1B, INPUT_PULLUP);
  pinMode(ENCODER_2A, INPUT_PULLUP);
  pinMode(ENCODER_2B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), encoder2ISR, CHANGE);

  // 3. WiFi 연결
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module failed!");
    while (true);
  }

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin(); // TCP 서버 (8888) 시작
  Serial.println("Server 8888 Started (Use 'w','a','s','d' to control)");
}

// ============== Main Loop ==============
void loop() {
  // 1. WiFi 명령 수신 및 처리
  processWiFiCommand();

  // 2. 시리얼 명령 수신 및 처리 (PC 직접 연결 시)
  processSerialCommand();

  // 3. 엔코더 값 항시 출력 (약 500ms 주기)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    printEncoderValues();
    lastPrint = millis();
  }
}

// ============== Modbus Client 설정 (서버로 데이터 전송) ==============
// ============== Modbus Client 설정 (서버로 데이터 전송) ==============
WiFiClient opcWifiClient;
ModbusTCPClient modbusTCPClient(opcWifiClient);
IPAddress serverIP(192, 168, 0, 19); // 서버 IP
uint16_t serverPort = 49320;         // 서버 포트 (uint16_t로 수정)

// ============== 함수 구현 ==============

// WiFi 명령 처리
void processWiFiCommand() {
  WiFiClient client = server.available();
  if (client) {
    if (client.connected()) {
      Serial.println("[WiFi] Client Connected");
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          // 유효한 명령이면 Mega로 전송
          if (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'x' || c == 'l' || c == 'p') {
            sendCommand(c);
            client.print("ACK: ");
            client.println(c);
          }
        }
        // 엔코더 값 출력 처리 등을 위해 루프 탈출
        if (millis() % 100 == 0) break; 
      }
    }
  }
}

// 시리얼 명령 처리
void processSerialCommand() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'x' || c == 'l' || c == 'p') {
      sendCommand(c);
    }
  }
}

// SPI 명령 전송 (Mega와 호환)
void sendCommand(char cmd) {
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(SPI_SS, LOW);
  delayMicroseconds(100);
  
  SPI.transfer('<');        // Header
  delayMicroseconds(50);
  SPI.transfer(cmd);        // Command
  delayMicroseconds(50);
  SPI.transfer('>');        // Footer
  
  delayMicroseconds(100); 
  digitalWrite(SPI_SS, HIGH);
  SPI.endTransaction();
  
  currentCommand = cmd;
  Serial.print("[Sent] ");
  Serial.println(cmd);
}

// 엔코더 값 출력 (값이 변했을 때만 출력)
void printEncoderValues() {
  noInterrupts();
  long e1 = encoderCount[0];
  long e2 = encoderCount[1];
  interrupts();
  
  static long lastE1 = -99999;
  static long lastE2 = -99999;
  
  // 값이 변하지 않았으면 리턴 (출력 안함)
  if (e1 == lastE1 && e2 == lastE2) {
    return;
  }
  
  // 값이 변했으면 갱신하고 출력
  lastE1 = e1;
  lastE2 = e2;
  
  Serial.print("ENC_L: ");
  Serial.print(e1);
  Serial.print("\t ENC_R: ");
  Serial.println(e2);
  
  // OPC 서버로도 전송 시도
  sendDataToOPC();
}

void resetEncoders() { 
  noInterrupts();
  encoderCount[0] = 0;
  encoderCount[1] = 0;
  interrupts();
  Serial.println("Encoders Reset");
}

// 외부 OPC 서버로 데이터 전송
void sendDataToOPC() {
  if (!modbusTCPClient.connected()) {
    // 연결 시도 (블로킹 방지를 위해 매번 시도하지 않도록 할 수도 있음)
    static unsigned long lastConnectAttempt = 0;
    if (millis() - lastConnectAttempt > 5000) {
      Serial.print("[Modbus CLI] Connecting to ");
      Serial.print(serverIP);
      Serial.print(":");
      Serial.println(serverPort);
      
      if (!modbusTCPClient.begin(serverIP, serverPort)) {
        Serial.println("[Modbus CLI] Connect Failed!");
      } else {
        Serial.println("[Modbus CLI] Connected!");
      }
      lastConnectAttempt = millis();
    }
    return;
  }

  // 데이터 전송
  noInterrupts();
  long e1 = encoderCount[0];
  long e2 = encoderCount[1];
  interrupts();

  // 레지스터 0번(40001)에 L, 1번(40002)에 R 값을 씀
  if (!modbusTCPClient.holdingRegisterWrite(0, (int)e1)) {
    Serial.print("[Modbus] Write Fail (L) Error: ");
    Serial.println(modbusTCPClient.lastError());
  }
  if (!modbusTCPClient.holdingRegisterWrite(1, (int)e2)) {
    Serial.print("[Modbus] Write Fail (R) Error: ");
    Serial.println(modbusTCPClient.lastError());
  } else {
    // 성공 시 점(.) 출력해서 동작 확인
    Serial.print("."); 
  }
}
