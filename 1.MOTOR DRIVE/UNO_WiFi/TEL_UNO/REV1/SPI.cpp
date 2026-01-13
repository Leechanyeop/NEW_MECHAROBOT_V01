/**
 * @file CTL_UNOWiFi.ino
 * @brief Arduino UNO WiFi Rev2 SPI Master - Motor Controller + 2 Encoders
 * @version 1.1
 * 
 * [중요: UNO WiFi Rev2 배선 주의사항]
 * UNO WiFi Rev2는 일반 UNO와 달리 디지털 11, 12, 13번 핀에 SPI가 연결되어 있지 않습니다.
 * 반드시 보드 중앙의 6핀 ICSP 헤더를 사용해야 합니다.
 * 
 * SPI 연결 (ICSP 헤더 위에서 보았을 때):
 *   (1) MISO [● 1] [2 ●] VCC      (3) SCK  [● 3] [4 ●] MOSI
 *   (5) RESET[● 5] [6 ●] GND       * SS는 디지털 핀 사용
 * 
 * 배선표:
 *   UNO WiFi Rev2 ICSP     Arduino Mega 2560
 *   ------------------     -----------------
 *   Pin 3 (SCK)        ->  Pin 52 (SCK)
 *   Pin 4 (MOSI)       ->  Pin 51 (MOSI)
 *   Pin 1 (MISO)       <-  Pin 50 (MISO)
 *   Pin 8 (SS)         ->  Pin 53 (SS)
 *   Pin 6 (GND)        --  GND (공통)
 * 
 * 엔코더 연결 (인터럽트 지원 핀):
 *   Encoder 1: Pin 2 (A), Pin 4 (B)
 *   Encoder 2: Pin 3 (A), Pin 5 (B)
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoRS485.h> // ArduinoModbus 의존성
#include <ArduinoModbus.h>
#include "SendOPC.h"


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
  delay(1000);
  
  Serial.println(F("=== UNO WiFi SPI Master + 2 Encoders ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  w: Forward    s: Backward"));
  Serial.println(F("  a: Left       d: Right"));
  Serial.println(F("  x: Stop"));
  Serial.println(F("  +: Speed Up   -: Speed Down"));
  Serial.println(F("  l: Line Trace (Simple)"));
  Serial.println(F("  p: Line Trace (PID)"));
  Serial.println(F("  e: Print Encoders"));
  Serial.println(F("  r: Reset Encoders"));
  Serial.println(F("========================================"));
  
  // SPI 초기화
  SPI.begin();
  
  // SS 핀 설정
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  
  Serial.print(F("[MASTER] SS Pin = "));
  Serial.println(SPI_SS);
  Serial.print(F("[MASTER] SS State = "));
  Serial.println(digitalRead(SPI_SS) ? "HIGH" : "LOW");
  
  // SS 핀 5회 토글 테스트 (LED 확인용)
  Serial.println(F("[TEST] Toggling SS 5 times..."));
  for (int i = 0; i < 5; i++) {
    digitalWrite(SPI_SS, LOW);
    Serial.println(F("  SS -> LOW"));
    delay(500);
    digitalWrite(SPI_SS, HIGH);
    Serial.println(F("  SS -> HIGH"));
    delay(500);
  }
  Serial.println(F("[TEST] SS toggle complete!"));
  
  // 엔코더 핀 설정
  pinMode(ENCODER_1A, INPUT_PULLUP);
  pinMode(ENCODER_1B, INPUT_PULLUP);
  pinMode(ENCODER_2A, INPUT_PULLUP);
  pinMode(ENCODER_2B, INPUT_PULLUP);
  
  // 인터럽트 연결 (A상 CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), encoder2ISR, CHANGE);
  
  // WiFi 연결 시도
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("[WiFi] WiFi shield not present"));
  } else {
    while (status != WL_CONNECTED) {
      Serial.print(F("[WiFi] Attempting to connect to SSID: "));
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      delay(5000); // 5초 대기
    }
    server.begin();
    
    // Modbus 서버 시작
    if (!modbusServer.begin()) {
      Serial.println(F("[Modbus] Failed to start Modbus TCP Server!"));
    } else {
      // 레지스터 구성
      // Holding Registers: 40001(Speed), 40002(Command)
      modbusServer.configureHoldingRegisters(0x00, 10); 
      // Input Registers: 30001-30004(Encoders)
      modbusServer.configureInputRegisters(0x00, 10);
      
      // 초기값 설정
      modbusServer.holdingRegisterWrite(0, DEFAULT_SPEED); 
      Serial.println("[Modbus SRV] TCP Server Started on Port 502");
    }

    // 외부 OPC 서버(Modbus Client) 연결 시도
    Serial.print("[Modbus CLI] Connecting to OPC Server: ");
    Serial.print(serverIP);
    Serial.print(":");
    Serial.println(serverPort);
    
    if (!modbusTCPClient.begin(serverIP, serverPort)) {
      Serial.println(F("[Modbus CLI] Failed to connect to OPC Server!"));
    } else {
      Serial.println(F("[Modbus CLI] Connected to OPC Server!"));
      detectRegisterOffset(); // 오프셋 자동 감지
    }
    
    Serial.println(F("[WiFi] Connected to Network!"));
    Serial.print(F("[WiFi] IP Address: "));
    Serial.println(WiFi.localIP());
  }

  Serial.println(F("[UNO] SPI Master + Encoders Ready!"));
  
  // SPI 실제 전송 테스트 (5회)
  Serial.println(F("[TEST] Sending 5 SPI test packets ('T')..."));
  Serial.println(F("[!] Make sure to use ICSP header for SCK(3), MOSI(4), MISO(1)"));
  delay(1000);  // Slave 준비 대기
  
  for (int i = 0; i < 5; i++) {
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_SS, LOW);
    delayMicroseconds(100);
    byte response = SPI.transfer('T');  // 테스트 문자 전송
    delayMicroseconds(100);
    digitalWrite(SPI_SS, HIGH);
    SPI.endTransaction();
    
    Serial.print(F("  Sent 'T', Response from Slave: 0x"));
    Serial.println(response, HEX);
    delay(500);
  }
  Serial.println(F("[TEST] SPI test complete! Check Slave serial if it received 'T'."));
}

// ============== Main Loop ==============
void loop() {
  // 시리얼 명령 처리
  processSerialCommand();
  
  // WiFi 명령 처리
  processWiFiCommand();
  
  // Modbus 명령 처리
  processModbus();
  
  // 주기적 엔코더 값 출력 (100ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    printEncoderValues();
    lastPrint = millis();
  }

  // 하트비트 전송 (500ms 주기)
  // 메가의 워치독(700ms) 유지를 위해 현재 명령을 다시 보냄
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 500) {
    sendCommand(currentCommand);
    lastHeartbeat = millis();
  }

  // 외부 OPC 서버로 데이터 전송 (500ms 주기)
  static unsigned long lastOPCSend = 0;
  if (millis() - lastOPCSend >= 500) {
    sendDataToOPC();
    lastOPCSend = millis();
  }
}

// ============== WiFi TCP 서버 명령 처리 (CMD 환경용) ==============
void processWiFiCommand() {
  WiFiClient client = server.available();
  if (client) {
    if (client.connected()) {
      while (client.available()) {
        char cmd = client.read();
        
        // 수신된 문자가 유효한 명령어인지 확인
        if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd' || cmd == 'x' || 
            cmd == '+' || cmd == '-' || cmd == 'l' || cmd == 'p') {
          sendCommand(cmd);
          client.print("ACK: "); client.println(cmd); // 통신 확인용 응답
        } else if (cmd == '\r' || cmd == '\n') {
          // 개행 문자는 무시
        } else {
          client.print("ERR: Invalid CMD '"); client.print(cmd); client.println("'");
        }
      }
    }
  }
}

// ============== Modbus TCP 서버 처리 ==============
void processModbus() {
  modbusServer.poll();

  // 1. 엔코더 값 입력 레지스터 업데이트 (30001~30004)
  long encL = getEncoderCount(0);
  long encR = getEncoderCount(1);
  
  modbusServer.inputRegisterWrite(0, (encL >> 16) & 0xFFFF); // High 16-bit
  modbusServer.inputRegisterWrite(1, encL & 0xFFFF);        // Low 16-bit
  modbusServer.inputRegisterWrite(2, (encR >> 16) & 0xFFFF);
  modbusServer.inputRegisterWrite(3, encR & 0xFFFF);

  // 2. 명령 제어 (40002번 레지스터 감시)
  static uint16_t lastModbusCmd = 0;
  uint16_t currentModbusCmd = modbusServer.holdingRegisterRead(1);
  
  if (currentModbusCmd != lastModbusCmd) {
    switch (currentModbusCmd) {
      case 0: sendCommand(CMD_STOP); break;
      case 1: sendCommand(CMD_FORWARD); break;
      case 2: sendCommand(CMD_BACKWARD); break;
      case 3: sendCommand(CMD_LEFT); break;
      case 4: sendCommand(CMD_RIGHT); break;
      case 5: sendCommand(CMD_LINE_TRACE); break;
      case 6: sendCommand(CMD_LINE_PID); break;
    }
    lastModbusCmd = currentModbusCmd;
    Serial.print(F("[Modbus] New Command: "));
    Serial.println(currentModbusCmd);
  }

  // 3. 속도 제어 (40001번 레지스터 감시)
  static uint16_t lastModbusSpeed = 0;
  uint16_t currentModbusSpeed = modbusServer.holdingRegisterRead(0);
  if (currentModbusSpeed != lastModbusSpeed) {
    lastModbusSpeed = currentModbusSpeed;
    // 참고: 여기서 Mega로 PWM 값을 직접 보내는 SPI 프로토콜을 추가하면 더 정확한 제어가 가능합니다.
  }
}

// ============== SPI 명령 전송 ==============
void sendCommand(char cmd) {
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(SPI_SS, LOW);
  delayMicroseconds(100);
  
  // 3바이트 패킷 전송: [HEADER('<')] [CMD] [FOOTER('>')]
  SPI.transfer('<');
  delayMicroseconds(50);
  char response = (char)SPI.transfer(cmd);
  delayMicroseconds(50);
  SPI.transfer('>');
  
  delayMicroseconds(100); 
  digitalWrite(SPI_SS, HIGH);
  SPI.endTransaction();
  
  currentCommand = cmd;
  
  Serial.print(F("[UNO] Sent: <"));
  Serial.print(cmd);
  Serial.print(F("> | ACK: '"));
  Serial.print(response);
  Serial.println(F("'"));
}

// ============== 엔코더 값 출력 ==============
void printEncoderValues() {
  Serial.print(F("ENC[L]:"));
  Serial.print(encoderCount[0]);
  Serial.print(F("  [R]:"));
  Serial.println(encoderCount[1]);
}

// ============== 엔코더 리셋 ==============
void resetEncoders() {
  noInterrupts();
  encoderCount[0] = 0;
  encoderCount[1] = 0;
  interrupts();
  Serial.println(F("[UNO] Encoders Reset!"));
}

// ============== 개별 엔코더 값 읽기 ==============
long getEncoderCount(int index) {
  if (index < 0 || index >= 2) return 0;
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
        Serial.println(F(">> Forward"));
        break;
        
      case 's': case 'S':
        sendCommand(CMD_BACKWARD);
        Serial.println(F(">> Backward"));
        break;
        
      case 'a': case 'A':
        sendCommand(CMD_LEFT);
        Serial.println(F(">> Turn Left"));
        break;
        
      case 'd': case 'D':
        sendCommand(CMD_RIGHT);
        Serial.println(F(">> Turn Right"));
        break;
        
      case 'x': case 'X':
        sendCommand(CMD_STOP);
        Serial.println(F(">> Stop"));
        break;
        
      case '+': case '=':
        sendCommand(CMD_SPEED_UP);
        Serial.println(F(">> Speed Up"));
        break;
        
      case '-': case '_':
        sendCommand(CMD_SPEED_DOWN);
        Serial.println(F(">> Speed Down"));
        break;
        
      case 'l': case 'L':
        sendCommand(CMD_LINE_TRACE);
        Serial.println(F(">> Line Trace (Simple)"));
        break;
        
      case 'p': case 'P':
        sendCommand(CMD_LINE_PID);
        Serial.println(F(">> Line Trace (PID)"));
        break;
        
      case 'e': case 'E':
        Serial.println(F("\n=== Encoder Values ==="));
        Serial.print(F("Encoder 1 (Left):  "));
        Serial.println(encoderCount[0]);
        Serial.print(F("Encoder 2 (Right): "));
        Serial.println(encoderCount[1]);
        Serial.println(F("======================\n"));
        break;
        
      case 'r': case 'R':
        resetEncoders();
        break;
        
      case '?':
        Serial.println(F("\n=== Commands ==="));
        Serial.println(F("w/s: Forward/Backward"));
        Serial.println(F("a/d: Left/Right"));
        Serial.println(F("x: Stop"));
        Serial.println(F("+/-: Speed Up/Down"));
        Serial.println(F("l/p: LineTrace/PID"));
        Serial.println(F("e: Print Encoders"));
        Serial.println(F("r: Reset Encoders"));
        Serial.println(F("================\n"));
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

// ============== 외부 OPC 서버(Modbus Client) 데이터 전송 ==============
void sendDataToOPC() {
  // 연결 확인 및 재연결
  if (!modbusTCPClient.connected()) {
    Serial.println(F("[Modbus CLI] Connection lost. Reconnecting..."));
    modbusTCPClient.stop();
    if (modbusTCPClient.begin(serverIP, serverPort)) {
      Serial.println(F("[Modbus CLI] Reconnected!"));
      detectRegisterOffset();
    } else {
      Serial.println(F("[Modbus CLI] Reconnection failed!"));
      return;
    }
  }

  // 데이터 수집
  long encL = getEncoderCount(0);
  long encR = getEncoderCount(1);
  
  // OPC 서버로 전송
  // 1. 엔코더 L (40003, 40004)
  writeFloatToRegisters(REG_ENC_L_LOW, (float)encL);
  
  // 2. 엔코더 R (40005, 40006)
  writeFloatToRegisters(REG_ENC_R_LOW, (float)encR);
  
  // 3. 현재 상태 (40008)
  writeRegisterVerify(reg(REG_STATUS), (uint16_t)currentCommand);
  
  Serial.println(F("[Modbus CLI] Data sent to OPC Server"));
}
