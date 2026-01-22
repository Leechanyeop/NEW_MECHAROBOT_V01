#include <WiFi.h>
#include <SPI.h>

/**
 * @file ESP32_ModbusSlave.ino
 * @brief ESP32 Slave: Modbus TCP Server (OPC UA Master가 데이터 읽어감)
 */

// ============== WiFi Settings ==============
const char* ssid = "hhme";            
const char* password = "hme*12345";   
const int modbusPort = 502;

// ============== Hardware Pins ==============
const int chipSelectPin = 5;
const int enc1A = 36;
const int enc1B = 39;
const int enc2A = 34;
const int enc2B = 35;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

// ============== Robot Physical Constants ==============
const float WHEEL_DIAMETER_MM = 65.0;            
const float GEAR_RATIO = 30.0;                   
const int ENCODER_CPR = 11;                      
const float WHEEL_BASE_MM = 160.0; 
const float TICKS_PER_REV = (float)ENCODER_CPR * 2 * GEAR_RATIO;  //TICKS_PER_REV: 바퀴가 한 바퀴 돌 때 발생하는 엔코더 펄스 수
                                                                  //계산: $11(CPR) \times 2(상) \times 30(기어비) = 660$ 펄스/회전
float posX = 0.0, posY = 0.0, posTheta = 0.0;
long lastEnc1 = 0, lastEnc2 = 0;

// ============== Modbus Holding Registers ==============
// KEPServerEX Tag 매핑:
// 40001-40002: POS_X   (Float)   -> holdingRegisters[0-1]
// 40003-40004: POS_Y   (Float)   -> holdingRegisters[2-3]
// 40005-40006: POS_T   (Float)   -> holdingRegisters[4-5]
// 40007.0:     TargetA (Boolean) -> holdingRegisters[6] bit0
// 40100.20H:   Control (String)  -> holdingRegisters[99-108] (10 registers = 20 bytes)
// 40200.20H:   State   (String)  -> holdingRegisters[199-208] (10 registers = 20 bytes)

const int TOTAL_REGISTERS = 220;  // 충분한 크기 확보 상위 시스템(KEPServerEX)과 공유하는 공용 메모리 공간
uint16_t holdingRegisters[TOTAL_REGISTERS] = {0};

// ============== Register Index ==============
enum RegisterIndex {
  REG_POS_X_LO = 0,      // 40001
  REG_POS_X_HI = 1,      // 40002
  REG_POS_Y_LO = 2,      // 40003
  REG_POS_Y_HI = 3,      // 40004
  REG_POS_T_LO = 4,      // 40005
  REG_POS_T_HI = 5,      // 40006
  REG_TARGET_A = 6,      // 40007.0 - TargetA (Boolean)
  REG_CONTROL = 99,      // 40100.20H - Control (String, 10 registers)  상위 시스템(KEPServerEX)과 공유하는 공용 메모리 공간
  REG_STATE = 199        // 40200.20H - State (String, 10 registers) 상위 시스템(KEPServerEX)과 공유하는 공용 메모리 공간
};

const int STRING_LENGTH = 20;  // 20 bytes = 10 registers

WiFiServer modbusServer(modbusPort);
WiFiClient modbusClient;

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100;//위치 계산과 Modbus 데이터 업데이트를 **0.1초(100ms)**마다 수행

// ============== ISR ==============
void IRAM_ATTR isr1A() {
  static unsigned long lastT1 = 0;
  unsigned long now = micros();
  if (now - lastT1 < 50) return;
  int a = digitalRead(enc1A);
  ets_delay_us(5);
  if (a != digitalRead(enc1A)) return; 
  if (a == digitalRead(enc1B)) encoder1Count--; else encoder1Count++;
  lastT1 = now;
}

void IRAM_ATTR isr2A() {
  static unsigned long lastT2 = 0;
  unsigned long now = micros();
  if (now - lastT2 < 50) return; 
  int a = digitalRead(enc2A);
  ets_delay_us(5);
  if (a != digitalRead(enc2A)) return;
  if (a == digitalRead(enc2B)) encoder2Count++; else encoder2Count--;
  lastT2 = now;
}

// ============== Float -> Register 변환 (KEPware 호환) ==============
void floatToRegisters(float value, uint16_t* regs) {
  union {
    float f;
    uint8_t b[4];
  } data;
  
  data.f = value;
  
  // KEPware: Modbus Byte Order=Disable (Intel), First Word Low=Enable
  regs[0] = ((uint16_t)data.b[1] << 8) | data.b[0];  // Low word
  regs[1] = ((uint16_t)data.b[3] << 8) | data.b[2];  // High word
}

// ============== Boolean 비트 설정 ==============
void setBooleanBit(uint16_t* reg, int bitNum, bool value) {
  if (value) {
    *reg |= (1 << bitNum);
  } else {
    *reg &= ~(1 << bitNum);
  }
}

bool getBooleanBit(uint16_t reg, int bitNum) {
  return (reg & (1 << bitNum)) != 0;
}

// ============== String <-> Register 변환 (HiLo byte order) ==============
void stringToRegisters(const char* str, uint16_t* regs, int maxBytes) {
  int len = strlen(str);
  int regCount = maxBytes / 2;
  
  // 레지스터 초기화
  for (int i = 0; i < regCount; i++) {
    regs[i] = 0;
  }
  
  // HiLo byte order: 상위 바이트가 먼저
  for (int i = 0; i < maxBytes && i < len; i++) {
    int regIndex = i / 2;
    if (i % 2 == 0) {
      regs[regIndex] = (uint16_t)str[i] << 8;  // High byte
    } else {
      regs[regIndex] |= (uint16_t)str[i];      // Low byte
    }
  }
}

void registersToString(uint16_t* regs, char* str, int maxBytes) {
  int regCount = maxBytes / 2;
  
  // HiLo byte order: 상위 바이트가 먼저
  for (int i = 0; i < maxBytes; i++) {
    int regIndex = i / 2;
    if (i % 2 == 0) {
      str[i] = (char)(regs[regIndex] >> 8);    // High byte
    } else {
      str[i] = (char)(regs[regIndex] & 0xFF);  // Low byte
    }
  }
  str[maxBytes] = '\0';
  
  // 널 문자 이후 제거
  for (int i = 0; i < maxBytes; i++) {
    if (str[i] == '\0') break;
  }
}

// ============== Holding Registers 업데이트 ==============
void updateHoldingRegisters() {
  float x_m = posX / 1000.0f;
  float y_m = posY / 1000.0f;
  float theta_deg = posTheta * 180.0f / 3.141592f;
  
  floatToRegisters(x_m, &holdingRegisters[REG_POS_X_LO]);       // 40001-40002
  floatToRegisters(y_m, &holdingRegisters[REG_POS_Y_LO]);       // 40003-40004
  floatToRegisters(theta_deg, &holdingRegisters[REG_POS_T_LO]); // 40005-40006
  
  // TargetA, Control, State는 외부에서 Write된 값 유지
}

// ============== SPI Command ==============
void sendCommand(char cmd) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer('<');
  delayMicroseconds(40);
  SPI.transfer(cmd);
  delayMicroseconds(40);
  SPI.transfer('>');
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  
  if (cmd != 'x') {
    Serial.print("[SPI] Sent CMD: "); Serial.println(cmd);
  }
}

// ============== Modbus TCP 요청 처리 ==============
void processModbusRequest(WiFiClient& client) {
  uint8_t request[256];
  int len = client.read(request, sizeof(request));
  
  if (len < 8) return;
  
  uint16_t transactionId = (request[0] << 8) | request[1];
  uint8_t unitId = request[6];
  uint8_t functionCode = request[7];
  
  uint8_t response[256];
  int responseLen = 0;
  
  switch (functionCode) {
    case 0x03: {  // Read Holding Registers
      if (len < 12) return;
      uint16_t startAddr = (request[8] << 8) | request[9];
      uint16_t quantity = (request[10] << 8) | request[11];
      
      if (startAddr + quantity > TOTAL_REGISTERS) {
        response[0] = request[0]; response[1] = request[1];
        response[2] = 0x00; response[3] = 0x00;
        response[4] = 0x00; response[5] = 3;
        response[6] = unitId;
        response[7] = 0x83;
        response[8] = 0x02;
        responseLen = 9;
      } else {
        uint8_t byteCount = quantity * 2;
        response[0] = request[0]; response[1] = request[1];
        response[2] = 0x00; response[3] = 0x00;
        response[4] = 0x00; response[5] = 3 + byteCount;
        response[6] = unitId;
        response[7] = 0x03;
        response[8] = byteCount;
        
        for (int i = 0; i < quantity; i++) {
          response[9 + i*2] = (holdingRegisters[startAddr + i] >> 8) & 0xFF;
          response[10 + i*2] = holdingRegisters[startAddr + i] & 0xFF;
        }
        responseLen = 9 + byteCount;
      }
      break;
    }
    
    case 0x05: {  // Write Single Coil
      if (len < 12) return;
      uint16_t coilAddr = (request[8] << 8) | request[9];
      uint16_t coilValue = (request[10] << 8) | request[11];
      
      // Coil 7 = TargetA
      if (coilAddr == 7) {
        bool value = (coilValue == 0xFF00);  // 0xFF00 = ON, 0x0000 = OFF
        setBooleanBit(&holdingRegisters[REG_TARGET_A], 0, value);
        
        // Echo request back as response
        memcpy(response, request, 12);
        response[4] = 0x00; response[5] = 6;
        responseLen = 12;
        
        Serial.printf("[MODBUS] Write Coil: Addr=%d, Value=%s\n", coilAddr, value ? "ON" : "OFF");
      } else {
        // Exception: Illegal Data Address
        response[0] = request[0]; response[1] = request[1];
        response[2] = 0x00; response[3] = 0x00;
        response[4] = 0x00; response[5] = 3;
        response[6] = unitId;
        response[7] = 0x85;
        response[8] = 0x02;
        responseLen = 9;
      }
      break;
    }

    case 0x06: {  // Write Single Register
      if (len < 12) return;
      uint16_t regAddr = (request[8] << 8) | request[9];
      uint16_t regValue = (request[10] << 8) | request[11];
      
      if (regAddr < TOTAL_REGISTERS) {
        holdingRegisters[regAddr] = regValue;
        
        memcpy(response, request, 12);
        response[4] = 0x00; response[5] = 6;
        responseLen = 12;
        
        Serial.printf("[MODBUS] Write Reg: Addr=%d, Value=0x%04X\n", regAddr, regValue);
      }
      break;
    }
    
    case 0x10: {  // Write Multiple Registers
      if (len < 13) return;
      uint16_t startAddr = (request[8] << 8) | request[9];
      uint16_t quantity = (request[10] << 8) | request[11];
      uint8_t byteCount = request[12];
      
      if (startAddr + quantity <= TOTAL_REGISTERS && len >= 13 + byteCount) {
        for (int i = 0; i < quantity; i++) {
          holdingRegisters[startAddr + i] = (request[13 + i*2] << 8) | request[14 + i*2];
        }
        
        response[0] = request[0]; response[1] = request[1];
        response[2] = 0x00; response[3] = 0x00;
        response[4] = 0x00; response[5] = 6;
        response[6] = unitId;
        response[7] = 0x10;
        response[8] = request[8]; response[9] = request[9];
        response[10] = request[10]; response[11] = request[11];
        responseLen = 12;
        
        Serial.printf("[MODBUS] Write Regs: Addr=%d, Qty=%d\n", startAddr, quantity);
        
        // String 태그 Write 감지
        if (startAddr >= REG_CONTROL && startAddr < REG_CONTROL + 10) {
          char control[STRING_LENGTH + 1];
          registersToString(&holdingRegisters[REG_CONTROL], control, STRING_LENGTH);
          Serial.printf("[MODBUS] Control written: '%s'\n", control);
        }
        if (startAddr >= REG_STATE && startAddr < REG_STATE + 10) {
          char state[STRING_LENGTH + 1];
          registersToString(&holdingRegisters[REG_STATE], state, STRING_LENGTH);
          Serial.printf("[MODBUS] State written: '%s'\n", state);
        }
      }
      break;
    }
    
    default:
      response[0] = request[0]; response[1] = request[1];
      response[2] = 0x00; response[3] = 0x00;
      response[4] = 0x00; response[5] = 3;
      response[6] = unitId;
      response[7] = functionCode | 0x80;
      response[8] = 0x01;
      responseLen = 9;
      break;
  }
  
  if (responseLen > 0) {
    client.write(response, responseLen);
  }
}

// ============== 오도메트리 업데이트 ==============
void updateOdometry() {
  long dE1 = encoder1Count - lastEnc1;
  long dE2 = encoder2Count - lastEnc2;
  lastEnc1 = encoder1Count;
  lastEnc2 = encoder2Count;
  
  float dD1 = ((float)dE1 / TICKS_PER_REV) * (3.141592f * WHEEL_DIAMETER_MM);
  float dD2 = ((float)dE2 / TICKS_PER_REV) * (3.141592f * WHEEL_DIAMETER_MM);
  float dS = (dD1 + dD2) / 2.0f;
  float dT = (dD2 - dD1) / WHEEL_BASE_MM;
  
  posX += dS * cos(posTheta + dT / 2.0f);
  posY += dS * sin(posTheta + dT / 2.0f);
  posTheta += dT;
  
  while (posTheta > 3.141592f) posTheta -= 2.0f * 3.141592f;
  while (posTheta < -3.141592f) posTheta += 2.0f * 3.141592f;
}

// ============== String 태그 값 읽기 헬퍼 ==============
String getControlString() {
  char str[STRING_LENGTH + 1];
  registersToString(&holdingRegisters[REG_CONTROL], str, STRING_LENGTH);
  return String(str);
}

String getStateString() {
  char str[STRING_LENGTH + 1];
  registersToString(&holdingRegisters[REG_STATE], str, STRING_LENGTH);
  return String(str);
}

void setControlString(const char* str) {
  stringToRegisters(str, &holdingRegisters[REG_CONTROL], STRING_LENGTH);
}

void setStateString(const char* str) {
  stringToRegisters(str, &holdingRegisters[REG_STATE], STRING_LENGTH);
}
//================ 실행 메커니즘=====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT);
  pinMode(enc2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);

  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();
  
  WiFi.begin(ssid, password);
  Serial.print("WiFi Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  
  modbusServer.begin();
  Serial.printf("Modbus TCP Server started on port %d\n", modbusPort);
  
  // 초기 State 설정
  setStateString("IDLE");
  
  Serial.println("\n========== KEPServerEX Tag Map ==========");
  Serial.println("40001-40002: POS_X   (Float)");
  Serial.println("40003-40004: POS_Y   (Float)");
  Serial.println("40005-40006: POS_T   (Float)");
  Serial.println("40007.0:     TargetA (Boolean)");
  Serial.println("40100.20H:   Control (String, 20 bytes)");
  Serial.println("40200.20H:   State   (String, 20 bytes)");
  Serial.println("==========================================\n");
}

void loop() {
  unsigned long now = millis();
  // WiFi 연결 확인
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.begin(ssid, password);
    delay(1000);
    return;
  }
  
  // Modbus TCP 연결 확인
  WiFiClient newClient = modbusServer.available();
  
  // Modbus TCP Client 연결
  if (newClient) {
    if (modbusClient && modbusClient.connected()) {
      modbusClient.stop();
    }
    modbusClient = newClient;
    modbusClient.setNoDelay(true);
    Serial.print("[MODBUS] Client connected from: ");
    Serial.println(modbusClient.remoteIP());
  }
  
  // Modbus TCP Client 데이터 처리
  if (modbusClient && modbusClient.connected()) {
    if (modbusClient.available()) {
      processModbusRequest(modbusClient);
    }
  }
  
  // Odometry 및 Register 업데이트
  if (now - lastUpdateTime >= updateInterval) {
    lastUpdateTime = now;
    
    updateOdometry();
    updateHoldingRegisters();
    
    // 현재 태그 값 읽기
    bool targetA = getBooleanBit(holdingRegisters[REG_TARGET_A], 0);

    // TargetA가 0에서 1로 변하는 순간(Rising Edge) 감지하여 메가보드에 명령 전달
    static bool lastTargetA = false;
    if (targetA && !lastTargetA) {
      Serial.println("\n[EVENT] TargetA Triggered! Sending '1' to Mega.");
      sendCommand('1');
    }
    lastTargetA = targetA;

    String control = getControlString();
    String state = getStateString();
    
    // --- 시퀀스 및 제어 변수 ---
    static enum { SEQ_IDLE, SEQ_FORWARD_4M, SEQ_TURN_360 } seqStep = SEQ_IDLE;
    static float basePosX = 0, basePosY = 0, cumulativeTurn = 0, prevTheta = 0;
    static String lastControlForSeq = "";

    // 1. Control 태그가 "MOVE"로 써지는 순간 시퀀스 시작 트리거
    if (control == "MOVE" && lastControlForSeq != "MOVE") {
      uint16_t targetVal = holdingRegisters[REG_TARGET_A]; // 40007 값 읽기
      
      if (targetVal % 2 == 0) {
        Serial.printf("\n[SEQUENCE] Started from START POINT (TargetA:%d)\n", targetVal);
      } else {
        Serial.printf("\n[SEQUENCE] Started from END POINT (TargetA:%d)\n", targetVal);
      }

      seqStep = SEQ_FORWARD_4M;
      basePosX = posX; 
      basePosY = posY;
      sendCommand('w'); 
      setStateString("RUN"); // 동작 시작 보고
      Serial.println("[SEQUENCE] Step 1: Forward 4M Started");
    }
    lastControlForSeq = control;

    // 2. Control 태그 처리 (수동 및 기타 명령)
    static String lastControlManual = "";
    if (control != lastControlManual) {
      if (control == "x") {
        seqStep = SEQ_IDLE;
        sendCommand('x'); 
        setStateString("STOP");
      } else if (control.length() > 0 && control != "MOVE") {
        // 단일 문자 수동 명령 처리
        seqStep = SEQ_IDLE; 
        char cmd = control.charAt(0);
        sendCommand(cmd);
        if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd') setStateString("RUN");
        else setStateString("STOP");
      }
      lastControlManual = control;
    }

    // 3. 시퀀스 상태 머신 (엔코더 기반)
    if (seqStep == SEQ_FORWARD_4M) {
        float dist = sqrt(pow(posX - basePosX, 2) + pow(posY - basePosY, 2));
        if (dist >= 4000.0f) { // 4m 도달
            seqStep = SEQ_TURN_360;
            prevTheta = posTheta;
            cumulativeTurn = 0;
            sendCommand('a'); // 왼쪽 회전 시작
            Serial.println("[SEQUENCE] Step 2: Turn 360 Degrees Started");
        }
    } 
    else if (seqStep == SEQ_TURN_360) {
        float diff = posTheta - prevTheta;
        if (diff > PI) diff -= 2 * PI;
        if (diff < -PI) diff += 2 * PI;
        cumulativeTurn += abs(diff);
        prevTheta = posTheta;

        if (cumulativeTurn >= (2.0f * PI - 0.05f)) { // 360도 도달
            sendCommand('x');
            seqStep = SEQ_IDLE;
            setStateString("IDLE"); // 시퀀스 완료 보고
            Serial.printf("[SUCCESS] Sequence Complete! State -> STOP\n");
        }
    }

    state = getStateString();
    
    Serial.printf("\rPose[X:%.3f Y:%.3f T:%.1f] TargetA:%d Ctrl:'%s' State:'%s'\n", 
                  posX/1000.0f, posY/1000.0f, posTheta*180.0f/3.141592f,
                  targetA, control.c_str(), state.c_str());
  }
  
  delay(1);
}