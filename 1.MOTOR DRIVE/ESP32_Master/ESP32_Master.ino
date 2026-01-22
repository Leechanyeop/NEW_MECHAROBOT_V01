#include <WiFi.h>
#include <SPI.h>

/**
 * @file ESP32_Master.ino
 * @brief ESP32 Master: Modbus TCP Relay with Odometry
 */

// ============== WiFi & Modbus Server Settings ==============
const char* ssid = "hhme";            
const char* password = "hme*12345";   
const char* serverIP = "192.168.0.19"; 
const int serverPort = 502;          

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
const float TICKS_PER_REV = (float)ENCODER_CPR * 2 * GEAR_RATIO; 

float posX = 0.0, posY = 0.0, posTheta = 0.0;
long lastEnc1 = 0, lastEnc2 = 0;

WiFiClient client;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 500; 

// 1번 모드(10회전)용 전역 변수
bool isTenRotationsMode = false;
long targetTick = 0;
const int TICKS_PER_REV_ACTUAL = (int)TICKS_PER_REV; 

// ============== ISR (Noise Filtered) ==============
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

void setup() {
  Serial.begin(115200);
  pinMode(enc1A, INPUT); pinMode(enc1B, INPUT);
  pinMode(enc2A, INPUT); pinMode(enc2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);

  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected!");
  Serial.print("Local IP: "); Serial.println(WiFi.localIP());
  delay(1000);
}

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

// ============== Modbus TCP Packet Builder ==============
void sendModbusRegisters() {
  if (!client.connected()) return;

  float x_m = posX / 1000.0f; 
  float y_m = posY / 1000.0f;
  uint16_t stateVal = isTenRotationsMode ? 1 : 0; 
  uint16_t toVal = isTenRotationsMode ? 10 : 0;

  uint8_t frame[25]; // Header(7) + FC(1) + Addr(2) + Qty(2) + BC(1) + Data(12)
  
  // MBAP Header
  static uint16_t tid = 0;
  tid++;
  frame[0] = (tid >> 8) & 0xFF; frame[1] = tid & 0xFF; 
  frame[2] = 0x00; frame[3] = 0x00; 
  frame[4] = 0x00; frame[5] = 19;   // Length (UnitID 1 + FC 1 + Addr 2 + Qty 2 + BC 1 + Data 12)
  frame[6] = 0x01;                  
  
  // FC 16: Write Multiple Registers
  frame[7] = 0x10;
  frame[8] = 0x00; frame[9] = 0x00;   // Start Addr: 40001
  frame[10] = 0x00; frame[11] = 0x06; // Qty: 6 Registers (X:2, Y:2, State:1, To:1)
  frame[12] = 12;                     // Byte Count: 12 bytes

  // Float to 32-bit (Big Endian IEEE 754)
  auto floatToBytes = [&](float val, int start) {
      uint32_t data;
      memcpy(&data, &val, 4);
      frame[start]   = (data >> 24) & 0xFF;
      frame[start+1] = (data >> 16) & 0xFF;
      frame[start+2] = (data >> 8) & 0xFF;
      frame[start+3] = data & 0xFF;
  };

  // 40001-40002: POS_X (Float)
  floatToBytes(x_m, 13);
  // 40003-40004: POS_Y (Float)
  floatToBytes(y_m, 17);
  // 40005: State (Int)
  frame[21] = (stateVal >> 8) & 0xFF;
  frame[22] = stateVal & 0xFF;
  // 40006: To (Int)
  frame[23] = (toVal >> 8) & 0xFF;
  frame[24] = toVal & 0xFF;

  client.write(frame, 25);
}

// ============== Modbus TCP Read (FC 03) ==============
void readModbusRegisters(uint16_t startAddr, uint16_t qty) {
  if (!client.connected()) return;

  uint8_t frame[12];
  static uint16_t tid_r = 0x8000; // 읽기용 TID는 구분하기 위해 높게 설정
  tid_r++;

  // MBAP Header
  frame[0] = (tid_r >> 8) & 0xFF; frame[1] = tid_r & 0xFF;
  frame[2] = 0x00; frame[3] = 0x00;
  frame[4] = 0x00; frame[5] = 6;    // 뒤에 오는 길이 (UnitID 1 + FC 1 + Addr 2 + Qty 2 = 6)
  frame[6] = 0x01;                 // Unit ID
  
  // FC 03: Read Holding Registers
  frame[7] = 0x03;
  frame[8] = (startAddr >> 8) & 0xFF;
  frame[9] = startAddr & 0xFF;
  frame[10] = (qty >> 8) & 0xFF;
  frame[11] = qty & 0xFF;

  client.write(frame, 12);
}

// 서버 응답 데이터를 처리하는 함수
void handleModbusResponse() {
  if (client.available() < 9) return; // 최소 헤더 길이는 되어야 함

  uint8_t res[64];
  int len = client.read(res, 64);

  // Function Code 03(읽기) 응답인지 확인
  if (res[7] == 0x03) {
    int byteCount = res[8];
    if (byteCount >= 4) { // 2개 레지스터이므로 최소 4바이트 데이터 필요
      // 40005 (State) 읽기
      uint16_t serverState = (res[9] << 8) | res[10];
      // 40006 (To) 읽기
      uint16_t serverTo = (res[11] << 8) | res[12];
      
      // 로그 출력 (확인용)
      static uint16_t lastState = 99;
      if (serverState != lastState) {
        Serial.printf("\n[MODBUS] Server State: %d, To: %d\n", serverState, serverTo);
        lastState = serverState;
        
        // 만약 서버에서 State를 1로 보내면 로봇 동작 시작
        if (serverState == 1 && !isTenRotationsMode) {
          processBuffer("1"); 
        }
      }
    }
  }
}

void processBuffer(String buf) {
  buf.trim();
  if (buf.length() > 0) {
    char serverCmd = buf[0];
    if (serverCmd == '1') {
      isTenRotationsMode = true;
      targetTick = abs(encoder1Count) + (TICKS_PER_REV_ACTUAL * 10);
      sendCommand('w');
    } else {
      isTenRotationsMode = false;
      sendCommand(serverCmd);
    }
    lastSendTime = millis(); 
  }
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(1000);
    return;
  }

  if (!client.connected()) {
    static uint32_t lastR = 0;
    if (now - lastR > 3000) {
      lastR = now;
      client.stop();
      Serial.print(">>>> Connecting to Modbus Server (");
      Serial.print(serverIP); Serial.print(":"); Serial.print(serverPort);
      Serial.println(") ...");
      
      if (client.connect(serverIP, serverPort)) {
        client.setNoDelay(true);
        Serial.println(">>>> SUCCESS! Modbus Connected. <<<<");
      } else {
        Serial.println(">>>> FAILED. (Check PC IP / Port / Firewall) <<<<");
      }
    }
    return;
  }

  // 서버 응답 데이터 처리
  handleModbusResponse();

  // 10회전 모드 감시
  if (isTenRotationsMode) {
    if (abs(encoder1Count) >= targetTick) {
      sendCommand('x');
      isTenRotationsMode = false;
      Serial.println("\n[MODE] Complete!");
    }
  }

  // 주기적 데이터 전송
  if (now - lastSendTime >= sendInterval) {
    lastSendTime = now;
    
    // 오도메트리 계산
    long dE1 = encoder1Count - lastEnc1;
    long dE2 = encoder2Count - lastEnc2;
    lastEnc1 = encoder1Count; lastEnc2 = encoder2Count;
    float dD1 = ((float)dE1 / TICKS_PER_REV) * (3.141592f * WHEEL_DIAMETER_MM);
    float dD2 = ((float)dE2 / TICKS_PER_REV) * (3.141592f * WHEEL_DIAMETER_MM);
    float dS = (dD1 + dD2) / 2.0f;
    float dT = (dD2 - dD1) / WHEEL_BASE_MM;
    posX += dS * cos(posTheta + dT / 2.0f);
    posY += dS * sin(posTheta + dT / 2.0f);
    posTheta += dT;

    // Modbus 전송 및 읽기 요청
    sendModbusRegisters();
    readModbusRegisters(4, 2); // 40005번 주소(인덱스 4)부터 2개 읽기 요청

    Serial.printf("\rPose[X:%.2f Y:%.2f] State:%d To:%d", posX/1000.0, posY/1000.0, isTenRotationsMode?1:0, isTenRotationsMode?10:0);
    Serial.println();
  }

  // 로컬 시리얼 명령
  if (Serial.available() > 0) {
    char sCmd = Serial.read();
    if (sCmd != '\n' && sCmd != '\r') {
      String cmdStr = "";
      cmdStr += sCmd;
      processBuffer(cmdStr);
    }
  }

  delay(1);
}
