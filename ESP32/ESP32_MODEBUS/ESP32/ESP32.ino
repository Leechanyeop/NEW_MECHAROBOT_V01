#include <WiFi.h>
#include <SPI.h>

/**
 * @file ESP32.ino
 * @brief Robot Control via Modbus TCP (Integrated for OPC UA Gateway)
 * Location: c:\#PROJECT\NEW_MECHROBOT_V01\ESP32\ESP32_MODEBUS\ESP32.ino
 */

// ============== WiFi & Network Settings ==============
const char* ssid = "hhme";
const char* password = "hme*12345";

// Port 502: Modbus TCP Server (Connect this to KEPServerEX / Ignition)
const uint16_t mbPort = 502;
WiFiServer mbServer(mbPort);
WiFiClient mbClient;

// ============== Hardware Pins ==============
const int chipSelectPin = 5;
const int enc1A = 32;
const int enc1B = 33;
const int enc2A = 25;
const int enc2B = 26;

// ============== Global Variables ==============
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

unsigned long lastHBTime = 0;
const unsigned long HBInterval = 100; // Modbus update interval (ms)

// Robot Physical Constants
const float WHEEL_DIAMETER_MM = 67.0f;
const float GEAR_RATIO = 30.0f;
const int ENCODER_CPR = 11;
const float TICKS_PER_REV = (float)ENCODER_CPR * 4.0f * GEAR_RATIO; // 1320
const float WHEEL_BASE_MM = 200.0f;
const float PI_F = 3.1415926535f;

float posX = 0.0f, posY = 0.0f, posTheta = 0.0f;
long lastEnc1 = 0, lastEnc2 = 0;

// Target Mode Variables
// 0:None, 1:PID_4m, 3:Forward300, 4:Backward300
int targetMode = 0; 
volatile float travelDist = 0.0f; 

// ============== Modbus Holding Registers ==============
// Mapping for OPC UA / KEPServerEX
const int TOTAL_REGISTERS = 220; 
uint16_t holdingRegisters[TOTAL_REGISTERS] = {0};

enum RegisterIndex {
  REG_POS_X_LO = 0,      // 40001
  REG_POS_X_HI = 1,      // 40002
  REG_POS_Y_LO = 2,      // 40003
  REG_POS_Y_HI = 3,      // 40004
  REG_POS_T_LO = 4,      // 40005
  REG_POS_T_HI = 5,      // 40006
  REG_TARGET_A = 6,      // 40007.0 (Boolean / Bit 0)
  REG_CONTROL = 99,      // 40100 (String, 10 regs)
  REG_STATE = 199        // 40200 (String, 10 regs)
};
const int STR_REG_LEN = 10; // 20 bytes for string

// ============== Encoder ISRs (4x Quadrature) ==============
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
void sendSPI(const char *frame) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  size_t len = strlen(frame);
  for (size_t i = 0; i < len; ++i) {
    SPI.transfer((uint8_t)frame[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  Serial.printf("[SPI] Sent: %s\n", frame);
}


// ============== Modbus Utilities ==============
void floatToMB(float value, uint16_t* regs) {
  union { float f; uint8_t b[4]; } data;
  data.f = value;
  // Standard Float Order: CD AB (Low word first)
  regs[0] = ((uint16_t)data.b[1] << 8) | data.b[0]; 
  regs[1] = ((uint16_t)data.b[3] << 8) | data.b[2]; 
}

void stringToMB(const char* str, uint16_t* regs) {
  int len = strlen(str);
  for (int i = 0; i < STR_REG_LEN; i++) regs[i] = 0;
  for (int i = 0; i < 20 && i < len; i++) {
    int idx = i / 2;
    if (i % 2 == 0) regs[idx] = (uint16_t)str[i] << 8;
    else regs[idx] |= (uint16_t)str[i];
  }
}

void MBToString(uint16_t* regs, char* str) {
  for (int i = 0; i < 20; i++) {
    int idx = i / 2;
    str[i] = (i % 2 == 0) ? (char)(regs[idx] >> 8) : (char)(regs[idx] & 0xFF);
  }
  str[20] = '\0';
}

// ============== Odometry ==============
void updateOdometry() {
  noInterrupts();
  long curE1 = encoder1Count;
  long curE2 = encoder2Count;
  interrupts();

  long dE1 = curE1 - lastEnc1;
  long dE2 = curE2 - lastEnc2;
  lastEnc1 = curE1;
  lastEnc2 = curE2;

  float dD1 = ((float)dE1 / TICKS_PER_REV) * (PI_F * WHEEL_DIAMETER_MM);
  float dD2 = ((float)dE2 / TICKS_PER_REV) * (PI_F * WHEEL_DIAMETER_MM);
  float dS  = (dD1+dD2)*0.5f;
  float dT  = (dD2-dD1)/360 *PI_F *1/WHEEL_BASE_MM;

  if (targetMode != 0) travelDist += dS;

  float midT = posTheta + dT * 0.5f;
  posX += dS * cosf(midT);
  posY += dS * sinf(midT);
  posTheta += dT;

  while (posTheta > PI_F) posTheta -= 2.0f * PI_F;
  while (posTheta < -PI_F) posTheta += 2.0f * PI_F;
}

// ============== Unified Command Handler ==============
void handleCommand(char cmd) {
  if (cmd == '\n' || cmd == '\r' || cmd == 0 || cmd == ' ') return;
  Serial.printf("[CMD] Received: %c\n", cmd);

  if (cmd == '1') {
    Serial.println("Mission: PID Line Trace 4m");
    targetMode = 1;
    travelDist = 0.0f;
    sendSPI("<p>");
    stringToMB("RUN_PID_4M", &holdingRegisters[REG_STATE]);
  }
  else if (cmd == 'w') {
    targetMode = 3;
    travelDist = 0.0f;
    sendSPI("<w>");
    stringToMB("MOVE_FWD", &holdingRegisters[REG_STATE]);
  }
  else if (cmd == 's') {
    targetMode = 4;
    travelDist = 0.0f;
    sendSPI("<s>");
    stringToMB("MOVE_BWD", &holdingRegisters[REG_STATE]);
  }
  else if (cmd == 'x') {
    targetMode = 0;
    sendSPI("<x>");
    stringToMB("IDLE", &holdingRegisters[REG_STATE]);
  }
  else {
    targetMode = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "<%c>", cmd);
    sendSPI(buf);
  }
}

// ============== Modbus Request Handler ==============
void processModbus() {
  uint8_t req[256];
  int len = mbClient.read(req, sizeof(req));
  if (len < 8) return;

  uint8_t fc = req[7];
  uint16_t addr = (req[8] << 8) | req[9];
  uint16_t qty = (req[10] << 8) | req[11];
  uint8_t res[256];
  int resLen = 0;

  if (fc == 0x03) { // Read Holding Registers
    resLen = 9 + (qty * 2);
    memcpy(res, req, 7); res[5] = 3 + (qty * 2); res[7] = 0x03; res[8] = qty * 2;
    for (int i = 0; i < qty; i++) {
        res[9 + i*2] = (holdingRegisters[addr + i] >> 8) & 0xFF; // MB is big-endian
        res[10 + i*2] = holdingRegisters[addr + i] & 0xFF;
    }
  } 
  else if (fc == 0x10) { // Write Multiple Registers
    uint8_t bc = req[12];
    for (int i = 0; i < qty; i++) {
        holdingRegisters[addr + i] = (req[13 + i*2] << 8) | req[14 + i*2];
    }
    memcpy(res, req, 12); res[5] = 6; resLen = 12;

    // Trigger: TargetA bit 0 (Coil 40007.0)
    static bool lastTargetA = false;
    bool currentTargetA = (holdingRegisters[REG_TARGET_A] & 0x01);
    if (currentTargetA && !lastTargetA) handleCommand('1');
    lastTargetA = currentTargetA;

    // Trigger: Control String (40100)
    if (addr >= REG_CONTROL && addr < REG_CONTROL + 10) {
        char control[21];
        MBToString(&holdingRegisters[REG_CONTROL], control);
        // If single character 'w','s','x','1','c','p', etc.
        if (strlen(control) >= 1) {
            handleCommand(control[0]);
            // Clear used command
            stringToMB("", &holdingRegisters[REG_CONTROL]);
        }
    }
  }

  if (resLen > 0) mbClient.write(res, resLen);
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.printf("\nModbus Server for OPC UA Gateway: %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

  // Encoder Pins
  pinMode(enc1A, INPUT_PULLUP); pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP); pinMode(enc2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), isr1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), isr2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2B), isr2B, CHANGE);

  // SPI Master
  pinMode(chipSelectPin, OUTPUT); digitalWrite(chipSelectPin, HIGH);
  SPI.begin();

  // Modbus Server
  mbServer.begin();
  stringToMB("IDLE", &holdingRegisters[REG_STATE]);
  Serial.println("=== ESP32 Modbus-OPC UA Ready ===");
}

// ============== Loop ==============
void loop() {
  uint32_t now = millis();

  // Modbus Client Accept
  WiFiClient newMB = mbServer.available();
  if (newMB) {
    if (mbClient) mbClient.stop();
    mbClient = newMB;
    Serial.println("[MB] OPC UA Gateway Connected");
  }

  // Command Handling: Modbus or Local Serial
  if (mbClient.connected() && mbClient.available() > 0) processModbus();
  if (Serial.available() > 0) handleCommand(Serial.read());

  updateOdometry();

  // Autonomous Mode Check
  if (targetMode == 1 && travelDist >= 4000.0f) { // PID 4m
    handleCommand('x'); Serial.println("4m MISSION COMPLETE");
  } else if (targetMode == 3 && travelDist >= 300.0f) {
    handleCommand('x');
  } else if (targetMode == 4 && travelDist <= -300.0f) {
    handleCommand('x');
  }

  // Status Update & Logging
  if (now - lastHBTime >= HBInterval) {
    lastHBTime = now;
    
    // Convert current pose to Modbus Registers (Floats)
    floatToMB(posX / 1000.0f, &holdingRegisters[REG_POS_X_LO]);
    floatToMB(posY / 1000.0f, &holdingRegisters[REG_POS_Y_LO]);
    floatToMB(posTheta * 180.0f / PI_F, &holdingRegisters[REG_POS_T_LO]);

    // Debug Print
    Serial.printf("X:%.3f Y:%.3f T:%.1f Dist:%.1f Mode:%d\n", posX/1000.0f, posY/1000.0f, posTheta*180.0f/PI_F, travelDist, targetMode);
  }

  yield();
}
