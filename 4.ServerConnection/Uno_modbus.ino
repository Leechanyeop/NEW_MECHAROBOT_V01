/*
 * Uno_modbus.ino
 * Arduino UNO용 Modbus RTU Slave - 로봇 위치 추적 시스템
 * 
 * 시나리오:
 * - 출발지 A에서 바코드(A) 읽음 → 주행 시작, 위치 추적 시작
 * - 도착지 B에서 바코드(B) 읽음 → 도착 신호 전송, 추적 종료
 * 
 * OPC UA 브리지(Raspberry Pi 등)가 이 UNO를 Modbus Master로 폴링하여
 * 로봇 상태/위치를 OPC UA 서버로 전달함
 * 
 * 하드웨어 연결:
 * - 바코드 리더: Software Serial (Pin 10 RX, Pin 11 TX)
 * - Modbus RS485: Hardware Serial (Pin 0 RX, Pin 1 TX) + DE/RE 제어 (Pin 2)
 * - 엔코더/속도 센서: Pin A0 (아날로그) 또는 Pin 3 (인터럽트)
 */

#include <SoftwareSerial.h>

// ===================== 핀 설정 =====================
#define BARCODE_RX_PIN    10    // 바코드 리더 RX
#define BARCODE_TX_PIN    11    // 바코드 리더 TX
#define RS485_DE_RE_PIN   2     // RS485 방향 제어 (DE/RE)
#define ENCODER_PIN       3     // 엔코더 인터럽트 핀
#define SPEED_ANALOG_PIN  A0    // 속도 센서 (아날로그)

// ===================== Modbus 설정 =====================
#define MODBUS_SLAVE_ID   1
#define MODBUS_BAUD       9600

// Modbus 레지스터 맵 (Holding Registers - FC 03/06/16)
// 브리지가 이 레지스터들을 읽어서 OPC UA 서버에 전달
#define REG_POSITION_X_HI     0   // 위치 X (고) - mm 단위, int32를 2개 레지스터로
#define REG_POSITION_X_LO     1   // 위치 X (저)
#define REG_POSITION_Y_HI     2   // 위치 Y (고)
#define REG_POSITION_Y_LO     3   // 위치 Y (저)
#define REG_POSITION_THETA    4   // 방향각 (0.1도 단위, 예: 900 = 90.0도)
#define REG_SPEED             5   // 현재 속도 (mm/s)
#define REG_STATUS            6   // 상태: 0=IDLE, 1=RUNNING, 2=ARRIVED, 3=ERROR
#define REG_BARCODE_0         7   // 바코드 문자 0-1 (2 chars packed)
#define REG_BARCODE_1         8   // 바코드 문자 2-3
#define REG_BARCODE_2         9   // 바코드 문자 4-5
#define REG_BARCODE_3         10  // 바코드 문자 6-7
#define REG_ARRIVAL_HOUR      11  // 도착 시간: 시
#define REG_ARRIVAL_MIN       12  // 도착 시간: 분
#define REG_ARRIVAL_SEC       13  // 도착 시간: 초
#define REG_DISTANCE_TOTAL    14  // 총 이동거리 (m)
#define REG_TRIP_TIME         15  // 주행 시간 (초)

#define HOLDING_REG_COUNT     16

// Modbus Coil 레지스터 (FC 01/05/15)
#define COIL_RESET_POSITION   0   // 위치 리셋 명령
#define COIL_EMERGENCY_STOP   1   // 긴급 정지
#define COIL_START_TRACKING   2   // 추적 시작 명령 (외부에서 트리거)

#define COIL_COUNT            8

// ===================== 상태 정의 =====================
enum RobotStatus {
  STATUS_IDLE    = 0,
  STATUS_RUNNING = 1,
  STATUS_ARRIVED = 2,
  STATUS_ERROR   = 3
};

// ===================== 전역 변수 =====================
SoftwareSerial barcodeSerial(BARCODE_RX_PIN, BARCODE_TX_PIN);

// Modbus 레지스터 저장소
uint16_t holdingRegisters[HOLDING_REG_COUNT];
uint8_t coilRegisters = 0;

// 로봇 상태 변수
int32_t positionX = 0;        // 위치 X (mm)
int32_t positionY = 0;        // 위치 Y (mm)
int16_t positionTheta = 0;    // 방향각 (0.1도)
uint16_t currentSpeed = 0;    // 현재 속도 (mm/s)
RobotStatus robotStatus = STATUS_IDLE;

// 바코드 관련
char lastBarcode[16] = "";
bool barcodeReceived = false;

// 시간 추적
unsigned long tripStartTime = 0;
unsigned long lastPositionUpdate = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 100;  // 100ms마다 위치 업데이트

// 엔코더 관련 (인터럽트로 펄스 카운트)
volatile unsigned long encoderPulses = 0;
unsigned long lastEncoderPulses = 0;
unsigned long lastSpeedCalcTime = 0;
const float PULSE_TO_MM = 0.5;  // 1 펄스 = 0.5mm 이동 (실제 값으로 조정 필요)

// Modbus 수신 버퍼
uint8_t modbusBuffer[64];
int modbusBufferIndex = 0;
unsigned long lastModbusRx = 0;

// ===================== 함수 선언 =====================
void updatePosition();
void processBarcode();
void updateModbusRegisters();
void processModbusRequest();
uint16_t calculateCRC(uint8_t* buffer, int length);
void sendModbusResponse(uint8_t* response, int length);

// ===================== 인터럽트 핸들러 =====================
void encoderISR() {
  encoderPulses++;
}

// ===================== 초기화 =====================
void setup() {
  // Modbus용 하드웨어 시리얼
  Serial.begin(MODBUS_BAUD);
  
  // 바코드 리더용 소프트웨어 시리얼
  barcodeSerial.begin(9600);
  
  // RS485 방향 제어 핀
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // 수신 모드
  
  // 엔코더 인터럽트 설정
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  
  // 레지스터 초기화
  memset(holdingRegisters, 0, sizeof(holdingRegisters));
  holdingRegisters[REG_STATUS] = STATUS_IDLE;
  
  // 초기 시간
  lastPositionUpdate = millis();
  lastSpeedCalcTime = millis();
}

// ===================== 메인 루프 =====================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. 바코드 읽기 처리
  processBarcode();
  
  // 2. 속도 계산 (100ms마다)
  if (currentTime - lastSpeedCalcTime >= 100) {
    unsigned long pulsesDiff = encoderPulses - lastEncoderPulses;
    unsigned long timeDiff = currentTime - lastSpeedCalcTime;
    
    // 속도 = (펄스 * 펄스당mm) / 시간(s) = mm/s
    currentSpeed = (pulsesDiff * PULSE_TO_MM * 1000) / timeDiff;
    
    lastEncoderPulses = encoderPulses;
    lastSpeedCalcTime = currentTime;
  }
  
  // 3. 위치 업데이트 (주행 중일 때만)
  if (robotStatus == STATUS_RUNNING) {
    if (currentTime - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
      updatePosition();
      lastPositionUpdate = currentTime;
    }
  }
  
  // 4. Modbus 레지스터 업데이트
  updateModbusRegisters();
  
  // 5. Modbus 요청 처리
  processModbusRequest();
}

// ===================== 바코드 처리 =====================
void processBarcode() {
  static char barcodeBuffer[16];
  static int barcodeIndex = 0;
  
  while (barcodeSerial.available()) {
    char c = barcodeSerial.read();
    
    // 바코드 종료 문자 (CR 또는 LF)
    if (c == '\r' || c == '\n') {
      if (barcodeIndex > 0) {
        barcodeBuffer[barcodeIndex] = '\0';
        strncpy(lastBarcode, barcodeBuffer, sizeof(lastBarcode) - 1);
        barcodeReceived = true;
        
        // 바코드에 따른 상태 변경
        if (strcmp(lastBarcode, "A") == 0 || strstr(lastBarcode, "START") != NULL) {
          // 출발지 A 도착 → 주행 시작
          robotStatus = STATUS_RUNNING;
          tripStartTime = millis();
          positionX = 0;
          positionY = 0;
          encoderPulses = 0;
          lastEncoderPulses = 0;
        }
        else if (strcmp(lastBarcode, "B") == 0 || strstr(lastBarcode, "END") != NULL) {
          // 도착지 B 도착 → 주행 완료
          robotStatus = STATUS_ARRIVED;
          
          // 도착 시간 기록 (상대 시간 - 시:분:초로 변환)
          unsigned long tripTime = (millis() - tripStartTime) / 1000;
          holdingRegisters[REG_ARRIVAL_HOUR] = tripTime / 3600;
          holdingRegisters[REG_ARRIVAL_MIN] = (tripTime % 3600) / 60;
          holdingRegisters[REG_ARRIVAL_SEC] = tripTime % 60;
          holdingRegisters[REG_TRIP_TIME] = tripTime;
        }
        
        barcodeIndex = 0;
      }
    }
    else if (barcodeIndex < sizeof(barcodeBuffer) - 1) {
      barcodeBuffer[barcodeIndex++] = c;
    }
  }
}

// ===================== 위치 업데이트 =====================
void updatePosition() {
  // 단순 직선 주행 가정: X축 방향으로만 이동
  // 실제로는 방향각(theta)을 고려한 2D 위치 계산 필요
  
  unsigned long pulsesDiff = encoderPulses - lastEncoderPulses;
  float distance = pulsesDiff * PULSE_TO_MM;  // mm
  
  // 방향각을 고려한 위치 계산 (theta는 0.1도 단위)
  float thetaRad = (positionTheta / 10.0) * PI / 180.0;
  positionX += (int32_t)(distance * cos(thetaRad));
  positionY += (int32_t)(distance * sin(thetaRad));
}

// ===================== Modbus 레지스터 업데이트 =====================
void updateModbusRegisters() {
  // 위치 X (32비트를 16비트 2개로)
  holdingRegisters[REG_POSITION_X_HI] = (positionX >> 16) & 0xFFFF;
  holdingRegisters[REG_POSITION_X_LO] = positionX & 0xFFFF;
  
  // 위치 Y
  holdingRegisters[REG_POSITION_Y_HI] = (positionY >> 16) & 0xFFFF;
  holdingRegisters[REG_POSITION_Y_LO] = positionY & 0xFFFF;
  
  // 방향각
  holdingRegisters[REG_POSITION_THETA] = positionTheta;
  
  // 속도
  holdingRegisters[REG_SPEED] = currentSpeed;
  
  // 상태
  holdingRegisters[REG_STATUS] = robotStatus;
  
  // 바코드 (문자열을 레지스터로 패킹)
  for (int i = 0; i < 4; i++) {
    char c1 = (i * 2 < strlen(lastBarcode)) ? lastBarcode[i * 2] : 0;
    char c2 = (i * 2 + 1 < strlen(lastBarcode)) ? lastBarcode[i * 2 + 1] : 0;
    holdingRegisters[REG_BARCODE_0 + i] = (c1 << 8) | c2;
  }
  
  // 총 이동거리 (mm를 m로 변환)
  holdingRegisters[REG_DISTANCE_TOTAL] = (encoderPulses * PULSE_TO_MM) / 1000;
  
  // 주행 중이면 주행 시간 업데이트
  if (robotStatus == STATUS_RUNNING) {
    holdingRegisters[REG_TRIP_TIME] = (millis() - tripStartTime) / 1000;
  }
}

// ===================== Modbus 요청 처리 =====================
void processModbusRequest() {
  // 시리얼 데이터 수신
  while (Serial.available()) {
    modbusBuffer[modbusBufferIndex++] = Serial.read();
    lastModbusRx = millis();
    
    if (modbusBufferIndex >= sizeof(modbusBuffer)) {
      modbusBufferIndex = 0;
    }
  }
  
  // 프레임 완료 체크 (3.5 문자 시간 = ~4ms at 9600 baud)
  if (modbusBufferIndex > 0 && millis() - lastModbusRx > 5) {
    // 최소 프레임 길이 체크 (ID + FC + CRC = 4 bytes)
    if (modbusBufferIndex >= 4) {
      // 슬레이브 ID 체크
      if (modbusBuffer[0] == MODBUS_SLAVE_ID) {
        // CRC 체크
        uint16_t receivedCRC = (modbusBuffer[modbusBufferIndex - 1] << 8) | modbusBuffer[modbusBufferIndex - 2];
        uint16_t calculatedCRC = calculateCRC(modbusBuffer, modbusBufferIndex - 2);
        
        if (receivedCRC == calculatedCRC) {
          uint8_t functionCode = modbusBuffer[1];
          uint8_t response[64];
          int responseLen = 0;
          
          switch (functionCode) {
            case 0x03:  // Read Holding Registers
            {
              uint16_t startAddr = (modbusBuffer[2] << 8) | modbusBuffer[3];
              uint16_t quantity = (modbusBuffer[4] << 8) | modbusBuffer[5];
              
              if (startAddr + quantity <= HOLDING_REG_COUNT) {
                response[0] = MODBUS_SLAVE_ID;
                response[1] = 0x03;
                response[2] = quantity * 2;
                responseLen = 3;
                
                for (int i = 0; i < quantity; i++) {
                  response[responseLen++] = (holdingRegisters[startAddr + i] >> 8) & 0xFF;
                  response[responseLen++] = holdingRegisters[startAddr + i] & 0xFF;
                }
                
                uint16_t crc = calculateCRC(response, responseLen);
                response[responseLen++] = crc & 0xFF;
                response[responseLen++] = (crc >> 8) & 0xFF;
                
                sendModbusResponse(response, responseLen);
              }
              break;
            }
            
            case 0x06:  // Write Single Register
            {
              uint16_t regAddr = (modbusBuffer[2] << 8) | modbusBuffer[3];
              uint16_t value = (modbusBuffer[4] << 8) | modbusBuffer[5];
              
              if (regAddr < HOLDING_REG_COUNT) {
                // 특정 레지스터 쓰기 처리
                if (regAddr == REG_STATUS) {
                  robotStatus = (RobotStatus)value;
                  if (robotStatus == STATUS_RUNNING) {
                    tripStartTime = millis();
                    positionX = 0;
                    positionY = 0;
                  }
                }
                else if (regAddr == REG_POSITION_THETA) {
                  positionTheta = value;
                }
                
                holdingRegisters[regAddr] = value;
                
                // 에코 응답
                sendModbusResponse(modbusBuffer, modbusBufferIndex);
              }
              break;
            }
            
            case 0x01:  // Read Coils
            {
              uint16_t startAddr = (modbusBuffer[2] << 8) | modbusBuffer[3];
              uint16_t quantity = (modbusBuffer[4] << 8) | modbusBuffer[5];
              
              response[0] = MODBUS_SLAVE_ID;
              response[1] = 0x01;
              response[2] = (quantity + 7) / 8;
              response[3] = coilRegisters >> startAddr;
              responseLen = 4;
              
              uint16_t crc = calculateCRC(response, responseLen);
              response[responseLen++] = crc & 0xFF;
              response[responseLen++] = (crc >> 8) & 0xFF;
              
              sendModbusResponse(response, responseLen);
              break;
            }
            
            case 0x05:  // Write Single Coil
            {
              uint16_t coilAddr = (modbusBuffer[2] << 8) | modbusBuffer[3];
              uint16_t value = (modbusBuffer[4] << 8) | modbusBuffer[5];
              
              if (coilAddr < COIL_COUNT) {
                if (value == 0xFF00) {
                  coilRegisters |= (1 << coilAddr);
                } else {
                  coilRegisters &= ~(1 << coilAddr);
                }
                
                // Coil 액션 처리
                if (coilAddr == COIL_RESET_POSITION && (coilRegisters & (1 << COIL_RESET_POSITION))) {
                  positionX = 0;
                  positionY = 0;
                  positionTheta = 0;
                  encoderPulses = 0;
                  coilRegisters &= ~(1 << COIL_RESET_POSITION);  // 자동 리셋
                }
                else if (coilAddr == COIL_EMERGENCY_STOP && (coilRegisters & (1 << COIL_EMERGENCY_STOP))) {
                  robotStatus = STATUS_IDLE;
                }
                else if (coilAddr == COIL_START_TRACKING && (coilRegisters & (1 << COIL_START_TRACKING))) {
                  robotStatus = STATUS_RUNNING;
                  tripStartTime = millis();
                  positionX = 0;
                  positionY = 0;
                  coilRegisters &= ~(1 << COIL_START_TRACKING);  // 자동 리셋
                }
                
                // 에코 응답
                sendModbusResponse(modbusBuffer, modbusBufferIndex);
              }
              break;
            }
          }
        }
      }
    }
    
    modbusBufferIndex = 0;
  }
}

// ===================== CRC 계산 (Modbus RTU) =====================
uint16_t calculateCRC(uint8_t* buffer, int length) {
  uint16_t crc = 0xFFFF;
  
  for (int i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}

// ===================== Modbus 응답 전송 =====================
void sendModbusResponse(uint8_t* response, int length) {
  // RS485 송신 모드로 전환
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(100);
  
  // 응답 전송
  Serial.write(response, length);
  Serial.flush();
  
  // RS485 수신 모드로 복귀
  delayMicroseconds(100);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// ===================== 디버그 함수 (SoftwareSerial로 출력 시 사용) =====================
/*
void debugPrint() {
  // 속도 센서 시뮬레이션용 (아날로그 입력으로 속도 추정)
  int analogValue = analogRead(SPEED_ANALOG_PIN);
  currentSpeed = map(analogValue, 0, 1023, 0, 1000);  // 0~1000 mm/s
  
  // 시뮬레이션: 100ms마다 속도에 따라 엔코더 펄스 증가
  encoderPulses += currentSpeed / (1000 / POSITION_UPDATE_INTERVAL) / PULSE_TO_MM;
}
*/
