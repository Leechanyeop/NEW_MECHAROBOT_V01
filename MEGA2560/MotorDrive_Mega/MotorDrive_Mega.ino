#include <SPI.h>
#include <QTRSensors.h>
#include "MotorDrive.h"
#include "LINETRACE.h"
#include <Servo.h>
// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

/**
 * BTS7960 드라이버 연결 정보
 * ... (existing comments)
 */

// 모터 1 (왼쪽)
#define MOTOR1_R_EN  4
#define MOTOR1_L_EN  5
#define MOTOR1_RPWM  2
#define MOTOR1_LPWM  3

// 모터 2 (오른쪽)
#define MOTOR2_R_EN  10
#define MOTOR2_L_EN  11
#define MOTOR2_RPWM  8
#define MOTOR2_LPWM  9

// QTR 라인 센서 핀 (28, 29, 30, 31, 32, 33, 34, 35)
const uint8_t qtrPins[] = {28, 29, 30, 31, 32, 33, 34, 35};



// QTR 센서 객체
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ... (existing SPI comments)

// SPI Slave 핀 (Mega 2560 전용)
#define SPI_SS_PIN   53
#define SPI_MISO_PIN 50
#define SPI_MOSI_PIN 51
#define SPI_SCK_PIN  52

// 디버그용 LED
#define DEBUG_LED 17
#define SIGNAL_PIN 7  // 'k' 명령으로 제어할 핀 (서보 또는 디지털 신호)

Servo myservo;        // 서보 객체 추가

// ============== 전역 변수 ==============
int pwmValue = 60;        // 현재 PWM 값 (기본 속도: 천천히 주행)
int turnPwmValue = 45;     // 회전 시 PWM 값 (조금 더 천천히)
int maxPIDSpeed = 120;     // PID 주행 시 최대 속도 제한 (회전 여유 확보)
volatile char lastReceivedCmd[32] = {0};  // 마지막 수신 명령 (문자열 지원)
volatile bool cmdReceived = false;        // 명령 수신 플래그
volatile uint32_t lastPacketTime = 0; // 마지막 유효 패킷 수신 시간 (ms)
volatile uint32_t byteCount = 0;    // 수신된 총 바이트 수 (디버그용)
int angle = 90;
// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// LineTracer 객체 (기존 로직이 있다면 유지하되, 센서값은 메인에서 넘겨주는 방식 고려)
// 여기서는 메인 루프에서 직접 PID를 돌리는 것이 가장 확실함.
// LineTracer lineTracer(motor); 

bool isLineTracing = false;
bool isPIDTracing = false;
bool isAutoMode = false;
bool isCheckSensors = false;  // 'c' 명령 시 센서값 확인용 플래그
uint32_t autoModeTimer = 0;
int autoModeStep = 0;

// PID 변수 (메인에서 직접 튜닝)
float Kp = 0.1; // 비례 상수 (너무 높으면 떨림)
float Ki = 0.0;
float Kd = 1.0;  // 미분 상수 (흔들림 보정)
int lastError = 0;

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  
  pinMode(SIGNAL_PIN, OUTPUT);
  myservo.attach(SIGNAL_PIN);   // 서보 핀 연결
  myservo.write(angle);         // 초기 각도 90도
  
  motor.begin();
  motor.setSpeed(pwmValue);
  motor.stop(); 

  // QTR 센서 초기화 (RC 타입)
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(255); // 이미터 LED를 항상 켬 (QTR-8RC 필수)
  
  Serial.println(F("\n=== Mega Motor Driver (QTR-8RC Integrated) ==="));
  Serial.println(F("Calibrating QTR Sensors... (Move sensor over line)"));
  
  // 캘리브레이션 (약 5초 동안)
  // 로봇을 손으로 라인 위에서 좌우로 흔들어주세요.
  digitalWrite(DEBUG_LED, HIGH);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(DEBUG_LED, LOW);
  
  Serial.println(F("Calibration Complete."));
  Serial.println(F("Protocol: < [CMD] >"));

  // ========== SPI Slave 설정 (Mega 2560) ==========
  pinMode(SPI_SS_PIN, INPUT_PULLUP);
  pinMode(SPI_MOSI_PIN, INPUT);
  pinMode(SPI_MISO_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, INPUT);
  
  DDRB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2)); 
  DDRB |= _BV(PB3); 
  
  SPCR = _BV(SPE) | _BV(SPIE); 
  
  SPDR = 0x00; 
  
  volatile byte dummy = SPSR;
  dummy = SPDR;
  (void)dummy;

  lastPacketTime = millis();
}

// ============== SPI 인터럽트 ==============
ISR(SPI_STC_vect) {
  uint8_t inByte = SPDR;
  static byte state = 0;
  static byte pos = 0;

  if (inByte == '<') {
    state = 1;
    pos = 0;
  } else if (state == 1) {
    if (inByte == '>') {
      lastReceivedCmd[pos] = '\0';
      cmdReceived = true;
      lastPacketTime = millis();
      state = 0;
    } else {
      if (pos < 30) {
        lastReceivedCmd[pos++] = (char)inByte;
      }
    }
  }
}
void processCommand(char input) {
  if (input == 'h') { 
    angle = 180; 
    Serial.println("서보: 정방향(H)"); 
  } 
  else if (input == 'j') { 
    angle = 0; 
    Serial.println("서보: 역방향(J)"); 
  } 
  else if (input == 'k') { 
    angle = 90; 
    Serial.println("서보: 정지(K)"); 
  }
  angle = constrain(angle, 0, 180);
  myservo.write(angle);
}
// ============== Main Loop ==============
void loop() {
  uint32_t now = millis();

  // 1. SPI 명령 처리 (최우선 순위)
  if (cmdReceived) {
    String cmd = String((char*)lastReceivedCmd);
    cmdReceived = false;

    Serial.print(F("[SPI] CMD: ")); Serial.println(cmd);

    // 새로운 명령 수신 시 기본적으로 모드 해제 (단, 개별 핀 제어 명령 'h', 'j', 'k'는 제외)
    if (cmd != "l" && cmd != "p" && cmd != "PID" && cmd != "MOVE" && cmd != "1" && cmd != "c" && cmd != "CHECK" && 
        cmd != "h" && cmd != "j" && cmd != "k") {
      isLineTracing = false;
      isPIDTracing = false;
      isAutoMode = false;
      isCheckSensors = false;
      motor.stop(); 
    }

    if (cmd == "w" || cmd == "FWD") motor.forward();
    else if (cmd == "s" || cmd == "BWD") motor.backward();
    else if (cmd == "a" || cmd == "LEFT") motor.turnLeft(turnPwmValue);
    else if (cmd == "d" || cmd == "RIGHT") motor.turnRight(turnPwmValue);
    else if (cmd == "x" || cmd == "STOP" || cmd == "ESTOP") motor.stop();
    else if (cmd == "l" || cmd == "LINE") {
      isLineTracing = true;
      isPIDTracing = false;
      isCheckSensors = false;
    }
    else if (cmd == "p" || cmd == "PID" || cmd == "MOVE") {
      isPIDTracing = true;
      isLineTracing = false;
      isAutoMode = false;
      isCheckSensors = false;
    }
    else if (cmd == "1" || cmd == "AUTO") {
      isAutoMode = true;          
      isLineTracing = false;      
      isPIDTracing = false;       
      isCheckSensors = false;
      autoModeTimer = now;   
      autoModeStep = 0;           
      motor.forward();
    }
    else if (cmd == "c" || cmd == "CHECK") {
      isCheckSensors = !isCheckSensors;
      if (isCheckSensors) {
          isLineTracing = false;
          isPIDTracing = false;
          isAutoMode = false;
          motor.stop();
          Serial.println(F("[MODE] Sensor Check ON (Motors Halted)"));
      } else {
          Serial.println(F("[MODE] Sensor Check OFF"));
      }
    }
    else if (cmd == "2") {
      motor.turnLeft();
    }
    else if (cmd == "h" || cmd == "j" || cmd == "k") {
      processCommand(cmd.charAt(0));
    }

  }

  // 2. 센서 읽기
  uint16_t position = 0;
  if (isPIDTracing || isAutoMode || isCheckSensors) {
      position = qtr.readLineBlack(sensorValues);
  }

  // 3. 라인트레이싱 및 오토 모드 로직 실행
  if (isAutoMode) {
    if (now - autoModeTimer >= 1000) {
      autoModeTimer = now;
      autoModeStep = (autoModeStep + 1) % 2;
      if (autoModeStep == 0) {
        motor.forward();
        Serial.println(F("[AUTO] Forwarding..."));
      } else {
        motor.turnLeft();
        Serial.println(F("[AUTO] Turning Left..."));
      }
    }
  } else if (isLineTracing) {
    // lineTracer.runBasic(); // 이 부분은 QTR 센서로 대체되거나 다른 방식으로 구현될 수 있음
    // 현재는 isLineTracing이 'l' 명령으로 켜지지만, 실제 동작은 없음.
    // 필요하다면 QTR 센서의 디지털 값으로 간단한 라인 추적 로직을 여기에 추가할 수 있음.
    motor.forward(); // 임시로 그냥 전진
  } else if (isPIDTracing) {
    // 라인을 놓쳤는지 확인 (모든 센서가 흰색 바닥일 때)
    bool lineVisible = false;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 200) { // 200 이상인 센서가 하나라도 있으면 라인이 있는 것
            lineVisible = true;
            break;
        }
    }

    if (!lineVisible) {
        motor.stop();
        isPIDTracing = false;
        Serial.println(F("[STOP] Line Lost! (All White)"));
    } else {
        // QTRSensors 라이브러리를 사용한 정밀 PID 제어
        int error = position - 3500; // 0~7000의 중간값은 3500
    
    int P = error;
    int D = error - lastError;
    lastError = error;
    
    int motorSpeedChange = P * Kp + D * Kd;
    
    int leftMotorSpeed = pwmValue - motorSpeedChange;
    int rightMotorSpeed = pwmValue + motorSpeedChange;
    
    // 속도 제한 (0 ~ maxPIDSpeed)
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxPIDSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxPIDSpeed);
    
    motor.setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    }
  }

  // 4. 상태 LED & 디버그 출력
  digitalWrite(DEBUG_LED, (now - lastPacketTime < 300));

  // 5. QTR 센서 값 디버깅 출력
  static unsigned long lastQtrPrint = 0;
  if ((isPIDTracing || isCheckSensors) && (now - lastQtrPrint > 200)) {
    lastQtrPrint = now;
    
    Serial.print(F("Pos: ")); Serial.print(position);
    Serial.print(F(" | ["));
    for (uint8_t i = 0; i < SensorCount; i++) {
        // 보정된 값이 500 이상이면 라인이 있다고 판단 (O), 아니면 (.)
        if (sensorValues[i] > 500) Serial.print(F("O"));
        else Serial.print(F("."));
    }
    Serial.print(F("] | "));

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(F("\t"));
    }
    Serial.println();
  }
  
  delay(1);
}
