#include "MotorDrive.h"
#include "LINETRACE.h"

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

// 모터 1 (왼쪽)
#define MOTOR1_R_EN  5
#define MOTOR1_L_EN  4
#define MOTOR1_RPWM  3
#define MOTOR1_LPWM  2

// 모터 2 (오른쪽)
#define MOTOR2_R_EN  10
#define MOTOR2_L_EN  11
#define MOTOR2_RPWM  8
#define MOTOR2_LPWM  9

// QTR 라인 센서 핀 (28, 29, 30, 31, 32, 33, 34, 35)
const uint8_t qtrPins[] = {28, 29, 30, 31, 32, 33, 34, 35};

// 디버그용 LED
#define DEBUG_LED 17

#include <SPI.h>

// ============== 전역 변수 ==============
int pwmValue = DEFAULT_SPEED;       // 현재 PWM 값

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// LineTracer 객체 생성
LineTracer lineTracer(motor);

bool isLineTracing = false;
bool isPIDTracing = false;

// SPI 수신 패킷 처리용
volatile char spiCommand = 0;
volatile bool newCommand = false;

// ============== SPI 인터럽트 핸들러 (Slave) ==============
ISR(SPI_STC_vect) {
  char c = SPDR;  // 수신된 데이터
  
  // [디버깅] 데이터 수신 시 LED 토글 (물리 연결 확인용)
  digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED)); 
  
  // 패킷 시작('<')과 종료('>')는 무시하고 명령어만 받음
  if (c != '<' && c != '>') {
    spiCommand = c;
    newCommand = true;
  }
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, HIGH); // 시작 시 LED 켜기
  delay(500);
  digitalWrite(DEBUG_LED, LOW); // 끄기
  
  motor.begin();
  motor.setSpeed(pwmValue);
  motor.stop(); 

  // QTR 센서 초기화
  lineTracer.begin(qtrPins);
  lineTracer.setBaseSpeed(pwmValue);

  // ========== SPI Slave 모드 설정 (Mega 2560) ==========
  pinMode(53, INPUT); // SS 핀은 반드시 INPUT으로 설정해야 Slave로 동작
  pinMode(50, OUTPUT); // MISO는 출력
  
  // SPI 제어 레지스터 설정: SPI 활성화 + 인터럽트 활성화
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE); 

  Serial.println(F("\n=== Mega Motor Driver (SPI Slave) ==="));
  Serial.println(F("Waiting for SPI commands..."));
  
  // 캘리브레이션은 수동으로 변경 (시작 시 바로 돌지 않도록)
  // lineTracer.calibrate(); 
}

// ============== Main Loop ==============
void loop() {
  uint32_t now = millis();

  // 1. SPI 명령 처리
  if (newCommand) {
    char cmd = spiCommand;
    newCommand = false;
    processCommand(cmd);
  }

  // 2. 라인트레이싱 로직 실행 (활성화 된 경우만)
  if (isLineTracing) {
    lineTracer.runBasic();
  } else if (isPIDTracing) {
    lineTracer.runPID();
    if (now % 100 == 0) lineTracer.printSensorValues(); 
  }

  // 3. 상태 LED
  digitalWrite(DEBUG_LED, (now % 500 < 250)); 
}

void processCommand(char cmd) {
  Serial.print("CMD: "); Serial.println(cmd);

  switch(cmd) {
    case 'w': // Forward
      isLineTracing = false; isPIDTracing = false;
      motor.forward(pwmValue);
      break;
    case 's': // Backward
      isLineTracing = false; isPIDTracing = false;
      motor.backward(pwmValue);
      break;
    case 'a': // Left
      isLineTracing = false; isPIDTracing = false;
      motor.turnLeft(pwmValue);
      break;
    case 'd': // Right
      isLineTracing = false; isPIDTracing = false;
      motor.turnRight(pwmValue);
      break;
    case 'x': // Stop
      isLineTracing = false; isPIDTracing = false;
      motor.stop();
      break;
    case 'l': // Line Trace Start
      if (!isLineTracing) {
          isLineTracing = true; isPIDTracing = false; 
          motor.setSpeed(pwmValue);
      }
      break;
    case 'p': // PID Trace Start
      if (!isPIDTracing) {
          Serial.println("Calibrating...");
          lineTracer.calibrate(); // PID 시작 전 캘리브레이션
          isPIDTracing = true; isLineTracing = false;
      }
      break;
  }
}

