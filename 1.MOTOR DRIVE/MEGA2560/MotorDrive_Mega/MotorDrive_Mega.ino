#include <SPI.h>
#include "MotorDrive.h"
#include "LINETRACE.h"

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

/**
 * BTS7960 드라이버 연결 정보
 * ... (existing comments)
 */

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

// ... (existing SPI comments)

// SPI Slave 핀 (Mega 2560 전용)
#define SPI_SS_PIN   53
#define SPI_MISO_PIN 50
#define SPI_MOSI_PIN 51
#define SPI_SCK_PIN  52

// 디버그용 LED
#define DEBUG_LED 17

// ============== 전역 변수 ==============
int pwmValue = DEFAULT_SPEED;       // 현재 PWM 값
volatile char lastReceivedCmd = 0;  // 마지막 수신 명령
volatile bool cmdReceived = false;  // 명령 수신 플래그
volatile uint32_t lastPacketTime = 0; // 마지막 유효 패킷 수신 시간 (ms)

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// LineTracer 객체 생성
LineTracer lineTracer(motor);

bool isLineTracing = false;
bool isPIDTracing = false;

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  
  motor.begin();
  motor.setSpeed(pwmValue);
  motor.stop(); 

  // QTR 센서 초기화
  lineTracer.begin(qtrPins);
  lineTracer.setBaseSpeed(pwmValue);

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

  Serial.println(F("\n=== Mega Motor Driver (QTR Integrated) ==="));
  Serial.println(F("Protocol: < [CMD] >"));
}

// ============== SPI 인터럽트 ==============
ISR(SPI_STC_vect) {
  byte inByte = SPDR;
  static byte state = 0; 
  
  if (state == 0) {
    if (inByte == '<') state = 1;
  } 
  else if (state == 1) {
    lastReceivedCmd = inByte;
    state = 2;
    SPDR = inByte; 
  }
  else if (state == 2) {
    if (inByte == '>') {
      cmdReceived = true;
      lastPacketTime = millis();
    }
    state = 0;
  }
  
  if (state == 0) SPDR = 0x00;
}

// ============== Main Loop ==============
void loop() {
  uint32_t now = millis();

  // 1. 세이프티 워치독
  if (now - lastPacketTime > 700) {
    static bool watchDogAction = false;
    if (!watchDogAction) {
      motor.stop();
      isLineTracing = false;
      isPIDTracing = false;
      Serial.println(F("[WATCHDOG] Communication lost! Stopped."));
      watchDogAction = true;
    }
  }

  // 2. SPI 명령 처리
  if (cmdReceived) {
    char cmd = lastReceivedCmd;
    cmdReceived = false;

    Serial.print(F("[SPI] CMD: '")); Serial.print(cmd); Serial.println(F("'"));

    // 새로운 명령 수신 시 기본적으로 라인트레이싱 모드 해제 (명시적 명령 전까지)
    if (cmd != 'l' && cmd != 'p') {
      isLineTracing = false;
      isPIDTracing = false;
    }

    switch (cmd) {
      case 'w': motor.forward();   break;
      case 's': motor.backward();  break;
      case 'a': motor.turnLeft();  break;
      case 'd': motor.turnRight(); break;
      case 'x': motor.stop();      break;
      case '+': 
        pwmValue = constrain(pwmValue + 20, 0, 255);
        motor.setSpeed(pwmValue);
        lineTracer.setBaseSpeed(pwmValue);
        break;
      case '-': 
        pwmValue = constrain(pwmValue - 20, 0, 255);
        motor.setSpeed(pwmValue);
        lineTracer.setBaseSpeed(pwmValue);
        break;
      case 'l': 
        isLineTracing = true; 
        isPIDTracing = false;
        break;
      case 'p': 
        isPIDTracing = true;
        isLineTracing = false;
        break;
      case 'c': // Calibration command
        lineTracer.calibrate();
        break;
    }
  }

  // 3. 라인트레이싱 로직 실행
  if (isLineTracing) {
    lineTracer.runBasic();
  } else if (isPIDTracing) {
    lineTracer.runPID();
  }

  // 4. 상태 LED
  digitalWrite(DEBUG_LED, (now - lastPacketTime < 300));
  
  delay(1);
}

