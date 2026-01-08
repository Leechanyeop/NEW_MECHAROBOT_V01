#include <SPI.h>
#include "MotorDrive.h"

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

/**
 * BTS7960 드라이버 연결 정보
 * 
 * [드라이버 1 - 왼쪽 모터]
 * - R_EN: 5
 * - L_EN: 4
 * - RPWM: 7 (PWM)
 * - LPWM: 6 (PWM)
 * 
 * [드라이버 2 - 오른쪽 모터]
 * - R_EN2: 10
 * - L_EN2: 11
 * - RPWM2: 12 (PWM)
 * - LPWM2: 13 (PWM)
 */

// 모터 1 (왼쪽)
#define MOTOR1_R_EN  5
#define MOTOR1_L_EN  4
#define MOTOR1_RPWM  7
#define MOTOR1_LPWM  6

// 모터 2 (오른쪽)
#define MOTOR2_R_EN  10
#define MOTOR2_L_EN  11
#define MOTOR2_RPWM  12
#define MOTOR2_LPWM  13

// 라인 센서 핀 (필요시 사용)
int lineSensorPins[] = {A0, A1, A2, A3, A4};
#define LINE_SENSOR_COUNT 5

/**
 * SPI 통신 프로토콜 상세 정보
 * 
 * 1. 핀 연결 (UNO Master <-> Mega Slave)
 *    - Master 13 (SCK)  <-> Slave 52
 *    - Master 11 (MOSI) <-> Slave 51
 *    - Master 12 (MISO) <-> Slave 50
 *    - Master 8 (SS)    <-> Slave 53 (반드시 GND 공유 필수)
 * 
 * 2. 전송 명령 (Master -> Slave, 1 byte)
 *    - 'w': 전진 (Forward)
 *    - 's': 후진 (Backward)
 *    - 'a': 좌회전 (Turn Left)
 *    - 'd': 우회전 (Turn Right)
 *    - 'x': 정지 (Stop)
 *    - '+': 속도 증가 (Speed Up, +20)
 *    - '-': 속도 감소 (Speed Down, -20)
 *    - 'l': 라인트레이싱 기본 (Line Trace)
 *    - 'p': 라인트레이싱 PID (Line Trace PID)
 * 
 * 3. 응답 데이터 (Slave -> Master, long type / 4 bytes)
 *    - 현재 엔코더 위치(encoderPos)를 4바이트에 나눠서 순차적으로 응답
 */

// SPI Slave 핀 (Mega 2560 전용)
#define SPI_SS_PIN   53
#define SPI_MISO_PIN 50
#define SPI_MOSI_PIN 51
#define SPI_SCK_PIN  52

// 디버그용 LED
#define DEBUG_LED 13

// ============== 전역 변수 ==============
volatile long encoderPos = 0;       // 엔코더 위치
int pwmValue = DEFAULT_SPEED;       // 현재 PWM 값
volatile char lastReceivedCmd = 0;  // 마지막 수신 명령 (디버그용)
volatile bool cmdReceived = false;  // 명령 수신 플래그
volatile uint32_t spiInterruptCount = 0;  // SPI 인터럽트 카운터

// MotorDrive 객체 생성 (BTS7960 인터페이스로 변경됨)
// MotorDrive(R_EN1, L_EN1, RPWM1, LPWM1, R_EN2, L_EN2, RPWM2, LPWM2)
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // 시리얼 준비 대기
  
  // 디버그 LED 설정
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  
  // 모터 드라이버 초기화
  motor.begin();
  motor.setSpeed(pwmValue);
  
  // 라인 센서 설정 (라인트레이싱 사용시)
  motor.setLineSensors(lineSensorPins, LINE_SENSOR_COUNT);
  motor.setPIDGains(1.0, 0.0, 0.5);  // PID 게인 설정

  // ========== SPI Slave 설정 (Mega 2560) ==========
  // 1. 핀 모드 설정 (중요!)
  pinMode(SPI_SS_PIN, INPUT);      // SS: 반드시 INPUT
  pinMode(SPI_MOSI_PIN, INPUT);    // MOSI: INPUT
  pinMode(SPI_MISO_PIN, OUTPUT);   // MISO: OUTPUT (슬레이브가 응답 전송)
  pinMode(SPI_SCK_PIN, INPUT);     // SCK: INPUT
  
  // 2. DDR 레지스터 명시적 설정 (Mega 2560 PORTB)
  // PB0=SS(53), PB1=SCK(52), PB2=MOSI(51), PB3=MISO(50)
  DDRB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2));  // SS, SCK, MOSI = INPUT
  DDRB |= _BV(PB3);                           // MISO = OUTPUT
  
  // 3. SPI 레지스터 설정
  SPCR = 0;                // 초기화
  SPCR |= _BV(SPE);        // SPI Enable (슬레이브 모드는 MSTR=0)
  SPCR |= _BV(SPIE);       // SPI Interrupt Enable
  // CPOL=0, CPHA=0 (SPI Mode 0) - 기본값
  
  // 4. SPDR 초기값 설정 (첫 번째 응답용)
  SPDR = 0xAA;             // 디버그용: 슬레이브가 살아있다는 표시
  
  // 5. SPI 상태 레지스터 클리어 (이전 데이터 무시)
  volatile byte dummy = SPSR;
  dummy = SPDR;
  (void)dummy;

  Serial.println(F(""));
  Serial.println(F("=== SPI Slave Ready (Mega 2560) ==="));
  Serial.println(F("Pin Config: SS=53, SCK=52, MOSI=51, MISO=50"));
  Serial.print(F("SPCR = 0x")); Serial.println(SPCR, HEX);
  Serial.print(F("SPSR = 0x")); Serial.println(SPSR, HEX);
  Serial.println(F("MotorDrive initialized with L298N driver"));
  
  // SCK, MOSI 핀 현재 상태 출력
  Serial.println(F(""));
  Serial.println(F("[PIN TEST] Current pin states:"));
  Serial.print(F("  SS(53)="));   Serial.println(digitalRead(SPI_SS_PIN) ? "HIGH" : "LOW");
  Serial.print(F("  SCK(52)="));  Serial.println(digitalRead(SPI_SCK_PIN) ? "HIGH" : "LOW");
  Serial.print(F("  MOSI(51)=")); Serial.println(digitalRead(SPI_MOSI_PIN) ? "HIGH" : "LOW");
  Serial.print(F("  MISO(50)=")); Serial.println(digitalRead(SPI_MISO_PIN) ? "HIGH" : "LOW");
  Serial.println(F(""));
  Serial.println(F("Waiting for SPI commands..."));
}

// ============== SPI 인터럽트 (Master 명령 수신) ==============
ISR(SPI_STC_vect) {
  // 디버그: LED 토글 (인터럽트 발생 확인)
  digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  
  spiInterruptCount++;  // 인터럽트 카운트 증가
  
  char cmd = SPDR;  // Master가 보낸 명령
  lastReceivedCmd = cmd;  // 디버그용 저장
  cmdReceived = true;     // 수신 플래그

  // 명령 처리 - MotorDrive 클래스 메서드 사용
  switch (cmd) {
    case 'w':  // 전진
      motor.forward();
      break;
    case 's':  // 후진
      motor.backward();
      break;
    case 'a':  // 좌회전
      motor.turnLeft();
      break;
    case 'd':  // 우회전
      motor.turnRight();
      break;
    case 'x':  // 정지
      motor.stop();
      break;
    case '+':  // 속도 증가
      motor.speedUp(20);
      pwmValue = motor.getSpeed();
      break;
    case '-':  // 속도 감소
      motor.speedDown(20);
      pwmValue = motor.getSpeed();
      break;
    case 'l':  // 라인트레이싱 모드 (1회 실행)
      motor.lineTrace();
      break;
    case 'p':  // 라인트레이싱 PID 모드 (1회 실행)
      motor.lineTracePID();
      break;
    default:
      break;
  }

  // 응답으로 엔코더 값 전송
  static byte idx = 0;
  union {
    long val;
    byte b[4];
  } data;

  data.val = encoderPos;
  SPDR = data.b[idx];
  idx++;
  if (idx >= 4) idx = 0;
}

// ============== Main Loop ==============
void loop() {
  // SPI 명령 수신 확인 (디버그)
  if (cmdReceived) {
    Serial.print(F("[SPI] Received: '"));
    Serial.print(lastReceivedCmd);
    Serial.print(F("' (0x"));
    Serial.print((byte)lastReceivedCmd, HEX);
    Serial.print(F(") | ISR Count: "));
    Serial.println(spiInterruptCount);
    cmdReceived = false;
  }
  
  // SS 핀 상태 모니터링 (디버그)
  static bool lastSSState = HIGH;
  bool currentSS = digitalRead(SPI_SS_PIN);
  if (currentSS != lastSSState) {
    Serial.print(F("[SS] Pin 53 = "));
    Serial.print(currentSS ? "HIGH" : "LOW");
    Serial.print(F(" | SPCR=0x"));
    Serial.print(SPCR, HEX);
    Serial.print(F(" | SPSR=0x"));
    Serial.println(SPSR, HEX);
    lastSSState = currentSS;
  }
  
  // SCK 핀 상태 모니터링 (디버그)
  static bool lastSCKState = LOW;
  bool currentSCK = digitalRead(SPI_SCK_PIN);
  if (currentSCK != lastSCKState) {
    Serial.print(F("[SCK] Pin 52 = "));
    Serial.println(currentSCK ? "HIGH" : "LOW");
    lastSCKState = currentSCK;
  }
  
  // MOSI 핀 상태 모니터링 (디버그)
  static bool lastMOSIState = LOW;
  bool currentMOSI = digitalRead(SPI_MOSI_PIN);
  if (currentMOSI != lastMOSIState) {
    Serial.print(F("[MOSI] Pin 51 = "));
    Serial.println(currentMOSI ? "HIGH" : "LOW");
    lastMOSIState = currentMOSI;
  }
  
  // 주기적 상태 출력 (5초마다)
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 5000) {
    Serial.print(F("[STATUS] SPI ISR Count: "));
    Serial.print(spiInterruptCount);
    Serial.print(F(" | SPCR=0x"));
    Serial.print(SPCR, HEX);
    Serial.print(F(" | SS="));
    Serial.println(digitalRead(SPI_SS_PIN) ? "HIGH" : "LOW");
    lastStatus = millis();
  }
  
  delay(10);  // 더 빠른 폴링
}
