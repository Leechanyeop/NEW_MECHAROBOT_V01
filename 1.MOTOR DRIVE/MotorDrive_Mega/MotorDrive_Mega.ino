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
int pwmValue = DEFAULT_SPEED;       // 현재 PWM 값
volatile char lastReceivedCmd = 0;  // 마지막 수신 명령
volatile bool cmdReceived = false;  // 명령 수신 플래그
volatile uint32_t lastPacketTime = 0; // 마지막 유효 패킷 수신 시간 (ms)

// SPI 패킷 상태 머신 변수
volatile byte packetBuffer[3];
volatile byte packetIdx = 0;

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  
  motor.begin();
  motor.setSpeed(pwmValue);
  motor.stop(); // 초기 상태는 반드시 정합

  // ========== SPI Slave 설정 (Mega 2560) ==========
  pinMode(SPI_SS_PIN, INPUT_PULLUP);
  pinMode(SPI_MOSI_PIN, INPUT);
  pinMode(SPI_MISO_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, INPUT);
  
  DDRB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2)); 
  DDRB |= _BV(PB3); 
  
  SPCR = _BV(SPE) | _BV(SPIE); // SPI Enable, Interrupt Enable
  
  SPDR = 0x00; // 초기 응답값
  
  volatile byte dummy = SPSR;
  dummy = SPDR;
  (void)dummy;

  lastPacketTime = millis();

  Serial.println(F("\n=== Mega Motor Driver (ASCII SPI) ==="));
  Serial.println(F("Protocol: < [CMD] >"));
  Serial.println(F("Safety: Watchdog active (700ms)"));
}

// ============== SPI 인터럽트 (ASCII 패킷 처리) ==============
ISR(SPI_STC_vect) {
  byte inByte = SPDR;
  static byte state = 0; // 0: Wait Header, 1: Wait CMD, 2: Wait Footer
  
  if (state == 0) {
    if (inByte == '<') {
      state = 1;
    }
  } 
  else if (state == 1) {
    lastReceivedCmd = inByte;
    state = 2;
    // 명령어를 받았으므로 다음 전송(Footer 수신 시) 때 보낼 ACK 미리 설정
    SPDR = inByte; 
  }
  else if (state == 2) {
    if (inByte == '>') {
      cmdReceived = true;
      lastPacketTime = millis();
    }
    state = 0;
  }
  
  // Footer 수신 이후에는 다음 Header 대기를 위해 초기화 응답
  if (state == 0) SPDR = 0x00;
}

// ============== Main Loop ==============
void loop() {
  uint32_t now = millis();

  // 1. 세이프티 워치독 (Safety Watchdog)
  // 700ms 동안 유효한 패킷이 없으면 강제 정지
  if (now - lastPacketTime > 700) {
    static bool watchDogAction = false;
    if (!watchDogAction) {
      motor.stop();
      Serial.println(F("[WATCHDOG] Communication lost! Motor Stopped."));
      watchDogAction = true;
    }
    // 패킷 수신 시 watchDogAction이 false로 리셋되도록 패킷 처리부 수정 필요 없음 (stop()은 중복 호출되어도 안전)
  }

  // 2. SPI 명령 처리
  if (cmdReceived) {
    char cmd = lastReceivedCmd;
    cmdReceived = false;
    // 워치독 액션 상태 해제
    // (여기서 lastPacketTime이 ISR에서 업데이트되므로 자연스럽게 해제됨)

    Serial.print(F("[SPI] CMD: '")); Serial.print(cmd); Serial.println(F("'"));

    switch (cmd) {
      case 'w': motor.forward();   break;
      case 's': motor.backward();  break;
      case 'a': motor.turnLeft();  break;
      case 'd': motor.turnRight(); break;
      case 'x': motor.stop();      break;
      case '+': motor.speedUp(20); break;
      case '-': motor.speedDown(20); break;
      case 'l': motor.lineTrace(); break;
      case 'p': motor.lineTracePID(); break;
    }
  }

  // 3. 상태 LED (통신 중임을 표시)
  digitalWrite(DEBUG_LED, (now - lastPacketTime < 300));
  
  delay(1);
}
