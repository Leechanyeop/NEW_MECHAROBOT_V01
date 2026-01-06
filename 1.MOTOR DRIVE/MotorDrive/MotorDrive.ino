#include <SPI.h>
#include "MotorDrive.h"

// ============== 핀 설정 ==============


// L298N 모터 드라이버 핀
#define ENABLE_A 8    // 모터 A PWM
#define ENABLE_B 9    // 모터 B PWM
#define IN1_A    10   // 모터 A 방향 1
#define IN2_A    11   // 모터 A 방향 2
#define IN3_B    12   // 모터 B 방향 1
#define IN4_B    13   // 모터 B 방향 2

// 라인 센서 핀 (필요시 사용)
int lineSensorPins[] = {A0, A1, A2, A3, A4};
#define LINE_SENSOR_COUNT 5

// ============== 전역 변수 ==============
volatile long encoderPos = 0;   // 엔코더 위치
int pwmValue = DEFAULT_SPEED;   // 현재 PWM 값

// MotorDrive 객체 생성
MotorDrive motor(IN1_A, IN2_A, ENABLE_A, IN3_B, IN4_B, ENABLE_B);

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  
  // 모터 드라이버 초기화
  motor.begin();
  motor.setSpeed(pwmValue);
  
  // 라인 센서 설정 (라인트레이싱 사용시)
  motor.setLineSensors(lineSensorPins, LINE_SENSOR_COUNT);
  motor.setPIDGains(1.0, 0.0, 0.5);  // PID 게인 설정
  
  // 엔코더 핀 설정
 // pinMode(ENCODER_A, INPUT_PULLUP);
 // pinMode(ENCODER_B, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // SPI Slave 설정
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);   // SPI Enable
  SPCR |= _BV(SPIE);  // SPI Interrupt Enable

  Serial.println("=== SPI Slave Ready ===");
  Serial.println("MotorDrive initialized with L298N driver");
}


// ============== SPI 인터럽트 (Master 명령 수신) ==============
ISR(SPI_STC_vect) {
  char cmd = SPDR;  // Master가 보낸 명령

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
  // SPI 인터럽트로 대부분 처리됨
  Serial.print(" | Speed: ");
  Serial.print(motor.getSpeed());
  Serial.print(" | dir: ");
  Serial.println(" ");
  
  
  delay(100);
}