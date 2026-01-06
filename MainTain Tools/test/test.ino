/**
 * @file test.ino
 * @brief MotorDrive 라이브러리를 사용한 시리얼 키 입력 주행 테스트
 * @version 1.0
 * 
 * 시리얼 모니터에서 키 입력으로 모터 제어
 * MotorDrive.h / MotorDrive.cpp 참조
 * 보드: Arduino Mega / Uno
 */

#include "MotorDrive.h"

// ============== L298N 모터 드라이버 핀 설정 ==============
#define ENABLE_A 8    // 모터 A PWM
#define ENABLE_B 9    // 모터 B PWM
#define IN1_A    10   // 모터 A 방향 1
#define IN2_A    11   // 모터 A 방향 2
#define IN3_B    12   // 모터 B 방향 1
#define IN4_B    13   // 모터 B 방향 2

// 라인 센서 핀 (필요시 사용)
int lineSensorPins[] = {A0, A1, A2, A3, A4};
#define LINE_SENSOR_COUNT 5

// MotorDrive 객체 생성
MotorDrive motor(IN1_A, IN2_A, ENABLE_A, IN3_B, IN4_B, ENABLE_B);

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  
  // 모터 드라이버 초기화
  motor.begin();
  motor.setSpeed(DEFAULT_SPEED);
  
  // 라인 센서 설정 (옵션)
  motor.setLineSensors(lineSensorPins, LINE_SENSOR_COUNT);
  motor.setPIDGains(1.0, 0.0, 0.5);
  
  Serial.println(F("=== Motor Test (MotorDrive) ==="));
  Serial.println(F("w: Forward    s: Backward"));
  Serial.println(F("a: Left       d: Right"));
  Serial.println(F("q: CurveLeft  e: CurveRight"));
  Serial.println(F("x: Stop"));
  Serial.println(F("+: Speed Up   -: Speed Down"));
  Serial.println(F("l: LineTrace  p: LineTrace PID"));
  Serial.println(F("==============================="));
}

// ============== Main Loop ==============
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'w': case 'W':  // 전진
        motor.forward();
        Serial.println(F(">> Forward"));
        break;
        
      case 's': case 'S':  // 후진
        motor.backward();
        Serial.println(F(">> Backward"));
        break;
        
      case 'a': case 'A':  // 좌회전 (제자리)
        motor.turnLeft();
        Serial.println(F(">> Turn Left"));
        break;
        
      case 'd': case 'D':  // 우회전 (제자리)
        motor.turnRight();
        Serial.println(F(">> Turn Right"));
        break;
        
      case 'q': case 'Q':  // 좌측 곡선
        motor.curveLeft();
        Serial.println(F(">> Curve Left"));
        break;
        
      case 'e': case 'E':  // 우측 곡선
        motor.curveRight();
        Serial.println(F(">> Curve Right"));
        break;
        
      case 'x': case 'X':  // 정지
        motor.stop();
        Serial.println(F(">> Stop"));
        break;
        
      case '+': case '=':  // 속도 증가
        motor.speedUp(20);
        Serial.print(F(">> Speed: "));
        Serial.println(motor.getSpeed());
        break;
        
      case '-': case '_':  // 속도 감소
        motor.speedDown(20);
        Serial.print(F(">> Speed: "));
        Serial.println(motor.getSpeed());
        break;
        
      case 'l': case 'L':  // 라인트레이싱 (단순)
        motor.lineTrace();
        Serial.println(F(">> Line Trace"));
        break;
        
      case 'p': case 'P':  // 라인트레이싱 (PID)
        motor.lineTracePID();
        Serial.println(F(">> Line Trace PID"));
        break;
        
      case 'c': case 'C':  // 교차로 감지
        if (motor.isCrossDetected()) {
          Serial.println(F(">> Cross Detected!"));
        } else {
          Serial.println(F(">> No Cross"));
        }
        break;
        
      case 'i': case 'I':  // 라인 감지
        if (motor.isLineDetected()) {
          Serial.println(F(">> Line Detected!"));
        } else {
          Serial.println(F(">> No Line"));
        }
        break;
        
      case '?':  // 도움말
        Serial.println(F("\n=== Commands ==="));
        Serial.println(F("w/s: Forward/Backward"));
        Serial.println(F("a/d: Turn Left/Right"));
        Serial.println(F("q/e: Curve Left/Right"));
        Serial.println(F("x: Stop"));
        Serial.println(F("+/-: Speed Up/Down"));
        Serial.println(F("l/p: LineTrace/PID"));
        Serial.println(F("c/i: Cross/Line Check"));
        Serial.println(F("================\n"));
        break;
    }
  }
}
