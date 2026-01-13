/**
 * @file linetrace.ino
 * @brief 독립형 PID 라인트레이서 (명령어 없이 자동 동작)
 * 
 * 동작 방식:
 * 1. 전원이 켜지면 5초 동안 캘리브레이션(센서 보정) 모드로 진입합니다.
 *    - 이 5초 동안 로봇을 검은 선 위에서 좌우로 왔다갔다 흔들어주세요.
 *    - 모든 센서가 검은색과 흰색을 최소 한 번씩은 봐야 정확히 인식됩니다.
 * 2. 5초 후 보정이 끝나면 자동으로 라인트레이싱(PID 제어)을 시작합니다.
 * 3. 별도의 정지 명령은 없으며, 멈추려면 전원을 꺼야 합니다.
 */

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

// QTR 라인 센서 핀 (28~32번 핀 사용 - 5채널로 변경)
const uint8_t qtrPins[] = {28, 29, 30, 31, 32};

// 디버그 확인용 LED (보정 중일 때 켜짐)
#define DEBUG_LED 17

// ============== 전역 변수 ==============
int baseSpeed = 100; // 기본 주행 속도 (필요시 수정하세요)

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// LineTracer 객체 생성
LineTracer lineTracer(motor);

// ============== 초기 설정 (Setup) ==============
void setup() {
  Serial.begin(115200);
  pinMode(DEBUG_LED, OUTPUT);
  
  // 모터 초기화
  motor.begin();
  motor.setSpeed(baseSpeed);
  motor.stop(); 

  // 라인 센서 초기화
  lineTracer.begin(qtrPins);
  lineTracer.setBaseSpeed(baseSpeed);

  Serial.println(F("\n=== 라인트레이서 시작 (Digital) ==="));
  Serial.println(F("디지털 센서 모드로 동작합니다. (보정 없음)"));
  
  // 캘리브레이션 불필요 (하드웨어 감도 조절)
  // lineTracer.setGenericCalibration(); // [삭제]
  
  delay(500); // 0.5초 대기
  Serial.println(F("[완료] 주행 준비 끝"));
  
  digitalWrite(DEBUG_LED, LOW); // 보정 끝 LED 꺼짐
  delay(1000); // 1초 대기 후 출발
  
  Serial.println(F(">> PID 주행을 시작합니다! (멈추려면 전원 OFF)"));
}

// ============== 메인 루프 (Loop) ==============
void loop() {
  // PID 라인트레이싱 실행
  // (LINETRACE.cpp 내부에서 센서 값을 읽고 모터 속도를 자동 조절합니다)
  lineTracer.runPID();
  
  // (디버깅용) 0.5초마다 현재 센서값 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lineTracer.printSensorValues(); // 필요하면 주석 해제하여 값 확인
    lastPrint = millis();
  }
}
