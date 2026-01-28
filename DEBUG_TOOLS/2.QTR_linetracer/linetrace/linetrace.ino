#include "MotorDrive.h"

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

// 모터 1 (왼쪽)
#define MOTOR1_RPWM  2
#define MOTOR1_LPWM  3
#define MOTOR1_R_EN  4
#define MOTOR1_L_EN  5

// 모터 2 (오른쪽)
#define MOTOR2_RPWM  8
#define MOTOR2_LPWM  9
#define MOTOR2_R_EN  10
#define MOTOR2_L_EN  11

// 5채널 라인 센서 설정 (디지털 IR 센서 기준)
const uint8_t SensorCount = 5;
const uint8_t sensorPins[] = {26, 29, 30, 31, 32};

// 디버그 확인용 LED
#define DEBUG_LED 13

// ============== PID 제어 변수 ==============
float Kp = 40.0;  // 디지털 센서용 게인 (새로 튜닝 필요)
float Ki = 0.0;
float Kd = 20.0;

int16_t lastError = 0;
float integral = 0;

int baseSpeed = 64; 
const int MAX_MOTOR_SPEED = 128; 

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// ============== 초기 설정 (Setup) ==============
void setup() {
  Serial.begin(115200);
  pinMode(DEBUG_LED, OUTPUT);

  // 1. 모터 초기화
  motor.begin();
  motor.stop(); 

  // 2. 센서 핀 초기화
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println(F("\n=== 5채널 디지털 라인트레이서 시작 ==="));
  
  // 출발 카운트다운
  Serial.println(F("주행 시작 3초 전..."));
  for (int i = 3; i > 0; i--) {
    Serial.print(i); Serial.println(F("..."));
    digitalWrite(DEBUG_LED, HIGH); delay(500); 
    digitalWrite(DEBUG_LED, LOW);  delay(500);
  }
  
  Serial.println(F(">> 주행 시작!"));
}

// ============== 메인 루프 (Loop) ==============
void loop() {
  // 1. 센서 읽기 및 캘리브레이션 적용 (고정값: Min 550, Max 2500)
  uint16_t sensorValues[SensorCount];
  uint32_t avg = 0;
  uint32_t sum = 0;
  bool isLineFound = false;
  bool isAllBlack = true;

  for (uint8_t i = 0; i < SensorCount; i++) {
    // 8채널과 동일한 핀(28-32) 사용 시 analogRead 가능한 핀인지 확인 필요
    // Mega 2560에서 28번 등은 디지털 핀입니다. 
    // 만약 아날로그 값을 쓰시려면 A0~A15 핀을 쓰셔야 합니다.
    // 여기서는 일단 기존 핀 28~32에서 analogRead를 시도하지만, 
    // 하드웨어 연결에 따라 digitalRead(0 or 1000으로 처리)가 될 수 있습니다.
    uint16_t rawValue = analogRead(sensorPins[i]); 
    
    // 고정된 캘리브레이션 적용 (Min: 550, Max: 2500)
    if (rawValue <= 550) rawValue = 0;
    else if (rawValue >= 2500) rawValue = 1000;
    else rawValue = map(rawValue, 550, 2500, 0, 1000);
    
    sensorValues[i] = rawValue;

    // 라인 감지 여부 (하나라도 200 이상이면 라인 있음)
    if (rawValue > 200) isLineFound = true;
    // 전체 검은색 여부 (하나라도 700 미만이면 false)
    if (rawValue < 700) isAllBlack = false;

    // 가중치 평균 계산 (0~4000 범위의 Position 생성)
    avg += (uint32_t)rawValue * (i * 1000);
    sum += (uint32_t)rawValue;
  }

  // 2. 특수 상황 처리 (정지)
  if (!isLineFound) {
    motor.stop();
    return;
  }
  
  if (isAllBlack) {
    motor.stop();
    Serial.println(F("!!! 정지선 감지 !!!"));
    delay(500);
    return;
  }

  // 3. 위치 및 에러 계산
  uint16_t position = avg / sum; // 0 ~ 4000 (중앙 2000)
  int16_t error = 2000 - (int16_t)position;

  // 3. PID 제어 계산
  integral += error;
  integral = constrain(integral, -1000, 1000);
  int16_t derivative = error - lastError;
  
  int16_t correction = (int16_t)(Kp * (error / 100.0) + Ki * integral + Kd * (derivative / 100.0));
  lastError = error;

  // 4. 가변 속도 (심한 커브 시 감속)
  int currentBaseSpeed = baseSpeed;
  if (abs(error) > 1500) currentBaseSpeed = baseSpeed * 0.6;

  // 5. 모터 속도 설정
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_MOTOR_SPEED);

  motor.setMotorSpeeds(leftSpeed, rightSpeed);

  // 디버깅 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("Sensors: ");
    for(int i=0; i<5; i++) { Serial.print(sensorValues[i]); Serial.print(" "); }
    Serial.print("| Err: "); Serial.print(error);
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" R: "); Serial.println(rightSpeed);
    lastPrint = millis();
  }
}
