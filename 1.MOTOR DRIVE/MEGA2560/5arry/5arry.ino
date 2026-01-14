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
const uint8_t sensorPins[] = {28, 29, 30, 31, 32};

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
  // 1. 디지털 센서 읽기 (Pins 28-32)
  int s[5];
  int activeCount = 0;
  long weightSum = 0;
  bool isLineFound = false;

  for (uint8_t i = 0; i < SensorCount; i++) {
    // 센서가 검은 선을 감지했을 때 HIGH(1)인지 LOW(0)인지 확인 필요
    // 일반적인 TCRT5000 모듈은 검은색일 때 LOW(0)인 경우가 많습니다.
    // 여기서는 검은선 감지 시 HIGH(1)인 경우로 작성합니다. 
    // 반대로 작동한다면 digitalRead 앞에 ! 를 붙이세요.
    s[i] = digitalRead(sensorPins[i]);
    
    if (s[i] == HIGH) { // 검은 선 감지
      // 가중치 부여: -2000, -1000, 0, 1000, 2000
      weightSum += (int16_t(i) - 2) * 1000;
      activeCount++;
      isLineFound = true;
    }
  }

  // 2. 특수 상황 처리
  if (!isLineFound) {
    // 라인을 완전히 놓쳤을 때: 정지 (또는 마지막 에러 방향으로 회전 시도 가능)
    motor.stop();
    return;
  }
  
  if (activeCount == SensorCount) {
    // 모든 센서가 검은색 (정지선 감지)
    motor.stop();
    Serial.println(F("!!! 정지선 감지 - 정지 !!!"));
    delay(1000);
    return;
  }

  // 3. 에러 계산 (가중치 평균)
  int16_t error = weightSum / activeCount;

  // 4. PID 제어 계산
  integral += error;
  integral = constrain(integral, -1000, 1000); // 적분값 제한
  int16_t derivative = error - lastError;
  
  // 디지털 센서는 반응이 계단식이므로 Kp 값을 아날로그보다 높여야 할 수 있습니다.
  int16_t correction = (int16_t)(Kp * (error / 1000.0) + Ki * (integral / 1000.0) + Kd * (derivative / 1000.0));
  lastError = error;

  // 5. 가변 속도 및 모터 제어
  int currentBaseSpeed = baseSpeed;
  if (abs(error) > 1000) currentBaseSpeed = baseSpeed * 0.7; // 급커브 감속

  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_MOTOR_SPEED);

  motor.setMotorSpeeds(leftSpeed, rightSpeed);

  // 디버깅 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("Sensors: ");
    for(int i=0; i<5; i++) { Serial.print(s[i]); Serial.print(" "); }
    Serial.print("| Err: "); Serial.print(error);
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" R: "); Serial.println(rightSpeed);
    lastPrint = millis();
  }
  }

