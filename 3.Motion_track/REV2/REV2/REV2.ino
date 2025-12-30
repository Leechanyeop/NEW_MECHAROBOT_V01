// 5채널 라인트레이서 PID 제어 예제
// 센서 배열: S0, S1, S2, S3, S4 (왼쪽 → 오른쪽)
// 모터: LeftMotor, RightMotor
// Mega_LineFollower_AFMotor_R4.ino
#include "AFMotor_R4.h"   // 변경된 라이브러리 헤더

// 모터 설정 (Adafruit Motor Shield - R4 라이브러리 사용)
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

#define NUM_SENSORS 5
int sensors[NUM_SENSORS] = {A0, A1, A2, A3, A4};  // 라인트레이서 센서 핀
int sensorValues[NUM_SENSORS];
leftMotorPWM
rightMotorPWM
// PID 게인 값 (튜닝 필요)
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

float error = 0, lastError = 0, integral = 0;
int baseSpeed = 150;  // 기본 모터 속도

void setup() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i], INPUT);
  }
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
}

void loop() {
  // 센서 값 읽기
  int position = readLinePosition();

  // 오차 계산 (중앙: 2000, 범위: 0~4000)
  error = position - 2000;

  // PID 제어 계산
  integral += error;
  float derivative = error - lastError;
  float output = Kp * error + Ki * integral + Kd * derivative;

  // 모터 속도 제어
  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  // 속도 제한
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);

  lastError = error;
}

// 라인 위치 계산 함수 (가중 평균 방식)
int readLinePosition() {
  long avg = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensors[i]);
    avg += (long)sensorValues[i] * (i * 1000);  // 위치 가중치
    sum += sensorValues[i];
  }

  if (sum == 0) return 2000;  // 라인 못 찾으면 중앙 반환
  return avg / sum;
}