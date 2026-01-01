// Mega_LineFollower_5chDigital_AFMotor_R4_PID.ino
#include "AFMotor_R4.h"

// ===================== 모터 설정 =====================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ===================== 5채널 디지털 센서 핀 (예시) =====================
// 너 결선대로 바꿔줘!
const int S0 = 41;  // Leftmost
const int S1 = 42;  // Left
const int S2 = 43;  // Center
const int S3 = 44;  // Right
const int S4 = 45;  // Rightmost

// ===================== 센서 논리 옵션 =====================
// true  : 검정선 감지 = HIGH(1)
// false : 검정선 감지 = LOW(0)  (많이 흔한 타입)
const bool SENSOR_ACTIVE_HIGH = true;

// ===================== 주행 설정 =====================
int speedVal = 150;      // 기본 속도(0~255)
char lastCommand = 'A';  // 'A'=Auto PID, 'F','B','S' 수동

// ===================== PID =====================
// 디지털은 error 단위가 작아/커질 수 있어서 Kp 스케일이 아날로그와 다름!
float Kp = 35.0;   
float Ki = 0.0;
float Kd = 18.0;

float integral = 0;
float lastError = 0;
unsigned long lastPidMs = 0;

// 라인 유실 시 마지막 방향 기억
int lastDirSign = 0; // -1:왼쪽, +1:오른쪽

// ===================== 모터 제어 (너 하드웨어 기준: BACKWARD가 전진) =====================
void stopAll() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
}

void backward() { // 실제로는 후진 (너 기준)
  motor1.setSpeed(speedVal); motor2.setSpeed(speedVal);
  motor3.setSpeed(speedVal); motor4.setSpeed(speedVal);
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(BACKWARD); motor4.run(BACKWARD);
}

void forward() { // 실제로는 전진 (너 기준)
  motor1.setSpeed(speedVal); motor2.setSpeed(speedVal);
  motor3.setSpeed(speedVal); motor4.setSpeed(speedVal);
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(FORWARD); motor4.run(FORWARD);
}

// 좌/우 속도 차등 주행 (전진 방향은 BACKWARD로 고정)
void driveLR(int leftSp, int rightSp) {
  leftSp  = constrain(leftSp, 0, 255);
  rightSp = constrain(rightSp, 0, 255);

  // 좌측: motor1, motor4 / 우측: motor2, motor3 (네 기구가 다르면 여기만 바꾸면 됨)
  motor1.setSpeed(leftSp);
  motor2.setSpeed(rightSp);
  motor3.setSpeed(rightSp);
  motor4.setSpeed(leftSp);

  // 전진 방향(너 하드웨어 기준)
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// ===================== 5채널 디지털 읽기 =====================
bool readLineSensor(int pin) {
  int v = digitalRead(pin);
  if (SENSOR_ACTIVE_HIGH) return (v == HIGH);
  else return (v == LOW);
}

// 센서 상태를 5비트로(디버그용)
uint8_t read5Bits(bool &anyOn) {
  bool b0 = readLineSensor(S0);
  bool b1 = readLineSensor(S1);
  bool b2 = readLineSensor(S2);
  bool b3 = readLineSensor(S3);
  bool b4 = readLineSensor(S4);

  anyOn = (b0 || b1 || b2 || b3 || b4);

  uint8_t bits = (b0 ? 0x10 : 0) | (b1 ? 0x08 : 0) | (b2 ? 0x04 : 0) | (b3 ? 0x02 : 0) | (b4 ? 0x01 : 0);

  // --- 노이즈/엣지 패턴 보정 ---
  if (bits == 0x1B) { // 11011 -> 00100
    bits = 0x04;
  }
  if (bits == 0x11) { // 10001 -> 00100 (선택)
    bits = 0x04;
  }
  // 필요하면 다른 패턴도 여기에 추가
  // ------------------------------------

  return bits;

  }
  // ---------------------------------------



// ===================== ★ PID 라인추종 (디지털 5채널) =====================
// 가중치: [-2, -1, 0, +1, +2]
// error = 평균 포지션(검정으로 감지된 센서들)
float computeErrorFromBits(uint8_t bits, bool anyOn) {
  if (!anyOn) {
    // 라인 유실: 마지막 방향으로 크게 돌기
    return (lastDirSign >= 0) ? +3.0f : -3.0f;
  }

  // bits: 0x10(S0) 0x08(S1) 0x04(S2) 0x02(S3) 0x01(S4)
  int sum = 0;
  int cnt = 0;

  if (bits & 0x10) { sum += -2; cnt++; }
  if (bits & 0x08) { sum += -1; cnt++; }
  if (bits & 0x04) { sum +=  0; cnt++; }
  if (bits & 0x02) { sum += +1; cnt++; }
  if (bits & 0x01) { sum += +2; cnt++; }

  float pos = (cnt > 0) ? ((float)sum / (float)cnt) : 0.0f;

  // 방향 기억(라인 유실 대비)
  if (pos > 0.2f) lastDirSign = +1;
  else if (pos < -0.2f) lastDirSign = -1;

  return pos; // -2 ~ +2 근처
}
//
void lineFollowPID_5ch() {
  bool anyOn = false;
  uint8_t bits = read5Bits(anyOn);

  // dt
  unsigned long now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  lastPidMs = now;

  // error
  float error = computeErrorFromBits(bits, anyOn);

  // PID
  integral += error * dt;
  integral = constrain(integral, -3.0f, 3.0f); // 디지털이라 적분 제한을 작게

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // output 제한(너무 과격하면 튐)
  output = constrain(output, -120.0f, 120.0f);

  int base = -speedVal;
  int leftSp  = base - (int)output;
  int rightSp = base + (int)output;

  // 최소 속도 보장
  leftSp  = constrain(leftSp,  180, 200);
  rightSp = constrain(rightSp, 180, 200);

  driveLR(leftSp, rightSp);

  // 디버그 출력
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("BITS: ");
    // 5비트 출력(좌->우)
    Serial.print((bits & 0x10) ? "1" : "0");
    Serial.print((bits & 0x08) ? "1" : "0");
    Serial.print((bits & 0x04) ? "1" : "0");
    Serial.print((bits & 0x02) ? "1" : "0");
    Serial.print((bits & 0x01) ? "1" : "0");

    Serial.print(" | err: "); Serial.print(error, 3);
    Serial.print(" out: "); Serial.print(output, 1);
    Serial.print(" | sp L/R: "); Serial.print(leftSp); Serial.print("/"); Serial.println(rightSp);
    lastPrint = millis();
  }
}

// ===================== 시리얼 명령 처리 =====================
void handleSerialCommand(char cmd) {
  switch (cmd) {
    case 'A':
      lastCommand = 'A';
      Serial.println("Mode: Auto (5ch digital PID)");
      break;

    case 'F': // 전진(너 하드웨어 기준: backward()가 전진)
      lastCommand = 'F'; 
      forward();
      Serial.println("Command: Forward (your HW = backward())");
      break;

    case 'B': // 후진
      lastCommand = 'B';
      backward();
      Serial.println("Command: Backward (your HW = forward())");
      break;

    case 'S':
      lastCommand = 'S';
      stopAll();
      Serial.println("Command: Stop");
      break;

    case '+':
      speedVal = min(255, speedVal + 10);
      Serial.print("Speed: "); Serial.println(speedVal);
      break;

    case '-':
      speedVal = max(0, speedVal - 10);
      Serial.print("Speed: "); Serial.println(speedVal);
      break;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  stopAll();
  lastPidMs = millis();

  Serial.println("Ready: 5ch Digital Line Follower + PID (AFMotor_R4)");
  Serial.println("A=Auto, F=Forward, B=Backward, S=Stop, +=speed, -=speed");
  Serial.print("SENSOR_ACTIVE_HIGH = "); Serial.println(SENSOR_ACTIVE_HIGH ? "true" : "false");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') handleSerialCommand(c);
  }

  if (lastCommand == 'A') {
    lineFollowPID_5ch();
  }

  delay(10);
}
