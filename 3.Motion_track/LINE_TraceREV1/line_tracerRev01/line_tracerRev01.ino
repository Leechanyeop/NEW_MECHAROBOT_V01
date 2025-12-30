// Mega_LineFollower_AFMotor_R4_PID.ino
#include "AFMotor_R4.h"

// 모터 설정 (Adafruit Motor Shield - R4 라이브러리 사용)
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// 센서 핀 (아날로그)
const int sensorLeftPin   = A8;
const int sensorCenterPin = A9;
const int sensorRightPin  = A10;

// ====== 센서 임계값(네 환경값 기반으로 조정) ======
const int sensorCenterThreshold = 800;
const int sensorLeftThreshold   = 800;
const int sensorRightThreshold  = 800;

// (선택) 라인 완전 유실 판단용 (너무 높게 잡으면 항상 false 됨)
// const int sensorStopThreshold = 999;

int speedVal = 127;     // 기본 속도 (0-255)
char lastCommand = 'A'; // 'A' = Auto, 'F','B','L','R','S' = 수동

// ====== PID 파라미터 ======
float Kp = 0.7;   // 먼저 Kp만으로 튜닝 추천
float Ki = 0.00;   // 조명/바닥 변화 많으면 0 권장
float Kd = 0.20;   // 코너에서 반응 빠르게

float integral = 0.0;
float lastError = 0.0;
unsigned long lastPidMs = 0;

void setup() {
  Serial.begin(115200);
  setAllSpeed(speedVal);
  stopAll();
  lastPidMs = millis();

  Serial.println("Ready: Line follower (AFMotor_R4) - PID Auto mode by default");
  Serial.println("Serial commands: A=Auto F=Forward B=Backward L=Left R=Right S=Stop +=speed -=speed");
  Serial.println("PID: tune Kp/Ki/Kd in code (Kp then Kd, Ki last)");
}

/* ---------------- motor control functions ---------------- */
void setAllSpeed(int sp) {
  motor1.setSpeed(sp);
  motor2.setSpeed(sp);
  motor3.setSpeed(sp);
  motor4.setSpeed(sp);
}

// ⚠️ 주의: 너희 코드에서 backward()가 실제로 '직진(전진)'처럼 동작한다고 했으니 유지
void forward() { // (실제로는 후진이든 뭐든 너희 wiring 기준)
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void backward() { // ✅ 이걸 "직진"으로 쓰는 상황 유지
  setAllSpeed(speedVal);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// PID에서는 속도 차등이 필요해서 "좌/우 따로 속도+방향" 함수 추가 (기존 구조 크게 안바꿈)
void driveLR(int leftSp, int rightSp) {
  // leftSp/rightSp: -255~255 (부호로 방향)
  leftSp  = constrain(leftSp,  -255, 255);
  rightSp = constrain(rightSp, -255, 255);

  // 왼쪽: motor1, motor4 / 오른쪽: motor2, motor3 (너희 turn 코드 기준으로 묶음)
  // 부호에 따라 FORWARD/BACKWARD를 바꿔야 하는데,
  // 현재 backward()가 "직진" 역할이라, 여기서도 "양수=backward방향"으로 통일함.
  auto runOne = [](AF_DCMotor& m, int sp) {
    int a = abs(sp);
    m.setSpeed(a);
    if (sp > 0) m.run(BACKWARD);   // ✅ 양수면 너희 직진 방향
    else if (sp < 0) m.run(FORWARD);
    else m.run(RELEASE);
  };

  runOne(motor1, leftSp);
  runOne(motor4, leftSp);
  runOne(motor2, rightSp);
  runOne(motor3, rightSp);
}

void turnRight() {
  setAllSpeed(speedVal);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
}

void turnLeft() {
  setAllSpeed(speedVal);
  motor1.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
/* -------------------------------------------------------- */


/* ================= PID 라인추종 로직 =================
   - 오차(error) = (right - left)
   - output이 +면 오른쪽이 더 진하다는 뜻 -> 왼쪽으로 꺾어야 하므로 좌/우 속도 차등
   - center는 라인 유실/코너 판단에만 사용 (필요시 error에 가중치로 넣어도 됨)
*/
void lineFollowLoop() {
  int left   = analogRead(sensorLeftPin);
  int center = analogRead(sensorCenterPin);
  int right  = analogRead(sensorRightPin);

  bool leftOn   = (left   > sensorLeftThreshold);
  bool centerOn = (center > sensorCenterThreshold);
  bool rightOn  = (right  > sensorRightThreshold);

  // ====== 라인 유실 판단 ======
  // 3개 다 false면 라인 벗어남(흰바닥) 가능성이 큼 -> 마지막 에러 방향으로 회복
  bool lineLost = (!leftOn && !centerOn && !rightOn);

  // PID 시간차(dt)
  unsigned long now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f; // 안전값
  lastPidMs = now;

  // ====== error 계산 ======
  float error;
  if (!lineLost) {
    // 기본: right-left
    error = (float)(right - left);

    // (선택) 코너에서 center가 강하게 들어오면 살짝 보정하고 싶으면 가중치 추가 가능
    // error += 0.0f * (center - 800);

  } else {
    // 라인 유실 시: 마지막 에러 부호로 크게 돌면서 찾기
    error = (lastError >= 0) ? 800.0f : -800.0f;
  }

  // ====== PID 계산 ======
  integral += error * dt;
  // 적분 과다 누적 방지(필수)
  integral = constrain(integral, -5000.0f, 5000.0f);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // ====== 모터 속도 차등 ======
  // output이 +면 right가 더 크다 -> 왼쪽으로 돌아야 함
  // => leftSpeed를 낮추고 rightSpeed를 높이는 방식(차동)
  int base = speedVal;

  // output 스케일 제한 (너무 큰 값이면 급격히 튐)
  output = constrain(output, -200.0f, 200.0f);

  int leftSp  = base - (int)output;
  int rightSp = base + (int)output;

  // 최소 속도 보장(멈춰버리면 코너에서 못나옴)
  leftSp  = constrain(leftSp,  60, 255);
  rightSp = constrain(rightSp, 60, 255);

  // ====== 실제 주행 ======
  // 기존 코드에서 "centerOn이면 backward()가 직진"이었으니,
  // PID에서도 직진은 driveLR()로 처리 (양수=직진방향으로 매핑해둠)
  driveLR(leftSp, rightSp);

  // 라인 완전 유실 시에는 회복이 더 강해야 해서 살짝 추가 회전(선택)
  if (lineLost) {
    // lastError 부호에 따라 한쪽만 더 주기
    if (lastError > 0) {
      // 라인이 오른쪽에 있었음 -> 더 좌로 찾기
      driveLR(80, 200);
    } else {
      driveLR(200, 80);
    }
  }
}

/* 시리얼 명령 처리 (수동 제어 및 속도 조절) */
void handleSerialCommand(char cmd) {
  switch (cmd) {
    case 'A':
      lastCommand = 'A';
      Serial.println("Mode: Auto (PID line follow)");
      break;
    case 'F':
      lastCommand = 'F';
      backward(); // 너희 기준 직진
      Serial.println("Command: Forward(uses backward())");
      break;
    case 'B':
      lastCommand = 'B';
      forward();
      Serial.println("Command: Backward(uses forward())");
      break;
    case 'L':
      lastCommand = 'L';
      turnLeft();
      Serial.println("Command: Turn Left");
      break;
    case 'R':
      lastCommand = 'R';
      turnRight();
      Serial.println("Command: Turn Right");
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
    default:
      break;
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') return;
    handleSerialCommand(c);
  }

  if (lastCommand == 'A') {
    lineFollowLoop();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    int l = analogRead(sensorLeftPin);
    int c = analogRead(sensorCenterPin);
    int r = analogRead(sensorRightPin);
    Serial.print("Sensors L C R: ");
    Serial.print(l); Serial.print(" ");
    Serial.print(c); Serial.print(" ");
    Serial.print(r);
    Serial.print(" | PID Kp Ki Kd: ");
    Serial.print(Kp, 3); Serial.print(" ");
    Serial.print(Ki, 3); Serial.print(" ");
    Serial.println(Kd, 3);
    lastPrint = millis();
  }

  delay(20);
}
