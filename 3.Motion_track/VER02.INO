// Mega_LineFollower_AFMotor_R4_PID_Normalized.ino
#include "AFMotor_R4.h"

// 모터 (Adafruit Motor Shield R4)
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// 센서 핀 (아날로그)
const int sensorLeftPin   = A7;
const int sensorCenterPin = A8;
const int sensorRightPin  = A9;

// ====== 너가 측정한 min/max (10mm 올린 상태) ======
// 필요하면 여기만 다시 바꾸면 됨
const int L_MIN = 454;
const int L_MAX = 931;

const int C_MIN = 657;
const int C_MAX = 836;

const int R_MIN = 490;
const int R_MAX = 930;

// “라인이 존재한다” 판단용 최소 합 (정규화 합 기준)
const int LINE_SUM_MIN = 120;   // 환경에 따라 80~200 사이 튜닝

// 속도
int speedVal = 140;             // 기본 속도(0~255) - 너무 빠르면 낮춰
char lastCommand = 'A';         // A=Auto

// ===== PID 파라미터 =====
float Kp = 1.10f;               // 정규화(0~1000) 기준이므로 이전(0.35)보다 커지는 게 정상
float Ki = 0.00f;
float Kd = 0.25f;

// PID 상태값
float integral = 0;
float lastError = 0;
unsigned long lastPidMs = 0;

void setup() {
  Serial.begin(115200);
  setAllSpeed(speedVal);
  stopAll();

  Serial.println("Ready: PID Line follower (Normalized sensors)");
  Serial.println("Commands: A=Auto S=Stop +=speed -=speed");
  lastPidMs = millis();
}

/* ---------------- motor control helpers ---------------- */
void setAllSpeed(int sp) {
  motor1.setSpeed(sp);
  motor2.setSpeed(sp);
  motor3.setSpeed(sp);
  motor4.setSpeed(sp);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// 좌/우 속도(0~255)로 차등 구동 (차동주행)
// ※ 너희 코드에서 "backward()가 직진" 혼동이 있었으니 여기서는 명확히 FORWARD로 직진 처리
void driveLR(int leftSp, int rightSp) {
  leftSp  = constrain(leftSp, 0, 255);
  rightSp = constrain(rightSp, 0, 255);

  // 왼쪽 바퀴: motor1, motor4
  motor1.setSpeed(leftSp);
  motor4.setSpeed(leftSp);
  motor1.run(FORWARD);
  motor4.run(FORWARD);

  // 오른쪽 바퀴: motor2, motor3
  motor2.setSpeed(rightSp);
  motor3.setSpeed(rightSp);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
}
/* -------------------------------------------------------- */

// 값 정규화: min~max를 0~1000으로 변환 (클램프 포함)
int norm1000(int v, int vmin, int vmax) {
  if (vmax <= vmin) return 0;
  v = constrain(v, vmin, vmax);
  long n = (long)(v - vmin) * 1000L / (long)(vmax - vmin);
  return (int)n; // 0~1000
}

/* ================= PID 라인추종 로직 (정규화 기반) =================
   - 각 센서를 0~1000으로 정규화
   - 라인 위치 pos를 -1 ~ +1로 계산:
       pos = (R - L) / (L + C + R)
     -> pos>0: 라인이 오른쪽에 더 가까움(오른쪽이 더 진함)
     -> pos<0: 라인이 왼쪽에 더 가까움
   - 목표는 pos=0(가운데) => error = pos
*/
void lineFollowLoop() {
  int rawL = analogRead(sensorLeftPin);
  int rawC = analogRead(sensorCenterPin);
  int rawR = analogRead(sensorRightPin);

  int L = norm1000(rawL, L_MIN, L_MAX);
  int C = norm1000(rawC, C_MIN, C_MAX);
  int R = norm1000(rawR, R_MIN, R_MAX);

  int sum = L + C + R;
  bool lineLost = (sum < LINE_SUM_MIN);

  // dt
  unsigned long now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  lastPidMs = now;

  float error;

  if (!lineLost) {
    // -1 ~ +1 범위 근처
    float pos = (float)(R - L) / (float)sum;
    error = pos;  // 목표 0
  } else {
    // 라인 유실: 마지막 방향으로 크게 돌면서 찾기
    error = (lastError >= 0) ? 0.8f : -0.8f;
  }

  // PID
  integral += error * dt;
  integral = constrain(integral, -1.0f, 1.0f); // 정규화라 적분도 작은 범위로 제한

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // output을 속도차로 변환
  // error가 1이면 꽤 큰 편 => 속도차는 0~120 정도면 충분
  int diff = (int)(output * 120.0f);
  diff = constrain(diff, -120, 120);

  int base = speedVal;
  int leftSp  = base - diff;
  int rightSp = base + diff;

  // 너무 느려서 멈추는 것 방지
  leftSp  = constrain(leftSp,  60, 255);
  rightSp = constrain(rightSp, 60, 255);

  driveLR(leftSp, rightSp);

  // 라인 유실이면 더 강하게 회복(선택)
  if (lineLost) {
    if (lastError > 0) driveLR(70, 200);  // 라인이 오른쪽에 있었음 -> 좌로 찾기
    else               driveLR(200, 70);  // 라인이 왼쪽에 있었음 -> 우로 찾기
  }

  // 디버그 출력 (원시 + 정규화)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("RAW L C R: ");
    Serial.print(rawL); Serial.print(" ");
    Serial.print(rawC); Serial.print(" ");
    Serial.print(rawR);

    Serial.print(" | NORM L C R: ");
    Serial.print(L); Serial.print(" ");
    Serial.print(C); Serial.print(" ");
    Serial.print(R);

    Serial.print(" | err: ");
    Serial.print(error, 3);

    Serial.print(" | PID Kp Ki Kd: ");
    Serial.print(Kp, 2); Serial.print(" ");
    Serial.print(Ki, 2); Serial.print(" ");
    Serial.println(Kd, 2);

    lastPrint = millis();
  }

  delay(10);
}

/* 시리얼 명령 */
void handleSerialCommand(char cmd) {
  switch (cmd) {
    case 'A':
      lastCommand = 'A';
      Serial.println("Mode: Auto (PID)");
      break;
    case 'S':
      lastCommand = 'S';
      stopAll();
      Serial.println("Stop");
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
    if (c != '\n' && c != '\r') handleSerialCommand(c);
  }

  if (lastCommand == 'A') {
    lineFollowLoop();
  }
}
