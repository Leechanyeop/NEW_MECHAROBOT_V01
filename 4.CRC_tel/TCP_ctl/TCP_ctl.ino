// Mega_LineFollower_5chDigital_AFMotor_R4_PID_SearchStop_withRemote.ino
#include "AFMotor_R4.h"

// ===================== 모터 설정 =====================
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// ===================== 5채널 디지털 센서 핀 =====================
const int S0 = 41;  // Leftmost
const int S1 = 42;  // Left
const int S2 = 43;  // Center
const int S3 = 44;  // Right
const int S4 = 45;  // Rightmost

// ===================== 주행/튜닝 파라미터 =====================
int baseSpeed = 160;     // 기본 속도(0~255) - 자동 주행용 기본값 조정
int minSpeed  = 100;     // 너무 느려서 멈추는 것 방지(라인 있을 때만 적용)
int maxSpeed  = 220;

// 코너/급회전 감지 시 감속(스핀 불안정 줄이기)
int slowSpeed = 120;     // 코너 패턴에서 base를 낮춤

// ===================== PID =====================
float Kp = 35.0;
float Ki = 0.0;
float Kd = 22.0;

float integral = 0;
float lastError = 0;
unsigned long lastPidMs = 0;

// ===================== 라인 유실/탐색 상태 =====================
enum Mode { FOLLOW, SEARCH, STOPPED, MANUAL };
Mode mode = STOPPED;

int lastDirSign = +1;  // 마지막 라인 방향(-1 왼쪽, +1 오른쪽)

// SEARCH 동작(제자리 회전) 튜닝
const unsigned long SEARCH_PAUSE_MS = 140;   // 유실 직후 잠깐 멈춤(관성 정리)
const unsigned long SEARCH_SPIN_MS  = 1100;   // 한 번 스핀 시도 시간
const int SEARCH_SPIN_SPEED_L = 180;         // 제자리 회전 좌측 바퀴 속도
const int SEARCH_SPIN_SPEED_R = 180;         // 제자리 회전 우측 바퀴 속도

unsigned long searchStartMs = 10;
bool searchPaused = false;

// ===================== 수동 제어 변수 =====================
int manualLeftSp = 0;
int manualRightSp = 0;

// ===================== 모터 제어 =====================
void stopAll() {
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
}

// 전진(좌/우 속도 차등)
void driveForwardLR(int leftSp, int rightSp) {
  leftSp  = constrain(leftSp, 0, 255);
  rightSp = constrain(rightSp, 0, 255);

  // 좌측: motor1, motor4 / 우측: motor2, motor3
  motor1.setSpeed(leftSp);
  motor4.setSpeed(leftSp);
  motor2.setSpeed(rightSp);
  motor3.setSpeed(rightSp);

  motor1.run(FORWARD); motor4.run(FORWARD);
  motor2.run(FORWARD); motor3.run(FORWARD);
}

// 제자리 회전: left는 후진, right는 전진 => 좌회전 스핀
void spinRight(int sp) {
  sp = constrain(sp, 0, 255);

  motor1.setSpeed(sp); motor4.setSpeed(sp);
  motor2.setSpeed(sp); motor3.setSpeed(sp);

  motor1.run(BACKWARD); motor4.run(BACKWARD);
  motor2.run(FORWARD);  motor3.run(FORWARD);
}

// 제자리 회전: left 전진, right 후진 => 우회전 스핀
void spinLeft(int sp) {
  sp = constrain(sp, 0, 255);

  motor1.setSpeed(sp); motor4.setSpeed(sp);
  motor2.setSpeed(sp); motor3.setSpeed(sp);

  motor1.run(FORWARD);  motor4.run(FORWARD);
  motor2.run(BACKWARD); motor3.run(BACKWARD);
}

// ===================== 센서 읽기 =====================
// 너 환경: 흰바닥=1, 검정선=0 -> 반전해서 내부는 검정=1로 통일
bool readLine(int pin) {
  return !digitalRead(pin);
}

uint8_t readBits(bool &anyOn) {
  bool b0 = readLine(S0);
  bool b1 = readLine(S1);
  bool b2 = readLine(S2);
  bool b3 = readLine(S3);
  bool b4 = readLine(S4);

  anyOn = (b0 || b1 || b2 || b3 || b4);

  uint8_t bits = (b0 ? 0x10 : 0) | (b1 ? 0x08 : 0) | (b2 ? 0x04 : 0) | (b3 ? 0x02 : 0) | (b4 ? 0x01 : 0);

  // (선택) 너가 겪은 흔들림 패턴 보정
  // 11011 같은 이상패턴이 들어오면 중앙으로 간주
  if (bits == 0x1B) bits = 0x04;

  return bits;
}

// ===================== 에러 계산(-2 ~ +2) =====================
float computeError(uint8_t bits, bool anyOn) {
  if (!anyOn) return 0.0f;

  int sum = 0;
  int cnt = 0;

  if (bits & 0x10) { sum += -2; cnt++; }
  if (bits & 0x08) { sum += -1; cnt++; }
  if (bits & 0x04) { sum +=  0; cnt++; }
  if (bits & 0x02) { sum += +1; cnt++; }
  if (bits & 0x01) { sum += +2; cnt++; }

  float pos = (cnt > 0) ? (float)sum / (float)cnt : 0.0f;

  // 마지막 방향 기억(라인 유실 시 스핀 방향 결정)
  if (pos > 0.2f) lastDirSign = +1;
  else if (pos < -0.2f) lastDirSign = -1;

  return pos;
}

// ===================== 코너 패턴 감지(감속용) =====================
bool isCornerPattern(uint8_t bits) {
  // 가장자리 쪽 강하게 잡히면 코너로 보고 감속
  if (bits == 0x10 || bits == 0x18 || bits == 0x01 || bits == 0x03) return true;
  if (bits == 0x1C || bits == 0x07) return true;
  return false;
}

// ===================== FOLLOW(PID) =====================
void doFollowPID(uint8_t bits, bool anyOn) {
  // dt
  unsigned long now = millis();
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.02f;
  lastPidMs = now;

  float error = computeError(bits, anyOn);

  integral += error * dt;
  integral = constrain(integral, -3.0f, 3.0f);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -120.0f, 120.0f);

  // 코너면 base를 낮춰서 “진입 과속” 줄이기
  int base = isCornerPattern(bits) ? slowSpeed : baseSpeed;

  int leftSp  = base - (int)output;
  int rightSp = base + (int)output;

  leftSp  = constrain(leftSp,  minSpeed, maxSpeed);
  rightSp = constrain(rightSp, minSpeed, maxSpeed);

  driveForwardLR(leftSp, rightSp);
}

// ===================== SEARCH(라인 유실 시) =====================
void doSearch() {
  unsigned long now = millis();

  // 1) 유실 직후 잠깐 멈춤(관성 때문에 라인 더 놓치는거 방지)
  if (!searchPaused) {
    stopAll();
    searchPaused = true;
    searchStartMs = now;
    return;
  }

  if (now - searchStartMs < SEARCH_PAUSE_MS) {
    // 계속 정지 유지
    stopAll();
    return;
  }

  // 2) 제자리 회전으로 라인 재탐색
  unsigned long t = now - (searchStartMs + SEARCH_PAUSE_MS);

  unsigned long phase = t % (2 * SEARCH_SPIN_MS);

  int spinSp = SEARCH_SPIN_SPEED_L; // 동일 사용

  if (phase < SEARCH_SPIN_MS) {
    if (lastDirSign < 0) spinLeft(spinSp);
    else spinRight(spinSp);
  } else {
    if (lastDirSign < 0) spinRight(spinSp);
    else spinLeft(spinSp);
  }
}

// ===================== 원격(Serial1) 명령 처리 =====================
void handleSerialCommands() {
  // Serial1: ESP32와 연결 (Mega: RX1/TX1 -> 핀 19/18)
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // 대문자로 통일
    String cmd = line;
    cmd.toUpperCase();

    // MODE 명령
    if (cmd.startsWith("MODE ")) {
      String arg = cmd.substring(5);
      arg.trim();
      if (arg == "AUTO") {
        mode = FOLLOW;        // 자동 모드로 전환 (센서에 따라 FOLLOW/SEARCH)
        searchPaused = false;
        Serial.println("REMOTE: MODE AUTO");
      } else if (arg == "MANUAL") {
        mode = MANUAL;
        Serial.println("REMOTE: MODE MANUAL");
      } else {
        Serial.print("REMOTE: Unknown MODE ");
        Serial.println(arg);
      }
      continue;
    }

    // STOP
    if (cmd == "STOP") {
      mode = STOPPED;
      stopAll();
      Serial.println("REMOTE: STOP");
      continue;
    }

    // DRIVE L R  (예: DRIVE 150 150)
    if (cmd.startsWith("DRIVE ")) {
      // 숫자 파싱
      int sp1 = 0, sp2 = 0;
      // 원본 line 사용해서 숫자 파싱(대문자 변환으로 숫자 손상 방지)
      String payload = line.substring(6);
      payload.trim();
      int spaceIdx = payload.indexOf(' ');
      if (spaceIdx > 0) {
        sp1 = payload.substring(0, spaceIdx).toInt();
        sp2 = payload.substring(spaceIdx + 1).toInt();
        sp1 = constrain(sp1, 0, 255);
        sp2 = constrain(sp2, 0, 255);
        manualLeftSp = sp1;
        manualRightSp = sp2;
        mode = MANUAL;
        Serial.print("REMOTE: DRIVE ");
        Serial.print(sp1);
        Serial.print(" ");
        Serial.println(sp2);
      } else {
        Serial.println("REMOTE: DRIVE requires two values");
      }
      continue;
    }

    // SPIN LEFT/RIGHT speed
    if (cmd.startsWith("SPIN LEFT")) {
      int sp = 150;
      // 숫자 있으면 파싱
      int idx = cmd.indexOf(' ', 9);
      if (idx > 0) sp = cmd.substring(idx + 1).toInt();
      sp = constrain(sp, 0, 255);
      mode = MANUAL;
      spinLeft(sp);
      Serial.print("REMOTE: SPIN LEFT ");
      Serial.println(sp);
      continue;
    }
    if (cmd.startsWith("SPIN RIGHT")) {
      int sp = 150;
      int idx = cmd.indexOf(' ', 10);
      if (idx > 0) sp = cmd.substring(idx + 1).toInt();
      sp = constrain(sp, 0, 255);
      mode = MANUAL;
      spinRight(sp);
      Serial.print("REMOTE: SPIN RIGHT ");
      Serial.println(sp);
      continue;
    }

    // 파라미터 조정: SET KP 30  또는 SET KD 20 등
    if (cmd.startsWith("SET ")) {
      String rest = cmd.substring(4);
      rest.trim();
      int spc = rest.indexOf(' ');
      if (spc > 0) {
        String key = rest.substring(0, spc);
        int val = rest.substring(spc + 1).toInt();
        if (key == "KP") { Kp = val; Serial.print("REMOTE: Kp="); Serial.println(Kp); }
        else if (key == "KI") { Ki = val; Serial.print("REMOTE: Ki="); Serial.println(Ki); }
        else if (key == "KD") { Kd = val; Serial.print("REMOTE: Kd="); Serial.println(Kd); }
        else if (key == "BASE") { baseSpeed = constrain(val, 0, 255); Serial.print("REMOTE: baseSpeed="); Serial.println(baseSpeed); }
        else { Serial.print("REMOTE: Unknown SET "); Serial.println(key); }
      } else {
        Serial.println("REMOTE: SET requires key and value");
      }
      continue;
    }

    // 알 수 없는 명령
    Serial.print("REMOTE: Unknown command: ");
    Serial.println(line);
  }
}

// ===================== 메인 루프 =====================

void setup() {
  Serial.begin(115200);      // 디버그 출력
  Serial1.begin(115200);     // ESP32와 통신 (Mega: Serial1)

  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  stopAll();
  lastPidMs = millis();

  Serial.println("Ready: 5ch Digital Line Follower + PID + Remote Control");
  Serial.println("Commands: MODE AUTO|MANUAL, STOP, DRIVE L R, SPIN LEFT|RIGHT [spd], SET KP|KI|KD|BASE val");
}

void loop() {
  // 먼저 원격 명령 처리(비동기)
  handleSerialCommands();

  bool anyOn = false;
  uint8_t bits = readBits(anyOn);

  // ---- 상태 전이 규칙 (원격 모드 우선) ----
  if (mode != MANUAL && mode != STOPPED) {
    // 원격에서 AUTO로 설정된 경우 센서에 따라 FOLLOW/SEARCH 전환
    if (anyOn) {
      mode = FOLLOW;
      searchPaused = false; // SEARCH 종료
    } else {
      mode = SEARCH;
    }
  }

  // ---- 실행 ----
  if (mode == FOLLOW) {
    doFollowPID(bits, anyOn);
  } else if (mode == SEARCH) {
    doSearch();
  } else if (mode == MANUAL) {
    // 수동 모드: 수동 속도 적용 (DRIVE 명령으로 설정된 값 유지)
    // 만약 수동에서 0,0이면 정지
    if (manualLeftSp == 0 && manualRightSp == 0) {
      stopAll();
    } else {
      driveForwardLR(manualLeftSp, manualRightSp);
    }
  } else { // STOPPED
    stopAll();
  }

  // ---- 디버그 ----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("BITS: ");
    Serial.print((bits & 0x10) ? "1" : "0");
    Serial.print((bits & 0x08) ? "1" : "0");
    Serial.print((bits & 0x04) ? "1" : "0");
    Serial.print((bits & 0x02) ? "1" : "0");
    Serial.print((bits & 0x01) ? "1" : "0");

    Serial.print(" | mode: ");
    Serial.print(mode == FOLLOW ? "FOLLOW" : (mode == SEARCH ? "SEARCH" : (mode == MANUAL ? "MANUAL" : "STOP")));

    Serial.print(" | lastDir: ");
    Serial.print(lastDirSign);

    Serial.print(" | manual L/R: ");
    Serial.print(manualLeftSp);
    Serial.print("/");
    Serial.println(manualRightSp);

    lastPrint = millis();
  }

  delay(10);
}