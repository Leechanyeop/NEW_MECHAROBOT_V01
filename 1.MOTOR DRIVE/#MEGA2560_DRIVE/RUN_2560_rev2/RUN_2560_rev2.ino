word(2)#include <AFMotor.h>

// AFMotor: 모터 포트 1~4에 각각 연결된 DC 모터
AF_DCMotor motor1(1); // 앞 왼쪽
AF_DCMotor motor2(2); // 앞 오른쪽
AF_DCMotor motor3(3); // 뒤 왼쪽
AF_DCMotor motor4(4); // 뒤 오른쪽

int speedVal = 150; // 초기 속도 (0-255)

// 적외선/아날로그 센서 핀 (Arduino Mega 등에서 A8, A9 사용 가능)
const int sensorLeftPin  = A8;
const int sensorRightPin = A9;
const int sensorThreshold = 400; // 감지 임계값(0-1023) 필요시 조정
const unsigned long stopMillis = 1000; // 멈춤 시간(ms)

char lastCommand = 'S'; // 'F' forward, 'B' backward, 'L' left, 'R' right, 'S' stop

void setup() {
  Serial.begin(9600);
  stopAll();
  setAllSpeed(speedVal);
  Serial.println("Ready: w/s/a/d = forward/back/left/right, x = stop, 0-9 = speed");
}

void loop() {
  // 센서 우선 체크: 감지되면 잠시 멈춤 처리
  if (!checkSensors()) {
    // 물체 감지 시 동작
    Serial.println("물체감지 일단정지합니다");
    // 현재 동작을 잠시 멈추고 저장된 lastCommand를 기억
    char prev = lastCommand;
    stopAll();
    delay(stopMillis);
    // 멈춘 뒤 이전 동작 재개 (이전이 정지 상태면 그대로 정지)
    resumeCommand(prev);
    // 센서 감지 후 잠깐 더 센서 읽기 여유를 주기 위해 짧은 지연
    delay(100);
    // 루프 계속
  }

  // 시리얼 입력 처리 (사용자 명령)
  if (Serial.available()) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.length() == 0) return;
    char c = in.charAt(0);

    // 숫자키로 속도 설정: '0'->0, '1'->28, ... '9'->255
    if (c >= '0' && c <= '9') {
      int n = c - '0';
      speedVal = map(n, 0, 9, 0, 255);
      setAllSpeed(speedVal);
      Serial.print("Speed set to ");
      Serial.println(speedVal);
      return;
    }

    switch (c) {
      case 'w': case 'W':
        forward();
        lastCommand = 'F';
        Serial.println("Forward");
        break;
      case 's': case 'S':
        backward();
        lastCommand = 'B';
        Serial.println("Backward");
        break;
      case 'a': case 'A':
        turnLeft();
        lastCommand = 'L';
        Serial.println("Turn Left");
        break;
      case 'd': case 'D':
        turnRight();
        lastCommand = 'R';
        Serial.println("Turn Right");
        break;
      case 'x': case 'X':
        stopAll();
        lastCommand = 'S';
        Serial.println("Stop");
        break;
      default:
        // 무시
        break;
    }
  }
}

// 센서 읽고 임계값 초과하면 true 반환
bool checkSensors() {
  int leftVal  = analogRead(sensorLeftPin);
  int rightVal = analogRead(sensorRightPin);
  // 디버그용 출력(원하면 주석 해제)
  // Serial.print("L:"); Serial.print(leftVal); Serial.print(" R:"); Serial.println(rightVal);
  if (leftVal >= sensorThreshold || rightVal >= sensorThreshold) return true;
  return false;
}

// 이전 명령 재개
void resumeCommand(char cmd) {
  switch (cmd) {
    case 'F': forward(); break;
    case 'B': backward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopAll(); break;
    default: stopAll(); break;
  }
}

// 모든 모터 속도 설정 (AFMotor는 setSpeed 후 run 필요)
void setAllSpeed(int sp) {
  motor1.setSpeed(sp);
  motor2.setSpeed(sp);
  motor3.setSpeed(sp);
  motor4.setSpeed(sp);
}

// 전진: 모든 모터 정방향
void forward() {
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// 후진: 모든 모터 역방향
void backward() {
  setAllSpeed(speedVal);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// 좌회전: 왼쪽 모터 후진, 오른쪽 모터 전진 (제자리 회전)
void turnLeft() {
  setAllSpeed(speedVal);
  motor1.run(BACKWARD); // 앞 왼쪽
  motor3.run(BACKWARD); // 뒤 왼쪽
  motor2.run(FORWARD);  // 앞 오른쪽
  motor4.run(FORWARD);  // 뒤 오른쪽
}

// 우회전: 왼쪽 모터 전진, 오른쪽 모터 후진 (제자리 회전)
void turnRight() {
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor3.run(FORWARD);
  motor2.run(BACKWARD);
  motor4.run(BACKWARD);
}

// 모든 모터 정지
void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}