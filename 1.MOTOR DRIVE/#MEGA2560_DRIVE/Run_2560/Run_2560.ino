#include <AFMotor.h>

// AFMotor: 모터 포트 1~4에 각각 연결된 DC 모터
AF_DCMotor motor1(1); // 앞 왼쪽 (예)
AF_DCMotor motor2(2); // 앞 오른쪽 (예)
AF_DCMotor motor3(3); // 뒤 왼쪽 (예)
AF_DCMotor motor4(4); // 뒤 오른쪽 (예)

int speedVal = 150; // 초기 속도 (0-255)

void setup() {
  Serial.begin(9600);
  // 초기 상태: 정지
  stopAll();
  Serial.println("Ready: w/s/a/d = forward/back/left/right, x = stop, 0-9 = speed");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    // 숫자키로 속도 설정: '0'->0, '1'->28, ... '9'->255 (대략 비례)
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
        Serial.println("Forward");
        break;
      case 's': case 'S':
        backward();
        Serial.println("Backward");
        break;
      case 'a': case 'A':
        turnLeft();
        Serial.println("Turn Left");
        break;
      case 'd': case 'D':
        turnRight();
        Serial.println("Turn Right");
        break;
      case 'x': case 'X':
        stopAll();
        Serial.println("Stop");
        break;
      default:
        // 무시
        break;
    }
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