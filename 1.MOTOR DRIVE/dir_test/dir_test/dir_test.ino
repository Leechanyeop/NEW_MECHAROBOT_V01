#include <AFMotor_R4.h>

// 모터 포트 매핑 (실제 보드 포트에 맞게 변경)
AF_DCMotor leftFront(1);   // 왼쪽 앞 모터
AF_DCMotor leftRear(3);    // 왼쪽 뒤 모터
AF_DCMotor rightFront(2);  // 오른쪽 앞 모터
AF_DCMotor rightRear(4);   // 오른쪽 뒤 모터

int currentSpeed = 120;    // 초기 속도 (0..255)
const int speedStep = 10;  // + / - 키로 변경되는 단위

void setup() {
  Serial.begin(9600);

  // 초기화: 속도 0, RELEASE 상태
  leftFront.setSpeed(0); leftRear.setSpeed(0);
  rightFront.setSpeed(0); rightRear.setSpeed(0);
  leftFront.run(RELEASE); leftRear.run(RELEASE);
  rightFront.run(RELEASE); rightRear.run(RELEASE);

  Serial.println("AFMotor R4 4-motor Keyboard Control Ready");
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    processKey(c);
  }
}

// 개별 모터 제어 헬퍼: speed -255..255 (음수는 역회전)
void setMotorSpeed(AF_DCMotor &m, int speed) {
  if (speed > 0) {
    m.setSpeed(constrain(speed, 0, 255));
    m.run(FORWARD);
  } else if (speed < 0) {
    m.setSpeed(constrain(-speed, 0, 255));
    m.run(BACKWARD);
  } else {
    m.run(RELEASE);
  }
}

// 좌측 모터 모두 설정
void setLeftMotors(int speed) {
  setMotorSpeed(leftFront, speed);
  setMotorSpeed(leftRear, speed);
}

// 우측 모터 모두 설정
void setRightMotors(int speed) {
  setMotorSpeed(rightFront, speed);
  setMotorSpeed(rightRear, speed);
}

// 기본 동작
void driveForward(int sp) {
  setLeftMotors(sp);
  setRightMotors(sp);
}

void driveBackward(int sp) {
  setLeftMotors(-sp);
  setRightMotors(-sp);
}

// 제자리 회전: 좌회전은 좌측 역, 우측 정방향
void turnLeft(int sp) {
  setLeftMotors(sp);
  setRightMotors(-sp);
}

void turnRight(int sp) {
  setLeftMotors(-sp);
  setRightMotors(sp);
}

void stopMotors() {
  leftFront.run(RELEASE); leftRear.run(RELEASE);
  rightFront.run(RELEASE); rightRear.run(RELEASE);
}

void processKey(char k) {
  k = tolower(k);
  switch (k) {
    case 'w':
      driveForward(currentSpeed);
      Serial.print("Forward "); Serial.println(currentSpeed);
      break;
    case 's':
      driveBackward(currentSpeed);
      Serial.print("Backward "); Serial.println(currentSpeed);
      break;
    case 'a':
      turnLeft(currentSpeed);
      Serial.print("Turn Left "); Serial.println(currentSpeed);
      break;
    case 'd':
      turnRight(currentSpeed);
      Serial.print("Turn Right "); Serial.println(currentSpeed);
      break;
    case 'x':
      stopMotors();
      Serial.println("Stop");
      break;
    case '+':
      currentSpeed = constrain(currentSpeed + speedStep, 0, 255);
      Serial.print("Speed + -> "); Serial.println(currentSpeed);
      break;
    case '-':
      currentSpeed = constrain(currentSpeed - speedStep, 0, 255);
      Serial.print("Speed - -> "); Serial.println(currentSpeed);
      break;
    case '0':
      currentSpeed = 0;
      stopMotors();
      Serial.println("Speed set 0, Stop");
      break;
    case 'p':
      Serial.print("Current Speed: "); Serial.println(currentSpeed);
      break;
    case 'q':
      Serial.println("Status OK");
      break;
    case 'h':
      printHelp();
      break;
    default:
      Serial.print("Unknown key: "); Serial.println(k);
  }
}

void printHelp() {
  Serial.println("Controls:");
  Serial.println(" w : forward");
  Serial.println(" s : backward");
}