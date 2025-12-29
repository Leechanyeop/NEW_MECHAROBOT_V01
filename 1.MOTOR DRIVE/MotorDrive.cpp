// MotorDrive.cpp

#include "Arduino.h"
#include "MotorDrive.h"
#include <AFMotor.h>


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

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

void initMotor() {
  // 모터 핀 초기화
}


void setAllSpeed(int sp) {
  motor1.setSpeed(sp);
  motor2.setSpeed(sp);
  motor3.setSpeed(sp);
  motor4.setSpeed(sp);
}

void forward() {
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void backward() {
  setAllSpeed(speedVal);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft() {
  setAllSpeed(speedVal);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void turnRight() {
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}