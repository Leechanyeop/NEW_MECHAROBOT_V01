#include "MotorDrive.h"

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============
// 모터 1 (왼쪽)
#define MOTOR1_R_EN  5
#define MOTOR1_L_EN  4
#define MOTOR1_RPWM  3
#define MOTOR1_LPWM  2

// 모터 2 (오른쪽)
#define MOTOR2_R_EN  10
#define MOTOR2_L_EN  11
#define MOTOR2_RPWM  8
#define MOTOR2_LPWM  9

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  motor.begin();
  
  Serial.println(F("\n=== Direction Test Mode ==="));
  Serial.println(F(" Commands:"));
  Serial.println(F("  'w': Forward"));
  Serial.println(F("  's': Backward"));
  Serial.println(F("  'a': Turn Left (Pivot)"));
  Serial.println(F("  'd': Turn Right (Pivot)"));
  Serial.println(F("  'q': Curve Left"));
  Serial.println(F("  'e': Curve Right"));
  Serial.println(F("  ' ': Stop"));
  Serial.println(F("  '1': Speed 60"));
  Serial.println(F("  '2': Speed 100"));
  Serial.println(F("  '3': Speed 150"));
}

int currentSpeed = 60; // 기본 테스트 속도 (느리게 안전하게)

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // 개행 문자 무시
    if (cmd == '\n' || cmd == '\r') return;

    Serial.print("Command: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'w':
        Serial.println("Moving Forward");
        motor.forward(currentSpeed);
        break;
      case 's':
        Serial.println("Moving Backward");
        motor.backward(currentSpeed);
        break;
      case 'a':
        Serial.println("Pivot Turn Left");
        motor.turnLeft(currentSpeed);
        break;
      case 'd':
        Serial.println("Pivot Turn Right");
        motor.turnRight(currentSpeed);
        break;
      case 'q':
        Serial.println("Curve Left");
        motor.curveLeft(currentSpeed, 0.5);
        break;
      case 'e':
        Serial.println("Curve Right");
        motor.curveRight(currentSpeed, 0.5);
        break;
      case ' ':
        Serial.println("STOP");
        motor.stop();
        break;

      // 속도 조절
      case '1':
        currentSpeed = 60;
        Serial.println("Speed set to 60");
        break;
      case '2':
        currentSpeed = 100;
        Serial.println("Speed set to 100");
        break;
      case '3':
        currentSpeed = 150;
        Serial.println("Speed set to 150");
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        motor.stop();
        break;
    }
  }
}
