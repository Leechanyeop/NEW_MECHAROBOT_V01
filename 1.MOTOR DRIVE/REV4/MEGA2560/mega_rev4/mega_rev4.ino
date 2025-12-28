// Mega_SPI_Slave_Motor_fixed.ino
#include <SPI.h>
#include <AFMotor.h>

// 모터 설정
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// 센서 핀
const int sensorLeftPin  = A0;
const int sensorRightPin = A1;
const int sensorThreshold = 400;
const unsigned long stopMillis = 1000;

volatile byte commandByte = 0;
volatile bool newCmdFlag = false;

// 응답 버퍼
volatile uint8_t respBuf[4];
volatile uint8_t respLen = 0;
volatile uint8_t respIndex = 0;

int speedVal = 150;
char lastCommand = 'S';

ISR(SPI_STC_vect) {
  byte received = SPDR; // 마스터가 보낸 바이트
  commandByte = received;
  newCmdFlag = true;

  // 마스터가 읽을 수 있도록 준비된 응답이 있으면 SPDR에 넣음
  if (respIndex < respLen) {
    SPDR = respBuf[respIndex++];
  } else {
    SPDR = 0x00;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();

  setAllSpeed(speedVal);
  stopAll();

  Serial.println("Mega ready: SPI Slave + Motor control + Sensor response");
}

void loop() {
  // 센서 자동 정지 체크
  if (!checkSensors()) {
    Serial.println("물체감지: 일시정지");
    char prev = lastCommand;
    stopAll();
    delay(stopMillis);
    resumeCommand(prev);
    delay(100);
  }

  if (newCmdFlag) {
    noInterrupts();
    byte cmd = commandByte;
    newCmdFlag = false;
    interrupts();

    // 디버그 출력
    Serial.print("SPI 명령 수신: 0x");
    Serial.println(cmd, HEX);

    // 'R' 요청: 센서값 읽어 응답 버퍼에 넣기 (0-1023 -> 0-255 스케일)
    if (cmd == 'R') {
      int leftRaw  = analogRead(sensorLeftPin);
      int rightRaw = analogRead(sensorRightPin);
      uint8_t leftByte  = map(constrain(leftRaw, 0, 1023), 0, 1023, 0, 255);
      uint8_t rightByte = map(constrain(rightRaw,0,1023), 0, 1023, 0, 255);

      // 응답 버퍼 설정
      respBuf[0] = leftByte;
      respBuf[1] = rightByte;
      respLen = 2;
      respIndex = 0;

      // SPDR에 첫 바이트 준비
      SPDR = respBuf[respIndex++];

      // Serial.printf 대체: snprintf 사용
      char buf[80];
      snprintf(buf, sizeof(buf), "Sensor read L=%d R=%d -> send %u,%u", leftRaw, rightRaw, leftByte, rightByte);
      Serial.println(buf);

      // 처리 완료, 다음 루프로
    }
    else if (cmd >= '0' && cmd <= '9') {
      int n = cmd - '0';
      speedVal = map(n, 0, 9, 0, 255);
      setAllSpeed(speedVal);
      Serial.print("속도 설정: ");
      Serial.println(speedVal);

      respBuf[0] = 0xB0;
      respLen = 1;
      respIndex = 0;
      SPDR = respBuf[respIndex++];
    }
    else {
      // 이동 명령 처리
      if (cmd == 'w' || cmd == 'W') {
        forward(); lastCommand = 'F'; Serial.println("Forward"); respBuf[0] = 0xA1;
      } else if (cmd == 's' || cmd == 'S') {
        backward(); lastCommand = 'B'; Serial.println("Backward"); respBuf[0] = 0xA2;
      } else if (cmd == 'a' || cmd == 'A') {
        turnLeft(); lastCommand = 'L'; Serial.println("Left"); respBuf[0] = 0xA3;
      } else if (cmd == 'd' || cmd == 'D') {
        turnRight(); lastCommand = 'R'; Serial.println("Right"); respBuf[0] = 0xA4;
      } else if (cmd == 'x' || cmd == 'X') {
        stopAll(); lastCommand = 'S'; Serial.println("Stop"); respBuf[0] = 0xA5;
      } else {
        Serial.println("Unknown cmd");
        respBuf[0] = 0xFF;
      }

      respLen = 1;
      respIndex = 0;
      SPDR = respBuf[respIndex++];
    }
  }

  // 필요시 다른 작업 추가
  delay(2);
}

// 센서 읽기 (장애물 감지)
bool checkSensors() {
  int leftVal  = analogRead(sensorLeftPin);
  int rightVal = analogRead(sensorRightPin);
  if (leftVal >= sensorThreshold || rightVal >= sensorThreshold) return true;
  return false;
}

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
  motor3.run(BACKWARD);
  motor2.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  setAllSpeed(speedVal);
  motor1.run(FORWARD);
  motor3.run(FORWARD);
  motor2.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}