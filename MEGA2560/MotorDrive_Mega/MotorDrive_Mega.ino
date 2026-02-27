#include "LINETRACE.h"
#include "MotorDrive.h"
#include <QTRSensors.h>
#include <SPI.h>
#include <Servo.h>

// ============== 핀 설정 (Mega 2560 + BTS7960) ==============

/**
 * BTS7960 드라이버 연결 정보
 * ... (existing comments)
 */

// 모터 1 (왼쪽)
#define MOTOR1_R_EN 4
#define MOTOR1_L_EN 5
#define MOTOR1_RPWM 2
#define MOTOR1_LPWM 3

// 모터 2 (오른쪽)
#define MOTOR2_R_EN 10
#define MOTOR2_L_EN 11
#define MOTOR2_RPWM 8
#define MOTOR2_LPWM 9

// QTR 라인 센서 핀 (현재 8개 사용: 28 ~ 35)

// QTR 센서 객체
QTRSensors qtr;
const uint8_t SensorCount = 8;
const uint8_t qtrPins[] = {28, 29, 30, 31, 32, 33, 34, 35};

uint16_t sensorValues[SensorCount];

// ... (existing SPI comments)

// SPI Slave 핀 (Mega 2560 전용)
#define SPI_SS_PIN 53
#define SPI_MISO_PIN 50
#define SPI_MOSI_PIN 51
#define SPI_SCK_PIN 52

// 디버그용 LED
#define DEBUG_LED 17
#define SIGNAL_PIN 7 // 'k' 명령으로 제어할 핀 (서보 또는 디지털 신호)

Servo myservo; // 서보 객체 추가

// ============== 전역 변수 ==============
int pwmValue = 70;      // 현재 PWM 값 (직진 기본 속도)
int turnPwmValue = 100; // 회전 시 PWM 값
int maxPIDSpeed = 120;  // 매뉴얼 PID 주행 시 최대 속도 제한

// 스테이션 시퀀스 타이밍 설정 (ms 단위)
#define STATION_PAUSE_TIME 30000 // 도착 후 대기 시간 (3초)
#define CONVEYOR_RUN_TIME 3000   // 컨베이어 작동 시간 (3초)
#define STATION_ESCAPE_TIME 1000 // 스테이션 탈출 전진 시간 (1초)

// 컨베이어 속도 설정 (0~180, 90이 정지)
// 90에서 멀어질수록 빠릅니다. (예: 110은 느린 정방향, 180은 최대 정방향)
#define CONVEYOR_FWD_VAL 150             // 컨베이어 정방향 속도
#define CONVEYOR_BWD_VAL 30              // 컨베이어 역방향 속도
#define CONVEYOR_STOP_VAL 90             // 컨베이어 정지 값
volatile char lastReceivedCmd[32] = {0}; // 마지막 수신 명령 (문자열 지원)
volatile bool cmdReceived = false;       // 명령 수신 플래그
volatile uint32_t lastPacketTime = 0;    // 마지막 유효 패킷 수신 시간 (ms)
volatile uint32_t byteCount = 0;         // 수신된 총 바이트 수 (디버그용)
uint32_t stationTravelStartTime = 0;     // 스테이션 주행 시작 시간 측정용
int angle = 90;

// SPI 응답용 버퍼
char spiResponseBuffer[32] = {0};
volatile byte spiResponseIndex = 0;
volatile byte spiResponseLen = 0;

// MotorDrive 객체 생성
MotorDrive motor(MOTOR1_R_EN, MOTOR1_L_EN, MOTOR1_RPWM, MOTOR1_LPWM,
                 MOTOR2_R_EN, MOTOR2_L_EN, MOTOR2_RPWM, MOTOR2_LPWM);

// LineTracer 객체 (기존 로직이 있다면 유지하되, 센서값은 메인에서 넘겨주는 방식
// 고려) 여기서는 메인 루프에서 직접 PID를 돌리는 것이 가장 확실함. LineTracer
// lineTracer(motor);

bool isLineTracing = false;
bool isPIDTracing = false;
bool isAutoMode = false;
bool isAutoStationMode = false; // '2' 명령용 자동 스테이션 모드
int stationStep = 0; // 0: PID, 1: Conveyor, 2: Escape, 3: 90-Turn, 4: Stop Wait
uint32_t stationTimer = 0;
int stationCount = 0;        // 스테이션 감지 횟수
int currentConveyorDir = 90; // 현재 컨베이어 방향 (180: 정방향, 0: 역방향)

bool isMotorTest = false; // 'm' 명령: 모터 좌우 확인용
int motorTestStep = 0;
uint32_t motorTestTimer = 0;

bool isCheckSensors = false; // 'c' 명령 시 센서값 확인용 플래그
uint32_t autoModeTimer = 0;
int autoModeStep = 0;

// PID 변수 (80~120 속도에 최적화)
float Kp = 0.2; // 더 민첩하게 반응하도록 상향 (0.18 -> 0.25)
float Ki = 0.0;
float Kd = 2.5; // 흔들림 보정 및 급커브 대응 강화 (1.6 -> 2.5)
int lastError = 0;

// 주행 환경 설정
bool isWhiteLineMode = false;     // 흰색 선을 따라가려면 true (검은 바닥)
bool isDirectionReversed = false; // 센서 매핑 수정 완료 -> 표준(false) 설정

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  pinMode(SIGNAL_PIN, OUTPUT);
  myservo.attach(SIGNAL_PIN); // 서보 핀 연결
  myservo.write(angle);       // 초기 각도 90도

  motor.begin();
  motor.setSpeed(pwmValue);
  motor.stop();

  // QTR 센서 초기화 (RC 타입)
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setTimeout(2500);   // 읽기 시간 안정화 (2.5ms, QTR_Test와 동기화)
  qtr.setEmitterPin(255); // 이미터 LED를 항상 켬 (QTR-8RC 필수)

  Serial.println(F("\n=== Mega Motor Driver (QTR-8RC Integrated) ==="));
  Serial.println(F("Calibrating QTR Sensors... (Move sensor over line)"));

  // 캘리브레이션 (약 5초 동안)
  // 로봇을 손으로 라인 위에서 좌우로 흔들어주세요.
  digitalWrite(DEBUG_LED, HIGH);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(DEBUG_LED, LOW);

  Serial.println(F("Calibration Complete."));
  Serial.println(F("Protocol: < [CMD] >"));

  // ========== SPI Slave 설정 (Mega 2560) ==========
  pinMode(SPI_SS_PIN, INPUT_PULLUP);
  pinMode(SPI_MOSI_PIN, INPUT);
  pinMode(SPI_MISO_PIN, OUTPUT);
  pinMode(SPI_SCK_PIN, INPUT);

  DDRB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2));
  DDRB |= _BV(PB3);

  SPCR = _BV(SPE) | _BV(SPIE);

  SPDR = 0x00;

  volatile byte dummy = SPSR;
  dummy = SPDR;
  (void)dummy;

  lastPacketTime = millis();
}

// ============== SPI 인터럽트 ==============
ISR(SPI_STC_vect) {
  uint8_t inByte = SPDR;
  static byte state = 0;
  static byte pos = 0;

  // 다음 바이트 준비 (Master가 전송 시 Slave의 SPDR 값이 Master로 이동함)
  if (spiResponseIndex < spiResponseLen) {
    SPDR = spiResponseBuffer[spiResponseIndex++];
  } else {
    SPDR = 0x00; // 보낼 데이터 없음
  }

  if (inByte == '<') {
    state = 1;
    pos = 0;
  } else if (state == 1) {
    if (inByte == '>') {
      lastReceivedCmd[pos] = '\0';
      cmdReceived = true;
      lastPacketTime = millis();
      state = 0;
    } else {
      if (pos < 30) {
        lastReceivedCmd[pos++] = (char)inByte;
      }
    }
  }
}
void processCommand(char input) {
  if (input == 'h') {
    angle = CONVEYOR_FWD_VAL;
    Serial.println("서보: 정방향(H)");
  } else if (input == 'j') {
    angle = CONVEYOR_BWD_VAL;
    Serial.println("서보: 역방향(J)");
  } else if (input == 'k') {
    angle = CONVEYOR_STOP_VAL;
    Serial.println("서보: 정지(K)");
  }
  angle = constrain(angle, 0, 180);
  myservo.write(angle);
}
// ============== Main Loop ==============
void loop() {
  uint32_t now = millis();

  // 1. SPI 명령 처리 (최우선 순위)
  if (cmdReceived) {
    String cmd = String((char *)lastReceivedCmd);
    cmdReceived = false;

    Serial.print(F("[SPI] CMD: "));
    Serial.println(cmd);

    if (cmd != "l" && cmd != "p" && cmd != "PID" && cmd != "MOVE" &&
        cmd != "1" && cmd != "c" && cmd != "CHECK" && cmd != "h" &&
        cmd != "j" && cmd != "k") {
      isLineTracing = false;
      isPIDTracing = false;
      isAutoMode = false;
      isCheckSensors = false;
      isAutoStationMode = false;
      isMotorTest = false;
      motor.stop();
    }

    if (cmd == "w" || cmd == "FWD")
      motor.forward();
    else if (cmd == "s" || cmd == "BWD")
      motor.backward();
    else if (cmd == "a" || cmd == "LEFT")
      motor.turnLeft(turnPwmValue);
    else if (cmd == "d" || cmd == "RIGHT")
      motor.turnRight(turnPwmValue);
    else if (cmd == "x" || cmd == "STOP" || cmd == "ESTOP")
      motor.stop();
    else if (cmd == "l" || cmd == "LINE") {
      isLineTracing = true;
      isPIDTracing = false;
      isCheckSensors = false;
    } else if (cmd == "p" || cmd == "PID" || cmd == "MOVE") {
      isPIDTracing = true;
      isLineTracing = false;
      isAutoMode = false;
      isCheckSensors = false;
    } else if (cmd == "1" || cmd == "AUTO") {
      isAutoMode = true;
      isLineTracing = false;
      isPIDTracing = false;
      isCheckSensors = false;
      autoModeTimer = now;
      autoModeStep = 0;
      motor.forward();
    } else if (cmd == "c" || cmd == "CHECK") {
      isCheckSensors = !isCheckSensors;
      if (isCheckSensors) {
        isLineTracing = false;
        isPIDTracing = false;
        isAutoMode = false;
        motor.stop();
        Serial.println(F("[MODE] Sensor Check ON (Motors Halted)"));
      } else {
        Serial.println(F("[MODE] Sensor Check OFF"));
      }
    } else if (cmd == "2") {
      isAutoStationMode = true;
      isPIDTracing = false;
      isLineTracing = false;
      isAutoMode = false;
      stationStep = 0;
      stationCount = 0;             // 카운트 초기화
      stationTravelStartTime = now; // 주행 시간 측정 시작
      Serial.println(F("[MODE] Auto-Station Mode 2 Started. Count Reset."));
    } else if (cmd == "i") {
      // 수동 QTR 보정 시작 + 자동 전후진 (0.1초 간격)
      Serial.println(F("[SYSTEM] Auto-Movement Calibration Starting (5s)..."));
      digitalWrite(DEBUG_LED, HIGH);
      for (uint16_t i = 0; i < 250; i++) {
        // 0.1초(5회 루프)마다 방향 전환
        if ((i / 5) % 2 == 0) {
          motor.forward(pwmValue);
        } else {
          motor.backward(pwmValue);
        }

        qtr.calibrate();

        if (i % 50 == 0)
          Serial.print(".");
        delay(20);
      }
      motor.stop();
      digitalWrite(DEBUG_LED, LOW);
      Serial.println(F("\n[SYSTEM] Calibration Done!"));
    } else if (cmd == "m") {
      isMotorTest = true;
      motorTestStep = 1;
      motorTestTimer = now;
      isPIDTracing = false;
      isLineTracing = false;
      isAutoMode = false;
      isAutoStationMode = false;
      motor.stop();
      Serial.println(F("[DIAG] Motor Test Started: 1.LEFT(A), 2.RIGHT(B)"));
    } else if (cmd == "h" || cmd == "j" || cmd == "k") {
      processCommand(cmd.charAt(0));
    }
  }

  // 2. 센서 읽기
  uint16_t position = 0;
  if (isPIDTracing || isAutoMode || isCheckSensors || isAutoStationMode) {
    position = qtr.readLineBlack(sensorValues);
  }

  // 3. 라인트레이싱 및 오토 모드 로직 실행
  if (isAutoMode) {
    if (now - autoModeTimer >= 1000) {
      autoModeTimer = now;
      autoModeStep = (autoModeStep + 1) % 2;
      if (autoModeStep == 0) {
        motor.forward();
        Serial.println(F("[AUTO] Forwarding..."));
      } else {
        motor.turnLeft();
        Serial.println(F("[AUTO] Turning Left..."));
      }
    }
  } else if (isLineTracing) {
    // lineTracer.runBasic(); // 이 부분은 QTR 센서로 대체되거나 다른 방식으로
    // 구현될 수 있음 현재는 isLineTracing이 'l' 명령으로 켜지지만, 실제 동작은
    // 없음. 필요하다면 QTR 센서의 디지털 값으로 간단한 라인 추적 로직을 여기에
    // 추가할 수 있음.
    motor.forward(); // 임시로 그냥 전진
  } else if (isPIDTracing) {
    // 라인을 놓쳤는지 확인 (모든 센서가 흰색 바닥일 때)
    bool lineVisible = false;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 200) { // 0 초과 -> 200 상향 (노이즈 방지)
        lineVisible = true;
        break;
      }
    }

    if (!lineVisible) {
      motor.stop();
      isPIDTracing = false;
      Serial.println(F("[STOP] Line Lost! (All White)"));
    } else {
      // QTRSensors 라이브러리를 사용한 정밀 PID 제어 (8핀: 0 ~ 7000)
      int center = 3500;
      int error = (int)position - center;

      // 환경에 따른 에러 반전
      if (isWhiteLineMode)
        error = -error;
      if (isDirectionReversed)
        error = -error;

      int P = error;
      int D = error - lastError;
      lastError = error;

      float motorSpeedChange = (float)P * Kp + (float)D * Kd;

      // 표준 조향 공식 (left += change, right -= change)
      // Line이 오른쪽에 있으면 error > 0 -> Left 증가 -> Right 회전
      int leftMotorSpeed = (int)((float)pwmValue + motorSpeedChange);
      int rightMotorSpeed = (int)((float)pwmValue - motorSpeedChange);

      // 속도 제한 (0 ~ maxPIDSpeed)
      leftMotorSpeed = constrain(leftMotorSpeed, 0, maxPIDSpeed);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, maxPIDSpeed);

      motor.setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    }
  } else if (isAutoStationMode) {
    bool lineVisible = false;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 500) { // 200 -> 500 상향 (안정적 라인 인지)
        lineVisible = true;
        break;
      }
    }

    if (stationStep == 0) {
      // 모든 센서가 검은색이면 스테이션 도착으로 간주 (임계값 500)
      int blackSensorCount = 0;
      for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 700) { // 800 -> 500 하향 (안정적 도착 인지)
          blackSensorCount++;
        }
      }

      if (blackSensorCount >= SensorCount) {
        motor.stop();
        stationStep = 4; // 3초 대기 단계로
        stationTimer = now;
        stationCount++; // 스테이션 카운트 증가

        // 주행 시간 계산
        uint32_t duration = now - stationTravelStartTime;

        // 방향 결정: 홀수(1,3,5...) -> 역방향, 짝수(2,4,6...) -> 정방향
        if (stationCount % 2 == 0) {
          currentConveyorDir = CONVEYOR_FWD_VAL;
          Serial.print(F("[STATION] #"));
          Serial.print(stationCount);
          Serial.println(F(" (EVEN): Forward Dir selected."));
        } else {
          currentConveyorDir = CONVEYOR_BWD_VAL;
          Serial.print(F("[STATION] #"));
          Serial.print(stationCount);
          Serial.println(F(" (ODD): Reverse Dir selected."));
        }

        // SPI 응답 버퍼에 저장 (ESP32 전송용: C:count,T:duration)
        snprintf(spiResponseBuffer, sizeof(spiResponseBuffer), "C:%d,T:%lu",
                 stationCount, duration);
        spiResponseLen = strlen(spiResponseBuffer);
        spiResponseIndex = 0;
        SPDR = spiResponseBuffer[spiResponseIndex++]; // 첫 바이트 즉시 로드

        Serial.println(F("[STATION] Arrived. Waiting 3s..."));
      } else {
        if (!lineVisible) {
          // 라인 유실 감지 시 복구 단계로 전환
          motor.stop();
          stationStep = 5;
          stationTimer = now;
          Serial.println(F("[STATION] Line Lost! Starting Recovery..."));
        } else {
          // PID 제어 (중앙값 3500)
          int center = 3500;
          int error = (int)position - center;
          if (isWhiteLineMode)
            error = -error;
          if (isDirectionReversed)
            error = -error;

          int P = error;
          int D = error - lastError;
          lastError = error;
          float motorSpeedChange = (float)P * Kp + (float)D * Kd;

          int leftMotorSpeed = (int)((float)pwmValue + motorSpeedChange);
          int rightMotorSpeed = (int)((float)pwmValue - motorSpeedChange);
          leftMotorSpeed = constrain(leftMotorSpeed, 0, maxPIDSpeed);
          rightMotorSpeed = constrain(rightMotorSpeed, 0, maxPIDSpeed);
          motor.setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
        }
      }
    } else if (stationStep == 4) {
      // Step 4: 멈춘 후 대기
      if (now - stationTimer >= STATION_PAUSE_TIME) {
        stationStep = 1;
        stationTimer = now;
        myservo.write(currentConveyorDir); // 결정된 방향으로 작동
        Serial.print(F("[STATION] Operating Conveyor (3s) Dir: "));
        Serial.println(currentConveyorDir);
      }
    } else if (stationStep == 1) {
      // Step 1: 컨베이어 작동
      if (now - stationTimer >= CONVEYOR_RUN_TIME) {
        myservo.write(CONVEYOR_STOP_VAL); // 컨베이어 정지
        stationStep = 2;
        stationTimer = now;
        motor.forward(pwmValue); // 탈출 전진 시작
        Serial.println(F("[STATION] Conveyor Done. Escaping (1s)..."));
      }
    } else if (stationStep == 2) {
      // Step 2: 스테이션 영역 탈출 (무조건 전진)
      if (now - stationTimer >= STATION_ESCAPE_TIME) {
        stationStep = 0; // 다시 라인 트래킹(Step 0)으로 복귀
        Serial.println(F("[STATION] Escape Complete. Resuming PID..."));
      }
    } else if (stationStep == 5) {
      // Step 5: 라인 유실 복구 (좌우 회전 탐색)
      if (lineVisible) {
        stationStep = 0;
        Serial.println(F("[STATION] Line Recovered. Resuming PID..."));
      } else {
        uint32_t elapsed = now - stationTimer;
        if (elapsed < 500) {
          // 왼쪽으로 회전
          motor.setMotorSpeeds(-turnPwmValue, turnPwmValue);
        } else if (elapsed < 1500) {
          // 오른쪽으로 회전 (더 길게)
          motor.setMotorSpeeds(turnPwmValue, -turnPwmValue);
        } else if (elapsed < 2000) {
          // 다시 제자리로 (왼쪽으로 짧게)
          motor.setMotorSpeeds(-turnPwmValue, turnPwmValue);
        } else {
          // 검색 실패 시 일단 정지 후 반복 시도
          motor.stop();
          if (elapsed > 10000) { // 10초 이상 못 찾으면 모드 종료
            isAutoStationMode = false;
            Serial.println(F("[STATION] Recovery Failed. Mode OFF."));
          } else if (elapsed % 3000 < 100) {
            // 주기적으로 메시지 출력
            Serial.println(F("[STATION] Still searching for line..."));
          }
        }
      }
    }

  } else if (isMotorTest) {
    if (motorTestStep == 1) {
      if (now - motorTestTimer < 1000) {
        motor.setMotorSpeeds(pwmValue, 0); // Motor A (Left) Only
      } else {
        motor.stop();
        motorTestStep = 2;
        motorTestTimer = now;
        Serial.println(
            F("[DIAG] LEFT (Motor A) Test Done. Now RIGHT (Motor B)..."));
      }
    } else if (motorTestStep == 2) {
      if (now - motorTestTimer < 1000) {
        motor.setMotorSpeeds(0, pwmValue); // Motor B (Right) Only
      } else {
        motor.stop();
        isMotorTest = false;
        Serial.println(F("[DIAG] Motor Test Complete. All Halted."));
      }
    }
  }

  // 4. 상태 LED & 디버그 출력
  digitalWrite(DEBUG_LED, (now - lastPacketTime < 300));

  // 5. QTR 센서 값 디버깅 출력
  static unsigned long lastQtrPrint = 0;
  if ((isPIDTracing || isCheckSensors || isAutoStationMode) &&
      (now - lastQtrPrint > 200)) {
    lastQtrPrint = now;

    Serial.print(F("Pos: "));
    Serial.print(position);
    Serial.print(F(" | Error: "));
    Serial.print(lastError);
    if (lastError > 300)
      Serial.print(F(" [Line: R] "));
    else if (lastError < -300)
      Serial.print(F(" [Line: L] "));
    else
      Serial.print(F(" [Line: C] "));

    Serial.print(F(" | ["));
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 500)
        Serial.print(i); // 인덱스 번호 출력 (가독성 증대)
      else
        Serial.print(F("."));
    }
    Serial.print(F("] | "));

    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(F("\t"));
    }
    Serial.println();
  }

  // delay(1); // 루프 지연 제거 (반응 속도 극대화)
}
