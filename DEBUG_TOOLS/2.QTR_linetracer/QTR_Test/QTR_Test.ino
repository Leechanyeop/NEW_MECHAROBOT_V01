/**
 * @file QTR_Test.ino
 * @brief QTR-8RC 센서 값 읽기 테스트 프로그램
 *
 * 기능:
 * 1. 센서 보정 (처음 5초간)
 * 2. 보정된 센서 값 (0~1000) 및 라인 위치 값 (0~7000) 출력
 *
 * 연결:
 * QTR 센서 핀: 22, 23, 24, 25, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
 * 39, 40 (총 18개)
 */

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 18;
const uint8_t qtrPins[] = {22, 23, 24, 25, 27, 28, 29, 30, 31,
                           32, 33, 34, 35, 36, 37, 38, 39, 40};
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(115200);

  // 센서 설정
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setTimeout(2500);   // 읽기 시간 안정화 (2.5ms)
  qtr.setEmitterPin(255); // 이미터 항상 켜짐
  // 즉, 아두이노가 소프트웨어적으로 LED를 껐다 켰다 할 수 있는 핀이 지정되지
  // 않았음을 선언하는 것입니다.

  Serial.println("\n=== QTR-8RC Sensor Test ===");
  Serial.println("Initial Calibration (5 seconds)...");
  Serial.println("Move the sensor back and forth over the line.");

  // 캘리브레이션 (약 5초)
  // 센서가 검은색(최대값)과 흰색(최소값)을 모두 읽어야 정확합니다.
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    if (i % 50 == 0)
      Serial.print(".");
    delay(20);
  }
  Serial.println("\nCalibration Done!");

  // 보정된 최소/최대값 출력
  Serial.print("Min: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Max: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(" ");
  }
  Serial.println("\n");
  delay(1000);
}

void loop() {
  // 라인 위치 읽기 (0 ~ 17000)
  // 0: 왼쪽 끝, 8500: 중앙, 17000: 오른쪽 끝
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 센서 개별 값 출력 (0 ~ 1000, 1000이 검은색)
  Serial.print("Val: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    // 임계값 조정 (500 이상이면 감지된 것으로 표시)
    if (sensorValues[i] > 1000) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    Serial.print("\t");
  }

  // 계산된 위치 출력
  Serial.print("| Pos: ");
  Serial.println(position);

  delay(250); // 0.25초마다 출력
}
