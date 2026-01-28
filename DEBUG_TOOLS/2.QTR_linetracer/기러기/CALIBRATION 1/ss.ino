#include <QTRSensors.h>

// QTR-8RC: RC 타입이므로 QTRSensorsRC 사용
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// 너가 꽂을 핀(임의 지정)
uint8_t qtrPins[SensorCount] = {28, 29, 30, 31, 32, 33, 34, 35};

// LEDON 제어 핀(선택)
const uint8_t LEDON_PIN = 22;

// 라인이 "검정 테이프"라서 일반적으로 검정이 반사 낮음(= 더 큰 값/더 작은 값은 환경따라 다름)
// QTR 라이브러리는 calibration을 해주면 readLineBlack / readLineWhite로 처리 가능
// 보통 검정라인이면 readLineBlack 사용하면 편함.

void setup() {
  Serial.begin(115200);

  pinMode(LEDON_PIN, OUTPUT);
  digitalWrite(LEDON_PIN, HIGH); // LED ON (보드에 따라 LOW가 ON인 것도 있는데 대부분 HIGH=ON)

  // qtr 설정
  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);
  // RC 타임아웃(마이크로초) - 환경에 따라 2500~4000 정도 많이 씀
  qtr.setTimeout(2500);

  delay(500);

  // 캘리브레이션: 로봇을 손으로 라인 위/바a깥을 왔다갔다 시키면서 값 학습
  Serial.println("Calibrating... move sensor across line and background");
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(10);
  }
  Serial.println("Calibration done.");

  // 참고용: min/max 확인
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void loop() {
  // 검정 라인을 따라가면 readLineBlack
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 값 출력
  Serial.print("pos: ");
  Serial.print(position);
  Serial.print("  |  ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(20);
}