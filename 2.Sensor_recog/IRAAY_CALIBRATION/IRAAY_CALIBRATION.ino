// 5채널 라인센서 캘리브레이션 + 위치 계산 스케치
// 센서: A0..A4 (왼쪽 -> 오른쪽)
// 정규화 스케일: 0..1000 (검정=0, 흰=1000)
// 위치 가중치: -2000, -1000, 0, 1000, 2000

const int NUM_SENSORS = 5;
const int sensorPin[NUM_SENSORS] = {A1, A2, A3, A4,A5};

// 캘리브레이션 샘플 수
const int CAL_SAMPLES = 50;

// 정규화 값 저장
long whiteVal[NUM_SENSORS];
long blackVal[NUM_SENSORS];
int calibrated = 0; // 0: 미캘리브레이션, 1: 캘리브레이션 완료

// 이동평균 버퍼 (간단 윈도우 3)
const int MA_WINDOW = 3;
int maBuf[NUM_SENSORS][MA_WINDOW];
int maIndex = 0;

// 가중치 (왼쪽 -> 오른쪽)
const int weights[NUM_SENSORS] = {-2000, -1000, 0, 1000, 2000};

// PID 자리
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;
float integral = 0;
float lastError = 0;

// 유틸: 아날로그 읽기 평균
long avgRead(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5); // 센서 안정화
  }
  return sum / samples;
}

// 정규화: raw -> 0..1000 (검정=0, 흰=1000)
int normalizeSensor(long raw, long black, long white) {
  if (white == black) return 0;
  long val = (raw - black) * 1000L / (white - black);
  if (val < 0) val = 0;
  if (val > 1000) val = 1000;
  return (int)val;
}

// 이동평균 업데이트 및 반환
int movingAverage(int sensorIndex, int newVal) {
  maBuf[sensorIndex][maIndex] = newVal;
  int sum = 0;
  for (int i = 0; i < MA_WINDOW; i++) sum += maBuf[sensorIndex][i];
  return sum / MA_WINDOW;
}

// 위치 계산: 정규화값 배열 -> position (-2000..+2000)
// 반환: position, and total (sum of values) via reference
int computePosition(int normVals[], int &total) {
  long num = 0;
  total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    num += (long)normVals[i] * weights[i];
    total += normVals[i];
  }
  if (total == 0) {
    // 라인을 잃었을 때 처리: 이전 에러 기반 추정 또는 0 반환
    return 0;
  }
  int position = (int)(num / total); // -2000..+2000
  return position;
}

// PID 제어(간단)
float computePID(float error, float dt) {
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float out = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  return out;
}

void setup() {
  Serial.begin(115200);
  Serial.println("5채널 라인센서 캘리브레이션 스케치");
  Serial.println("명령: w=화이트 캘리브레이션, b=블랙 캘리브레이션, s=시작");
  // 초기화: 이동평균 버퍼 0으로
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < MA_WINDOW; j++) maBuf[i][j] = 0;
    whiteVal[i] = 1023; // 기본값
    blackVal[i] = 0;
  }
}

void loop() {
  // 시리얼 명령 처리 (간단)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      Serial.println("화이트(바닥) 캘리브레이션 시작: 라인 없는 흰 바닥에 올려두세요.");
      for (int i = 0; i < NUM_SENSORS; i++) {
        whiteVal[i] = avgRead(sensorPin[i], CAL_SAMPLES);
      }
      Serial.print("화이트값: ");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(whiteVal[i]);
        if (i < NUM_SENSORS - 1) Serial.print(", ");
      }
      Serial.println();
    } else if (c == 'b') {
      Serial.println("블랙(라인) 캘리브레이션 시작: 센서를 라인 위에 올려두세요.");
      for (int i = 0; i < NUM_SENSORS; i++) {
        blackVal[i] = avgRead(sensorPin[i], CAL_SAMPLES);
      }
      Serial.print("블랙값: ");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(blackVal[i]);
        if (i < NUM_SENSORS - 1) Serial.print(", ");
      }
      Serial.println();
    } else if (c == 's') {
      // 간단 유효성 검사
      bool ok = true;
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (whiteVal[i] <= blackVal[i]) {
          ok = false;
          Serial.print("캘리브레이션 오류: 센서 ");
          Serial.print(i);
          Serial.println(" 의 white <= black 입니다. 다시 캘리브레이션 하세요.");
        }
      }
      if (ok) {
        calibrated = 1;
        Serial.println("캘리브레이션 완료. 실시간 출력 시작.");
      } else {
        Serial.println("캘리브레이션 실패. w와 b를 다시 수행하세요.");
      }
    }
  }

  // 메인 루프: 캘리브레이션 완료 시 센서 읽고 정규화 -> 위치 계산 -> PID 출력
  if (calibrated) {
    int raw[NUM_SENSORS];
    int norm[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      raw[i] = analogRead(sensorPin[i]);
      norm[i] = normalizeSensor(raw[i], blackVal[i], whiteVal[i]);
      // 이동평균 적용
      norm[i] = movingAverage(i, norm[i]);
    }
    maIndex = (maIndex + 1) % MA_WINDOW;

    int total = 0;
    int position = computePosition(norm, total); // -2000..+2000

    // 에러: 중앙(0) 기준으로 position 사용 (원하면 offset 추가)
    float error = (float)position; // 목표는 0
    // dt 계산 (간단 고정 dt 사용 예: 0.02s -> 50Hz)
    float dt = 0.02;
    float correction = computePID(error, dt);

    // 시리얼 출력: raw, norm, position, correction
    Serial.print("RAW: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(raw[i]);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
    Serial.print(" | NORM: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(norm[i]);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
    Serial.print(" | POS: ");
    Serial.print(position);
    Serial.print(" | TOT: ");
    Serial.print(total);
    Serial.print(" | CORR: ");
    Serial.println(correction);

    // TODO: correction을 모터 속도로 변환하여 모터 드라이버에 출력
    // 예: leftSpeed = base - correction, rightSpeed = base + correction
    // setMotorSpeed(leftSpeed, rightSpeed);

    delay(20); // 루프 주기 조절
  } else {
    delay(50);
  }
}