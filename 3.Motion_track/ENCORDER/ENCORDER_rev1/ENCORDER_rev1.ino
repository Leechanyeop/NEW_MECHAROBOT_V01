// === 엔코더 핀 설정 ===
const uint8_t pinA = A0;   // Encoder A
const uint8_t pinB = A1;   // Encoder B

volatile long pulseCount = 0;
volatile int direction = 0; // +1 정방향, -1 역방향
volatile uint8_t prevAB = 0;

// === 모터 제어 핀 설정 ===
const int enA = 9;   // PWM 핀 (속도 제어)
const int in1 = 2;   // 방향 제어 1
const int in2 = 3;   // 방향 제어 2

void setup() {
  Serial.begin(115200);

  // 엔코더 핀
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  prevAB = (a << 1) | b;

  PCICR |= (1 << PCIE1);        // PCINT[14:8] 인터럽트 허용
  PCMSK1 |= (1 << PCINT8);      // A0 에지 감지
  PCMSK1 |= (1 << PCINT9);      // A1 에지 감지

  // 모터 핀
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

// === 엔코더 인터럽트 ===
ISR(PCINT1_vect) {
  uint8_t a = (PINC & (1 << PC0)) ? HIGH : LOW; // A0 = PC0
  uint8_t b = (PINC & (1 << PC1)) ? HIGH : LOW; // A1 = PC1
  uint8_t ab = (a << 1) | b;

  if (ab != prevAB) {
    if (((prevAB >> 1) & 0x1) != ((ab >> 1) & 0x1)) { // A가 바뀐 경우
      if (a == b) {
        direction = +1;
        pulseCount++;
      } else {
        direction = -1;
        pulseCount--;
      }
    }
    prevAB = ab;
  }
}

// === 모터 제어 함수 ===
void motorForward(int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed); // 0~255
}

void motorBackward(int speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
}

void motorStop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

// === 메인 루프 ===
void loop() {
  static long lastReported = 0;
  if (lastReported != pulseCount) {
    lastReported = pulseCount;
    noInterrupts();
    long p = pulseCount;
    int dir = direction;
    interrupts();

    Serial.print("pulses: ");
    Serial.print(p);
    Serial.print("  dir: ");
    Serial.println(dir > 0 ? "CW" : "CCW");
  }

  // 예시: 모터 제어
  motorForward(200);   // 정방향, 속도 200
  delay(2000);

  motorBackward(200);  // 역방향, 속도 200
  delay(2000);

  motorStop();         // 정지
  delay(2000);
}