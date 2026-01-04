#include <SPI.h>

// 엔코더 핀
#define ENCODER_A 20
#define ENCODER_B 21

//PWM
#define IN1_A     22
#define IN2_A     23
#define IN3_B     24
#define IN4_B     25
#define ENABLE_A  26
#define ENABLE_B  27

volatile long encoderPos = 0;
int pwmValue = 150;

void setup() {
  pinMode(ENABLE_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);
  pinMode(IN3_B, OUTPUT);
  pinMode(IN4_B, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // SPI Slave 설정
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);   // SPI Enable
  SPCR |= _BV(SPIE);  // SPI Interrupt Enable

  Serial.begin(115200);
  Serial.println("=== SPI Slave Ready ===");
}

void motor_speed(int spd) {
  analogWrite(ENABLE_A, spd);
  analogWrite(ENABLE_B, spd);
}

void motor_dir(int dir) {
  if (dir == 0) { // forward
    digitalWrite(IN1_A, HIGH);
    digitalWrite(IN2_A, LOW);
    digitalWrite(IN3_B, HIGH);
    digitalWrite(IN4_B, LOW);
  } else if (dir == 1) { // right
    digitalWrite(IN1_A, HIGH);
    digitalWrite(IN2_A, LOW);
    digitalWrite(IN3_B, LOW);
    digitalWrite(IN4_B, HIGH);
  } else if (dir == 2) { // left
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, HIGH);
    digitalWrite(IN3_B, HIGH);
    digitalWrite(IN4_B, LOW);
  } else if (dir == 4) { // backward
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, HIGH);
    digitalWrite(IN3_B, LOW);
    digitalWrite(IN4_B, HIGH);
  } else { // stop
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);
    digitalWrite(IN3_B, LOW);
    digitalWrite(IN4_B, LOW);
  }
}

// 엔코더 인터럽트
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b) encoderPos++;
  else encoderPos--;
}

// SPI 인터럽트 (Master가 보낸 데이터 수신)
ISR(SPI_STC_vect) {
  char cmd = SPDR;  // Master가 보낸 명령

  // 명령 처리
  if (cmd == 'w') motor_dir(0);
  else if (cmd == 's') motor_dir(4);
  else if (cmd == 'a') motor_dir(2);
  else if (cmd == 'd') motor_dir(1);
  else if (cmd == 'x') motor_dir(9);
  else if (cmd == '+') {
    pwmValue += 20;
    if (pwmValue > 255) pwmValue = 255;
  }
  else if (cmd == '-') {
    pwmValue -= 20;
    if (pwmValue < 0) pwmValue = 0;
  }

  motor_speed(pwmValue);

  
  static byte idx = 0; // 응답으로 엔코더 값 전송
  union {//- 같은 메모리 공간을 4바이트 배열(b[0]..b[3])과 32비트 정수(val) 로 공유합니다.
    long val;
    byte b[4];
  } data;

  data.val = encoderPos; //4 바이트 값 한번에 읽음
  SPDR = data.b[idx];
  idx++;
  if (idx >= 4) idx = 0;
}

void loop() {
  // 메인 루프는 SPI 인터럽트로 처리되므로 특별히 필요 없음
  Serial.println(encoderPos);
  delay(100);
}