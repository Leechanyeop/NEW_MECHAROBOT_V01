// 모터1 (드라이버1)
const int R_EN  = 5;   // Right Enable
const int L_EN  = 4;   // Left Enable
const int RPWM  = 7;   // Right PWM
const int LPWM  = 6;   // Left PWM

// 모터2 (드라이버2)


const int R_EN2  = 10;
const int L_EN2  = 11;
const int RPWM2  = 12;
const int LPWM2  = 13;

void setup() {
  // 모터1 핀 설정
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // 모터2 핀 설정
  pinMode(R_EN2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);

  // Enable 핀 항상 HIGH
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);
}

void loop() {
  // === 모터1 정방향 ===
  analogWrite(RPWM, 200);   // 속도 제어 (0~255)
  analogWrite(LPWM, 0);

  // === 모터2 정방향 ===
  analogWrite(RPWM2, 200);
  analogWrite(LPWM2, 0);

  delay(3000);

  // === 모터1 역방향 ===
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 200);

  // === 모터2 역방향 ===
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 200);

  delay(3000);

  // === 정지 ===
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 0);

  delay(2000);
}