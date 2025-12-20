#include <ESP32Servo.h>

#define TRIG_PIN 14
#define ECHO_PIN 27

Servo myServo;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  myServo.attach(13);  // 서보 핀
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // cm
}

void loop() {
  // 카메라에서 인식한 x 좌표 (예시: 시리얼로 전달받음)
  int objectX = 120; // 실제로는 DFR1154 카메라 코드에서 받아옴
  int frameWidth = 240;

  // 좌표 → 각도 변환
  int angle = map(objectX, 0, frameWidth, 0, 180);

  // 거리 측정
  float distance = getDistance();

  // 조건: 50cm 이내일 때만 추적
  if (distance < 50) {
    myServo.write(angle);
    Serial.printf("Tracking object at angle %d, distance %.2f cm\n", angle, distance);
  } else {
    Serial.println("No object in range");
  }

  delay(200);
}