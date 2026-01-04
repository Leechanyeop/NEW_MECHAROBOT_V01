#include <SPI.h>

#define CS_PIN 5   // Chip Select 핀


void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
}

// 엔코더 값 읽기
long readEncoder() {
  union { //- 같은 메모리 공간을 4바이트 배열(b[0]..b[3])과 32비트 정수(val) 로 공유합니다.
    long val;
    byte b[4];
  } data;

  digitalWrite(CS_PIN, LOW);// slave 선택
  for (int i = 0; i < 4; i++) {
    data.b[i] = SPI.transfer(0x00);  // 더미 데이터 전송, 응답만 수신
  }
  digitalWrite(CS_PIN, HIGH);//통신 종료

  return data.val;
}

void loop() {
  long enc = readEncoder();   // 엔코더 값 읽기
  Serial.println(enc);        // 값 출력
  delay(100);
}