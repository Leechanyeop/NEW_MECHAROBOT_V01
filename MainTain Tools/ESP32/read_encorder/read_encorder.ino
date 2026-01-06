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
  union {
    long val;
    byte b[4];
  } data;

  digitalWrite(CS_PIN, LOW); // slave 선택
  for (int i = 0; i < 4; i++) {
    data.b[i] = SPI.transfer(0x00);  // 더미 데이터 전송, 응답 수신
  }
  digitalWrite(CS_PIN, HIGH); // 통신 종료

  return data.val;
}

// Slave에게 제어 명령 보내기
void sendCommand(char cmd) {
  digitalWrite(CS_PIN, LOW);   // Slave 선택
  SPI.transfer(cmd);           // 명령 전송
  digitalWrite(CS_PIN, HIGH);  // 통신 종료
}

void loop() {
  // 예시: 앞으로 → 2초 → 정지 → 1초 → 뒤로 → 2초 → 정지
  sendCommand('w'); // forward
  delay(2000);

  sendCommand('x'); // stop
  delay(1000);

  sendCommand('s'); // backward
  delay(2000);

  sendCommand('x'); // stop
  delay(1000);

  // 엔코더 값 읽기
  long enc = readEncoder();
  Serial.print("Encoder: ");
  Serial.println(enc);

  delay(500);
}