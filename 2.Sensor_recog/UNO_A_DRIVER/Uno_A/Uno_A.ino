#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

int sensorPin = A0;

void setup() {
  mySerial.begin(9600);
  
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  mySerial.println(sensorValue); // 센서값 전송
  delay(500);
}