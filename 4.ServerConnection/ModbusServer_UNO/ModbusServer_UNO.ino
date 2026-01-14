/*
 * Modbus TCP Server for UNO WiFi Rev2
 * 
 * 기능:
 * 1. 502번 포트를 열고(Listening) 기다립니다.
 * 2. 외부 클라이언트(PC, HMI 등)가 접속하면 응답합니다.
 * 3. Holding Register(40001~)에 값을 쓰고 읽을 수 있게 해줍니다.
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoModbus.h>

// 네트워크 설정
const char* ssid     = "hhme";
const char* pass     = "hme*12345";

// Modbus TCP 서버 객체 (502 포트) -> 실제로는 Raw TCP 서버로 동작 (Telnet 문자열 수신용)
WiFiServer wifiServer(502);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Raw TCP Server on Port 502 (UNO WiFi Rev2)");

  // 1. WiFi 연결
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // 2. 서버 시작
  wifiServer.begin();

  Serial.println("Server is listening on port 502 (Ready for Telnet)...");
}

void loop() {
  // 클라이언트의 접속을 기다리고 처리
  WiFiClient client = wifiServer.available()

  if (client) {
    Serial.println("New Client Connected via Telnet!");

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c); // 받은 문자 시리얼에 출력
        
        // 여기에 SPI 전송 코드 추가 가능
        // sendToMega(c); 
      }
    }
    Serial.println("Client Disconnected");
  }
}

