/**
 * @file ESP32_SPI_Master_Test.ino
 * @brief ESP32 SPI Master Test Code
 * 
 * Sends 'A', 'B', 'C', ... to the Slave every 1 second.
 * 
 * Wiring:
 * ESP32 (Master) <-> Uno (Slave)
 * GPIO18 (SCK)   <-> Pin 13
 * GPIO23 (MOSI)  <-> Pin 11
 * GPIO19 (MISO)  <-> Pin 12
 * GPIO5  (SS)    <-> Pin 10
 * GND            <-> GND
 * 
 * Note: Use level shifters if necessary (ESP32 is 3.3V, Uno is 5V).
 */

#include <SPI.h>
#include <WiFi.h>

// ============== WiFi & Server 설정 ==============
const char* ssid = "hhme";            // 와이파이 이름
const char* password = "hme*12345";   // 와이파이 비밀번호
const char* serverIP = "192.168.0.73"; // 서버 IP (예: PC 또는 PLC)
const int serverPort = 8888;          // 서버 포트

#define SPI_SS 5

WiFiClient client;

void setup() {
  Serial.begin(115200);
  
  // 1. SPI 초기화
  SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, SS
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);

  // 2. WiFi 연결
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println("ESP32 SPI Master + WiFi Test Started");
}

char data = 'A';

void loop() {
  // --- 1. SPI 통신 (메가 슬레이브에게 명령 전송) ---
  // 메가가 인식할 수 있도록 <와 >로 감싸서 전송합니다.
  Serial.print("SPI Sending: <");
  Serial.print(data);
  Serial.println(">");

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_SS, LOW);
  
  SPI.transfer('<');                   // 패킷 시작
  delayMicroseconds(20);               // Mega 처리 시간 확보
  char response = SPI.transfer(data);  // 명령 본체 전송
  delayMicroseconds(20);
  SPI.transfer('>');                   // 패킷 종료
  
  digitalWrite(SPI_SS, HIGH);
  SPI.endTransaction();

  Serial.print("SPI Slave Response: ");
  Serial.println(response);

  // --- 2. WiFi를 통한 서버 데이터 전송 (옵션) ---
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      client.connect(serverIP, serverPort); 
    }
    if (client.connected()) {
      client.printf("Slave Response: %c\n", response);
    }
  }

  // 데이터 변경 (A, B, C... 순환 전송)
  // 메가 코드에서 '1'은 자동 모드, 'w','a','s','d' 등은 이동입니다.
  // 테스트를 위해 '1'을 고정해서 보내거나 돌아가면서 보낼 수 있습니다.
  data++;
  if (data > 'Z') data = 'A';

  // --- 3. 중요: 전송 주기 ---
  // 메가의 워치독이 700ms 이므로, 최소한 500ms 마다는 한번씩 보내야 합니다.
  delay(500); 
}
