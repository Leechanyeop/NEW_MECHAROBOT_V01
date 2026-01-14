#include <WiFiNINA.h>
#include <ArduinoModbus.h>

// WiFi 설정
const char* ssid     = "hhme";
const char* password = "hme*12345";

// ModbusTCP 서버 IP
IPAddress serverIP(192, 168, 0, 20);
int serverPort = 502;

WiFiClient wifiClient;
ModbusTCPClient modbusTCPClient(wifiClient);

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" 연결 완료!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());

  if (!modbusTCPClient.begin(serverIP, serverPort)) {
    Serial.println("ModbusTCP 서버 연결 실패!");
    while (1);
  }
  Serial.println("ModbusTCP 서버 연결 성공!");
}

void loop() {
  // Position.X (40001-40002, Float)
  uint16_t low = modbusTCPClient.holdingRegisterRead(40001);
  uint16_t high = modbusTCPClient.holdingRegisterRead(40002);
  float posX = registersToFloat(low, high);
  Serial.print("Robot.Position.X = ");
  Serial.println(posX);

  // Speed (40007, Int16)
  int speed = modbusTCPClient.holdingRegisterRead(40007);
  Serial.print("Robot.Speed = ");
  Serial.println(speed);

  // Status (40008, Int16)
  int status = modbusTCPClient.holdingRegisterRead(40008);
  Serial.print("Robot.Status = ");
  Serial.println(status); // 0=IDLE, 1=RUNNING, 2=ARRIVED

  // ArrivalTime (40021-40022, Int32 UNIX Timestamp)
  uint16_t tLow = modbusTCPClient.holdingRegisterRead(40021);
  uint16_t tHigh = modbusTCPClient.holdingRegisterRead(40022);
  uint32_t timestamp = registersToInt32(tLow, tHigh);
  Serial.print("Robot.ArrivalTime (UNIX) = ");
  Serial.println(timestamp);

  delay(2000);
}

// 유틸 함수: 2레지스터 → Float
float registersToFloat(uint16_t low, uint16_t high) {
  union {
    float f;
    uint16_t regs[2];
  } data;
  data.regs[0] = low;
  data.regs[1] = high;
  return data.f;
}

// 유틸 함수: 2레지스터 → Int32
uint32_t registersToInt32(uint16_t low, uint16_t high) {
  return ((uint32_t)high << 16) | low;
}