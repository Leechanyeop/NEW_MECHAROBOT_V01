
// ============== WiFi 설정 ==============
char ssid[] = "codelab";      // 와이파이 이름
char pass[] = "20380800";  // 와이파이 비밀번호
int status = WL_IDLE_STATUS;
WiFiServer server(8888);        // 8888번 포트 TCP 서버 (CMD 제어용)
ModbusTCPServer modbusServer;   // Modbus TCP 서버 (포트 502)

// ============== 기본 설정 ==============
#define DEFAULT_SPEED 200

// ============== SPI 핀 설정 (Arduino UNO WiFi Rev2) ==============
#define SPI_SS    8 // Slave Select (표준 SPI SS 핀)

// ============== 엔코더 핀 설정 (UNO 인터럽트 핀 2, 3) ==============
#define ENCODER_1A 2   // 엔코더 1 - A상 (인터럽트 0)
#define ENCODER_1B 4   // 엔코더 1 - B상
#define ENCODER_2A 3   // 엔코더 2 - A상 (인터럽트 1)
#define ENCODER_2B 5   // 엔코더 2 - B상

// ============== 명령어 정의 ==============
#define CMD_FORWARD     'w'
#define CMD_BACKWARD    's'
#define CMD_LEFT        'a'
#define CMD_RIGHT       'd'
#define CMD_STOP        'x'
#define CMD_SPEED_UP    '+'
#define CMD_SPEED_DOWN  '-'
#define CMD_LINE_TRACE  'l'
#define CMD_LINE_PID    'p'

// ============== 전역 변수 ==============
char currentCommand = CMD_STOP;

// 엔코더 카운트 (volatile - 인터럽트에서 사용)
volatile long encoderCount[2] = {0, 0};

// ============== 함수 선언 ==============
void sendCommand(char cmd);
void processSerialCommand();
void printEncoderValues();
void resetEncoders();
void processModbus(); // Modbus 처리 함수 추가
void sendDataToOPC(); // 외부 OPC 서버 데이터 전송 함수 추가