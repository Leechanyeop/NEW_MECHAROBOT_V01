# OPC UA 서버 ↔ ESP32 Modbus TCP 태그 통신 메커니즘

## 📡 **아키텍처 개요**

```
┌─────────────────────────────────────────────────────────────┐
│           OPC UA Master (KEPServerEX)                        │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Tag Definitions (KEP Server에서 설정)               │  │
│  │  ├─ POS_X, POS_Y, POS_T (위치 데이터)               │  │
│  │  ├─ TargetA (Boolean 플래그)                         │  │
│  │  ├─ Control (명령 문자열)                             │  │
│  │  └─ State (상태 응답)                                 │  │
│  └──────────────────────────────────────────────────────┘  │
│              ↕ Modbus TCP 프로토콜                         │
└─────────────────────────────────────────────────────────────┘
                          │
                    TCP 포트 502
                          │
┌─────────────────────────────────────────────────────────────┐
│           ESP32 (Modbus TCP Server)                          │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  holdingRegisters[] - 공유 메모리                    │  │
│  │  ├─ [0-1]: POS_X (Float)   → 레지스터 40001-40002   │  │
│  │  ├─ [2-3]: POS_Y (Float)   → 레지스터 40003-40004   │  │
│  │  ├─ [4-5]: POS_T (Float)   → 레지스터 40005-40006   │  │
│  │  ├─ [6]: TargetA (Boolean) → 레지스터 40007.0       │  │
│  │  ├─ [99-108]: Control (String) → 40100-40109        │  │
│  │  └─ [199-208]: State (String)  → 40200-40209        │  │
│  └──────────────────────────────────────────────────────┘  │
│              ↓ SPI 프로토콜 (GPIO5)                        │
│          ┌──────────────────┐                              │
│          │   Mega 2560      │ ← 모터, 센서 제어            │
│          └──────────────────┘                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔄 **데이터 흐름 메커니즘**

### 1️⃣ **OPC 서버 → ESP32 (쓰기 요청)**

#### 메커니즘
```
KEPServerEX (OPC Master)
    ↓
Modbus TCP Write 요청 발송
    ↓ TCP Port 502
ESP32 (서버)
    ↓
processModbusRequest() 함수 처리
    ├─ Function Code 0x05: 단일 코일 쓰기
    │   └─ TargetA 업데이트 (boolean)
    │
    ├─ Function Code 0x06: 단일 레지스터 쓰기
    │   └─ 개별 레지스터 값 업데이트
    │
    └─ Function Code 0x10: 다중 레지스터 쓰기
        └─ Control/State 문자열 업데이트 (20바이트)
    ↓
Control 문자열 파싱
    └─ "MOVE", "2", "3", "4", "5", "c", "x" 등 처리
    ↓
명령 실행
    └─ SPI → Mega 전송
```

#### 예시 1: 자동 모드 시작
```
OPC Tag: Control = "MOVE"

ESP32 수신:
  1. KEPServerEX가 Control 태그 "MOVE"를 쓰기
  2. Modbus Write Multiple (0x10) 요청
  3. ESP32: holdingRegisters[99-108]에 "MOVE" 저장
  4. processModbusRequest()에서 감지
  5. seqStep = SEQ_FORWARD_4M 설정
  6. sendCommand('w') → SPI로 Mega에 전송
  7. 로봇 전진 시작
```

#### 예시 2: 컨베이어 제어
```
OPC Tag: Control = "h"

ESP32 수신:
  1. Control = "h" 쓰기
  2. processCommand('h') 호출
  3. sendCommand('h') → SPI 전송
  4. Mega: 서보 125도 (정방향)
  5. 컨베이어 작동
```

---

### 2️⃣ **ESP32 → OPC 서버 (읽기 응답)**

#### 메커니즘
```
KEPServerEX (주기적으로 읽기)
    ↓
Modbus TCP Read 요청 발송 (Function Code 0x03)
    ↓
ESP32:
    1. 엔코더 입력 → 위치 계산
    2. updateOdometry() → posX, posY, posTheta 갱신
    3. updateHoldingRegisters() → 레지스터 업데이트
    ↓
Modbus Read 응답 전송
    └─ holdingRegisters[] 값 반환
    ↓
KEPServerEX:
    └─ OPC 태그 값 갱신
```

#### 예시: 위치 데이터 읽기
```
OPC가 읽기 요청 (주기: 100ms)
    ↓
ESP32 응답:
  - POS_X (레지스터 0-1): 1.234m (Float)
  - POS_Y (레지스터 2-3): 0.567m (Float)
  - POS_T (레지스터 4-5): 45.6° (Float)
    ↓
KEPServerEX 태그 업데이트:
  POS_X = 1.234
  POS_Y = 0.567
  POS_T = 45.6
```

---

## 📊 **Modbus 레지스터 상세 매핑**

### 읽기 전용 (ESP32 → OPC)
| 레지스터 주소 | 레지스터 범위 | 데이터 타입 | 설명 | 값 범위 |
|-------------|-----------|---------|------|--------|
| **40001-40002** | [0-1] | Float | **POS_X** (위치 X) | -999.999 ~ 999.999 m |
| **40003-40004** | [2-3] | Float | **POS_Y** (위치 Y) | -999.999 ~ 999.999 m |
| **40005-40006** | [4-5] | Float | **POS_T** (회전 각도) | -180 ~ 180 ° |

### 읽기/쓰기 (양방향)
| 레지스터 주소 | 레지스터 범위 | 데이터 타입 | 설명 | 값 |
|-------------|-----------|---------|------|-----|
| **40007.0** | [6] bit0 | Boolean | **TargetA** (제어 플래그) | ON(1) / OFF(0) |
| **40100.20H** | [99-108] | String (20B) | **Control** (명령) | "MOVE", "2", "3", "4", "5", "c", "x" |
| **40200.20H** | [199-208] | String (20B) | **State** (상태) | "RUN", "STOP", "IDLE", "ESTOP", "CHECK" |

---

## 🔤 **Control 명령어 상세 설명**

### 명령어 전송 방법
```cpp
// KEPServerEX에서 (OPC UA 클라이언트)
WriteTag("Control", "MOVE");   // 문자열 쓰기
WriteTag("Control", "2");      // 문자열 쓰기

// ESP32에서 (수신)
String control = getControlString();  // "MOVE" 읽기
if (control == "MOVE") {
    // 자동 시퀀스 시작
}
```

### 명령어 목록

| 명령 | ESP32 처리 | Mega 동작 | State | 설명 |
|------|-----------|---------|-------|------|
| **"w"** | forward() | 전진 | "RUN" | 직진 |
| **"s"** | backward() | 후진 | "RUN" | 후진 |
| **"a"** | turnLeft() | 좌회전 | "RUN" | 좌향 |
| **"d"** | turnRight() | 우회전 | "RUN" | 우향 |
| **"x"** | stop() | 정지 | "STOP" | 즉시 정지 |
| **"p"** | PID 주행 | PID 제어 | "RUN" | PID 라인 트레이싱 |
| **"h"** | compConveyor(h) | 서보 125° | "RUN" | 컨베이어 정방향 |
| **"j"** | processCommand(j) | 서보 55° | "RUN" | 컨베이어 역방향 |
| **"k"** | processCommand(k) | 서보 90° | "STOP" | 컨베이어 정지 |
| **"MOVE"** | seqStep = SEQ_FORWARD_4M | 자동 시작 | "RUN" | Mode 1 시작 |
| **"2"** | targetMode = 20 | 스테이션 순환 | "RUN" | Mode 2 순환 |
| **"3"** | targetMode = 30 | 스테이션 1회 | "RUN" | Mode 3 1회 후 정지 |
| **"4"** | EStop | 즉시 정지 | "ESTOP" | 긴급 정지 |
| **"5"** | Resume | 상태 복원 | "RUN" | 재개 |
| **"c"** | Oscillation ON | 좌우 진동 | "CHECK" | 센서 테스트 |

---

## 🔐 **Float 데이터 변환 (IEEE-754)**

### POS_X 예시
```
실제 값: 1.234m

ESP32에서:
  1. posX = 1234.0 (mm 단위)
  2. x_m = posX / 1000.0 = 1.234f (m 단위)
  3. floatToRegistersSafe(x_m) 호출
  
Float 비트 표현:
  1.234f = 0x3F9D70A4 (IEEE-754)
  
레지스터 저장:
  High Word (MSW): 0x3F9D = holdingRegisters[0]
  Low Word (LSW):  0x70A4 = holdingRegisters[1]

OPC에서 읽기:
  1. 레지스터 [0-1] 읽음: 0x3F9D70A4
  2. Float 변환: 1.234
  3. POS_X 태그 = 1.234m ✓
```

### Word Order
```cpp
// Modbus 표준: HIGH_FIRST (Big Endian)
const WordOrder modbusWordOrder = WORD_ORDER_HIGH_FIRST;

dest[0] = high;   // 하위 주소에 상위 워드
dest[1] = low;    // 상위 주소에 하위 워드
```

---

## ⏱️ **주기 및 동기화**

### 업데이트 주기
```
ESP32 루프:
  100ms마다:
    ├─ 엔코더 읽기
    ├─ 위치 계산
    ├─ Modbus 레지스터 갱신
    └─ State 업데이트
    
OPC 서버 (KEPServerEX):
  설정에 따라 (일반적으로 100-500ms):
    ├─ 위치 데이터 읽기
    ├─ 상태 확인
    └─ 명령 전송
```

### 타이밍 다이어그램
```
시간 →

ESP32 루프:
|←─100ms─→|←─100ms─→|←─100ms─→|
└─ 오도메트리 │  레지스터 │  상태 업데이트
  계산    │  갱신   │

OPC 읽기:
|←─200ms─→|←─200ms─→|
└─ 위치 읽기     │  명령 상태 확인

수신:
|←─50ms─→|
└─ Modbus 명령 처리 (거의 실시간)
```

---

## 🔌 **Modbus TCP 함수 코드 상세**

### Function Code 0x03: Read Holding Registers
```
요청 (12 바이트):
  [0-1]: Transaction ID
  [2-3]: Protocol ID (0x0000 = Modbus)
  [4-5]: Length (6 바이트)
  [6]:   Unit ID
  [7]:   Function Code (0x03)
  [8-9]: Starting Address (예: 0)
  [10-11]: Quantity (예: 2)

응답:
  [0-8]: MBAP 헤더 + Function Code
  [9]: Byte Count (4)
  [10-13]: Register Value[0] & [1]

예시 - POS_X 읽기:
  요청: Address=0, Quantity=2 (레지스터 [0-1])
  응답: 0x3F (고바이트) + 0x9D (저바이트) + 0x70 + 0xA4
```

### Function Code 0x06: Write Single Register
```
요청:
  [8-9]: Register Address
  [10-11]: Register Value

예시 - TargetA 설정:
  Address: 6, Value: 1 (ON) or 0 (OFF)
```

### Function Code 0x10: Write Multiple Registers
```
요청:
  [8-9]: Starting Address
  [10-11]: Quantity
  [12]: Byte Count
  [13+]: Register Data

예시 - Control 문자열 쓰기:
  Address: 99 (Control 시작)
  Quantity: 10 (20바이트)
  Data: "MOVE" (+ 패딩)
```

---

## 🧪 **실제 통신 시나리오**

### 시나리오 1: Auto Mode 2 실행
```
시간  OPC UA                ESP32               Mega
─────────────────────────────────────────────────────
0ms   Control = "2" 쓰기    
      ↓ Modbus Write
      
10ms                        수신 + 파싱
                            targetMode = 20
                            State = "RUN" ← 레지스터 갱신
                            
100ms                       오도메트리 계산
                            2.5m 진행 감시...
                            
200ms OPC: POS_X = 0.25m ← 위치 읽기
      OPC: State = "RUN"
      
2600ms  (2.5m 도달)         
                            컨베이어 명령 → 'h'
                            ↓ SPI
                                              서보 125° 작동
                            State = "처리중"
                            
5600ms  (3초 후)            
                            컨베이어 정지 → 'k'
                            State = "역사탈출"
                            
6100ms  (500mm 탈출)        
                            다시 탐색 모드
                            State = "검색중"
                            
OPC 읽기:
  POS_X = 최종 좌표
  POS_Y = 최종 좌표
  State = "검색중"
```

### 시나리오 2: 긴급 정지
```
진행 중...

시간: 4000ms
OPC: Control = "4" (EStop) 쓰기
     ↓ Modbus Write (즉시)
     
ESP32: isEStopActive = true
       상태 저장 (targetMode, 위치 등)
       State = "ESTOP"
       모터 정지
       ↓ SPI: 'x' 명령
       
Mega: 모터 OFF
      
OPC 읽기:
  State = "ESTOP" ← 확인

(문제 해결)

OPC: Control = "5" (Resume) 쓰기
     
ESP32: 저장된 상태 복원
       State = "RUN"
       계속 실행
```

---

## 📈 **성능 지표**

| 항목 | 값 | 비고 |
|------|-----|------|
| Modbus TCP 레이턴시 | ~50ms | 로컬 네트워크 |
| 위치 업데이트 주기 | 100ms | ESP32 loop() |
| OPC 읽기 주기 | 100-500ms | KEP 설정값 |
| 명령 응답 시간 | <100ms | SPI 포함 |
| Float 정확도 | ±0.001m | IEEE-754 32bit |

---

## ✅ **통신 확인 방법**

### 1. Modbus 디버거로 확인
```
Tool: Modbus TCP Client (예: QModMaster)

연결:
  Host: ESP32 IP (예: 192.168.1.100)
  Port: 502
  
읽기:
  Function: 03 (Read Holding Registers)
  Address: 0
  Quantity: 6
  
결과:
  [0-1]: POS_X
  [2-3]: POS_Y
  [4-5]: POS_T
```

### 2. ESP32 Serial Monitor
```
[MODBUS] Read Regs: Addr=0, Qty=6
[MODBUS] Write Regs: Addr=99, Qty=10
[MODE 2] Step 0: Forward (seeking station)
```

### 3. KEPServerEX OPC 탐색기
```
탐색기에서:
  └─ ESP32
     ├─ POS_X (값 읽기 가능)
     ├─ POS_Y
     ├─ POS_T
     ├─ TargetA (쓰기 가능)
     ├─ Control (쓰기 가능)
     └─ State (읽기 가능)
```

---

## 📚 **요약**

**OPC → ESP32 경로**:
1. KEPServerEX에서 태그 쓰기 (Control)
2. Modbus TCP로 전송 (포트 502)
3. ESP32: processModbusRequest() 처리
4. holdingRegisters[] 업데이트
5. 명령 파싱 → SPI로 Mega 제어

**ESP32 → OPC 경로**:
1. ESP32: 엔코더로 위치 계산
2. updateHoldingRegisters()로 레지스터 갱신
3. OPC가 Modbus Read 요청
4. holdingRegisters[] 반환
5. KEPServerEX 태그 업데이트

**핵심**: Modbus TCP를 통한 공유 메모리 구조로 OPC와 ESP32가 실시간 통신 ✅
