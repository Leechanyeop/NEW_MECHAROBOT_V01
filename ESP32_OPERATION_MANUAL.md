# ESP32 ModbusTCP_NewTags_260121.ino 완벽 조작 가이드

## 📋 목차
1. [시스템 개요](#시스템-개요)
2. [하드웨어 연결](#하드웨어-연결)
3. [Modbus 레지스터 맵](#modbus-레지스터-맵)
4. [제어 명령 (Control Tag)](#제어-명령-control-tag)
5. [작동 모드](#작동-모드)
6. [상태 피드백 (State Tag)](#상태-피드백-state-tag)
7. [응답 메커니즘](#응답-메커니즘)
8. [실행 흐름](#실행-흐름)
9. [디버깅 팁](#디버깅-팁)

---

## 시스템 개요

**ESP32 역할:** Modbus TCP 서버
- **포트:** 502 (기본 Modbus 포트)
- **WiFi 연결:** SSID "hhme", 암호 "hme*12345"
- **주요 기능:**
  - 엔코더로부터 위치(X, Y, Theta) 계산
  - Modbus TCP를 통해 KEPware(OPC UA)와 통신
  - SPI를 통해 Mega2560 보드에 명령 전송
  - 자동 모드, 긴급 정지, 센서 체크 모드 지원

**통신 체인:**
```
KEPware (OPC UA Master)
   ↓↑ (Modbus TCP 502번 포트)
ESP32 (Modbus TCP Server)
   ↓↑ (SPI, GPIO 5 CS)
Mega2560 (모터/센서 제어)
```

---

## 하드웨어 연결

### 엔코더 (GPIO 인터럽트)
```
Encoder 1:
  - GPIO 36 (enc1A) → ISR: isr1A()
  - GPIO 39 (enc1B)
  결과: encoder1Count 증가/감소

Encoder 2:
  - GPIO 34 (enc2A) → ISR: isr2A()
  - GPIO 35 (enc2B)
  결과: encoder2Count 증가/감소
```

### SPI (Mega2560과의 통신)
```
ESP32 SPI:
  - GPIO 5: CS (Chip Select)
  - GPIO 18: CLK
  - GPIO 19: MOSI
  - GPIO 23: MOSI (Backup)
  
통신 프로토콜: 1MHz, MSBFIRST, SPI_MODE0
명령 형식: '<' + cmd + '>' 
예: '<w>' → 전진
```

---

## Modbus 레지스터 맵

### 읽기 전용 (ESP32 → OPC)
| 주소 | 레지스터 | 타입 | 설명 | 값 범위 |
|------|---------|------|------|--------|
| 40001-40002 | 0-1 | Float | POS_X (미터) | -∞ ~ +∞ |
| 40003-40004 | 2-3 | Float | POS_Y (미터) | -∞ ~ +∞ |
| 40005-40006 | 4-5 | Float | POS_T (도) | -180 ~ +180 |
| 40200-40209 | 199-208 | String | State (상태) | "IDLE", "RUN", "ESTOP" 등 |

### 읽기/쓰기 (양방향)
| 주소 | 레지스터 | 타입 | 설명 | 사용법 |
|------|---------|------|------|--------|
| **00007** (Coil) | 6 (bit0) | Boolean | TargetA (스테이션 선택) | FC 0x01(읽기), FC 0x05(쓰기) |
| 40100-40109 | 99-108 | String | Control (제어 명령) | FC 0x10(쓰기), FC 0x03(읽기) |

### 상세 설명

#### TargetA (Coil 00007 / Register 6)
```
쓰기: KEPware → FC 0x05 (Write Single Coil)
읽기: KEPware ← FC 0x01 (Read Coils)

값:
  0xFF00 = TRUE (1) → 스테이션 B 또는 END 포인트
  0x0000 = FALSE (0) → 스테이션 A 또는 START 포인트

응답 형식 (FC 0x01):
  [MBAP Header: 6 bytes]
  [Function Code: 0x01]
  [Byte Count: 1]
  [Coil Value: 0x01 (ON) 또는 0x00 (OFF)]
```

#### Control (Register 99-108 / 40100-40109)
```
쓰기: KEPware → FC 0x10 (Write Multiple Registers)
읽기: KEPware ← FC 0x03 (Read Holding Registers)

형식: 20바이트 문자열
바이트 순서: HiLo (상위 바이트가 먼저)

저장 방식:
  Register 99  = bytes[0-1]: 'h' + 'j' (두 문자)
  Register 100 = bytes[2-3]: 'k' + ...
  ...
```

#### State (Register 199-208 / 40200-40209)
```
읽기만 가능 (OPC ← ESP32)

상태 값 (예시):
  "IDLE"   → 대기 중
  "RUN"    → 실행 중
  "ESTOP"  → 긴급 정지 상태
  "CHECK"  → 센서 체크 모드
  "STOP"   → 정지됨
```

---

## 제어 명령 (Control Tag)

### 명령 형식
```
Control 태그에 다음 값을 쓰면 ESP32가 해당 명령 실행:

"x"    → 정지 (Stop)
"w"    → 전진 (Forward)
"s"    → 후진 (Backward)
"a"    → 좌회전 (Turn Left)
"d"    → 우회전 (Turn Right)

"2"    → Auto Mode 2 (스테이션 순환)
"3"    → Auto Mode 3 (스테이션 1회 후 정지)

"4"    → 긴급 정지 (EStop)
"5"    → 긴급 정지 해제 (Resume)

"c"    → 센서 체크 모드 (진동 테스트)

"h"    → 컨베이어 정방향
"j"    → 컨베이어 역방향
"k"    → 컨베이어 정지

"MOVE" → 시퀀스 시작 (4m 전진 + 360도 회전)
```

### 처리 흐름
```
loop() 함수:
  ↓
Control 문자열 읽기 (getControlString())
  ↓
이전 Control 값과 비교 (lastControlManual)
  ↓
변화 감지 시 해당 처리:
  ├─ "x" → sendCommand('x') + State = "STOP"
  ├─ "2" → targetMode=20, State = "RUN"
  ├─ "3" → targetMode=30, State = "RUN"
  ├─ "4" → isEStopActive=true, State = "ESTOP"
  ├─ "5" → isEStopActive=false, 이전 상태 복원
  ├─ "c" → isSensorCheckMode 토글
  └─ ...
```

---

## 작동 모드

### 1️⃣ **Manual Control (수동 조작)**
```
명령: 단일 문자 ('w', 's', 'a', 'd' 등)

처리:
  Control = "w" → sendCommand('w') → Mega가 전진
  
흐름:
  ESP32: Control 감지
    ↓
  sendCommand('w') via SPI
    ↓
  Mega: '<w>' 수신 → 전진 시작
    ↓
  ESP32: 엔코더로 위치 추적
    ↓
  ESP32: Position 값 주기적으로 업데이트 (100ms마다)
    ↓
  OPC: Modbus FC 0x03으로 위치 읽음
```

### 2️⃣ **Auto Mode 2: 스테이션 순환 (targetMode = 20)**
```
명령: Control = "2" 쓰기

시퀀스:
  Step 0 (위치 리셋):
    basePosX/Y = 현재 위치
    sendCommand('w') → 전진 시작
    ↓
  Step 0 (거리 감시):
    거리 계산: sqrt((X-basePosX)² + (Y-basePosY)²)
    거리 ≥ 2500mm (2.5m) 도달 시:
      ↓
  Step 1 (스테이션 도착):
    sendCommand('x') → 정지
    processCommand('h') → 컨베이어 정방향 (서보 동작)
    targetStartTime = 현재 시간
    ↓
  Step 1 (작업 대기):
    3초 경과 시:
      processCommand('k') → 컨베이어 정지
      basePosX/Y 재설정 (탈출 거리 측정용)
      sendCommand('w') → 전진 (탈출)
      ↓
  Step 2 (탈출 거리 감시):
    거리 ≥ 500mm 도달 시:
      sendCommand('x') → 정지
      targetStep = 0 → Step 0으로 돌아가기 (순환)

무한 반복 (targetMode=20이 유지되는 한)
```

**타이밍:**
```
t=0ms:      Control="2" 쓰임
t=0:        Step0 시작, Forward
t=0~2500mm: 거리 누적
t=2500mm:   Step1 → 컨베이어 ON
t=2500~5500ms: 컨베이어 작동 (3초)
t=5500ms:   Step2 → 탈출 시작
t=5500~6000mm: 탈출 (500mm)
t=6000mm:   Step0 → 다음 스테이션 탐색
```

### 3️⃣ **Auto Mode 3: 스테이션 1회 (targetMode = 30)**
```
명령: Control = "3" 쓰기

시퀀스: Mode 2와 동일하나 마지막 단계가 다름
  Step 0: 2.5m 주행
  Step 1: 컨베이어 3초 작동
  Step 1 종료 후: targetMode=0으로 설정 → 작동 종료

특징: 한 번의 작업만 수행 후 정지
```

### 4️⃣ **Sensor Check Mode (센서 진동 테스트)**
```
명령: Control = "c" 쓰기

동작:
  isSensorCheckMode = true
  oscillateTimer = 현재 시간
  oscillateStep = 0 (좌측부터 시작)
  
반복:
  500ms 마다:
    Step 0: sendCommand('a') → 좌회전
    Step 1: sendCommand('d') → 우회전
    
로그:
  "[CHECK] Left oscillation"
  "[CHECK] Right oscillation"

해제:
  다시 Control = "c" → isSensorCheckMode = false
```

### 5️⃣ **EStop (긴급 정지) & Resume**
```
명령:
  Control = "4" → 긴급 정지
  Control = "5" → 재개

EStop 처리:
  isEStopActive = true
  
  상태 저장:
    savedTargetMode = 현재 모드
    savedTargetStep = 현재 스텝
    savedTravelDist = 누적 거리
    savedPosX/Y/Theta = 위치 저장
  
  sendCommand('x') → Mega에 정지 명령
  State = "ESTOP"
  
  모든 자동 모드 중단
  수동 명령도 무시

Resume 처리:
  Control = "5" 수신 시:
    isEStopActive = false
    targetMode = savedTargetMode (이전 모드 복원)
    targetStep = savedTargetStep
    travelDist = savedTravelDist
    State = "RUN"
    
    이전 상태에서 계속 실행
```

---

## 상태 피드백 (State Tag)

### State 값의 의미

| State 값 | 의미 | 언제 설정되나 |
|---------|------|-------------|
| "IDLE" | 유휴 상태 | 시스템 시작, 작업 완료 |
| "RUN" | 실행 중 | Auto Mode, Manual 명령 후 |
| "STOP" | 정지됨 | Control="x" 수신 후 |
| "ESTOP" | 긴급 정지 | Control="4" 수신 후 |
| "CHECK" | 센서 체크 중 | Control="c" 수신 후 |

### State 읽기
```
OPC (KEPware):
  Modbus FC 0x03 요청 → Register 199-208 읽음
  ↓
ESP32:
  registersToString(&holdingRegisters[REG_STATE], ...)
  → 20바이트 문자열 반환
  ↓
OPC:
  State 값 표시
```

---

## 응답 메커니즘

### 위치 데이터 (Position) 업데이트

**주기:** 100ms마다 (updateInterval)

```cpp
if (now - lastUpdateTime >= updateInterval) {
    lastUpdateTime = now;
    
    // 1. 엔코더 값 읽기
    updateOdometry();
    
    // 2. Float → Modbus Register로 변환
    floatToRegistersSafe(posX_m, &holdingRegisters[REG_POS_X_LO]);
    floatToRegistersSafe(posY_m, &holdingRegisters[REG_POS_Y_LO]);
    floatToRegistersSafe(posTheta_deg, &holdingRegisters[REG_POS_T_LO]);
    
    // 3. OPC가 FC 0x03으로 읽을 수 있는 상태 준비 완료
}
```

**변환 공식 (IEEE-754):**
```
Float → 4 바이트 (32-bit)
  Byte 0: LSB of Low Word
  Byte 1: MSB of Low Word
  Byte 2: LSB of High Word
  Byte 3: MSB of High Word
  
Register에 저장:
  Register[0] = (Byte1 << 8) | Byte0   (Low Word)
  Register[1] = (Byte3 << 8) | Byte2   (High Word)
  
KEPware 바이트 순서: Intel (Byte Order=Disable)
```

### 위치 계산 (Odometry)

```cpp
void updateOdometry() {
  // 1. 엔코더 델타 계산
  long dE1 = encoder1Count - lastEnc1;  // 모터 1 펄스
  long dE2 = encoder2Count - lastEnc2;  // 모터 2 펄스
  
  // 2. 펄스 → 거리 (mm)
  TICKS_PER_REV = 11 CPR × 2 × 30 기어비 = 660 펄스/회전
  WHEEL_DIAMETER = 65mm
  
  dD1 = (dE1 / 660) × π × 65   // 모터 1 주행 거리
  dD2 = (dE2 / 660) × π × 65   // 모터 2 주행 거리
  
  // 3. 차동 주행 공식
  dS = (dD1 + dD2) / 2      // 평균 거리
  dT = (dD2 - dD1) / 160mm  // 회전각 (휠 베이스 = 160mm)
  
  // 4. 위치 업데이트 (항법 좌표계)
  posX += dS × cos(posTheta + dT/2)
  posY += dS × sin(posTheta + dT/2)
  posTheta += dT
}
```

---

## 실행 흐름

### Setup 함수
```
1. Serial 시작 (115200 baud)
2. 엔코더 GPIO 설정 + ISR 연결
3. SPI 초기화 (1MHz)
4. WiFi 연결 시도
   → "hhme" SSID 연결
   → 192.168.x.x IP 할당
5. Modbus TCP 서버 시작 (포트 502)
6. 초기 State = "IDLE"
7. 태그 맵 정보 시리얼 출력
```

### Loop 함수 (반복)
```
1. WiFi 상태 확인 (연결 끊김 시 재연결 시도)

2. Modbus TCP 클라이언트 수락
   newClient = modbusServer.available()
   
3. Modbus 요청 처리 (100ms마다)
   if (processModbusRequest()) 호출
   
4. 오도메트리 계산 (100ms마다)
   updateOdometry()
   floatToRegisters()
   
5. Control/State 문자열 읽기
   String control = getControlString()
   
6. 모드별 로직 처리
   ├─ Manual Control
   ├─ Sequence Mode (MOVE)
   ├─ Auto Mode 2 (targetMode=20)
   ├─ Auto Mode 3 (targetMode=30)
   └─ EStop/Resume
   
7. Sensor Check Mode 처리
   500ms마다 좌/우 진동
   
8. 상태 로그 출력 (매 100ms)
   [X, Y, Theta, TargetA, Control, State]

delay(1);  // 1ms 대기
```

---

## Modbus 프로토콜

### Function Code 지원

| FC | 이름 | 목적 | 처리 |
|----|------|------|------|
| **0x01** | Read Coils | Boolean 읽기 | TargetA (Coil 00007) |
| 0x03 | Read Holding Registers | 레지스터 읽기 | Position, State, Control |
| 0x05 | Write Single Coil | Boolean 쓰기 | TargetA (Coil 00007) |
| 0x06 | Write Single Register | 레지스터 쓰기 | 미사용 |
| **0x10** | Write Multiple Registers | 다중 레지스터 쓰기 | Control (99-108) |

### 요청/응답 형식

#### FC 0x01 (Read Coils) - TargetA 읽기
```
요청:
  [MBAP Header]
  [FC: 0x01]
  [Starting Address: 00007]
  [Quantity: 0001]

응답:
  [MBAP Header]
  [FC: 0x01]
  [Byte Count: 1]
  [Coil Value: 0x01 (ON) / 0x00 (OFF)]
```

#### FC 0x03 (Read Registers) - Position 읽기
```
요청:
  [MBAP Header]
  [FC: 0x03]
  [Starting Address: 0000]    // Register 0 (POS_X_LO)
  [Quantity: 6]               // 6 registers (POS_X, Y, T)

응답:
  [MBAP Header]
  [FC: 0x03]
  [Byte Count: 12]
  [Value 0: POS_X_LO >> 8, POS_X_LO & 0xFF]
  [Value 1: POS_X_HI >> 8, POS_X_HI & 0xFF]
  ...
  [Value 5: POS_T_HI >> 8, POS_T_HI & 0xFF]
```

#### FC 0x05 (Write Single Coil) - TargetA 쓰기
```
요청:
  [MBAP Header]
  [FC: 0x05]
  [Coil Address: 00007]
  [Coil Value: 0xFF00 (ON) / 0x0000 (OFF)]

응답:
  [요청과 동일 반환]
```

#### FC 0x10 (Write Multiple Registers) - Control 쓰기
```
요청:
  [MBAP Header]
  [FC: 0x10]
  [Starting Address: 99]      // Register 99 (Control)
  [Quantity: 10]              // 10 registers (20 bytes)
  [Byte Count: 20]
  [Data: "2" 등 명령]

응답:
  [MBAP Header]
  [FC: 0x10]
  [Starting Address: 99]
  [Quantity: 10]
```

---

## 디버깅 팁

### 시리얼 모니터 로그 읽기

**WiFi 연결:**
```
WiFi Connecting...
WiFi Connected!
ESP32 IP Address: 192.168.1.123
```

**Modbus 요청:**
```
[MODBUS] FC 0x03 Request: Addr=0, Qty=6     ← Position 읽음
[MODBUS] FC 0x03 Response: 6 registers sent ← 응답 완료
```

**제어 명령:**
```
[MODBUS] Write Regs: Addr=99, Qty=10   ← Control 쓰임
[MODBUS] Control written: '2'           ← 모드 2 시작
```

**위치 정보:**
```
Pose[X:1.234 Y:5.678 T:45.2] TargetA:0 Ctrl:'2' State:'RUN'
```

### 일반적인 문제

#### 1. "FC 0x01 요청이 안 옴"
```
원인: KEPware가 FC 0x03으로 읽으려고 함
해결: KEPware 설정에서 TargetA를 다음으로 변경:
     - 데이터 타입: UInt16 (또는 Boolean)
     - Function Code: 1 (Auto 아님)
```

#### 2. "TargetA 값이 항상 0"
```
원인: Write는 되지만 Read가 안 됨 (FC 0x01 미지원)
해결: ESP32 코드에서 FC 0x01 처리 추가 ✓ (이미 수정됨)
```

#### 3. "위치 값이 이상함"
```
원인: 
  - 엔코더 카운트 오류
  - 바퀴 지름 상수 오류
  - 기어비 값 틀림

확인:
  WHEEL_DIAMETER_MM = 65.0
  GEAR_RATIO = 30.0
  ENCODER_CPR = 11
  
로그:
  시리얼에서 Pose 값 확인
```

#### 4. "Mega가 명령을 받지 못함"
```
원인: SPI 통신 오류
확인:
  - GPIO 5 (CS) 연결 확인
  - SPI 케이블 양쪽 모두 연결
  
로그:
  "[SPI] Sent CMD: w" 확인
```

### 유용한 명령

**전체 상태 확인:**
```
Control = "x"  → 정지 + 상태 초기화
```

**시퀀스 테스트:**
```
Control = "2"  → Auto Mode 2 시작
Control = "x"  → 정지 (중간에 멈추기)
```

**센서 체크:**
```
Control = "c"  → 진동 시작
시리얼에서 "[CHECK] Left/Right oscillation" 로그 확인
Control = "c"  → 진동 정지
```

---

## 요약: 최소 실행 단계

```
1. ESP32 업로드 및 부팅
   → 시리얼 모니터: "WiFi Connected!" 확인

2. KEPware에서 다음 연결 설정:
   - IP: ESP32 IP (예: 192.168.1.123)
   - Port: 502
   - Protocol: Modbus TCP

3. 다음 태그 추가:
   TargetA  (Coil 00007, FC 1/5)
   POS_X    (Register 40001-40002, Float)
   POS_Y    (Register 40003-40004, Float)
   POS_T    (Register 40005-40006, Float)
   Control  (Register 40100-40109, String)
   State    (Register 40200-40209, String)

4. 제어:
   Control = "2" 쓰기 → Auto Mode 2 시작
   시리얼 로그: Position 업데이트 확인
   
5. 상태 확인:
   State 읽기 → "RUN" 확인
   Position 읽기 → 실시간 위치 확인
```

---

## 참고 자료

- **IEEE-754 Float 형식:** 엔코더 데이터 → Float 변환 참조
- **Modbus TCP 표준:** MB_TCP.pdf
- **SPI 프로토콜:** Mega2560 SPI Slave 코드와 비교
- **차동 주행 모델:** Differential Drive Kinematics

