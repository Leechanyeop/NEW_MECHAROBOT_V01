# OPC에서 Control="2"를 받았을 때 Mega2560의 처리 흐름

## 🔄 통신 경로
```
KEPware (OPC UA)
  ↓ Modbus TCP: Control 레지스터 (99-108) 에 "2" 쓰기
ESP32 (Modbus Server)
  ↓ SPI: '<' + '2' + '>' 형식으로 Mega에 전송
Mega2560 (SPI Slave)
  ↓ ISR에서 수신 → processCommand('2') 호출
```

---

## 📍 Step-by-Step 실행 흐름

### **1단계: 명령 수신 (SPI ISR)**

```cpp
// Mega2560이 ESP32로부터 SPI로 데이터 수신
// ISR (Interrupt Service Routine)

ISR(SPI_STC_vect) {
  char data = SPDR;  // SPI 데이터 레지스터 읽기
  
  if (data == '<') {
    // 명령 시작
    inCommand = true;
    byteCount = 0;
  } 
  else if (data == '>') {
    // 명령 종료
    inCommand = false;
    cmdReceived = true;    // ← 메인 루프가 이것을 감지
    lastPacketTime = millis();
  } 
  else if (inCommand && byteCount < sizeof(lastReceivedCmd)-1) {
    lastReceivedCmd[byteCount++] = data;
    lastReceivedCmd[byteCount] = '\0';  // Null 터미네이터
  }
}

// 예: ESP32가 보낸 것
ESP32 전송: '<' + '2' + '>'
Mega 수신: lastReceivedCmd[0] = '2'
```

### **2단계: 메인 루프에서 명령 처리**

```cpp
void loop() {
  // ... (센서 읽음, 시간 계산 등)
  
  // 명령 처리
  if (cmdReceived) {
    cmdReceived = false;
    char first = lastReceivedCmd[0];  // first = '2'
    
    // 이전 모드 전부 비활성화
    isLineTracing = false;
    isPIDTracing = false;
    isAutoMode = false;
    isCheckSensors = false;
    
    // Auto Mode 2 활성화
    if (first == '2') {
      isAutoMode2 = true;        // ✓ Auto Mode 2 플래그 ON
      isAutoMode3 = false;
      autoModeStep = 0;          // ✓ Step 0 (초기 상태)
      Serial.println(F("[MODE] Auto Station Mode (2) - Cycle"));
    }
  }
}
```

### **3단계: Auto Mode 2 실행 루프**

```cpp
void loop() {
  // ... (주기적으로 실행)
  
  if (isAutoMode2) {
    // 센서 상태 확인 (흰 바닥 감지)
    bool lineVisible = false;
    for (int i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 600) {  // QTR 센서값 > 600 = 흰 바닥
        lineVisible = true;
        break;
      }
    }
    
    // 상태 머신 시작
    if (autoModeStep == 0) {
      // Step 0: PID 주행 중 스테이션 감시
      ...
    }
    else if (autoModeStep == 1) {
      // Step 1: 컨베이어 작동 3초 대기
      ...
    }
    else if (autoModeStep == 2) {
      // Step 2: 역사 탈출
      ...
    }
  }
}
```

---

## 🎯 Auto Mode 2 상태 머신 상세

### **Step 0: 스테이션 탐색 (PID 주행)**

```
시작 조건: isAutoMode2=true, autoModeStep=0

동작:
  1. QTR 센서로 흰 바닥(스테이션) 감시
     - sensorValues[i] ≤ 600: 비블랙/흰색 바닥 감지
     - sensorValues[i] > 600: 검은색 바닥
  
  2. 흰 바닥이 감지될 때까지:
     - PID 제어로 직선 주행
     - error = position - 3500 (중앙선 유지)
     - motorSpeedChange = Kp * error + Kd * (error - lastError)
     - 좌측 모터: pwmValue - motorSpeedChange
     - 우측 모터: pwmValue + motorSpeedChange
  
  3. 흰 바닥 감지 시:
     ✓ motor.stop() → 로봇 정지
     ✓ autoModeStep = 1 → 다음 단계로
     ✓ processCommand('h') → 컨베이어 정방향 가동
     ✓ 시리얼 로그: "[AUTO2] Station Matched! Conveyor ON (3s)"

타이밍: 즉시 반응 (매 루프마다 센서 체크)
```

### **Step 1: 스테이션에서 작업 (3초 대기)**

```
진입 조건: Step 0에서 흰 바닥 감지

동작:
  1. 컨베이어 정방향 작동 (이미 processCommand('h')로 시작됨)
  2. autoModeTimer = now (진입 시간 기록)
  3. 3초 대기: now - autoModeTimer >= 3000
  4. 3초 경과 시:
     ✓ processCommand('k') → 컨베이어 정지
     ✓ motor.forward() → 역사 탈출 시작
     ✓ autoModeStep = 2 → 다음 단계로
     ✓ 시리얼 로그: "[AUTO2] Work Done. Clearing Station (1s)..."

타이밍: 
  t=0ms:    Step 1 진입
  t=0~3000ms: 컨베이어 작동
  t=3000ms: Step 2로 전환
```

### **Step 2: 역사 탈출 (100ms)**

```
진입 조건: Step 1에서 3초 경과

동작:
  1. motor.forward() 지속 (Step 1에서 이미 시작)
  2. autoModeTimer 초기화 (Step 1에서 재설정)
  3. 100ms 대기: now - autoModeTimer >= 100
  4. 100ms 경과 시:
     ✓ isSoftStop 확인:
       - isSoftStop이 FALSE (정상) → Step 0으로 복귀 (순환)
       - isSoftStop이 TRUE (정지 요청) → 모드 종료
     ✓ 시리얼 로그: "[AUTO2] Station Escape Done. Next Station Search..."

타이밍:
  t=3000ms: Step 2 진입, 전진 시작
  t=3100ms: Step 2 완료
  t=3100ms: Step 0으로 복귀 → 다음 스테이션 탐색 시작

무한 반복:
  Step 0 → Step 1 → Step 2 → Step 0 → ...
  (isAutoMode2=true가 유지되는 한)
```

---

## 📊 타이밍 다이어그램

```
t=0ms:      OPC: Control="2" 쓰기
            ↓
            ESP32: SPI로 '<2>' 전송
            ↓
            Mega: cmdReceived=true
            ↓
            isAutoMode2=true, autoModeStep=0

t=0~Ts:     Step 0: PID 주행 (스테이션 탐색)
            센서 읽음 → 흰 바닥 감지까지 반복
            
t=Ts:       흰 바닥 감지!
            ↓
            motor.stop()
            processCommand('h') ← 컨베이어 ON
            autoModeStep=1

t=Ts~Ts+3s: Step 1: 컨베이어 작동 (3초)
            
t=Ts+3s:    3초 경과!
            ↓
            processCommand('k') ← 컨베이어 OFF
            motor.forward() ← 탈출 시작
            autoModeStep=2

t=Ts+3s~Ts+3.1s: Step 2: 역사 탈출 (100ms)

t=Ts+3.1s:  100ms 경과!
            ↓
            autoModeStep=0 ← Step 0으로 복귀
            
t=Ts+3.1s~Ts+3.1s+T2: Step 0: 다음 스테이션 탐색 (무한 반복)
```

---

## 🛑 Auto Mode 2 정지 방법

### **방법 1: Soft Stop (현재 작업 완료 후 정지)**
```cpp
isSoftStop = true;  // 이 플래그를 설정

→ 현재 작업(Step 1, 2) 완료
→ Step 2 마지막에서:
   if (isSoftStop) {
     motor.stop();
     isAutoMode2 = false;  // 모드 종료
     Serial.println("[AUTO2] Soft stop - Task complete");
   }
```

### **방법 2: 즉시 정지 (Control="x")**
```cpp
if (first == 'x') {
  motor.stop();
  isAutoMode2 = false;  // 모드 즉시 해제
  isAutoMode3 = false;
  isLineTracing = false;
  Serial.println(F("[STOP] All stop"));
}
```

### **방법 3: EStop (긴급 정지)**
```cpp
if (first == '4') {
  isEStop = true;
  motor.stop();
  
  // 상태 저장
  estopState.wasAutoMode2 = isAutoMode2;
  estopState.savedAutoModeStep = autoModeStep;
  
  // 모든 모드 비활성화
  isAutoMode2 = false;
  isAutoMode3 = false;
  isPIDTracing = false;
}
```

---

## 📋 관련 변수 정리

```cpp
// 플래그
isAutoMode2 = true;         // Mode 2 활성 여부
autoModeStep = 0/1/2;       // 현재 단계
isSoftStop = false;         // Soft stop 요청 플래그
isEStop = false;            // EStop 상태 플래그

// 타이머
autoModeTimer = now;        // Step 1/2에서 경과 시간 측정
now = millis();             // 현재 시간

// 센서
sensorValues[8];            // QTR 센서 값 (0~1023)
lineVisible;                // 흰 바닥 감지 여부

// PID 제어
Kp = 0.25;                  // 비례 상수
Kd = 2.0;                   // 미분 상수
error = position - 3500;    // 편차 (중앙선 = 3500)
motorSpeedChange;           // PID 계산 결과
pwmValue = 80;              // 기본 속도
maxPIDSpeed = 150;          // 최대 속도 제한
```

---

## 🔧 실제 동작 시나리오

### **시나리오 1: 정상 동작**
```
1. OPC: Control="2" 쓰기
   ↓
2. Mega Step 0: 검은 선 따라 전진 (1.5m 정도 주행)
   ↓
3. 흰 바닥(스테이션) 감지!
   ↓
4. Mega Step 1: 컨베이어 ON (3초)
   예: 물체 적재/제거
   ↓
5. 3초 경과 → Mega Step 2: 탈출 (100ms 전진)
   ↓
6. Mega Step 0으로 복귀: 다음 스테이션 탐색
   ↓
7. 반복... (isAutoMode2=true 유지)
```

### **시나리오 2: 2번 작업 후 정지**
```
1. OPC: Control="2" 쓰기 → 시작
2. 첫 번째 스테이션 작업 완료 (Step 2)
3. 다시 Step 0 → 두 번째 스테이션 탐색
4. 두 번째 스테이션 도착 → Step 1 (컨베이어 3초)
5. OPC: Control="x" 쓰기 (또는 isSoftStop=true)
6. Step 2 완료 후:
   if (isSoftStop) { isAutoMode2=false; }
   ↓
7. 정지
```

---

## 📊 시리얼 로그 예시

```
[MODE] Auto Station Mode (2) - Cycle
[AUTO2] Station Matched! Conveyor ON (3s)
[AUTO2] Work Done. Clearing Station (1s)...
[AUTO2] Station Escape Done. Next Station Search...
[AUTO2] Station Matched! Conveyor ON (3s)
[AUTO2] Work Done. Clearing Station (1s)...
[AUTO2] Station Escape Done. Next Station Search...
... (반복)
```

---

## ✅ 요약: Control="2" 전달 후 동작

| 항목 | 내용 |
|------|------|
| **명령 전달** | OPC → ESP32 (Modbus) → Mega (SPI) |
| **플래그 설정** | isAutoMode2=true, autoModeStep=0 |
| **Step 0** | QTR 센서로 흰 바닥(스테이션) 탐색, PID 주행 |
| **Step 1** | 스테이션 도착 후 컨베이어 3초 작동 |
| **Step 2** | 역사 탈출 (100ms) |
| **반복** | Step 0으로 돌아가 무한 순환 |
| **정지** | Control="x" 또는 isSoftStop=true로 해제 |
| **긴급정지** | Control="4" (EStop) |

