// ==========================================
// 4-Channel Encoder Test (Interactive Serial)
// Pins: CH1(2,3), CH2(4,5), CH3(6,7), CH4(9,10)
// ==========================================

// 엔코더 핀 정의 (A상, B상)
const int ENC1_A = 12; const int ENC1_B = 13;
const int ENC2_A = 4; const int ENC2_B = 5;
const int ENC3_A = 6; const int ENC3_B = 7;
const int ENC4_A = 9; const int ENC4_B = 1;

// 엔코더 카운트 변수
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

// 이전 상태 저장용
int lastA1, lastA2, lastA3, lastA4;

// 표시 모드 ('a': 전체, '1'~'4': 개별 채널)
char displayMode = 'a';

void printMenu() {
  Serial.println(F("\n--- Encoder Test Menu ---"));
  Serial.println(F("'a' : 모든 채널 보기 (기본)"));
  Serial.println(F("'1'~'4' : 해당 채널만 보기"));
  Serial.println(F("'z' : 모든 카운트 초기화 (Zero)"));
  Serial.println(F("'m' : 이 메뉴 다시 보기"));
  Serial.println(F("------------------------"));
}

void setup() {
  Serial.begin(9600);
  
  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP); pinMode(ENC4_B, INPUT_PULLUP);

  lastA1 = digitalRead(ENC1_A);
  lastA2 = digitalRead(ENC2_A);
  lastA3 = digitalRead(ENC3_A);
  lastA4 = digitalRead(ENC4_A);

  printMenu();
}

void loop() {
  // --- 1. 시리얼 입력 처리 (메뉴 선택) ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'a' || (cmd >= '1' && cmd <= '4')) {
      displayMode = cmd;
      Serial.print(F(">> Display Mode: ")); 
      if (displayMode == 'a') Serial.println(F("ALL"));
      else { Serial.print(F("CH")); Serial.println(displayMode); }
    } 
    else if (cmd == 'z') {
      count1 = count2 = count3 = count4 = 0;
      Serial.println(F(">> Counts Reset to 0"));
    }
    else if (cmd == 'm') {
      printMenu();
    }
  }

  // --- 2. 엔코더 폴링 (고속 읽기) ---
  int currA1 = digitalRead(ENC1_A);
  if (currA1 != lastA1) {
    if (digitalRead(ENC1_B) != currA1) count1++; else count1--;
  }
  lastA1 = currA1;

  int currA2 = digitalRead(ENC2_A);
  if (currA2 != lastA2) {
    if (digitalRead(ENC2_B) != currA2) count2++; else count2--;
  }
  lastA2 = currA2;

  int currA3 = digitalRead(ENC3_A);
  if (currA3 != lastA3) {
    if (digitalRead(ENC3_B) != currA3) count3++; else count3--;
  }
  lastA3 = currA3;

  int currA4 = digitalRead(ENC4_A);
  if (currA4 != lastA4) {
    if (digitalRead(ENC4_B) != currA4) count4++; else count4--;
  }
  lastA4 = currA4;

  // --- 3. 설정된 모드에 따라 출력 ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    
    switch (displayMode) {
      case '1': Serial.print(F("CH1: ")); Serial.println(count1); break;
      case '2': Serial.print(F("CH2: ")); Serial.println(count2); break;
      case '3': Serial.print(F("CH3: ")); Serial.println(count3); break;
      case '4': Serial.print(F("CH4: ")); Serial.println(count4); break;
      case 'a':
        Serial.print(F("C1:")); Serial.print(count1);
        Serial.print(F("\tC2:")); Serial.print(count2);
        Serial.print(F("\tC3:")); Serial.print(count3);
        Serial.print(F("\tC4:")); Serial.println(count4);
        break;
    }
  }
}
