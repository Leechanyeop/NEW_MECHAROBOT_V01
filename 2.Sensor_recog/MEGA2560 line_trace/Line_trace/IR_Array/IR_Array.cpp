#include "IR_Array.h"
#include "Arduino.h"

IR_Array::IR_Array(int p0, int p1, int p2, int p3, int p4) {
    pins[0] = p0;
    pins[1] = p1;
    pins[2] = p2;
    pins[3] = p3;
    pins[4] = p4;

    // 핀 모드 설정
    for (int i = 0; i < 5; i++) {
        pinMode(pins[i], INPUT);
        Serial.println("linetrace_ready");
    }
}
    void IR_Array::readValues(int values[5]) {
    for (int i = 0; i < 5; i++) {
        values[i] = analogRead(pins[i]);
    }


}