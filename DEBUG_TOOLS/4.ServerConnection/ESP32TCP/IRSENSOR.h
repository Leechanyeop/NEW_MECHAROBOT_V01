#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

class IRSensor {
private:
    int pins[6];   // IR 센서가 연결된 핀 번호 저장
public:
    IRSensor(int p0, int p1, int p2, int p3, int p4, int p5);
    void readValues(int values[6]);  // 6개 센서 값 읽기
};

#endif