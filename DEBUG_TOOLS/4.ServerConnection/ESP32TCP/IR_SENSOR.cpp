#include "IRSensor.h"

IRSensor::IRSensor(int p0, int p1, int p2, int p3, int p4, int p5) {
    pins[0] = p0;
    pins[1] = p1;
    pins[2] = p2;
    pins[3] = p3;
    pins[4] = p4;
    pins[5] = p5;

    // �� ��� ����
    for (int i = 0; i < 6; i++) {
        pinMode(pins[i], INPUT);
    }
}

void IRSensor::readValues(int values[6]) {
    for (int i = 0; i < 6; i++) {
        values[i] = analogRead(pins[i]);  // �� ���� �� �б�
    }
}
