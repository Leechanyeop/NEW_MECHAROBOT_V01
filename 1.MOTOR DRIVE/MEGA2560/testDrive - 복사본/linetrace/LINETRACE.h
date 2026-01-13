#ifndef LINETRACE_H
#define LINETRACE_H

#include "MotorDrive.h"

class LineTracer {
private:
    static const uint8_t _sensorCount = 5;
    uint8_t _sensorPins[_sensorCount]; // 센서 핀 번호 저장
    
    MotorDrive& _motor;
    
    // PID constants
    float _kp;
    float _ki;
    float _kd;
    int16_t _lastError;
    float _integral;
    
    int _baseSpeed;

public:
    LineTracer(MotorDrive& motor);
    
    void begin(const uint8_t pins[]);
    void runPID();
    void stop();
    
    void setPID(float p, float i, float d);
    void setBaseSpeed(int speed);
    
    void printSensorValues();
};

#endif
