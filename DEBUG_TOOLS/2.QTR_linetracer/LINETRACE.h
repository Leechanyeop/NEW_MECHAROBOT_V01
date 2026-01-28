#ifndef LINETRACE_H
#define LINETRACE_H

#include <QTRSensors.h>
#include "MotorDrive.h"

class LineTracer {
private:
    QTRSensors _qtr;
    static const uint8_t _sensorCount = 8;
    uint16_t _sensorValues[_sensorCount];
    
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
    void calibrate();
    void runPID();
    void runBasic();
    void stop();
    
    void setPID(float p, float i, float d);
    void setBaseSpeed(int speed);
    
    uint16_t* getSensorValues() { return _sensorValues; }
};

#endif
