#include "LINETRACE.h"

LineTracer::LineTracer(MotorDrive& motor) : _motor(motor) {
    _kp = 0.1; // Initial PID values, adjust as needed
    _ki = 0.0;
    _kd = 0.5;
    _lastError = 0;
    _integral = 0;
    _baseSpeed = DEFAULT_SPEED;
}

void LineTracer::begin(const uint8_t pins[]) {
    _qtr.setTypeRC();
    _qtr.setSensorPins(pins, _sensorCount);
    _qtr.setEmitterPin(2); // As per user example
}

void LineTracer::calibrate() {
    Serial.println(F("[LineTracer] Setting Manual Calibration..."));
    
    // 1. 메모리 할당을 위해 한번은 호출해야 함 (더미 호출)
    _qtr.calibrate(); 

    // 2. 수동 값 설정 (Min Values)
    // 사용자가 제공한 값: {1216, 964, 964, 916, 872, 872, 1056, 1308}
    uint16_t minValues[] = {1216, 964, 964, 916, 872, 872, 1056, 1308};
    
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (i < 8) {
             _qtr.calibrationOn.minimum[i] = minValues[i];
        } else {
             _qtr.calibrationOn.minimum[i] = 1000; // 예외 처리
        }
        _qtr.calibrationOn.maximum[i] = 2500; // Max 값은 모두 2500으로 고정
    }

    _motor.stop();
    Serial.println(F("[LineTracer] Manual Calibration Set."));
    
    // Print calibration results to verify
    Serial.println(F("Calibration Results (Manual):"));
    Serial.print(F("Min: "));
    for (uint8_t i = 0; i < _sensorCount; i++) {
        Serial.print(_qtr.calibrationOn.minimum[i]);
        Serial.print('\t');
    }
    Serial.println();
    
    Serial.print(F("Max: "));
    for (uint8_t i = 0; i < _sensorCount; i++) {
        Serial.print(_qtr.calibrationOn.maximum[i]);
        Serial.print('\t');
    }
    Serial.println();
}

void LineTracer::runPID() {
    // readLineBlack returns a value from 0 to 7000 (for 8 sensors)
    uint16_t position = _qtr.readLineBlack(_sensorValues);
    
    // Calculate error: 3500 is the center for 8 sensors (0-7000)
    // Calculate error: Center is (_sensorCount - 1) * 1000 / 2
    int16_t center = (_sensorCount - 1) * 500;
    int16_t error = (int16_t)center - (int16_t)position;
    
    _integral += error;
    int16_t derivative = error - _lastError;
    
    int16_t correction = (int16_t)(_kp * (float)error + _ki * _integral + _kd * (float)derivative);
    _lastError = error;
    
    int leftSpeed = _baseSpeed - correction;
    int rightSpeed = _baseSpeed + correction;
    
    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
    
    _motor.setMotorSpeeds(leftSpeed, rightSpeed);
}

void LineTracer::runBasic() {
    _qtr.read(_sensorValues);
    
    // Simple logic based on threshold
    // (This is a placeholder, usually PID is preferred for 8 sensors)
    uint16_t position = _qtr.readLineBlack(_sensorValues);
    if (position < 2500) {
        _motor.turnLeft(_baseSpeed);
    } else if (position > 4500) {
        _motor.turnRight(_baseSpeed);
    } else {
        _motor.forward(_baseSpeed);
    }
}

void LineTracer::stop() {
    _motor.stop();
}

void LineTracer::setPID(float p, float i, float d) {
    _kp = p; _ki = i; _kd = d;
}

void LineTracer::setBaseSpeed(int speed) {
    _baseSpeed = speed;
}
