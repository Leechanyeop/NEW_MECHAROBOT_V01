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
    Serial.println(F("[LineTracer] Calibrating sensors..."));
    for (uint16_t i = 0; i < 400; i++) {
        _qtr.calibrate();
        
        // Blink LED or move slowly during calibration if desired
        if (i % 20 == 0) {
            if (i % 40 == 0) _motor.turnLeft(100);
            else _motor.turnRight(100);
        }
    }
    _motor.stop();
    Serial.println(F("[LineTracer] Calibration done."));
}

void LineTracer::runPID() {
    // readLineBlack returns a value from 0 to 7000 (for 8 sensors)
    uint16_t position = _qtr.readLineBlack(_sensorValues);
    
    // Calculate error: 3500 is the center for 8 sensors (0-7000)
    int16_t error = (int16_t)3500 - (int16_t)position;
    
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
