#include "LINETRACE.h"

LineTracer::LineTracer(MotorDrive& motor) : _motor(motor) {
    // PID 제어 상수 초기화
    _kp = 0.03; 
    _ki = 0.0; 
    _kd = 0.5; 
    _baseSpeed = DEFAULT_SPEED; 
}

void LineTracer::begin(const uint8_t pins[]) {
    // 5개 핀을 입력으로 설정
    for(int i = 0; i < _sensorCount; i++) {
        _sensorPins[i] = pins[i];
        pinMode(_sensorPins[i], INPUT);
    }
}

void LineTracer::runPID() {
    // ========== 위치 계산 (Weighted Average) ==========
    long weightedSum = 0;
    int activeSensors = 0;
    
    // 5개 센서 읽기 (검은색=1, 흰색=0 가정)
    // 일반적으로 TCRT5000 모듈은 검은색일 때 HIGH(1)가 나옵니다.
    // 만약 반대라면 !digitalRead(...) 로 수정해야 합니다.
    
    for(int i = 0; i < _sensorCount; i++) {
        int val = digitalRead(_sensorPins[i]);
        
        if(val == HIGH) { // 검은색 감지
            weightedSum += i * 1000;
            activeSensors++;
        }
    }
    
    // 1. 라인 이탈 (모두 흰색) -> 정지
    if (activeSensors == 0) {
        _motor.stop();
        return;
    }
    
    // 2. 공중 부양 / 교차로 (모두 검은색) -> 정지
    if (activeSensors == 5) {
        _motor.stop();
        return;
    }
    
    // 위치값 계산 (0 ~ 4000)
    int16_t position = weightedSum / activeSensors;
    
    // PID 오차 (중앙 2000 기준)
    int16_t error = 2000 - position;
    
    _integral += error;
    int16_t derivative = error - _lastError;
    
    int16_t correction = (int16_t)(_kp * (float)error + _ki * _integral + _kd * (float)derivative);
    _lastError = error;
    
    int leftSpeed = _baseSpeed - correction;
    int rightSpeed = _baseSpeed + correction;
    
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
    
    _motor.setMotorSpeeds(leftSpeed, rightSpeed);
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

void LineTracer::printSensorValues() {
    // 단순 디지털 값 출력 (0 or 1)
    long weightedSum = 0;
    int activeSensors = 0;
    
    Serial.print("Sensors: ");
    for (uint8_t i = 0; i < _sensorCount; i++) {
        int val = digitalRead(_sensorPins[i]);
        
        // 보기 좋게 출력 (검은색 1, 흰색 0)
        Serial.print(val);
        Serial.print(" ");
        
        if(val == HIGH) {
            weightedSum += i * 1000;
            activeSensors++;
        }
    }
    
    int position = -1;
    if(activeSensors > 0) position = weightedSum / activeSensors;
    
    Serial.print(" | Pos: ");
    Serial.println(position);
}
