#include "LINETRACE.h"

LineTracer::LineTracer(MotorDrive& motor) : _motor(motor) {
    // PID 제어 상수 초기화
    _kp = 0.1; // 비례항 (반응 속도)
    _ki = 0.0; // 적분항 (누적 오차 보정)
    _kd = 0.5; // 미분항 (급격한 변화 억제)
    _baseSpeed = DEFAULT_SPEED; // 기본 주행 속도
}

void LineTracer::begin(const uint8_t pins[]) {
    _qtr.setTypeRC(); // QTR-8RC (Digital) 타입 설정
    _qtr.setSensorPins(pins, _sensorCount);
    // [중요 수정사항] 2번 핀 충돌 방지
    // 기존: _qtr.setEmitterPin(2); -> Motor1 LPWM과 충돌
    _qtr.setEmitterPin(255); //-> Emitter 제어 끔 (항상 켜짐으로 가정)
}

void LineTracer::calibrate() {
    Serial.println(F("[LineTracer] Calibrating sensors..."));
    
    
    int calSpeed = 100; // 캘리브레이션 속도 설정 

    for (uint16_t i = 0; i < 400; i++) {
        _qtr.calibrate(); // 센서 값 읽어서 Min/Max 갱신
        
        // 제자리 좌우 회전 (Pivot Turn)
        // 20ms마다 방향 전환 체크
        if (i % 20 == 0) {
            // 주기적으로 왼쪽/오른쪽 번갈아가며 회전
            if (i % 40 == 0) {
                // 왼쪽으로 제자리 회전 (왼쪽 후진, 오른쪽 전진)
                // _motor.turnLeft 함수가 제자리 회전인지 확인 필요하지만, 보통 한쪽만 돌거나 제자리 회전임.
                // 더 확실한 제자리 회전을 위해 직접 제어 명령을 줄 수도 있음.
                _motor.turnLeft(calSpeed); 
            } else {
                // 오른쪽으로 제자리 회전
                _motor.turnRight(calSpeed);
            }
        }
        delay(1); // 루프 속도 조절
    }
    _motor.stop();
    Serial.println(F("[LineTracer] Calibration done."));
    
    // 캘리브레이션 결과 출력 (디버깅)
    Serial.print("Min: ");
    for (uint8_t i = 0; i < _sensorCount; i++) {
        Serial.print(_qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();
    
    Serial.print("Max: ");
    for (uint8_t i = 0; i < _sensorCount; i++) {
        Serial.print(_qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void LineTracer::runPID() {
    // readLineBlack returns a value from 0 to 7000 (for 8 sensors)
    uint16_t position = _qtr.readLineBlack(_sensorValues);

    // ========== 안전 장치 (Safety Stop) ==========
    // 모든 센서 값이 극단적인 경우 정지
    uint32_t sensorSum = 0;
    for (uint8_t i = 0; i < _sensorCount; i++) {
        sensorSum += _sensorValues[i];
    }

    // 1. 공중 부양 또는 흰 바닥 (모든 센서값 합이 매우 낮음)
    // 검출값이 1000 미만이면 그라우드(Ground)로 인식
    if (sensorSum < 1000) {
        _motor.stop();
        return;
    }

    // 2. 모든 센서가 검은색 (교차로 또는 종료 지점, 혹은 공중)
    // 공중에서 5000 정도 나오므로, 4500 이상이면 Stop 처리
    if (sensorSum > 4500) {
        _motor.stop();
        return;
    }
    // ==========================================
    
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

void LineTracer::printSensorValues() {
    uint16_t position = _qtr.readLineBlack(_sensorValues);
    
    Serial.print("Sensors: ");
    for (uint8_t i = 0; i < _sensorCount; i++) {
        Serial.print(_sensorValues[i]);
        Serial.print('\t');
    }
    Serial.print(" | Pos: ");
    Serial.println(position);
}
