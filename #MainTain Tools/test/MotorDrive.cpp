/**
 * @file MotorDrive.cpp
 * @brief L298N 모터 드라이버 제어 및 라인트레이싱 기능 구현
 * @version 1.1
 * @note AVR (Arduino Mega/Uno) 호환 버전
 */

#include "MotorDrive.h"

// ============== 생성자 및 초기화 ==============

MotorDrive::MotorDrive(int in1, int in2, int ena, int in3, int in4, int enb) {
    _in1 = in1;
    _in2 = in2;
    _ena = ena;
    _in3 = in3;
    _in4 = in4;
    _enb = enb;

    _baseSpeed = DEFAULT_SPEED;
    _turnSpeed = DEFAULT_TURN_SPEED;
    _lineSensorCount = 0;

    // PID 기본값
    _kp = 25.0;
    _ki = 0.0;
    _kd = 15.0;
    _lastError = 0;
    _integral = 0;
}

void MotorDrive::begin() {
    // 모터 A 핀 설정
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_ena, OUTPUT);

    // 모터 B 핀 설정
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
    pinMode(_enb, OUTPUT);

    // AVR에서는 analogWrite 사용 (PWM 핀이어야 함)
    // PWM 핀: Arduino Mega - 2~13, 44~46
    // PWM 핀: Arduino Uno - 3, 5, 6, 9, 10, 11

    // 초기 상태: 정지
    stop();

    Serial.println(F("[MotorDrive] Initialized - L298N Ready"));
}

// ============== 내부 헬퍼 함수 ==============

void MotorDrive::setMotorA(int direction, int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    if (direction > 0) {
        // 전진
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else if (direction < 0) {
        // 후진
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    } else {
        // 정지
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }
    analogWrite(_ena, speed);  // AVR용 PWM
}

void MotorDrive::setMotorB(int direction, int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    if (direction > 0) {
        // 전진
        digitalWrite(_in3, HIGH);
        digitalWrite(_in4, LOW);
    } else if (direction < 0) {
        // 후진
        digitalWrite(_in3, LOW);
        digitalWrite(_in4, HIGH);
    } else {
        // 정지
        digitalWrite(_in3, LOW);
        digitalWrite(_in4, LOW);
    }
    analogWrite(_enb, speed);  // AVR용 PWM
}

// ============== 기본 주행 함수 ==============

void MotorDrive::forward(int speed) {
    if (speed < 0) speed = _baseSpeed;
    setMotorA(1, speed);
    setMotorB(1, speed);
}

void MotorDrive::backward(int speed) {
    if (speed < 0) speed = _baseSpeed;
    setMotorA(-1, speed);
    setMotorB(-1, speed);
}

void MotorDrive::turnLeft(int speed) {
    if (speed < 0) speed = _turnSpeed;
    setMotorA(-1, speed);  // 왼쪽 모터 후진
    setMotorB(1, speed);   // 오른쪽 모터 전진
}

void MotorDrive::turnRight(int speed) {
    if (speed < 0) speed = _turnSpeed;
    setMotorA(1, speed);   // 왼쪽 모터 전진
    setMotorB(-1, speed);  // 오른쪽 모터 후진
}

void MotorDrive::curveLeft(int speed, float ratio) {
    if (speed < 0) speed = _baseSpeed;
    ratio = constrain(ratio, 0.0, 1.0);
    setMotorA(1, (int)(speed * ratio));  // 왼쪽 느리게
    setMotorB(1, speed);                  // 오른쪽 정상
}

void MotorDrive::curveRight(int speed, float ratio) {
    if (speed < 0) speed = _baseSpeed;
    ratio = constrain(ratio, 0.0, 1.0);
    setMotorA(1, speed);                  // 왼쪽 정상
    setMotorB(1, (int)(speed * ratio));  // 오른쪽 느리게
}

void MotorDrive::stop() {
    setMotorA(0, 0);
    setMotorB(0, 0);
}

void MotorDrive::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // 왼쪽 모터 (모터 A)
    if (leftSpeed >= 0) {
        setMotorA(1, leftSpeed);
    } else {
        setMotorA(-1, -leftSpeed);
    }

    // 오른쪽 모터 (모터 B)
    if (rightSpeed >= 0) {
        setMotorB(1, rightSpeed);
    } else {
        setMotorB(-1, -rightSpeed);
    }
}

// ============== 속도 설정 함수 ==============

void MotorDrive::setSpeed(int speed) {
    _baseSpeed = constrain(speed, MIN_SPEED, MAX_SPEED);
}

int MotorDrive::getSpeed() {
    return _baseSpeed;
}

void MotorDrive::speedUp(int amount) {
    _baseSpeed = constrain(_baseSpeed + amount, MIN_SPEED, MAX_SPEED);
    Serial.print(F("[MotorDrive] Speed: "));
    Serial.println(_baseSpeed);
}

void MotorDrive::speedDown(int amount) {
    _baseSpeed = constrain(_baseSpeed - amount, MIN_SPEED, MAX_SPEED);
    Serial.print(F("[MotorDrive] Speed: "));
    Serial.println(_baseSpeed);
}

// ============== 라인트레이싱 함수 ==============

void MotorDrive::setLineSensors(int pins[], int count) {
    _lineSensorCount = min(count, LINE_SENSOR_COUNT);
    
    for (int i = 0; i < _lineSensorCount; i++) {
        _lineSensorPins[i] = pins[i];
        pinMode(_lineSensorPins[i], INPUT);
    }
    
    Serial.print(F("[MotorDrive] Line sensors set: "));
    Serial.print(_lineSensorCount);
    Serial.println(F(" sensors"));
}

void MotorDrive::setPIDGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _lastError = 0;
    _integral = 0;
    
    Serial.print(F("[MotorDrive] PID: Kp="));
    Serial.print(_kp);
    Serial.print(F(", Ki="));
    Serial.print(_ki);
    Serial.print(F(", Kd="));
    Serial.println(_kd);
}

int MotorDrive::readLineSensor(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= _lineSensorCount) {
        return 0;
    }
    return digitalRead(_lineSensorPins[sensorIndex]);
}

void MotorDrive::readAllLineSensors(int values[]) {
    for (int i = 0; i < _lineSensorCount; i++) {
        values[i] = digitalRead(_lineSensorPins[i]);
    }
}

int MotorDrive::calculateLinePosition() {
    if (_lineSensorCount == 0) return 0;

    int sensorValues[LINE_SENSOR_COUNT];
    readAllLineSensors(sensorValues);

    // 가중치 기반 위치 계산
    // 센서 배치: [0]=가장 왼쪽, [n-1]=가장 오른쪽
    // 가중치 예시 (5개 센서): -2, -1, 0, +1, +2
    int weightSum = 0;
    int sensorSum = 0;

    for (int i = 0; i < _lineSensorCount; i++) {
        int weight = i - (_lineSensorCount / 2);  // 중앙을 0으로
        if (sensorValues[i] == LINE_BLACK) {
            weightSum += weight * 100;  // 스케일 조정
            sensorSum++;
        }
    }

    if (sensorSum == 0) {
        // 라인 없음 - 마지막 위치 유지
        return (int)_lastError;
    }

    return weightSum / sensorSum;  // -100 ~ +100 범위
}

void MotorDrive::lineTrace() {
    if (_lineSensorCount < 3) {
        Serial.println(F("[MotorDrive] Error: At least 3 line sensors required"));
        return;
    }

    int sensorValues[LINE_SENSOR_COUNT];
    readAllLineSensors(sensorValues);

    // 3개 센서 기준 간단한 라인트레이싱
    // 센서 배치: [0]=왼쪽, [1]=중앙, [2]=오른쪽
    bool left = (sensorValues[0] == LINE_BLACK);
    bool center = (sensorValues[_lineSensorCount / 2] == LINE_BLACK);
    bool right = (sensorValues[_lineSensorCount - 1] == LINE_BLACK);

    if (center && !left && !right) {
        // 중앙에 라인 → 직진
        forward();
    } else if (left && !right) {
        // 왼쪽에 라인 → 좌회전
        curveLeft(_baseSpeed, 0.3);
    } else if (right && !left) {
        // 오른쪽에 라인 → 우회전
        curveRight(_baseSpeed, 0.3);
    } else if (left && right) {
        // 교차로 감지 → 직진 (또는 사용자 정의 동작)
        forward();
    } else {
        // 라인 없음 → 정지 또는 이전 방향 유지
        stop();
    }
}

void MotorDrive::lineTracePID() {
    if (_lineSensorCount == 0) {
        Serial.println(F("[MotorDrive] Error: No line sensors configured"));
        return;
    }

    // 현재 위치 계산
    float position = calculateLinePosition();

    // PID 제어
    float error = position;  // 목표는 0 (중앙)
    _integral += error;
    float derivative = error - _lastError;

    float correction = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    _lastError = error;

    // 모터 속도 계산
    int leftSpeed = _baseSpeed + (int)correction;
    int rightSpeed = _baseSpeed - (int)correction;

    // 속도 제한
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    // 모터 구동
    setMotorSpeeds(leftSpeed, rightSpeed);
}

bool MotorDrive::isLineDetected() {
    for (int i = 0; i < _lineSensorCount; i++) {
        if (digitalRead(_lineSensorPins[i]) == LINE_BLACK) {
            return true;
        }
    }
    return false;
}

bool MotorDrive::isCrossDetected() {
    if (_lineSensorCount < 3) return false;

    int blackCount = 0;
    for (int i = 0; i < _lineSensorCount; i++) {
        if (digitalRead(_lineSensorPins[i]) == LINE_BLACK) {
            blackCount++;
        }
    }

    // 대부분의 센서가 검은색을 감지하면 교차로로 판단
    return (blackCount >= _lineSensorCount - 1);
}