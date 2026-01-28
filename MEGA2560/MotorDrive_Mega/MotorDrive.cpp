/**
 * @file MotorDrive.cpp
 * @brief BTS7960 모터 드라이버 제어 및 라인트레이싱 기능 구현
 * @version 1.1
 */

#include "MotorDrive.h"

// ============== 생성자 및 초기화 ==============

MotorDrive::MotorDrive(int r_en1, int l_en1, int rpwm1, int lpwm1, 
                       int r_en2, int l_en2, int rpwm2, int lpwm2) {
    _r_en1 = r_en1;
    _l_en1 = l_en1;
    _rpwm1 = rpwm1;
    _lpwm1 = lpwm1;

    _r_en2 = r_en2;
    _l_en2 = l_en2;
    _rpwm2 = rpwm2;
    _lpwm2 = lpwm2;

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
    pinMode(_r_en1, OUTPUT);
    pinMode(_l_en1, OUTPUT);
    pinMode(_rpwm1, OUTPUT);
    pinMode(_lpwm1, OUTPUT);

    pinMode(_r_en2, OUTPUT);
    pinMode(_l_en2, OUTPUT);
    pinMode(_rpwm2, OUTPUT);
    pinMode(_lpwm2, OUTPUT);

    digitalWrite(_r_en1, HIGH);
    digitalWrite(_l_en1, HIGH);
    digitalWrite(_r_en2, HIGH);
    digitalWrite(_l_en2, HIGH);

    stop();
    Serial.println(F("[MotorDrive] Initialized - BTS7960 Ready"));
}

// ============== 내부 헬퍼 함수 ==============

void MotorDrive::setMotorA(int direction, int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    if (direction > 0) {
        analogWrite(_lpwm1, speed);
        analogWrite(_rpwm1, 0);
    } else if (direction < 0) {
        analogWrite(_lpwm1, 0);
        analogWrite(_rpwm1, speed);
    } else {
        analogWrite(_lpwm1, 0);
        analogWrite(_rpwm1, 0);
    }
}

void MotorDrive::setMotorB(int direction, int speed) {
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    if (direction > 0) {
        analogWrite(_lpwm2, speed);
        analogWrite(_rpwm2, 0);
    } else if (direction < 0) {
        analogWrite(_lpwm2, 0);
        analogWrite(_rpwm2, speed);
    } else {
        analogWrite(_lpwm2, 0);
        analogWrite(_rpwm2, 0);
    }
}

// ============== 기본 주행 함수 ==============

void MotorDrive::forward(int speed) {
    if (speed < 0) speed = _baseSpeed;
    // 왼쪽 모터 속도를 95%로 줄여서 직진성 보정 (왼쪽으로 휜다면 왼쪽 모터가 너무 강한 것일 수도 있고 오른쪽이 약한 것일 수도 있음)
    // 일반적으로 "왼쪽으로 휜다" = "오른쪽 바퀴가 더 많이 굴러간다" (오버스티어) -> 오른쪽을 줄여야 함
    // 질문자님이 "좌우 밸런스가 안 맞는다"고만 했고 방향은 특정하지 않았으나,
    // 보통 직진 보정은 한쪽을 줄이는 식입니다.
    // 여기서는 임의로 Motor A(왼쪽)에 0.95를 곱합니다. 테스트 후 반대면 B를 줄이세요.
    setMotorA(1, (int)(speed * 1.3)); 
    setMotorB(1, speed);
}

void MotorDrive::backward(int speed) {
    if (speed < 0) speed = _baseSpeed;
    setMotorA(-1, speed);
    setMotorB(-1, speed);
}

void MotorDrive::turnLeft(int speed) {
    if (speed < 0) speed = _turnSpeed;
    setMotorA(-1, speed);
    setMotorB(1, speed);
}

void MotorDrive::turnRight(int speed) {
    if (speed < 0) speed = _turnSpeed;
    setMotorA(1, speed);
    setMotorB(-1, speed);
}

void MotorDrive::curveLeft(int speed, float ratio) {
    if (speed < 0) speed = _baseSpeed;
    ratio = constrain(ratio, 0.0, 1.0);
    setMotorA(1, (int)(speed * ratio));
    setMotorB(1, speed);
}

void MotorDrive::curveRight(int speed, float ratio) {
    if (speed < 0) speed = _baseSpeed;
    ratio = constrain(ratio, 0.0, 1.0);
    setMotorA(1, speed);
    setMotorB(1, (int)(speed * ratio));
}

void MotorDrive::stop() {
    setMotorA(0, 0);
    setMotorB(0, 0);
}

void MotorDrive::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) setMotorA(1, leftSpeed);
    else setMotorA(-1, -leftSpeed);

    if (rightSpeed >= 0) setMotorB(1, rightSpeed);
    else setMotorB(-1, -rightSpeed);
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
}

void MotorDrive::speedDown(int amount) {
    _baseSpeed = constrain(_baseSpeed - amount, MIN_SPEED, MAX_SPEED);
}

// ============== 라인트레이싱 함수 ==============

void MotorDrive::setLineSensors(int pins[], int count) {
    _lineSensorCount = min(count, LINE_SENSOR_COUNT);
    for (int i = 0; i < _lineSensorCount; i++) {
        _lineSensorPins[i] = pins[i];
        pinMode(_lineSensorPins[i], INPUT);
    }
}

void MotorDrive::setPIDGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
    _lastError = 0; _integral = 0;
}

int MotorDrive::readLineSensor(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= _lineSensorCount) return 0;
    return digitalRead(_lineSensorPins[sensorIndex]);
}

void MotorDrive::readAllLineSensors(int values[]) {
    for (int i = 0; i < _lineSensorCount; i++) values[i] = digitalRead(_lineSensorPins[i]);
}

int MotorDrive::calculateLinePosition() {
    if (_lineSensorCount == 0) return 0;
    int sensorValues[LINE_SENSOR_COUNT];
    readAllLineSensors(sensorValues);
    int weightSum = 0; int sensorSum = 0;
    for (int i = 0; i < _lineSensorCount; i++) {
        int weight = i - (_lineSensorCount / 2);
        if (sensorValues[i] == LINE_BLACK) {
            weightSum += weight * 100;
            sensorSum++;
        }
    }
    if (sensorSum == 0) return (int)_lastError;
    return weightSum / sensorSum;
}

void MotorDrive::lineTrace() {
    if (_lineSensorCount < 3) return;
    int sensorValues[LINE_SENSOR_COUNT];
    readAllLineSensors(sensorValues);
    bool left = (sensorValues[0] == LINE_BLACK);
    bool center = (sensorValues[_lineSensorCount / 2] == LINE_BLACK);
    bool right = (sensorValues[_lineSensorCount - 1] == LINE_BLACK);
    if (center && !left && !right) forward();
    else if (left && !right) curveLeft(_baseSpeed, 0.3);
    else if (right && !left) curveRight(_baseSpeed, 0.3);
    else if (left && right) forward();
    else stop();
}

void MotorDrive::lineTracePID() {
    if (_lineSensorCount == 0) return;
    float position = calculateLinePosition();
    float error = position;
    _integral += error;
    float derivative = error - _lastError;
    float correction = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    _lastError = error;
    int leftSpeed = constrain(_baseSpeed + (int)correction, -MAX_SPEED, MAX_SPEED);
    int rightSpeed = constrain(_baseSpeed - (int)correction, -MAX_SPEED, MAX_SPEED);
    setMotorSpeeds(leftSpeed, rightSpeed);
}

bool MotorDrive::isLineDetected() {
    for (int i = 0; i < _lineSensorCount; i++) if (digitalRead(_lineSensorPins[i]) == LINE_BLACK) return true;
    return false;
}

bool MotorDrive::isCrossDetected() {
    if (_lineSensorCount < 3) return false;
    int blackCount = 0;
    for (int i = 0; i < _lineSensorCount; i++) if (digitalRead(_lineSensorPins[i]) == LINE_BLACK) blackCount++;
    return (blackCount >= _lineSensorCount - 1);
}