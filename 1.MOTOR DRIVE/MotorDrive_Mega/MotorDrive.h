/**
 * @file MotorDrive.h
 * @brief BTS7960 모터 드라이버 제어 및 라인트레이싱 기능 정의
 * @version 1.1
 */

#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include <Arduino.h>

// ============== 기본 설정 값 ==============
#define DEFAULT_SPEED       200     // 기본 속도 (0~255)
#define DEFAULT_TURN_SPEED  150     // 회전 속도
#define MIN_SPEED           0
#define MAX_SPEED           255

// 라인 센서 설정
#define LINE_SENSOR_COUNT   5       // 라인 센서 개수 (최대 5개)
#define LINE_BLACK          1       // 검은 라인 감지
#define LINE_WHITE          0       // 흰 배경

/**
 * @class MotorDrive
 * @brief BTS7960 드라이버를 사용한 DC 모터 제어 클래스
 */
class MotorDrive {
private:
    // 모터 A 핀 (BTS7960 드라이버 1)
    int _r_en1;
    int _l_en1;
    int _rpwm1;
    int _lpwm1;

    // 모터 B 핀 (BTS7960 드라이버 2)
    int _r_en2;
    int _l_en2;
    int _rpwm2;
    int _lpwm2;

    // 라인 센서 핀 배열
    int _lineSensorPins[LINE_SENSOR_COUNT];
    int _lineSensorCount;

    // 속도 설정
    int _baseSpeed;
    int _turnSpeed;

    // 라인트레이싱 PID 변수
    float _kp;          // 비례 게인
    float _ki;          // 적분 게인
    float _kd;          // 미분 게인
    float _lastError;
    float _integral;

    // 내부 헬퍼 함수
    void setMotorA(int direction, int speed);
    void setMotorB(int direction, int speed);

public:
    /**
     * @brief 생성자 - BTS7960 모터 핀 설정
     */
    MotorDrive(int r_en1, int l_en1, int rpwm1, int lpwm1, 
               int r_en2, int l_en2, int rpwm2, int lpwm2);

    /**
     * @brief 모터 드라이버 초기화
     */
    void begin();

    // ============== 기본 주행 함수 ==============
    void forward(int speed = -1);
    void backward(int speed = -1);
    void turnLeft(int speed = -1);
    void turnRight(int speed = -1);
    void curveLeft(int speed = -1, float ratio = 0.5);
    void curveRight(int speed = -1, float ratio = 0.5);
    void stop();
    void setMotorSpeeds(int leftSpeed, int rightSpeed);

    // ============== 속도 설정 함수 ==============
    void setSpeed(int speed);
    int getSpeed();
    void speedUp(int amount = 20);
    void speedDown(int amount = 20);

    // ============== 라인트레이싱 함수 ==============
    void setLineSensors(int pins[], int count);
    void setPIDGains(float kp, float ki = 0.0, float kd = 0.0);
    int readLineSensor(int sensorIndex);
    void readAllLineSensors(int values[]);
    int calculateLinePosition();
    void lineTrace();
    void lineTracePID();
    bool isLineDetected();
    bool isCrossDetected();
};

#endif // MOTORDRIVE_H
