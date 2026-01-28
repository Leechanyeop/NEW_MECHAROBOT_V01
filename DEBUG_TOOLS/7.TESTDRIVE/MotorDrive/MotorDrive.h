/**
 * @file MotorDrive.h
 * @brief L298N 모터 드라이버 제어 및 라인트레이싱 기능 정의
 * @version 1.0
 * 
 * ESP32 + L298N 드라이버를 사용한 DC 모터 제어 클래스
 * 라인 센서 기반 라인트레이싱 기능 포함
 */

#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include <Arduino.h>

// ============== 기본 설정 값 ==============
#define DEFAULT_SPEED       200     // 기본 속도 (0~255)
#define DEFAULT_TURN_SPEED  150     // 회전 속도
#define MIN_SPEED           0
#define MAX_SPEED           255

// PWM 설정 (AVR에서는 analogWrite 사용)
// Arduino Mega PWM 핀: 2~13, 44~46
// Arduino Uno PWM 핀: 3, 5, 6, 9, 10, 11

// 라인 센서 설정
#define LINE_SENSOR_COUNT   5       // 라인 센서 개수 (최대 5개)
#define LINE_BLACK          1       // 검은 라인 감지
#define LINE_WHITE          0       // 흰 배경

/**
 * @class MotorDrive
 * @brief L298N 드라이버를 사용한 DC 모터 제어 클래스
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
     * @param r_en1, l_en1 모터 1 이네이블 핀
     * @param rpwm1, lpwm1 모터 1 PWM 핀
     * @param r_en2, l_en2 모터 2 이네이블 핀
     * @param rpwm2, lpwm2 모터 2 PWM 핀
     */
    MotorDrive(int r_en1, int l_en1, int rpwm1, int lpwm1, 
               int r_en2, int l_en2, int rpwm2, int lpwm2);

    /**
     * @brief 모터 드라이버 초기화
     */
    void begin();

    // ============== 기본 주행 함수 ==============
    
    /**
     * @brief 전진
     * @param speed 속도 (0~255), 기본값 사용시 -1
     */
    void forward(int speed = -1);

    /**
     * @brief 후진
     * @param speed 속도 (0~255), 기본값 사용시 -1
     */
    void backward(int speed = -1);

    /**
     * @brief 좌회전 (제자리)
     * @param speed 속도 (0~255), 기본값 사용시 -1
     */
    void turnLeft(int speed = -1);

    /**
     * @brief 우회전 (제자리)
     * @param speed 속도 (0~255), 기본값 사용시 -1
     */
    void turnRight(int speed = -1);

    /**
     * @brief 좌측 곡선 주행 (좌측 모터 느리게)
     * @param speed 기준 속도
     * @param ratio 좌측 모터 속도 비율 (0.0~1.0)
     */
    void curveLeft(int speed = -1, float ratio = 0.5);

    /**
     * @brief 우측 곡선 주행 (우측 모터 느리게)
     * @param speed 기준 속도
     * @param ratio 우측 모터 속도 비율 (0.0~1.0)
     */
    void curveRight(int speed = -1, float ratio = 0.5);

    /**
     * @brief 정지
     */
    void stop();

    /**
     * @brief 개별 모터 속도 제어
     * @param leftSpeed 왼쪽 모터 속도 (-255~255, 음수는 후진)
     * @param rightSpeed 오른쪽 모터 속도 (-255~255, 음수는 후진)
     */
    void setMotorSpeeds(int leftSpeed, int rightSpeed);

    // ============== 속도 설정 함수 ==============

    /**
     * @brief 기본 속도 설정
     * @param speed 속도 (0~255)
     */
    void setSpeed(int speed);

    /**
     * @brief 현재 설정된 속도 반환
     * @return 현재 속도
     */
    int getSpeed();

    /**
     * @brief 속도 증가
     * @param amount 증가량 (기본값 20)
     */
    void speedUp(int amount = 20);

    /**
     * @brief 속도 감소
     * @param amount 감소량 (기본값 20)
     */
    void speedDown(int amount = 20);

    // ============== 라인트레이싱 함수 ==============

    /**
     * @brief 라인 센서 핀 설정
     * @param pins 센서 핀 배열
     * @param count 센서 개수
     */
    void setLineSensors(int pins[], int count);

    /**
     * @brief PID 게인 설정
     * @param kp 비례 게인
     * @param ki 적분 게인
     * @param kd 미분 게인
     */
    void setPIDGains(float kp, float ki = 0.0, float kd = 0.0);

    /**
     * @brief 라인 센서 값 읽기
     * @param sensorIndex 센서 인덱스
     * @return 센서 값 (0 또는 1)
     */
    int readLineSensor(int sensorIndex);

    /**
     * @brief 모든 라인 센서 값 읽기
     * @param values 센서 값 저장 배열
     */
    void readAllLineSensors(int values[]);

    /**
     * @brief 라인 위치 계산 (-100 ~ +100)
     * @return 라인 위치 (음수: 왼쪽, 양수: 오른쪽, 0: 중앙)
     */
    int calculateLinePosition();

    /**
     * @brief 라인트레이싱 실행 (1회 호출)
     * 반복적으로 loop()에서 호출하여 사용
     */
    void lineTrace();

    /**
     * @brief 라인트레이싱 (PID 제어)
     */
    void lineTracePID();

    /**
     * @brief 라인 감지 여부 확인
     * @return true: 라인 감지됨, false: 라인 없음
     */
    bool isLineDetected();

    /**
     * @brief 교차로 감지 여부 확인
     * @return true: 교차로 감지됨
     */
    bool isCrossDetected();
};

#endif // MOTORDRIVE_H
