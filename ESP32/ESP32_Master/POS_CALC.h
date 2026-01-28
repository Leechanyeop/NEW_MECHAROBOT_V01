#ifndef POS_CALC_H
#define POS_CALC_H

#include <Arduino.h>
#include <math.h>

class PosCalc {
public:
    // 생성자: 로봇 물리 상수 초기화
    // wheelDiameterMm: 바퀴 지름 (mm)
    // wheelBaseMm: 바퀴 간 거리 (mm)
    // ticksPerRev: 바퀴 1회전 당 엔코더 틱 수
    PosCalc(float wheelDiameterMm, float wheelBaseMm, float ticksPerRev);

    // 현재 위치를 0 또는 특정 값으로 초기화 (엔코더 값은 호출 시점 기준으로 동기화됨)
    void resetPose(float x = 0.0, float y = 0.0, float theta = 0.0);
    
    // 주기적으로 호출하여 오도메트리 계산
    // encLeft, encRight: 현재 누적 엔코더 카운트
    void update(long encLeft, long encRight);

    // 현재 위치 및 각도 가져오기
    float getX() const { return _x; }
    float getY() const { return _y; }
    float getTheta() const { return _theta; } // Radians
    float getThetaDeg() const; // Degrees

    // 필요시 값 직접 설정
    void setPose(float x, float y, float theta);

private:
    float _wheelDiameter;
    float _wheelBase;
    float _ticksPerRev;
    
    long _lastEncLeft;
    long _lastEncRight;
    bool _firstRun;
    
    float _x;
    float _y;
    float _theta; // Radians
};

#endif
