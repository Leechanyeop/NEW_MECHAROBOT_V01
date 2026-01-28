#include "POS_CALC.h"

PosCalc::PosCalc(float wheelDiameterMm, float wheelBaseMm, float ticksPerRev) 
    : _wheelDiameter(wheelDiameterMm), _wheelBase(wheelBaseMm), _ticksPerRev(ticksPerRev)
{
    _x = 0.0f;
    _y = 0.0f;
    _theta = 0.0f;
    _lastEncLeft = 0;
    _lastEncRight = 0;
    _firstRun = true;
}

void PosCalc::resetPose(float x, float y, float theta) {
    _x = x;
    _y = y;
    _theta = theta;
    // _lastEncLeft / Right 는 update() 호출 시 델타 계산을 위해 유지하거나,
    // 여기서 초기화하면 다음 update()에서 튈 수 있으므로 주의해야 함.
    // 보통 resetPose는 위치값만 0으로 밀고, 엔코더 흐름은 끊지 않는 것이 좋음.
    // 하지만 "완전 초기화"라면 _firstRun을 true로 설정하여 델타를 0으로 만드는 방법도 있음.
    // 여기서는 위치만 재설정합니다.
}

void PosCalc::setPose(float x, float y, float theta) {
    _x = x;
    _y = y;
    _theta = theta;
}

void PosCalc::update(long encLeft, long encRight) {
    if (_firstRun) {
        _lastEncLeft = encLeft;
        _lastEncRight = encRight;
        _firstRun = false;
        return;
    }

    long dL = encLeft - _lastEncLeft;
    long dR = encRight - _lastEncRight;
    
    _lastEncLeft = encLeft;
    _lastEncRight = encRight;
    
    if (_ticksPerRev == 0.0f || _wheelBase == 0.0f) return;

    // 거리 변환 (mm)
    // PI * Diameter * (ticks / ticksPerRev)
    float distL = (float)dL / _ticksPerRev * (M_PI * _wheelDiameter);
    float distR = (float)dR / _ticksPerRev * (M_PI * _wheelDiameter);
    
    float dDist = (distL + distR) * 0.5f;
    float dTheta = (distR - distL) / _wheelBase;
    
    // Mid-value approximation for better accuracy on curves
    float midTheta = _theta + dTheta * 0.5f;
    
    _x += dDist * cos(midTheta);
    _y += dDist * sin(midTheta);
    _theta += dTheta;
    
    // Normalize Theta (-PI ~ PI)
    while (_theta > M_PI) _theta -= 2.0 * M_PI;
    while (_theta < -M_PI) _theta += 2.0 * M_PI;
}

float PosCalc::getThetaDeg() const {
    return _theta * 180.0f / M_PI;
}