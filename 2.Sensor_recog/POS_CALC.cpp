#include <math.h>

/**
 * [물리적 상수 설정] 
 * 사용자 로봇의 실제 사양에 맞춰 아래 값을 수정해 주세요.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double WHEEL_DIAMETER = 6.8;    // 바퀴 지름 (cm)
const double WHEEL_BASE = 20.0;        // 좌우 바퀴 간의 거리 (cm) - 실제 측정값 입력 필요
const double TICKS_PER_REV = 658;    // 엔코더 1회전당 틱 수 (PPR * Gear Ratio) - 실제 사양 입력 필요

const double left_encoder = 0;
const double right_encoder = 0;



/**
 * @brief 왼쪽/오른쪽 엔코더 틱 값을 이용하여 차체의 현재 각도(Heading)를 계산합니다.
 * 
 * 원리: 두 바퀴의 이동 거리 차이를 바퀴 사이의 거리(Wheel Base)로 나누면 회전한 각도(Radian)가 나옵니다.
 * 
 * @param leftEncoder  왼쪽 모터 엔코더 카운트 값
 * @param rightEncoder 오른쪽 모터 엔코더 카운트 값
 * @return double      계산된 차체의 각도 (Degree 단위)
 */

    double distanceLeft;
    double distanceRight;

double calculateChassisAngle(long leftEncoder, long rightEncoder) {
    // 1. 각 바퀴가 이동한 총 거리 (cm) 계산
    // (현재 틱 / 1회전당 틱) * 바퀴 둘레
    double distanceLeft = (double)leftEncoder / TICKS_PER_REV * (M_PI * WHEEL_DIAMETER);
    double distanceRight = (double)rightEncoder / TICKS_PER_REV * (M_PI * WHEEL_DIAMETER);

    // 2. 각도 변화량 계산 (Radian)
    // 공식: theta = (dR - dL) / L
    double angleRad = (distanceRight - distanceLeft) / WHEEL_BASE;

    // 3. Radian을 Degree(도) 단위로 변환
    double angleDeg = angleRad * (180.0 / M_PI);

    // 4. (옵션) 각도를 -180 ~ 180도 범위로 정규화하거나 0~360 범위로 관리할 수 있습니다.
    // 여기서는 단순 누적 각도를 반환합니다.
    
    return angleDeg;
}

double calculate_distance(long leftEncoder, long rightEncoder) {
    // 1. 각 바퀴가 이동한 총 거리 (cm) 계산
    // (현재 틱 / 1회전당 틱) * 바퀴 둘레
    double distanceLeft = (double)leftEncoder / TICKS_PER_REV * (M_PI * WHEEL_DIAMETER);
    double distanceRight = (double)rightEncoder / TICKS_PER_REV * (M_PI * WHEEL_DIAMETER);

    // 2. 각도 변화량 계산 (Radian)
    // 공식: theta = (dR - dL) / L
    double distance = (distanceRight + distanceLeft) / 2;

    return distance;
}


double calculate_x(double angleDeg) {
 double x = calculate_distance() * cos(angleDeg);
 return x;
}

double calculate_y(double angleDeg) {
 double y = calculate_distance() * sin(angleDeg);
 return y;
}