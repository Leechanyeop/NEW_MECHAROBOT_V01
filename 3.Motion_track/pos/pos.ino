#include <math.h>

// 기본 상수
const double WHEEL_DIAMETER_CM = 6.8;    // 바퀴 지름 (cm)
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER_CM;

// 상태 구조체
struct Pose {
  double x;      // cm
  double y;      // cm
  double theta;  // rad, 0 = x축 방향
};

// ticksPerRev: 엔코더 1회전당 틱 수
// wheelBase: 좌우 바퀴 사이 거리 (cm)
Pose updatePose(Pose prev, long ticksL, long ticksR, double ticksPerRev, double wheelBase) {
  // 거리 계산
  double dL = (ticksL / ticksPerRev) * WHEEL_CIRCUMFERENCE;
  double dR = (ticksR / ticksPerRev) * WHEEL_CIRCUMFERENCE;
  double dc = (dR + dL) / 2.0;
  double dTheta = (dR - dL) / wheelBase;

  Pose next = prev;

  if (fabs(dTheta) < 1e-6) {
    // 거의 직선 이동
    next.x += dc * cos(prev.theta);
    next.y += dc * sin(prev.theta);
    // theta 변화 거의 없음
  } else {
    // 회전 중심(ICC) 이용한 정확한 갱신
    double R = dc / dTheta; // 회전 반경
    double cx = prev.x - R * sin(prev.theta);
    double cy = prev.y + R * cos(prev.theta);

    next.theta = prev.theta + dTheta;
    // theta 정규화 (-pi ~ pi)
    if (next.theta > M_PI) next.theta -= 2.0 * M_PI;
    if (next.theta <= -M_PI) next.theta += 2.0 * M_PI;

    next.x = cx + R * sin(next.theta);
    next.y = cy - R * cos(next.theta);
  }

  return next;
}