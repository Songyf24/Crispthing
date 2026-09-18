#include "config.h"

namespace {
float readJSNSR04Cm(int trigPin, int echoPin) {
#if SENSOR_ENABLED
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
#else
  (void)trigPin;
  (void)echoPin;
  return -1.0f;
#endif
}

float readToFFrontCm() {
#if SENSOR_ENABLED && TOF_ENABLED
  // TODO：根据实际 ToF 型号加入 VL53L0X/VL53L1X 等库代码。
  return -1.0f;
#else
  return -1.0f;
#endif
}
}  // namespace

void setupObstacleSensors() {
#if SENSOR_ENABLED
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);
  pinMode(ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(ULTRA_FRONT_ECHO, INPUT);
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
  pinMode(ULTRA_REAR_TRIG, OUTPUT);
  pinMode(ULTRA_REAR_ECHO, INPUT);
#endif
}

DistanceData makeInvalidDistanceData() {
  DistanceData d;
  d.ultraLeftCm = -1.0f;
  d.ultraFrontCm = -1.0f;
  d.ultraRightCm = -1.0f;
  d.ultraRearCm = -1.0f;
  d.tofFrontCm = -1.0f;
  return d;
}

DistanceData readDistanceSensors() {
  DistanceData d = makeInvalidDistanceData();

#if SENSOR_ENABLED
  // 超声波按顺序触发，避免相互串扰。
  d.ultraLeftCm = readJSNSR04Cm(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  delay(30);
  d.ultraFrontCm = readJSNSR04Cm(ULTRA_FRONT_TRIG, ULTRA_FRONT_ECHO);
  delay(30);
  d.ultraRightCm = readJSNSR04Cm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
  delay(30);
  d.ultraRearCm = readJSNSR04Cm(ULTRA_REAR_TRIG, ULTRA_REAR_ECHO);
  d.tofFrontCm = readToFFrontCm();
#endif

  return d;
}

bool isFrontBlocked(const DistanceData& d) {
  const bool ultrasonicBlocked =
      d.ultraFrontCm > 0 && d.ultraFrontCm < ULTRA_FRONT_SAFE_CM;
  const bool tofBlocked =
      d.tofFrontCm > 0 && d.tofFrontCm < TOF_TOO_CLOSE_CM;
  return ultrasonicBlocked || tofBlocked;
}

bool isLeftBlocked(const DistanceData& d) {
  return d.ultraLeftCm > 0 && d.ultraLeftCm < ULTRA_SIDE_SAFE_CM;
}

bool isRightBlocked(const DistanceData& d) {
  return d.ultraRightCm > 0 && d.ultraRightCm < ULTRA_SIDE_SAFE_CM;
}

bool isRearBlocked(const DistanceData& d) {
  return d.ultraRearCm > 0 && d.ultraRearCm < ULTRA_REAR_SAFE_CM;
}

bool obstacleOverrideRequired(const DistanceData& d) {
  // V1 先把“正前方危险”作为强制避障条件。
  return isFrontBlocked(d);
}

MotionCommand computeObstacleCommand(const DistanceData& d) {
  (void)d;

  // 当前 V1 采用最安全策略：检测到正前方障碍就停车。
  // 后续实车验证后，可在这里加入左/右绕行状态机。
  return makeStopCommand("front obstacle stop");
}
