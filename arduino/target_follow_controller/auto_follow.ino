#include "config.h"

namespace {

// 根据目标水平偏差计算转向速度
int computeTurnSpeed(float xError) {
  xError = constrain(xError, -1.0f, 1.0f);
  return (int)(K_TURN * xError * TURN_SIGN);
}

// 根据目标距离计算前进速度
int computeForwardSpeedByDistance(float distanceCm) {
  // 距离无效
  if (distanceCm <= 0.0f) {
    return 0;
  }

  // 已经达到跟随停止距离
  if (distanceCm <= FOLLOW_STOP_DISTANCE_CM) {
    return 0;
  }

  // 目标较远，使用最大速度
  if (distanceCm >= FOLLOW_FULL_SPEED_DISTANCE_CM) {
    return MAX_SPEED;
  }

  // 在停止距离和全速距离之间线性调速
  const float ratio =
      (distanceCm - FOLLOW_STOP_DISTANCE_CM) /
      (FOLLOW_FULL_SPEED_DISTANCE_CM - FOLLOW_STOP_DISTANCE_CM);

  const int speed =
      MIN_SPEED + (int)(ratio * (MAX_SPEED - MIN_SPEED));

  return constrain(speed, MIN_SPEED, MAX_SPEED);
}

}  // namespace

void setupAutoFollow() {
  // 当前自主跟随模块不需要额外硬件初始化
}

// 解析新协议：
// TARGET,序号,目标有效,x偏差,距离,相似度
bool parseTargetMessage(char* line, TargetData& target) {
  char* token = strtok(line, ",");

  // 消息类型
  if (token == NULL || strcmp(token, "TARGET") != 0) {
    return false;
  }

  // 1. 序号
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.sequence = strtoul(token, NULL, 10);

  // 2. 目标是否有效
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }

  const int validValue = atoi(token);

  if (validValue != 0 && validValue != 1) {
    return false;
  }

  target.valid = (validValue == 1);

  // 3. x 偏差
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.xError = atof(token);

  // 4. 目标距离，单位 cm
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.distanceCm = atof(token);

  // 5. 相似度
  token = strtok(NULL, ",");
  if (token == NULL) {
    return false;
  }
  target.similarity = atof(token);

  // 不允许出现额外字段
  if (strtok(NULL, ",") != NULL) {
    return false;
  }

  // 数值范围检查
  if (target.xError < -1.0f || target.xError > 1.0f) {
    return false;
  }

  if (target.similarity < 0.0f || target.similarity > 1.0f) {
    return false;
  }

  // 允许 -1 表示距离无效，不允许其他负数
  if (target.distanceCm < 0.0f &&
      target.distanceCm != -1.0f) {
    return false;
  }

  target.receivedAt = millis();

#if DEBUG_PRINT
  Serial.print("TARGET: seq=");
  Serial.print(target.sequence);

  Serial.print(", valid=");
  Serial.print(target.valid);

  Serial.print(", xError=");
  Serial.print(target.xError);

  Serial.print(", distanceCm=");
  Serial.print(target.distanceCm);

  Serial.print(", similarity=");
  Serial.println(target.similarity);
#endif

  return true;
}

MotionCommand computeAutoFollowCommand(
    const TargetData& target,
    const DistanceData& distance) {

  // 目标无效
  if (!target.valid) {
    return makeStopCommand("target invalid");
  }

  // 当前目标与指定跟随者相似度不足
  if (target.similarity < SIMILARITY_MIN) {
    return makeStopCommand("low target similarity");
  }

  // 距离无效
  if (target.distanceCm <= 0.0f) {
    return makeStopCommand("invalid target distance");
  }

  // 已经达到跟随停止距离
  if (target.distanceCm <= FOLLOW_STOP_DISTANCE_CM) {
    return makeStopCommand("target within stop distance");
  }

  // 根据距离计算前进速度
  const int forwardSpeed =
      computeForwardSpeedByDistance(target.distanceCm);

  // 根据水平偏差计算转向速度
  int turnSpeed = computeTurnSpeed(target.xError);

  // 左侧有障碍时，禁止继续向左修正
  if (target.xError < 0.0f &&
      isLeftBlocked(distance)) {
    turnSpeed = 0;
  }

  // 右侧有障碍时，禁止继续向右修正
  if (target.xError > 0.0f &&
      isRightBlocked(distance)) {
    turnSpeed = 0;
  }

  MotionCommand cmd;

  cmd.leftSpeed = constrain(
      forwardSpeed + turnSpeed,
      -MAX_SPEED,
      MAX_SPEED);

  cmd.rightSpeed = constrain(
      forwardSpeed - turnSpeed,
      -MAX_SPEED,
      MAX_SPEED);

  cmd.reason = "auto follow";

  return cmd;
}