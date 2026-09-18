#include "config.h"

// ============================================================
// 全局运行数据
// ============================================================
TargetData latestTarget = {
  0,        // sequence
  false,    // valid
  0.0f,     // xError
  -1.0f,    // distanceCm
  0.0f,     // similarity
  0         // receivedAt
};
DistanceData latestDistance;
RobotState currentState = STATE_IDLE;

bool hasReceivedTarget = false;
bool manualModeActive = true;
RemoteAction remoteAction = REMOTE_STOP;
int remoteSpeed = 70;
unsigned long lastRemoteCommandTime = 0;

unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  /*
   * 安全模块必须最先初始化：
   * 先把电机引脚设为输出并强制为 0，
   * 再初始化可能受干扰的 I2C 遥控模块。
   */
  setupSafetyRedundancy();
  stopCar();

  setupRemoteControl();
  setupObstacleSensors();
  setupAutoFollow();

  latestDistance = makeInvalidDistanceData();

  Serial.println(F("Controller ready."));
}

void loop() {
  static bool emergencyWasActive = false;

#if EMERGENCY_STOP_ENABLED
  const bool emergencyIsActive =
      digitalRead(EMERGENCY_STOP_PIN) == HIGH;

  if (emergencyIsActive) {
    if (!emergencyWasActive) {
      Serial.println(F("EMERGENCY PRESSED -> STOP"));
    }

    emergencyWasActive = true;
    currentState = STATE_EMERGENCY;

    // 清除之前的运动动作；释放急停后仍需方向键先回中。
    requireRemoteNeutralRearm();
    stopCar();
    return;
  }

  if (emergencyWasActive) {
    emergencyWasActive = false;
    currentState = STATE_IDLE;

    requireRemoteNeutralRearm();
    stopCar();

    Serial.println(
        F("EMERGENCY RELEASED -> release D-pad to rearm"));
    return;
  }
#endif

  readSerialCommunication();

  const unsigned long now = millis();

  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }
  lastControlTime = now;

  latestDistance = readDistanceSensors();

  const MotionCommand command = chooseSafeCommand(now);
  applySafeMotionCommand(command);
}