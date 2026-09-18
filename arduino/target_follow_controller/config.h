#ifndef TARGET_FOLLOW_CONFIG_H
#define TARGET_FOLLOW_CONFIG_H

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ============================================================
// 1. 功能开关
// ============================================================

// 0：只测试指令，不真正驱动电机
// 1：允许电机运行
#define MOTOR_ENABLED 1

// 当前暂不启用避障传感器
#define SENSOR_ENABLED 0
#define TOF_ENABLED 0

// 急停：A3 接 NC，COM 接 GND
#define EMERGENCY_STOP_ENABLED 1

// USB 手柄转接板通过 I2C 通信
#define I2C_REMOTE_ENABLED 1

// 串口调试输出
#define DEBUG_PRINT 1


// ============================================================
// 2. 电机与履带参数
// ============================================================
//
// 当前左右通道已经按照你的实车结果交换：
//
// 左侧履带：PWM=D6，方向=D9/D10
// 右侧履带：PWM=D5，方向=D7/D8
//
const int LEFT_PWM  = 6;
const int LEFT_IN1  = 12;
const int LEFT_IN2  = 13;

const int RIGHT_PWM = 5;
const int RIGHT_IN1 = 7;
const int RIGHT_IN2 = 8;

// 如果某一侧履带正反方向错误，将对应的 1 改为 -1
#define LEFT_TRACK_DIR  1
#define RIGHT_TRACK_DIR -1

// 自主跟随转向符号
#define TURN_SIGN 1


// ============================================================
// 3. USB 遥控器转接板：I2C
// ============================================================
//
// Arduino UNO：
// SDA = A4
// SCL = A5
//
const uint8_t REMOTE_I2C_ADDRESS = 0x19;

// 转接板每个标准数据包共 9 字节
const uint8_t REMOTE_PACKET_LENGTH = 9;

// 每 20 ms 轮询一次手柄
const unsigned long REMOTE_POLL_INTERVAL_MS = 20;

// 当前已经按实测值直接处理左右方向键，保持为 0
#define REMOTE_SWAP_LEFT_RIGHT 0


// ============================================================
// 4. 距离传感器引脚
// ============================================================
//
// 当前 SENSOR_ENABLED=0，这些引脚不会被初始化。
// 后续正式接避障传感器时再根据实际硬件确认。
//
const int ULTRA_LEFT_TRIG  = 2;
const int ULTRA_LEFT_ECHO  = 3;

const int ULTRA_FRONT_TRIG = 4;
const int ULTRA_FRONT_ECHO = 11;

const int ULTRA_RIGHT_TRIG = 12;
const int ULTRA_RIGHT_ECHO = A0;

const int ULTRA_REAR_TRIG  = A1;
const int ULTRA_REAR_ECHO  = A2;

// ToF 默认使用 I2C：
// SDA=A4，SCL=A5
// 可与遥控板共用 I2C 总线，但当前 TOF_ENABLED=0


// ============================================================
// 5. 安全输入
// ============================================================
//
// 急停连接：
// A3 信号针 → 急停 NC
// GND       → 急停 COM
//
// 正常释放：LOW
// 按下急停：HIGH
//
const int EMERGENCY_STOP_PIN = A3;


// ============================================================
// 6. 自主跟随参数
// ============================================================
const float SIMILARITY_MIN = 0.50f;

// 目标距离小于该值时停止跟随
const float FOLLOW_STOP_DISTANCE_CM = 100.0f;

// 目标距离达到该值时允许使用最大速度
const float FOLLOW_FULL_SPEED_DISTANCE_CM = 250.0f;


// ============================================================
// 7. 速度与转向参数
// ============================================================
//
// 当前仍处于架空调试阶段，最高速度先限制为 90。
// 后续稳定后可逐步提高，但不要直接恢复到很高速度。
//
const int MIN_SPEED = 70;
const int MAX_SPEED = 90;

const float K_TURN = 70.0f;
const float TARGET_CENTER_X_THRESHOLD = 0.25f;


// ============================================================
// 8. 避障参数
// ============================================================
const float ULTRA_FRONT_SAFE_CM = 45.0f;
const float ULTRA_SIDE_SAFE_CM  = 35.0f;
const float ULTRA_REAR_SAFE_CM  = 35.0f;

const float TOF_TOO_CLOSE_CM = 80.0f;
const float TOF_FAR_CM       = 180.0f;


// ============================================================
// 9. 超时与控制周期
// ============================================================

// OpenBot 目标数据超时
const unsigned long TARGET_TIMEOUT_MS = 500;

// 电脑串口单字符遥控超时
const unsigned long REMOTE_TIMEOUT_MS = 1500;

// 手柄完整数据帧超过 800 ms 未更新时强制停车
const unsigned long REMOTE_FAILSAFE_MS = 800;

/*
 * 最大连续运动时间。
 *
 * 这是针对“手柄失联后转接板仍重复最后一帧”的安全兜底。
 * 当前设为 5000 ms，即同一方向连续保持 5 秒后强制停车，
 * 必须松开方向键，再重新按下才能继续。
 *
 * 当前阶段不建议设为 0。
 */
const unsigned long REMOTE_MAX_CONTINUOUS_MS = 5000;

// 主控制周期：20 ms，即约 50 Hz
const unsigned long CONTROL_INTERVAL_MS = 20;


// ============================================================
// 10. 公共数据类型
// ============================================================
struct TargetData {
  unsigned long sequence;      // OpenBot 帧序号
  bool valid;                  // 目标是否有效
  float xError;                // 水平偏差，范围 -1～1
  float distanceCm;            // 目标距离，-1 表示无效
  float similarity;            // 相似度，范围 0～1
  unsigned long receivedAt;    // Arduino 收到新数据的时间
};

struct DistanceData {
  float ultraLeftCm;
  float ultraFrontCm;
  float ultraRightCm;
  float ultraRearCm;
  float tofFrontCm;
};

struct MotionCommand {
  int leftSpeed;
  int rightSpeed;
  const char* reason;
};

enum RobotState {
  STATE_IDLE,
  STATE_REMOTE,
  STATE_AVOID,
  STATE_AUTO,
  STATE_EMERGENCY
};

enum RemoteAction {
  REMOTE_STOP,
  REMOTE_FORWARD,
  REMOTE_BACKWARD,
  REMOTE_LEFT,
  REMOTE_RIGHT
};


// ============================================================
// 11. 全局数据
// 在主 .ino 文件中真正定义
// ============================================================
extern TargetData latestTarget;
extern DistanceData latestDistance;

extern RobotState currentState;

extern bool hasReceivedTarget;
extern bool manualModeActive;

extern RemoteAction remoteAction;
extern int remoteSpeed;

extern unsigned long lastRemoteCommandTime;


// ============================================================
// 12. 遥控模块接口
// ============================================================
void setupRemoteControl();
void readSerialCommunication();

MotionCommand computeRemoteCommand();

bool isRemoteCommandTimedOut(unsigned long now);

/*
 * 急停按下、急停释放、遥控失联后调用。
 * 清除当前运动命令，并要求方向键先回到中位，
 * 防止安全状态解除后小车自动恢复上一动作。
 */
void requireRemoteNeutralRearm();


// ============================================================
// 13. 避障模块接口
// ============================================================
void setupObstacleSensors();

DistanceData makeInvalidDistanceData();
DistanceData readDistanceSensors();

bool isFrontBlocked(const DistanceData& d);
bool isLeftBlocked(const DistanceData& d);
bool isRightBlocked(const DistanceData& d);
bool isRearBlocked(const DistanceData& d);

bool obstacleOverrideRequired(const DistanceData& d);

MotionCommand computeObstacleCommand(
    const DistanceData& d);


// ============================================================
// 14. 自主跟随模块接口
// ============================================================
void setupAutoFollow();

bool parseTargetMessage(
    char* line,
    TargetData& target);

MotionCommand computeAutoFollowCommand(
    const TargetData& target,
    const DistanceData& distance);


// ============================================================
// 15. 安全冗余与电机输出接口
// ============================================================
void setupSafetyRedundancy();

MotionCommand chooseSafeCommand(
    unsigned long now);

MotionCommand makeStopCommand(
    const char* reason);

void applySafeMotionCommand(
    const MotionCommand& cmd);

void stopCar();

#endif