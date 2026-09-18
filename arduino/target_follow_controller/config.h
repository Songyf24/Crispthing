#ifndef TARGET_FOLLOW_CONFIG_H
#define TARGET_FOLLOW_CONFIG_H

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ============================================================
// 1. 功能开关
// ============================================================
// 下地低速联调：允许电机运行。
// 上传前必须确认车头传感器、停车阈值和物理断电手段均可用。
#define MOTOR_ENABLED 1

// 总传感器开关。
// 未来：等待四路传感器接入 STM32。
// Arduino UNO 直接读取单个前向超声波传感器。
#define SENSOR_ENABLED 1

// 四个方向必须独立启用。STM32 尚未接入时，只启用车头传感器。
// 这样可避免初始化未接线的引脚，也可避免 D12 与电机方向引脚冲突。
#define ULTRA_LEFT_ENABLED  0
#define ULTRA_FRONT_ENABLED 1
#define ULTRA_RIGHT_ENABLED 0
#define ULTRA_REAR_ENABLED  0

#define TOF_ENABLED 0

// 下地联调阶段保留测距日志；稳定后可改为 0，减少串口输出。
#define ULTRASONIC_TEST_PRINT_ENABLED 1

// 急停：A3 接 NC，COM 接 GND
#define EMERGENCY_STOP_ENABLED 0

// USB 手柄转接板通过 I2C 通信
#define I2C_REMOTE_ENABLED 1

// 串口调试输出
#define DEBUG_PRINT 1

// 电池电压监测：总开关到货并完成接线后再改为 1
#define BATTERY_MONITOR_ENABLED 0

// ============================================================
// 2. 电机与履带参数
// ============================================================
//
// 当前左右通道已经按照你的实车结果交换：
//
// M2 左侧履带：PWM=D6，方向=D12/D13
// M1 右侧履带：PWM=D5，方向=D7/D8
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
#define TURN_SIGN -1


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
// 当前只启用车头：商家扩展板已将 TRIG/ECHO 接到 A4/A5，但考虑到要同时接手柄，改成A0, A1
// 其余方向留作 STM32 到货后的接口定义，不会被初始化或读取。
//
const int ULTRA_LEFT_TRIG  = 2;
const int ULTRA_LEFT_ECHO  = 3;

// 车头超声波接口：A0/A1
const int ULTRA_FRONT_TRIG = A0;
const int ULTRA_FRONT_ECHO = A1;

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

// 电压检测模块 OUT 接 A0
const int BATTERY_VOLTAGE_PIN = A0;

// 模块将输入电压缩小为 1/5
const float BATTERY_DIVIDER_RATIO = 5.0f;

// 万用表校准前保持 1.0
const float BATTERY_CALIBRATION = 1.0f;

// 每 200 ms 采样一次
const unsigned long BATTERY_SAMPLE_INTERVAL_MS = 200;

// ============================================================
// 6. 自主跟随参数
// ============================================================

// 指定跟随者的最低相似度
const float SIMILARITY_MIN = 0.50f;

// OpenBot 手机端已先完成连续 3 帧确认；
// Arduino 再确认 2 条序号不同的有效 TARGET 后才允许开始前进。
const uint8_t FAR_TARGET_CONFIRM_FRAMES = 2;

/*
 * 相对距离：
 * relativeDistance = 注册时肩髋尺度 / 当前肩髋尺度
 *
 * < 1：目标比注册位置近
 * ≈ 1：目标在注册位置附近
 * > 1：目标比注册位置远
 * -1 ：当前距离数据无效
 */

// 相对距离不超过 1.12 时，不再向前运动
const float FOLLOW_STOP_RELATIVE_DISTANCE = 1.12f;

/*
 * 停车后，目标必须重新远到 1.16，才开始累计 Arduino 端的
 * 2 条确认帧。1.12～1.16 是滞回区间，用于避免距离估计在
 * 停止阈值附近抖动时反复启停。
 */
const float FOLLOW_RESTART_RELATIVE_DISTANCE = 1.22f;

// 相对距离达到 1.80 后，允许使用最大跟随速度
const float FOLLOW_FULL_SPEED_RELATIVE_DISTANCE = 1.80f;

// ============================================================
// 7. 速度与转向参数
// ============================================================
//
// 当前仍处于架空调试阶段，最高速度先限制为 90。
// 后续稳定后可逐步提高，但不要直接恢复到很高速度。
//
const int MIN_SPEED = 80;
const int MAX_SPEED = 90;

// 电机最终输出的安全上限，与普通跟随最高速度分开
const int MOTOR_PWM_LIMIT = 90;

// 非线性 P 转向的最大修正量
const float K_TURN = 85.0f;

// 非线性指数：
// > 1 时，小偏差修正柔和，大偏差修正迅速增强
const float TURN_NONLINEAR_EXPONENT = 1.6f;

/*
 * 普通跟随转向滞回：
 * 尚未转向时，偏差达到 0.12 才启动转向；
 * 已经转向后，偏差降到 0.06 才停止转向。
 *
 * 0.06～0.12 内保持之前的转向状态，
 * 避免人体关键点在中心附近抖动时频繁左右切换。
 */
const float TURN_START_X_THRESHOLD = 0.12f;
const float TURN_STOP_X_THRESHOLD  = 0.06f;

// ============================================================
// 近距离原地对准参数
// ============================================================

// 尚未进入对准时，水平偏差达到 0.15 才开始累计确认帧。
const float ALIGN_START_X_THRESHOLD = 0.15f;

// 已进入对准后，偏差降到 0.08 才退出，形成启动/停止滞回。
const float ALIGN_STOP_X_THRESHOLD = 0.08f;

// 必须连续收到两条序号不同、且偏转方向一致的 TARGET 才启动。
const uint8_t ALIGN_CONFIRM_FRAMES = 2;

// 原地对准的最低、最高 PWM
const int MIN_ALIGN_TURN_SPEED = 60;
const int MAX_ALIGN_TURN_SPEED = 70;

// 水平偏差达到 0.30 时使用最高原地转向速度
const float ALIGN_FULL_SPEED_X_THRESHOLD = 0.30f;

// 原地对准减速曲线指数。
// > 1 时，接近中心后的降速更加明显。
const float ALIGN_TURN_NONLINEAR_EXPONENT = 1.5f;

// ============================================================
// 电机输出平滑
// ============================================================

// 是否启用电机速度斜坡
#define MOTOR_RAMP_ENABLED 1

/*
 * 每个控制周期最多变化的 PWM 数值。
 *
 * 当前控制周期为 20 ms：
 * 0 → 70 约需 14 个周期，即约 280 ms
 * 0 → 90 约需 18 个周期，即约 360 ms
 */
const int MOTOR_RAMP_STEP = 5;

// ============================================================
// 8. 避障参数
// ============================================================
// 下地初测保留较大的制动余量：连续两帧不超过 45 cm 即停车。
const float ULTRA_FRONT_SAFE_CM = 45.0f;

// 停车后必须连续测到至少 50 cm 才能解除，形成 5 cm 滞回区。
const float ULTRA_FRONT_RELEASE_CM = 50.0f;

const float ULTRA_SIDE_SAFE_CM  = 35.0f;
const float ULTRA_REAR_SAFE_CM  = 35.0f;

// JSN-SR04T 常见近距离盲区约 20 cm；低于此值仍按“太近”处理，
// 但串口会标成 BELOW_MIN，提醒该数值本身不宜当作精确距离。
const float ULTRA_MIN_RELIABLE_CM = 20.0f;
const float ULTRA_MAX_RELIABLE_CM = 500.0f;

// 单个方向每 80 ms 采样一次。两帧确认后，典型判定延迟约 160 ms。
const unsigned long ULTRA_SAMPLE_INTERVAL_MS = 80;

// 20 ms 约对应 3.4 m 单程测距，足够覆盖当前室内下地测试范围，
// 同时减少无回波时 pulseIn 对串口和跟随控制循环的阻塞。
const unsigned long ULTRA_ECHO_TIMEOUT_US = 20000;

// 停车要求连续 2 帧，兼顾响应速度与毛刺过滤；
// 解除要求连续 3 帧，避免障碍边缘晃动时过早重新起步。
const uint8_t ULTRA_BLOCK_CONFIRM_SAMPLES = 2;
const uint8_t ULTRA_RELEASE_CONFIRM_SAMPLES = 3;

const float TOF_TOO_CLOSE_CM = 80.0f;
const float TOF_FAR_CM       = 180.0f;


// ============================================================
// 9. 超时与控制周期
// ============================================================

// OpenBot 目标数据超时
const unsigned long TARGET_TIMEOUT_MS = 1000;

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
  unsigned long sequence;       // 手机端消息序号
  bool valid;                   // 目标是否有效
  float xError;                 // 水平偏差，范围 -1～1

  float relativeDistance;       // 注册尺度 / 当前尺度
                                // >1 更远，<1 更近，-1 无效

  float similarity;             // 身份相似度，范围 0～1
  unsigned long receivedAt;     // Arduino 收到数据的时间
};

struct DistanceData {
  float ultraLeftCm;
  float ultraFrontCm;
  float ultraRightCm;
  float ultraRearCm;
  float tofFrontCm;
  unsigned long sampledAt;
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

// CMD,序号,STOP 触发的停车锁定
extern bool commandStopActive;

// CMD,序号,ESTOP 触发的软件急停锁定
extern bool softwareEmergencyActive;

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

void resetAutoFollowConfirmation();

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