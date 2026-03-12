/**
 * @file em_monitor.cpp
 * @brief 运行状态监控实现
 *
 * 该模块提供串口状态输出，帮助验证系统各环节是否正常。
 * 输出分为多行包含键值对信息，帮助展示关键变量的当前值。
 *
 * 使用方式：
 *   - 调用 em_monitor_init() 启动定时输出（默认10秒一次）
 *   - 任意时刻调用 em_monitor_dump() 立即打印一行状态
 *
 * BLE 命令处理流程中已自动调用 em_monitor_dump()，因此每次状态
 * 变更均会尽快在串口反映。
 */

#include "em_monitor.h"
#include "hal/em_alg.h"   // 提供 moveX/Y/Z, absoluteX/Y/Z 等全局变量
#include "hal/em_ble.h"   // 提供 get_ble_connect() 蓝牙连接状态
#include "hal/em_motor.h" // 提供舵机列表状态
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ticker.h> // 周期性定时器辅助库

// ============ 外部引用声明 ============
// 重新声明 em_motor 中的舵机列表结构（与 em_motor.cpp 内部定义一致）
typedef struct
{
  Servo servo[4];
  float last_angle[4];
  float curr_speed[4];
  float max_step[4];
  float accel[4];
} t_servo_list;

// 声明 em_motor 中定义的全局变量（在 em_motor.cpp 中定义）
extern t_servo_list list;

// 来自 em_alg.cpp 的全局变量（在 em_alg.cpp 中定义）
extern int moveX;
extern int moveY;
extern int moveZ;
extern float angleA;
extern float angleB;
extern float angleC;
extern float absoluteX;
extern float absoluteY;
extern float absoluteZ;

// 此模块只有一个文件 em_monitor，可在程序任何地方调用 em_monitor_dump()
// 来触发立即打印。定时器间隔设为 10 秒，主要用于周期性心跳。
static Ticker monitorTicker;
// ================================

// ============ 初始化 ============

// 记录上一组状态，用于比对是否发生变化
// 定义命名结构体以支持简单赋值
typedef struct
{
  unsigned long t;
  size_t freeHeap;
  bool ble;
  int moveX, moveY, moveZ;
  float absX, absY, absZ;
  float angA, angB, angC;
  float servo[4];
} MonitorState;
static MonitorState prev = {0};

/**
 * @brief 初始化监控模块
 *
 * 启动10秒周期的Ticker定时器，定时调用 em_monitor_dump() 打印系统状态。
 * 在 setup() 中手动调用，位于 em_motor_init() 之后。
 */
void em_monitor_init()
{
  // 周期打印（毫秒），根据需要可调
  monitorTicker.attach_ms(10000, em_monitor_dump);
}
// =============================

// ============ 状态输出 ============

/**
 * 打印当前系统状态到串口
 *
 * 输出格式分为四行:
 * 1. t, freeHeap, ble, moveX, moveY, moveZ
 * 2. coord (absX, absY, absZ)
 * 3. ik_angle (angA, angB, angC)
 * 4. servos (s0, s1, s2, s3)
 *
 * 该函数可以被周期定时器或外部事件调用，确保在状态变化时
 * 尽快记录当前值。
 */
void em_monitor_dump()
{
  unsigned long t = millis();
  size_t freeHeap = ESP.getFreeHeap();
  bool ble = get_ble_connect();

  // 收集当前状态
  MonitorState cur;

  cur.t = t;
  cur.freeHeap = freeHeap;
  cur.ble = ble;
  cur.moveX = moveX;
  cur.moveY = moveY;
  cur.moveZ = moveZ;
  cur.absX = absoluteX;
  cur.absY = absoluteY;
  cur.absZ = absoluteZ;
  cur.angA = angleA;
  cur.angB = angleB;
  cur.angC = angleC;
  for (int i = 0; i < 4; i++)
    cur.servo[i] = list.last_angle[i];

  // 移除 "仅状态变化才打印" 的限制。
  // 这样只要你点击了按钮（收到BLE指令），就一定会触发打印，不会再觉得没反应。

  // 使用全英文排版输出，彻底解决在某些串口助手（特别是 Windows
  // 环境下）导致中文乱码问题。
  Serial.printf("\n============= System Status =============\n");
  Serial.printf("[ SYS ] Uptime : %lu ms | FreeHeap: %u B\n", t,
                (unsigned)freeHeap);
  Serial.printf("[ BLE ] Link   : %s\n", ble ? "Connected" : "Disconnected");
  Serial.printf("[ CTL ] Move   : X=%d  Y=%d  Z=%d\n", cur.moveX, cur.moveY,
                cur.moveZ);
  Serial.printf("[ POS ] End Pos: X=%.1f  Y=%.1f  Z=%.1f\n", cur.absX, cur.absY,
                cur.absZ);
  Serial.printf("[ALG_I] Angles : A=%.1f  B=%.1f  C=%.1f\n", cur.angA, cur.angB,
                cur.angC);
  Serial.printf("[SERVO] Actual : S0=%.1f  S1=%.1f  S2=%.1f  S3=%.1f\n",
                cur.servo[0], cur.servo[1], cur.servo[2], cur.servo[3]);
  Serial.printf("=========================================\n");

  // 更新上一次状态记录
  prev = cur;
}
// ==============================
