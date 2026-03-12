/**
 * @file em_motor.cpp
 * @brief 舵机驱动核心 + 七段式S曲线轨迹规划 + 前后台解耦架构实现
 *
 * ============================================================================
 * [核心架构演进]: 从裸机强耦合 -> 前后台标志位驱动（Bare-Metal Decoupling）
 * ============================================================================
 *
 * 【数据流转与任务分离】:
 *
 * 1. 异步指令接入层 (Interrupt Context)
 *    - 函数: ble_set_angle_command(), ble_set_move_command()
 *    - 行为: 原址更新单深度命令缓冲区 (ble_cmd)，置位 pending 标志。
 *    - 特性: 时间复杂度 O(1)，无耗时依赖模块调用，确保通讯协议层中断安全。
 *
 * 2. 硬件定时器层 (Hardware Timer ISR Context)
 *    - 函数: motor_timer_callbackfun()
 *    - 行为: 以 15ms 为节拍周期，置位全局同步标志 `flag_15ms_tick`。
 *    - 特性: 极短的执行时间，将耗时计算与硬件中断解耦，保障基础系统时钟稳定。
 *
 * 3. 后台轮询处理层 (Main Loop Context)
 *    - 函数: em_motor_tick_loop()
 *    - 行为: 轮询标志位 `flag_15ms_tick`。触发时：
 *          (a) 通过临界区 (noInterrupts/interrupts) 安全清除标志位。
 *          (b) 检索命令缓冲区：执行上位轨迹规划 (plan_scurve)
 * 及正逆运动学解算。 (c) 离散积分步进：对就绪的任务组，执行 S
 * 曲线的分段积分插补运算 (update_profile)。 (d)
 * 串行日志：输出结构化运动数据用于调试观测。
 *    - 特性: 承担所有重负载浮点运算，运行于非特权线程上下文。
 *
 * 【关键运动学参数】(em_motor_init 中定义):
 * - g_scurve_J     = 20.0~200.0  (deg/s^3)  - 跃度法则限制
 * (用于抵抗系统启动冲击)
 * - g_scurve_Amax  = 40.0~400.0  (deg/s^2)  - 加速度边界约束
 * (用于保证电机不过载失步)
 * - g_scurve_Vmax  = 120.0~200.0 (deg/s)    - 稳态最高巡航速度
 *
 * 更新频率边界：15ms/帧 (对应约 67Hz，针对 MG996R 和 MG90S 最优闭环匹配频率)
 * ============================================================================
 */

#include "em_motor.h" // 包含舵机控制头文件
#include <Ticker.h>   // 包含定时器功能头文件
#include <cstdint>    // 包含整数类型定义
#include <math.h>     // 数学函数：fabs等
#define SERVO_NUM 4   // 舵机数量

// ============ BLE命令缓冲（用于安全处理中断回调）============
typedef struct
{
  uint8_t cmd_type;          // 命令类型：0=空闲，1=角度指令，2=微动指令
  uint8_t angles[SERVO_NUM]; // 缓冲的舵机角度
  uint8_t move_dir[3];       // 缓冲的移动方向(X,Y,Z)
  bool pending;              // 是否有待处理命令
} BleCommandBuffer;

static BleCommandBuffer ble_cmd = {0, {0, 0, 0, 0}, {0, 0, 0}, false};

/**
 * @brief 缓冲绝对角度指令 (执行上下文：中断处理程序)
 *
 * 说明：
 * 采用覆盖策略的单深度缓冲机制。由于外部通讯中断的随机突发性，
 * 此处仅执行最小操作以记录最新数据，将复杂的验证与计算推迟至主循环进行。
 *
 * 此设计可有效充当低通滤波器，滤除频繁冗余指令，避免数据队列积压。
 *
 * @param angles 舵机目标角度数组指针
 */
void ble_set_angle_command(uint8_t *angles)
{
  ble_cmd.cmd_type = 1;
  for (int i = 0; i < SERVO_NUM; i++)
    ble_cmd.angles[i] = angles[i];
  ble_cmd.pending = true; // 拉高信箱标志，随后由 em_motor_tick_loop() 收件
}

/**
 * @brief 缓冲坐标系微动指令（供BLE中断回调安全调用）
 *
 * 将三个方向值复制到命令缓冲区。
 *
 * @param dir 指向三个 uint8_t 方向值的指针 (X,Y,Z: 0/1/255)
 */
void ble_set_move_command(uint8_t *dir)
{
  ble_cmd.cmd_type = 2;
  for (int i = 0; i < 3; i++)
    ble_cmd.move_dir[i] = dir[i];
  ble_cmd.pending = true;
}

/**
 * @brief 异步命令派发器 (执行上下文：后台主循环)
 *
 * 说明：
 * 读取并清空全局命令缓冲区 `ble_cmd`。
 * 根据指令类型转交对应的动作处理接口：绝对角度规划 (Type 1)
 * 或是 笛卡尔坐标方向标定 (Type 2)。
 */
static void process_ble_command()
{
  if (!ble_cmd.pending)
    return;

  if (ble_cmd.cmd_type == 1)
  {
    em_motor_run(ble_cmd.angles); // 角度指令 → S曲线规划
  }
  else if (ble_cmd.cmd_type == 2)
  {
    alg_set_move_action(ble_cmd.move_dir); // 微动指令 → 设置方向
  }

  ble_cmd.pending = false;
  ble_cmd.cmd_type = 0;

  // 状态变更后立即打印监控信息
  extern void em_monitor_dump();
  em_monitor_dump();
}
// =======================================================

// ============ S曲线数据结构与参数 ============
typedef struct
{
  float start;      // 起始角度
  float end;        // 目标角度
  float diff;       // 目标角度差（目标 - 起始）
  float dir;        // 方向 (+1/-1)
  float J;          // 最大跃度 (deg/s^3)
  float Amax;       // 最大加速度 (deg/s^2)
  float Vmax;       // 最大速度 (deg/s)
  float t[7];       // 七段时长
  float total_time; // 预计算的总时间，避免每次更新都重算
  float curr_t;     // 已经过的时间
  float pos;        // 当前位置
  float vel;        // 当前速度
  float acc;        // 当前加速度
  bool active;      // 是否正在运动
} ScurveProfile;

static ScurveProfile profiles[SERVO_NUM]; // S曲线规划结构体数组

// --------------- 规划函数原型 ---------------
static void plan_scurve(int id, float target);
static float update_profile(int id);

#define RESET_ANGLE 0 // 舵机复位角度

// #define PIN_SERVOA 23
// #define PIN_SERVOB 22
// #define PIN_SERVOC 21
// #define PIN_SERVOG 19

#define PIN_SERVOA 19 // 夹爪
#define PIN_SERVOB 21 // 大臂
#define PIN_SERVOC 22 // 小臂
#define PIN_SERVOG 23 // 底座

typedef struct
{
  Servo servo[SERVO_NUM];      // 舵机对象数组
  float last_angle[SERVO_NUM]; // 上一次的角度，用于控制舵机运动速度
  float curr_speed[SERVO_NUM]; // 当前速度，用于加减速
  float max_step[SERVO_NUM];   // 每个定时周期允许的最大角度变化（与型号相关）
  float accel[SERVO_NUM];      // 角速度加速度，用于生成渐变速度曲线
} t_servo_list;                // 定义舵机列表结构体

t_servo_list list;                    // 定义全局变量，保存舵机对象和运动参数
volatile bool flag_15ms_tick = false; // 定时器触发标志位，用于前后台解耦

Ticker read_state_timer; // 定义定时器对象，用于定时更新舵机状态

/**
 * @brief 符号辅助函数
 *
 * @param v 输入数值
 * @return +1.0f 正数, -1.0f 负数, 0.0f 零
 */
static float sgnf(float v)
{
  if (v > 0)
    return 1.0f;
  if (v < 0)
    return -1.0f;
  return 0.0f;
}
// ====================================

// ============ S曲线规划实现 ============

// S曲线参数（可调）
static float g_scurve_J = 200.0f;    // 跃度 (deg/s^3)
static float g_scurve_Amax = 400.0f; // 最大加速度 (deg/s^2)
static float g_scurve_Vmax = 200.0f; // 最大速度 (deg/s)
/**
 * @brief 单轴 S 曲线参数规划 (执行上下文：后台主循环)
 *
 * 计算原理：
 * 根据系统整定的跃度限制 (J)、全局最大约束加速度 (Amax) 与 最大约束速度
 * (Vmax)， 建立起始角至目标角的七段分段多项式模型 (Seven-Segment S-Curve
 * Model)。
 *
 * 预计算产出：各参数区间的转换边界时间节点 `t[0...6]`。
 *
 * @param id 舵机索引
 * @param target 目标绝对角度
 */
static void plan_scurve(int id, float target)
{
  float start = profiles[id].pos = list.last_angle[id]; // 从当前角度开始规划
  float diff = target - start;                          // 计算总的角度差

  // 如果已经接近目标，则直接设置为目标并结束
  if (fabs(diff) < 0.01f)
  {
    profiles[id].active = false;
    profiles[id].pos = target;
    return;
  }

  // 记录规划参数
  profiles[id].start = start;    // 记录起始角度
  profiles[id].end = target;     // 记录目标角度
  profiles[id].diff = diff;      // 计算总的角度差
  profiles[id].dir = sgnf(diff); // 计算运动方向

  // 参考参数，由于电机扭矩和机械臂负载，此处参数直接影响顺滑度
  // 必须大幅降低跃度(J)和最大加速度(Amax)，否则在短距离移动中 S
  // 曲线起步段在零点几毫秒内就完成了，
  // 从宏观数据上看就会退化成和纯线性算法一样的方波。
  profiles[id].J = 30.0f;     // deg/s^3 (原为200)
  profiles[id].Amax = 40.0f;  // deg/s^2 (原为400)
  profiles[id].Vmax = 120.0f; // deg/s   (原为200)

  float D = fabs(diff);
  float J = profiles[id].J;
  float Amax = profiles[id].Amax;
  float Vmax = profiles[id].Vmax;

  // 根据最大跃度和加速度计算理论能达到的最大速度边界
  float V_Amax = Amax * Amax / J;
  float A_sys = Amax;
  float V_sys = Vmax;

  if (Vmax < V_Amax)
  {
    A_sys = sqrtf(Vmax * J); // 必须降低加速度以保证在达到Vmax前jerk平滑过渡
  }

  float t_j = A_sys / J;
  float t_a = (V_sys - A_sys * t_j) / A_sys;
  float D_req = V_sys * (2.0f * t_j + t_a); // 完整的加速+减速所需位移

  // 如果距离太短，无法达到目标速度 V_sys
  if (D < D_req)
  {
    float D_A = 2.0f * (A_sys * A_sys * A_sys) / (J * J);
    if (D < D_A)
    {
      // 距离极短，只有加速跃度和减速跃度
      t_j = powf(D / (2.0f * J), 1.0f / 3.0f);
      t_a = 0;
    }
    else
    {
      // 能达到A_sys，但不能达到V_sys，需要计算新的t_a
      float b = 3.0f * t_j;
      float c = 2.0f * t_j * t_j - D / A_sys;
      float delta = b * b - 4.0f * c;
      if (delta < 0.0f)
        delta = 0.0f; // ★ 添加这行：防止因为浮点误差导致负数，算出 NaN
                      // 向后传递毁坏全部数字
      t_a = (-b + sqrtf(delta)) / 2.0f;
    }
    V_sys = A_sys * (t_j + t_a);
    D_req = D;
  }

  float t_v = 0;
  if (V_sys > 0.001f)
  {
    t_v = (D - D_req) / V_sys;
    if (t_v < 0)
      t_v = 0;
  }

  // 设置七段时长
  profiles[id].t[0] = t_j;
  profiles[id].t[1] = t_a;
  profiles[id].t[2] = t_j;
  profiles[id].t[3] = t_v;
  profiles[id].t[4] = t_j;
  profiles[id].t[5] = t_a;
  profiles[id].t[6] = t_j;
  // 预计算总时间
  profiles[id].total_time = 4.0f * t_j + 2.0f * t_a + t_v;
  // 重置当前时间和状态
  profiles[id].curr_t = 0;
  profiles[id].vel = 0;
  profiles[id].acc = 0;
  profiles[id].active = true;
}

/**
 * @brief 离散积分步进与插补更新 (执行上下文：后台主循环)
 *
 * 计算原理：
 * 基于时域增量 `step_dt` 对已激活的七段轨迹 `ScurveProfile` 执行递推。
 * - 层级积分流程：跃度查表 -> 加速度欧拉积分 -> 速度二次积分 -> 位移三次积分。
 *
 * @param id 舵机索引
 * @return float 离散计算生成的瞬时规划角度输出
 */
static float update_profile(int id)
{ // 如果当前没有活动的轨迹，则直接返回最后的角度值
  if (!profiles[id].active)
    return list.last_angle[id];
  // 计算当前时间点所在的轨迹段和对应的跃度
  float remaining_dt = 0.015f; // 15ms timer period
                               // 通过循环查找当前时间点所在的段，并计算该段剩余的时间
  while (remaining_dt > 0.0001f &&
         profiles[id].curr_t < profiles[id].total_time)
  {
    float t = profiles[id].curr_t;
    float cumulative = 0;
    float jerk = 0;
    float time_left_in_seg = 0;

    // 查找当前所处的段，并计算该段剩余的时间
    for (int seg = 0; seg < 7; seg++)
    {
      float seglen = profiles[id].t[seg];
      if (t < cumulative + seglen)
      {
        time_left_in_seg = (cumulative + seglen) - t;
        switch (seg)
        {
        case 0:
          jerk = profiles[id].J * profiles[id].dir; // 加速跃度
          break;
        case 1:
          jerk = 0; // 匀加速段无跃度
          break;
        case 2:
          jerk = -profiles[id].J * profiles[id].dir; // 减速跃度
          break;
        case 3:
          jerk = 0; // 匀速段无跃度
          break;
        case 4:
          jerk = -profiles[id].J * profiles[id].dir; // 加速跃度（反向）
          break;
        case 5:
          jerk = 0; // 匀加速段无跃度
          break;
        case 6:
          jerk = profiles[id].J * profiles[id].dir; // 减速跃度（反向）
          break;
        }
        break;
      }
      cumulative += seglen; // 累加前面段的时长
    }

    // 避免因为浮点数精度截断导致永远无法计算
    if (time_left_in_seg <= 0.0001f)
    {
      time_left_in_seg = remaining_dt;
    }

    // 决定当前这一小步的积分步长
    float step_dt = remaining_dt;
    if (step_dt > time_left_in_seg)
    {
      step_dt = time_left_in_seg;
    }

    // 执行精确数学积分：
    // V = V0 + A0*t + 0.5*J*t^2
    // P = P0 + V0*t + 0.5*A0*t^2 + (1/6)*J*t^3
    float prev_acc = profiles[id].acc;
    float prev_vel = profiles[id].vel;

    profiles[id].acc = prev_acc + jerk * step_dt;
    profiles[id].vel =
        prev_vel + prev_acc * step_dt + 0.5f * jerk * step_dt * step_dt;
    profiles[id].pos += prev_vel * step_dt +
                        0.5f * prev_acc * step_dt * step_dt +
                        (1.0f / 6.0f) * jerk * step_dt * step_dt * step_dt;

    profiles[id].curr_t += step_dt;
    remaining_dt -= step_dt;
  }

  // 终点检查与微小误差对齐
  if (profiles[id].curr_t >= profiles[id].total_time - 0.001f)
  {
    profiles[id].pos = profiles[id].end;
    profiles[id].vel = 0;
    profiles[id].acc = 0;
    profiles[id].active = false;
  }

  return profiles[id].pos;
}

// 已废弃：旧的速度控制函数（已被S曲线规划替代）
// static float em_motor_speed_ctl_run(...) // 不再使用
// =================================

// ============ 定时器回调 ============

/**
 * @brief 硬件定时器中断服务函数 (ISR)
 *
 * 说明：负责定频置位任务同步信号 `flag_15ms_tick`，执行时间维持在微秒级别。
 */
static void motor_timer_callbackfun() { flag_15ms_tick = true; }

/**
 * @brief 核心业务轮询调度任务 (Main Loop Handler)
 *
 * 说明：
 * 采用关中断的方式提取硬件标志位触发，保证访问原子性。
 * 执行序列包含：状态派发、模型数值积分计算、驱动层 PWM
 * 指令分发、控制台诊断日志输出。
 */
void em_motor_tick_loop()
{
  if (flag_15ms_tick)
  {
    // 关中断，清除标志位，防止并发冲突
    noInterrupts();
    flag_15ms_tick = false;
    interrupts();

    // 处理BLE缓冲命令（从中断上下文安全转移到后台上下文）
    process_ble_command();

    bool is_moving = false;
    // 更新每个舵机的位置（根据S曲线轨迹）
    for (int index = 0; index < SERVO_NUM; index++)
    {
      float next = update_profile(index);
      list.last_angle[index] = next;
      list.servo[index].write(next);
      if (profiles[index].active)
      {
        is_moving = true; // S曲线仍在规划/运动中
      }
    }
    // 仅执行一次位置算法（处理坐标系动作，不应每个舵机都执行一次）
    alg_move_run();

    // 记录上一个周期的运动状态，用于判断边缘
    static bool prev_moving = false;

    // ----- 当处于运动状态时，记录所有轴的数据以便后续画图 -----
    // 数据格式: "时间(ms), A轴目标, A轴实际, B轴目标, B轴实际, C轴目标,
    // C轴实际, 夹爪目标, 夹爪实际\n"
    if (is_moving)
    {
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(),
                    profiles[0].end, profiles[0].pos, profiles[1].end,
                    profiles[1].pos, profiles[2].end, profiles[2].pos,
                    profiles[3].end, profiles[3].pos);
    }
    else if (prev_moving == true)
    {
      // 当从运动状态刚刚变为停止时，打印一个分隔符
      Serial.println("================_END_OF_MOVEMENT_================");
    }

    prev_moving = is_moving;
  }
}
// ===============================

// ============ 外部控制接口 ============

/**
 * @brief 检查角度数组是否全部在合法范围内
 *
 * @param angle 指向四个 float 角度值的数组
 * @return true 所有角度合法
 * @return false 存在超范围角度
 */
static bool check_angle(float *angle)
{
  static const float angle_min[SERVO_NUM] = {SERVO0_MIN, SERVO1_MIN, SERVO2_MIN,
                                             SERVO3_MIN};
  static const float angle_max[SERVO_NUM] = {SERVO0_MAX, SERVO1_MAX, SERVO2_MAX,
                                             SERVO3_MAX};
  for (int index = 0; index < SERVO_NUM; index++)
  {
    if (angle[index] < angle_min[index] || angle[index] > angle_max[index])
      return false;
  }
  return true;
}

/**
 * @brief 根据 uint8_t 角度数组驱动舵机（BLE指令入口）
 *
 * 转换为 float 后检查角度合法性，然后为四个舵机
 * 各规划一条S曲线轨迹，并调用正运动学更新末端坐标。
 *
 * @param angle 指向四个 uint8_t 角度值的指针
 */
void em_motor_run(uint8_t *angle)
{
  float fangle[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; i++)
    fangle[i] = angle[i]; // uint8_t → float 转换

  if (check_angle(fangle) == false)
    return; // 角度超范围，拒绝执行

  // 为每个舵机规划S曲线轨迹
  for (int index = 0; index < SERVO_NUM; index++)
  {
    plan_scurve(index, fangle[index]);
  }
  // 正运动学：记录目标位置对应的末端坐标
  alg_positive_operation(fangle[0], fangle[1], fangle[2]);

  Serial.printf("动作反馈：A轴=%.1f° B轴=%.1f° C轴=%.1f°\n", fangle[2],
                fangle[0], fangle[1]);
  Serial.flush();
}

/**
 * @brief 根据 float 角度驱动舵机（代码直接调用版本）
 *
 * 与 em_motor_run() 功能相同，但接受 float 参数，
 * 不输出串口反馈（避免高频调用时缓冲溢出）。
 *
 * @param angle1 舵机0目标角度
 * @param angle2 舵机1目标角度
 * @param angle3 舵机2目标角度
 * @param angle4 舵机3目标角度
 */
void em_motor_run_by_angle(float angle1, float angle2, float angle3,
                           float angle4)
{
  float angles[SERVO_NUM] = {angle1, angle2, angle3, angle4};
  if (check_angle(angles) == false)
    return; // 角度超范围，拒绝执行

  // 规划各舵机的S曲线
  plan_scurve(0, angle1);
  plan_scurve(1, angle2);
  plan_scurve(2, angle3);
  plan_scurve(3, angle4);

  // 更新正运动学坐标
  alg_positive_operation(angle1, angle2, angle3);
}

/**
 * @brief 微动专用：直接设置前3个舵机的目标角度
 *
 *
 * 优化说明（Bug修复）：
 * 1. 不调用 plan_scurve()，避免每15ms重置速度导致微动卡顿（解决Bug1）
 * 2. 不操作舵机3（夹爪），微动时夹爪保持当前状态（解决Bug2：强制归零）
 * 3. 直接更新 last_angle 和停止 S曲线 active 标志，
 *    让定时器的 servo.write() 直接跟踪新位置
 *
 * 因为微动每tick仅偏移0.4°，步进足够小，无需S曲线加减速
 */
void em_motor_set_target_direct(float angle1, float angle2, float angle3)
{
  float angles[3] = {angle1, angle2, angle3};

  // 边界检测（仅检查前3个轴）
  static const float angle_min[3] = {SERVO0_MIN, SERVO1_MIN, SERVO2_MIN};
  static const float angle_max[3] = {SERVO0_MAX, SERVO1_MAX, SERVO2_MAX};
  for (int i = 0; i < 3; i++)
  {
    if (angles[i] < angle_min[i] || angles[i] > angle_max[i])
      return;
  }

  // 停止这3个轴的S曲线（如果有正在执行的），直接跟踪新位置
  for (int i = 0; i < 3; i++)
  {
    profiles[i].active = false;
    profiles[i].pos = angles[i];
    profiles[i].vel = 0;
    profiles[i].acc = 0;
    list.last_angle[i] = angles[i];
  }
  // 舵机3（夹爪）完全不动，保持 list.last_angle[3] 不变

  // 更新正运动学坐标
  alg_positive_operation(angle1, angle2, angle3);
}
// ================================

// ============ 初始化 ============

/**
 * @brief 舵机初始化
 *
 * 执行以下初始化步骤：
 * 1. 绑定四个舵机到GPIO引脚 (PWM 500~2500us)
 * 2. 设置每个舵机的速度参数（MG90S/MG996R）
 * 3. 初始化S曲线参数 (J/Amax/Vmax)
 * 4. 启动15ms定时器 (作为前台的独立时间节拍器，不含耗时计算)
 * 5. 设置初始角度并计算原点坐标
 */
void em_motor_init()
{
  // 例如，如果范围是500us到2000us，
  // 500us等于0的角，1500us等于90度，2500us等于1800 度。
  list.servo[0].attach(PIN_SERVOA, 500, 2500);
  list.servo[1].attach(PIN_SERVOB, 500, 2500);
  list.servo[2].attach(PIN_SERVOC, 500, 2500);
  list.servo[3].attach(PIN_SERVOG, 500, 2500);
  // 设置每个舵机的最大步进和加速度（MG90S 用于夹爪，其余为 MG996R）
  const float MG996R_MAX = 2.0f;   // 每15ms最多移动 2°
  const float MG996R_ACCEL = 0.1f; // 速度变化每周期增加 0.1°/拍
  const float MG90S_MAX = 1.0f;    // 更小步进，夹爪动作更细腻
  const float MG90S_ACCEL = 0.05f; // 夹爪加速度也较小

  for (int index = 0; index < SERVO_NUM; index++)
  {
    list.curr_speed[index] = 0;
    if (index == 0) // 夹爪为 MG90S
    {
      list.max_step[index] = MG90S_MAX;
      list.accel[index] = MG90S_ACCEL;
    }
    else
    {
      list.max_step[index] = MG996R_MAX;
      list.accel[index] = MG996R_ACCEL;
    }
  }

  // 初始化S曲线参数
  g_scurve_J = 200.0f;
  g_scurve_Amax = 400.0f;
  g_scurve_Vmax = 200.0f;

  // 启动定时器（15ms）：使用S曲线更新舵机位置 + 运行运动学计算
  read_state_timer.attach_ms(15, motor_timer_callbackfun);
  // 初始化位置
  for (int index = 0; index < SERVO_NUM; index++)
  {
    // 设置当前角度，因为舵机上电时无法知道自身角度，所以这里假设角度为原点+1
    // 这就要求我们上电前把机械臂调整到安装的原点角度，也就是每个轴都在0°附近
    // 否则会导致机械臂第一次控制运动非常快
    list.last_angle[index] = RESET_ANGLE + 1;
    profiles[index].pos = list.last_angle[index];
    profiles[index].active = false;
    if (index == 0)
    {
      list.last_angle[index] = 89;
      profiles[index].pos = 89;
    }
    if (index == 2)
    {
      list.last_angle[index] = 179;
      profiles[index].pos = 179;
    }
  }
  // 初始化原点角度，计算原点绝对坐标 absoluteX、absoluteY、absoluteZ
  alg_positive_operation(90, 0, 180);
}
// =============================
