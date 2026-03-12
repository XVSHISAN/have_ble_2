#ifndef _EM_MOTOR_H_
#define _EM_MOTOR_H_

#include <Arduino.h>
#include <ESP32Servo.h>

#include "em_alg.h"
#include "em_config.h"

/**
 * @file em_motor.h
 * @brief 舵机控制模块头文件 - 提供S曲线规划接口和低层驱动
 *
 * ============ 子模块职责 ============
 *
 * 1. 舵机驱动 (servo[].write)
 *    └─ 直接输出PWM信号给舵机，无轨迹平滑
 *
 * 2. 轨迹规划核心 (plan_scurve + update_profile)
 *    ├─ plan_scurve()      上位规划：给定当前角度和目标角度，
 *    │                     一次性计算7段轨迹的分段时间和参数
 *    └─ update_profile()   离散执行：每15ms调用一次，积分
 *                          jerk→加速度→速度→位置，驱动舵机光滑运动
 *
 * 3. BLE命令缓冲 (ble_set_angle_command / ble_set_move_command)
 *    ├─ BLE中断context调用：缓存8字节或6字节数据，立即返回
 *    └─ timer context中：process_ble_command() 解析并执行规划
 *
 * 4. 前后台解耦调度 (motor_timer_callbackfun + em_motor_tick_loop)
 *    ├─ motor_timer_callbackfun(): 15ms硬件中断前台，仅拉高标志位，极速退出
 *    └─ em_motor_tick_loop(): 主循环后台，轮询标志位并执行沉重负载：
 *       (1) 处理BLE缓冲指令
 *       (2) 执行4个舵机的S曲线离散步进
 *       (3) 触发坐标系微动运动学计算 (alg_move_run)
 *
 * ============ 全局变量说明 ============
 *
 * 舵机状态：
 *   float list.last_angle[4]         当前舵机角度(度)
 *   float list.curr_speed[4]          当前速度(度/s)，实时更新
 *
 * S曲线参数（可调）：
 *   float g_scurve_J    = 200.0      加加度 (deg/s³) 控制平稳性
 *   float g_scurve_Amax = 400.0      最大加速度 (deg/s²) 控制快慢
 *   float g_scurve_Vmax = 200.0      最大速度 (deg/s) 限制峰值速度
 *
 *   说明：J越大，加减速阶段越短，但可能有冲击感
 *        Amax越大，加速越快
 *        Vmax越大，整体速度越快
 *        对不同舵机类型可通过调整这三个值优化性能
 *
 * 坐标系微动：
 *   float moveX, moveY, moveZ        当前坐标系位移方向(-1/0/1)
 *   float absoluteX, absoluteY, absoluteZ  末端当前坐标(mm)
 *
 * ============ 舵机配置 ============
 *
 * GPIO 19: 夹爪 (MG90S, 小舵机，响应快，<0.12us)
 * GPIO 21: 肩关节 (MG996R, 大力矩，<0.2us)
 * GPIO 22: 肘关节 (MG996R, 大力矩，<0.2us)
 * GPIO 23: 底座转台 (MG996R, 大力矩，<0.2us)
 *
 * 角度范围：
 *   底座(servo3): 0 ~ 180°
 *   肩关节(servo1): 0 ~ 85°
 *   肘关节(servo2): 140-servo1 ~ 196-servo1 (取决于肩关节角度)
 *   夹爪(servo0): 0 ~ 180° (0闭合, 180张开)
 *
 * ============ S曲线工作流程 ============
 *
 * em_motor_run(uint8_t *angle):       主入口：舵机角度指令
 *   ├─ check_angle()                  角度范围检查
 *   └─ plan_scurve() ×4               为四个舵机各规划一条7段S曲线
 *      ├─ 计算：总里程 = 目标角度 - 当前角度
 *      ├─ 阶段1: jerk从0加到J (加速阶段1)
 *      ├─ 阶段2: jerk从J降到0 (加速阶段2，加速度达到Amax)
 *      ├─ 阶段3: jerk为0 (等速加速)
 *      ├─ 阶段4-6: 对称的减速三段 (jerk反向)
 *      ├─ 阶段7: 到达目标角度
 *      └─ 计算每段的持续时间和最终速度
 *
 * 然后：主循环 em_motor_tick_loop() 每15ms轮询执行
 *   ├─ update_profile() ×4             四个舵机各执行一步离散积分
 *   │  ├─ 当前段确定：找到时间所在的7段中哪一段
 *   │  ├─ jerk值：根据段号决定 (给定方向下的±J)
 *   │  ├─ 积分链：acc += jerk*dt, vel += acc*dt, pos += vel*dt
 *   │  └─ 输出PWM给舵机 (servo[].write)
 *   └─ 到达目标时：标记profile.active=false，S曲线完成
 *
 * ============ 控制接口 ============
 *
 * 1. 角度直接控制：
 *    em_motor_run_by_angle(a, b, c, d)
 *      ├─ 给定四个舵机的目标角度
 *      └─ 通过S曲线自动平滑过渡
 *
 *    em_motor_set_target_direct(a, b, c)
 *      ├─ 专用于微动控制的高频调用接口
 *      ├─ 直接设定前三轴目标位置，跳过S曲线耗时及其导致的速度重置
 *      └─ 保护第四轴（夹爪）状态不被覆盖归零
 *
 * 2. 坐标系微动标记：
 *    alg_set_move_action(data)
 *      ├─ 解析BLE方向数据，标记坐标系移动方向
 *      └─ 由 alg_move_run() 在定时器中处理
 *
 * 3. BLE指令缓冲（中断安全）：
 *    ble_set_angle_command(data)       缓冲角度指令
 *    ble_set_move_command(data)        缓冲坐标微动指令
 *    process_ble_command()             定时器context中执行
 */

void em_motor_init(); // 舵机初始化

// 外部接口仍支持 uint8_t 数据，内部会转换为
// float，并通过七段式S形算法进行上位规划
// 每给出一个新的角度目标，系统会计算完整的速度/加速度/跃度曲线，随后在定时器中离散下发给舵机
void em_motor_run(uint8_t *angle); // 根据角度运行舵机

// 供算法模块调用的版本，可直接传入 float 角度（相同规划机制）
void em_motor_run_by_angle(float angle1, float angle2, float angle3,
                           float angle4); // 根据角度运行舵机

// 微动专用：直接设置前3个舵机的目标角度，不触发S曲线重规划，不改变夹爪(servo3)状态
// 适用于每15ms高频调用的坐标系微动场景，避免S曲线速度被反复重置
void em_motor_set_target_direct(float angle1, float angle2, float angle3);

// BLE安全缓冲接口（供em_ble.cpp调用，避免中断context中执行复杂操作）
void ble_set_angle_command(uint8_t *angles); // 缓冲角度命令
void ble_set_move_command(uint8_t *dir);     // 缓冲移动命令

// 后台轮询任务，用于前后台解耦
void em_motor_tick_loop();

#endif
