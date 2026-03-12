/**
 * @file em_alg.h
 * @brief 机械臂系统运动学代数解算引擎与空间建模核心定义
 *
 * 核心功能模块划分：
 *
 * 1. 正运动学 (Forward Kinematics)
 *    函数：alg_positive_operation(angleA, angleB, angleC)
 *    功能：根据基础连杆几何参数将关节空间变量映射至末端绝对笛卡尔坐标空间。
 *
 * 2. 逆运动学 (Inverse Kinematics)
 *    函数：inverse_operation(x, y, z)
 *    功能：采用解析解法对给定的空间笛卡尔点位目标进行反演计算，求解对应关节组变量。
 *          具备特定奇异点防护设计。
 *
 * 3. 笛卡尔增量微动控制 (Cartesian Incremental Control)
 *    函数：alg_move_run()
 *    触发时序：绑定后台系统基准迭代循环 (15ms 节拍)。
 *    功能：处理离散化连续偏移需求，构建虚拟约束以保证动态执行边界安全性。
 */

#ifndef _EM_ALG_H_
#define _EM_ALG_H_

#include "em_config.h"
#include <math.h>

// 正运动学：角度 → 坐标
// 在以下场景调用：
// - em_motor_run()执行前（记录起始位置）
// - em_motor_init()初始化（计算原点坐标）
void alg_positive_operation(float angleA, float angleB,
                            float angleC); // 正运动学

// 设置坐标系微动指令
// 由BLE回调 → ble_set_move_command() → process_ble_command() → 本函数
void alg_set_move_action(uint8_t *data); // 设置移动动作

// 定时器驱动的坐标系微动执行
// 每15ms自动调用一次（由motor_timer_callbackfun()触发）
// 功能：利用逆运动学实现增量式坐标控制
void alg_move_run(); // 根据动作运行舵机

#endif



