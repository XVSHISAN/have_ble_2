/**
 * @file em_config.h
 * @brief 全局配置常数定义
 *
 * BLE服务配置：
 * - SERVICE_UUID: 自定义服务的唯一标识
 * - CHARACTERISTIC_UUID: 自定义特征的唯一标识
 * - BLE_NAME: 设备在BLE扫描中显示的名称
 *
 * 注意：UUID可修改，但需保持一致性
 */

#ifndef _EM_CONFIG_H_
#define _EM_CONFIG_H_

#include <Arduino.h>

// ============ BLE配置 ============
#define SERVICE_UUID \
  "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // 自定义蓝牙服务UUID
#define CHARACTERISTIC_UUID \
  "beb5483e-36e1-4688-b7f5-ea07361b26a8" // 自定义蓝牙特征UUID
#define BLE_NAME "Mini-Arm"
// ===============================

// ============ 各轴角度范围 ============
// 索引对应：servo[0]=旋转, servo[1]=B轴, servo[2]=C轴, servo[3]=夹爪
#define SERVO0_MIN 0   // 旋转最小角度
#define SERVO0_MAX 180 // 旋转最大角度
#define SERVO1_MIN 0   // B轴最小角度
#define SERVO1_MAX 80  // B轴最大角度
#define SERVO2_MIN 55  // C轴最小角度
#define SERVO2_MAX 180 // C轴最大角度
#define SERVO3_MIN 0   // 夹爪最小角度
#define SERVO3_MAX 37  // 夹爪最大角度
// ================================

#endif
