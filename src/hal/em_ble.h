/**
 * @file em_ble.h
 * @brief BLE蓝牙通信模块头文件
 *
 * 提供BLE初始化和状态查询接口。
 * 内部使用ESP32 BLE Arduino库实现BLE Server功能。
 */

#ifndef _EM_BLE_H_
#define _EM_BLE_H_

#include "em_config.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>

/**
 * @brief 初始化BLE蓝牙服务
 *
 * 创建BLE Server, Service, Characteristic，
 * 绑定回调并开始广播。
 */
void init_ble();

/**
 * @brief 获取当前蓝牙连接状态
 *
 * @return true 已连接
 * @return false 未连接
 */
bool get_ble_connect();

#endif
