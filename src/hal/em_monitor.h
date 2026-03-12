/**
 * @file em_monitor.h
 * @brief 运行状态监控模块头文件
 *
 * 提供串口状态输出功能，帮助调试和验证系统运行状态。
 */

#ifndef _EM_MONITOR_H_
#define _EM_MONITOR_H_

/**
 * @brief 初始化监控模块
 *
 * 启动10秒周期的定时器，定期输出系统状态。
 */
void em_monitor_init();

/**
 * @brief 立即打印当前系统状态
 *
 * 输出舵机角度、末端坐标、BLE连接状态等信息。
 * 若状态未变化则跳过输出。
 */
void em_monitor_dump();

#endif
