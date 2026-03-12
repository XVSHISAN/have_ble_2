/**
 * @file main.cpp
 * @brief 机械臂主程序入口
 *
 * ============ 系统总体架构 ============
 *
 * 数据流：
 *
 *   BLE客户端 (手机/上位机)
 *   发送指令：0xA5 0xA5 0x01 A B C D (角度模式)
 *         或：0xA5 0xA5 0x02 X Y Z (坐标微动模式)
 *                       │
 *                       │ BLE通信 (433ft)
 *                       ↓
 *
 *   onWrite() 回调 [中断context]
 *   - 解析8/6字节协议
 *   - 调用 ble_set_*_command() 缓冲数据
 *   (重点：不执行复杂计算，快速返回)
 *                       │
 *                       ↓
 *
 *          motor_timer_callbackfun()
 *          硬件定时器 15ms 触发 (前台 ISR)
 *          - 仅拉高 flag_15ms_tick 标志位
 *          - 极速退出，绝不阻塞CPU
 *
 *          em_motor_tick_loop()
 *          主循环(loop)异步轮询 (后台任务)
 *                       │
 *                       ↓
 *
 *   1. process_ble_command()                 处理BLE缓冲
 *      - em_motor_run() 上位规划S曲线
 *      - alg_positive_operation() 正运动学
 *      或：alg_set_move_action() 设置微动
 *                       │
 *                       ↓
 *
 *   2. 四个舵机轨迹离散执行
 *      update_profile()×4
 *      - 积分：jerk → acc → vel → pos
 *      - 输出PWM给舵机
 *                       │
 *                       ↓
 *
 *   3. alg_move_run() 坐标系微动
 *      - 累积坐标位移
 *      - inverse_operation() 反运动学
 *      - check_angle() 安全检查
 *      - em_motor_set_target_direct() 直接下发
 *                       │
 *                       ↓
 *              舵机位置实时更新
 *              末端坐标实时同步
 *
 * ============ 模块职责 ============
 *
 * [em_motor]
 * ✓ S曲线轨迹规划核心
 * ✓ 舵机PWM驱动
 * ✓ BLE命令缓冲队列
 * ✓ 定时器管理
 *
 * [em_alg]
 * ✓ 正/逆运动学计算
 * ✓ 坐标系微动执行
 * ✓ 边界检查与安全策略
 *
 * [em_ble]
 * ✓ BLE Server/Service/Characteristic
 * ✓ 连接管理 + 重连机制
 * ✓ 协议解析 + 缓冲转发
 *
 * ============ 初始化顺序 ============
 *
 * setup():
 * 1. Serial.begin(115200) ---- 调试/日志输出
 * 2. init_ble()             ---- BLE服务器启动，等待连接
 * 3. em_motor_init()        ---- 舵机初始化 + 15ms定时器启动
 *                                (此时开始定时处理BLE命令)
 * 4. em_monitor_init()      ---- 启动监控模块（10s周期心跳）
 *
 * ============ BLE通信协议 ============
 *
 * 协议格式：
 * 指令1 [8字节] - 舵机角度模式
 *   0xA5 0xA5 0x01 angle1 angle2 angle3 angle4
 *   功能：直接指定四个舵机的目标角度（度）
 *   范围：0~180
 *
 * 指令2 [6字节] - 坐标系微动模式
 *   0xA5 0xA5 0x02 moveX moveY moveZ
 *   功能：在末端坐标系中微小移动
 *   值：-1 负方向，0 停止，1 正方向，255 表示 -1
 *
 * 示例代码：
 * - 让机械臂转到某个角度后 1 秒再转回：
 *   em_motor_run_by_angle(45, 30, 120, 0);
 *   delay(1000);
 *   em_motor_run_by_angle(0, 0, 180, 0);
 */

#include "em_config.h"
#include "hal/em_alg.h"
#include "hal/em_ble.h"
#include "hal/em_monitor.h"
#include "hal/em_motor.h"

/**
 * @brief 系统初始化入口
 *
 * 按顺序初始化各模块：
 * 1. 串口 (115200bps)
 * 2. BLE蓝牙服务器
 * 3. 舵机 + 15ms定时器
 * 4. 监控模块 (10s心跳)
 */
void setup() {
  Serial.begin(115200);
  Serial.print("init_task\n");
  Serial.println("系统启动");
  Serial.flush();
  init_ble();      // 启动BLE服务器
  em_motor_init(); // 舵机初始化 + 定时器启动
  // em_monitor_init(); // 关闭监控模块，避免干扰串口绘图器
}

void loop() {
  em_motor_tick_loop(); // 轮询执行底层的重负载离散插补算法
}
