/**
 * @file em_ble.cpp
 * @brief BLE无线通信模块 - ESP32 BLE Server
 *
 * 系统架构：
 *
 *   手机APP / 上位机
 *          │
 *          │ BLE通信 (2.4GHz 433ft/132m)
 *          ↓
 *
 *   BLE Server (ESP32)
 *   - 服务 UUID: 4fafc201-...
 *   - 特征 UUID: beb5483e-...
 *          │
 *          ↓ (onWrite回调)
 *
 *   解析指令格式  [在中断context执行]
 *          │
 *          ↓
 *   ble_set_angle_command()  or  ble_set_move_command()
 *   (仅缓存数据，快速返回)
 *          │
 *          ↓ (定时器15ms后)
 *   process_ble_command()  [在timer context执行]
 *          │
 *          ↓
 *   em_motor_run() / alg_set_move_action()
 *
 * 通信协议（保持不变）：
 * 1. 角度指令（8字节）：
 *    0xA5 0xA5 0x01 angle1 angle2 angle3 angle4
 *    用途：直接控制四个舵机的目标角度
 *
 * 2. 移动指令（6字节）：
 *    0xA5 0xA5 0x02 moveX moveY moveZ
 *    说明：每个方向为 -1(负)/0(停止)/+1(正)，255表示-1
 *    用途：相对坐标系微动
 *
 * 安全设计：
 * ✓ BLE回调仅做数据解析 + 缓存，避免中断context中复杂计算
 * ✓ 舵机规划/运动学在定时器context中执行，线程安全
 * ✓ 连接断开时自动重新广播，无需手动重启
 */

#include "em_ble.h"   // 包含蓝牙功能头文件
#include "em_alg.h"   // 包含算法头文件
#include "em_motor.h" // 包含舵机控制头文件

// ============ 全局变量 ============
BLECharacteristic *pCharacteristic = NULL; // 定义全局变量，保存特征对象指针
bool bleConnected = false;                 // 定义全局变量，保存连接状态

/**
 * @brief 获取蓝牙连接状态
 * @return true 如果蓝牙已连接
 * @return false 如果蓝牙未连接
 */
bool get_ble_connect() { return bleConnected; }
// ==============================

// ============ BLE服务器回调 ============

/**
 * @brief BLE服务器连接/断开事件回调类
 *
 * 管理蓝牙连接生命周期：
 * - 连接时标记状态为已连接
 * - 断开时标记状态为未连接，并自动重新开启广播以允许再次连接
 */
class bleServerCallbacks : public BLEServerCallbacks
{
  /**
   * @brief 客户端连接回调
   * @param pServer 当前BLE服务器指针
   */
  void onConnect(BLEServer *pServer)
  {
    bleConnected = true;
    Serial.println("蓝牙已连接");
    Serial.flush();
  }

  /**
   * @brief 客户端断开回调
   *
   * BLE断开后Advertising会自动停止，必须手动重新开启广播，
   * 否则需要重启ESP32才能再次被扫描到。
   *
   * @param pServer 当前BLE服务器指针
   */
  void onDisconnect(BLEServer *pServer)
  {
    bleConnected = false;
    Serial.println("蓝牙已断开");
    Serial.flush();
    pServer->startAdvertising(); // 重新开启广播，允许再次连接
  }
};
// ==================================

// ============ BLE特征读写回调 ============

/**
 * @brief BLE特征读写事件回调类
 *
 * 处理客户端对Characteristic的读/写操作：
 * - onRead: 记录读取事件日志
 * - onWrite: 解析BLE协议，将指令缓冲到安全队列
 */
class bleCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  /**
   * @brief 客户端读取事件回调（目前仅日志记录）
   * @param pCharacteristic 被读取的特征对象
   */
  void onRead(BLECharacteristic *pCharacteristic)
  {
    Serial.println("触发读取事件");
  }

  /**
   * @brief 客户端写入事件回调 - 协议解析入口
   *
   * 在BLE中断context中执行，仅做数据解析和缓冲，不执行复杂计算。
   * 支持两种指令格式：
   * - 8字节角度指令：0xA5 0xA5 0x01 + 4个角度值
   * - 6字节微动指令：0xA5 0xA5 0x02 + 3个方向值
   *
   * @param pCharacteristic 被写入的特征对象
   */
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    size_t length = pCharacteristic->getLength();
    uint8_t *pdata = pCharacteristic->getData();

    // --- 角度指令处理（8字节） ---
    if (length == 8)
    {
      if (pdata[0] == 0xA5 && pdata[1] == 0xA5 && pdata[2] == 0x01)
      {
        Serial.printf("收到角度指令 A=%d B=%d C=%d D=%d\n", pdata[3], pdata[4],
                      pdata[5], pdata[6]);
        Serial.flush();
        ble_set_angle_command(pdata + 3); // 缓冲数据，由定时器处理
      }
    }
    // --- 微动指令处理（6字节） ---
    if (length == 6)
    {
      if (pdata[0] == 0xA5 && pdata[1] == 0xA5 && pdata[2] == 0x02)
      {
        Serial.printf("收到微动指令 X=%d Y=%d Z=%d\n", pdata[3], pdata[4],
                      pdata[5]);
        Serial.flush();
        ble_set_move_command(pdata + 3); // 缓冲数据，由定时器处理
      }
    }

    // 原始数据调试输出
    for (int index = 0; index < length; index++)
    {
      Serial.printf(" %d", pdata[index]);
    }
    Serial.printf("\n");
  }
};
// ===================================

// ============ BLE初始化 ============

/**
 * @brief 初始化BLE蓝牙功能
 *
 * 完成以下步骤：
 * 1. 初始化BLE设备（设备名称 = BLE_NAME）
 * 2. 创建BLE Server并绑定连接/断开回调
 * 3. 创建Service + Characteristic（支持读/写/通知）
 * 4. 绑定特征读写回调
 * 5. 启动服务并开始广播
 *
 * 调用位置：setup() 中，在 em_motor_init() 之前
 */
void init_ble()
{
  // 1. 初始化BLE设备
  BLEDevice::init(BLE_NAME);
  BLEDevice::startAdvertising();

  // 2. 创建服务器并绑定连接回调
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new bleServerCallbacks());

  // 3. 创建服务和特征
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ |
                               BLECharacteristic::PROPERTY_NOTIFY |
                               BLECharacteristic::PROPERTY_WRITE);

  // 4. 绑定特征回调 + 添加BLE2902描述符（用于通知功能）
  pCharacteristic->setCallbacks(new bleCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // 5. 启动服务并开始广播
  pService->start();
  BLEDevice::startAdvertising();
}
// ================================
  



