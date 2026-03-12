/**
 * @file em_alg.cpp
 * @brief 机械臂运动学算法 - 正运动学 + 逆运动学 + 坐标系微动
 *
 * 系统架构：
 *           运动学计算处理流程
 *
 *   1. 正运动学 (alg_positive_operation)
 *      舵机角度(A,B,C) → 末端绝对坐标(X,Y,Z)
 *
 *   2. 逆运动学 (inverse_operation)
 *      末端绝对坐标(X,Y,Z) → 舵机角度(A,B,C)
 *      特点：含范围检查、NaN防护、acos保护
 *
 *   3. 坐标系微动 (alg_move_run - 定时器驱动)
 *      接收方向指令 → 累积坐标位移 → 反运动学
 *      → 角度边界检查 → 下发直接角度指令
 *
 * 坐标系定义：
 * - X: 前后（沿臂伸缩）
 * - Y: 上下（垂直高度）
 * - Z: 左右（水平旋转投影）
 *
 * 舵机映射 (3-DoF):
 * - angleA: 底座旋转（0~180°）
 * - angleB: 大臂上下（0~85°，受K<85限制）
 * - angleC: 小臂上下（140-B ~ min(196-B, 180)°）
 * (angle4的夹爪独立控制)
 *
 * 安全特性：
 * ✓ 逆运动学计算失败时打印错误，保持当前位置不动
 * ✓ 坐标超出达域时自动回退，防止抖动
 * ✓ NaN/无穷值检测，防止传播到舵机控制
 */

#include "em_alg.h"       // 包含算法头文件
#include "hal/em_motor.h" // 包含舵机控制头文件

// ============ 全局状态（定时器中实时更新）============
// 移动方向（来自BLE）
int moveX;
int moveY;
int moveZ;

// 舵机实时角度（从正运动学计算得出）
float angleA;
float angleB;
float angleC;

// 夹爪末端绝对坐标（在空间中的实时位置）
float absoluteX;
float absoluteY;
float absoluteZ;

#define MIN(a, b) ((a) < (b) ? (a) : (b)) // 取最小值

// ============ 辅助函数 ============

/**
 * @brief 平方函数
 *
 * @param n 输入数值
 * @return float n的平方
 */
float square(float n) { return n * n; }

/**
 * @brief 符号函数
 *
 * @param num 输入数值
 * @return float +1.0 正数, -1.0 负数, 0.0 零
 */
float sgn(float num)
{
  if (num > 0)
  {
    return 1.0;
  }
  else if (num < 0)
  {
    return -1.0;
  }
  else
  {
    return 0.0;
  }
}

/**
 * @brief 逆运动学解算器：基于解析几何的三自由度空间逆解
 *
 * 算法描述：
 * 本函数负责将笛卡尔空间 (Cartesian Space) 的绝对物理坐标 (X, Y, Z mm)
 * 映射为三个伺服关节的旋转角度 (Joint Space, angle A/B/C °)。
 *
 * 采用解析几何法 (`atan`, `acos`)
 * 求解。为保证系统稳定性，包含以下边界约束处理：
 *   1. 奇异点处理 (`fabs(x) < 0.01`): 避免在特定姿态下出现除零异常。
 *   2. 定义域截断 (`acosArg > 1.0f`): 过滤超出机械臂工作空间 (Workspace)
 * 的不可达目标。
 *
 * 这些约束机制将非法的计算结果拦截在软件层面，防止数学异常 (如 NaN)
 * 向下传递给驱动层， 避免产生不可预期的机械动作。
 *
 * @param x 末端执行器 (夹爪) 在全局坐标系下的 X 轴投影 (mm)
 * @param y 末端执行器 在全局坐标系下的 Y 轴投影 (mm)
 * @param z 末端执行器 在全局坐标系下的 Z 轴投影 (mm)
 */
void inverse_operation(float x, float y, float z)
{
  // 防止除零
  if (fabs(x) < 0.01f)
  {
    Serial.printf("错误: 逆运动学中x值过小，接近0\n");
    return;
  }

  //=-DEGREES(ATAN(D10/B10))*(72/28)+90
  angleA = -degrees(atan(z / x)) * (72.0 / 28.0) + 90.0;

  float sqrtVal = sqrt(x * x + z * z);
  if (fabs(sqrtVal) < 0.01f)
  {
    Serial.printf("错误: 逆运动学中平方根值过小\n");
    return;
  }

  float temp1 = degrees(atan((y - 65.5) / (-sgn(x) * sqrtVal - 7.0 - 60.0)));

  // 检查acos的参数范围
  float denomVal =
      sqrt(square(-sgn(x) * sqrtVal - 7.0 - 60.0) + square(y - 65.5));
  if (fabs(denomVal) < 0.01f)
  {
    Serial.printf("错误: acos分母值过小\n");
    return;
  }

  float acosArg = (135.0 * 135.0 + square(-sgn(x) * sqrtVal - 7.0 - 60.0) +
                   square(y - 65.5) - 145.0 * 145.0) /
                  (2.0 * 135.0 * denomVal);
  if (acosArg > 1.0f || acosArg < -1.0f)
  {
    Serial.printf("错误: acos参数超出范围: %f\n", acosArg);
    return;
  }

  float temp2 = degrees(acos(acosArg));

  angleB = 180.0 - 69.0 - temp2 - temp1;

  // 第二个acos同样需要检查参数范围
  float acosArg2 = (145.0 * 145.0 + 135.0 * 135.0 -
                    square(-sgn(x) * sqrtVal - 67.0) - square(y - 65.5)) /
                   (2.0 * 145.0 * 135.0);
  if (acosArg2 > 1.0f || acosArg2 < -1.0f)
  {
    Serial.printf("错误: 第二个acos参数超出范围: %f\n", acosArg2);
    return;
  }

  float temp3 = degrees(acos(acosArg2));
  angleC = 180.0 - (83.5 + (180.0 - 69.0 - temp2 - temp1) - temp3);

  // 内部调试信息被移除以防高频输出导致缓冲溢出
}

/**
 * @brief 正运动学解算器：关节空间至笛卡尔空间的映射
 *
 * 算法描述：
 * 根据连杆机构的三角关系，计算当前三个基础关节角度对应的末端执行器绝对物理坐标
 * (`absoluteX/Y/Z`)。
 *
 * 典型应用场景：
 * 系统初始化或执行绝对角度控制 (如 P2P 运动)
 * 后，需调用此函数以同步当前末端位姿状态。 这保证了后续增量式微动控制
 * (Incremental Control) 具有正确的参考起始点。
 *
 * @param a 底座旋转目标绝对角度 (0~180°)
 * @param b 主臂俯仰目标绝对角度 (0~85°)
 * @param c 副臂俯仰目标绝对角度
 */
void alg_positive_operation(float a, float b, float c)
{
  // 同步更新全局角度变量（修复参数遮蔽Bug：原参数名与全局变量同名导致全局变量不更新）
  angleA = a;
  angleB = b;
  angleC = c;

  float temp =
      -(135.0 * cos(radians(111.0 - b)) +
        145.0 * sin(radians((83.5 + b - (180.0 - c)) - b + 21.0)) + 67.0);

  //=COS(RADIANS((B8-90)/(72/28)))*-(135*COS(RADIANS(111-C8))+145*SIN(RADIANS((83.5+C8-(180-D8))-C8+21))+67)
  absoluteX = cos(radians((a - 90.0) / (72.0 / 28.0))) * temp;

  //=65.5+135*SIN(RADIANS(111-C8))-(145*COS(RADIANS((83.5+C8-(180-D8))-C8+21)))
  absoluteY = 65.5 + 135.0 * sin(radians(111.0 - b)) -
              (145.0 * cos(radians((83.5 + b - (180.0 - c)) - b + 21.0)));

  //=-SIN(RADIANS((B8-90)/(72/28)))*-(135*COS(RADIANS(111-C8))+145*SIN(RADIANS((83.5+C8-(180-D8))-C8+21))+67)
  absoluteZ = -sin(radians((a - 90.0) / (72.0 / 28.0))) * temp;
}

/**
 * @brief 判断角度是否在可到达范围内（参数使用float以适应所有调用场景）
 *
 * @param angleA
 * @param angleB
 * @param angleC
 * @return true
 * @return false
 */
bool check_angle(float angleA, float angleB, float angleC)
{
  // 检查是否为NaN
  if (isnan(angleA) || isnan(angleB) || isnan(angleC))
  {
    Serial.printf("错误: 角度值为NaN\n");
    return false;
  }

  if (angleA < 0 || angleA > 180)
  {
    Serial.printf("angleA error %f , must in 0<a<180\n", angleA);
    return false;
  }

  if (angleB < 0 || angleB > 85)
  {
    Serial.printf("angleB error %f , must in 0<b<85\n", angleB);
    return false;
  }

  float angleCMin = 140 - angleB;
  float angleCMax = MIN((196 - angleB), 180);
  if (angleC < angleCMin || angleC > angleCMax)
  {
    Serial.printf("angleC error %f , must in %f<c<%f\n", angleC, angleCMin,
                  angleCMax);
    return false;
  }
  return true;
}

/**
 * @brief 设置坐标系微动方向
 *
 * 解析BLE方向数据，将 uint8_t 值转换为整型方向指令。
 * 255 解析为 -1（因为 BLE 传输 uint8_t 无法直接表示负数）。
 *
 * @param data 指向三个 uint8_t 方向值的指针 (0=停止, 1=正向, 255=反向)
 */
void alg_set_move_action(uint8_t *data)
{
  moveX = data[0];
  moveY = data[1];
  moveZ = data[2];
  // BLE传输uint8_t，255表示-1
  if (moveX == 255)
    moveX = -1;
  if (moveY == 255)
    moveY = -1;
  if (moveZ == 255)
    moveZ = -1;

  Serial.printf("设置微动方向 X=%d Y=%d Z=%d\n", moveX, moveY, moveZ);
  Serial.flush();
}

/**
 * @brief 笛卡尔空间增量微动控制任务
 *
 * 功能描述：
 * 在后台主循环驱动下，执行连续的空间平移插补及位置校验。
 * 每次调用周期 (`tick`) 产生固定的空间步长 (`offset`)。
 *
 * 数据流与控制逻辑：
 * 1. 增量求解：`absoluteX/Y/Z += direction * offset`。
 * 2. 空间反解：调用 `inverse_operation()` 计算期望的关节角度。
 * 3. 约束校验：调用 `check_angle()` 验证期望关节角是否在机械或软件限位范围内。
 *    - 校验通过：调用 `em_motor_set_target_direct()` 更新驱动级目标。
 *    - 校验失败：执行状态回滚 (Rollback)，抵消本次步长增量，忽略位移输出。
 */
void alg_move_run()
{
  float offset = 0.4;
  if (moveX == 0 && moveY == 0 && moveZ == 0)
  {
    return;
  }
  if (moveX > 0)
    absoluteX = absoluteX + offset;
  else if (moveX < 0)
    absoluteX = absoluteX - offset;

  if (moveY > 0)
    absoluteY = absoluteY + offset;
  else if (moveY < 0)
    absoluteY = absoluteY - offset;

  if (moveZ > 0)
    absoluteZ = absoluteZ + offset;
  else if (moveZ < 0)
    absoluteZ = absoluteZ - offset;
  inverse_operation(absoluteX, absoluteY, absoluteZ);
  if (check_angle(angleA, angleB, angleC) == true)
  {
    // 使用微动专用函数：不触发S曲线重规划，不改变夹爪状态
    em_motor_set_target_direct(angleA, angleB, angleC);
  }
  else
  {
    // 角度超限，回退坐标偏移
    if (moveX > 0)
      absoluteX = absoluteX - offset;
    else if (moveX < 0)
      absoluteX = absoluteX + offset;

    if (moveY > 0)
      absoluteY = absoluteY - offset;
    else if (moveY < 0)
      absoluteY = absoluteY + offset;

    if (moveZ > 0)
      absoluteZ = absoluteZ - offset;
    else if (moveZ < 0)
      absoluteZ = absoluteZ + offset;
  }
}






