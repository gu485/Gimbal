/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"
#include "gimbal.hpp"   // robot::Gimbal 完整定义(ins_robot.hpp 只有前向声明)
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() {
  /* TODO【新人任务】:
   * 返回本板顶层状态机 robot::Robot 单例。
   *
   * 你要做：
   *   1. 在本文件静态定义一个 robot::Robot 对象（静态存储期，程序启动即构造）。
   *   2. 返回它的地址。
   *
   * 约束 / 提示：
   *   - robot::Robot 类的定义由你在 RobotModules/component/robot.hpp 补全（见该文件 TODO），
   *     本文件只负责实例化并返回地址。
   *   - 参考：code_control/SteeredInfantry2025/Gimbal/Instance/src/ins_robot.cpp。
   */
  return nullptr;
}

robot::Gimbal *GetGimbal() {
  /* 返回云台运动模块单例。对象用 new 创建,不是栈对象:
   * 栈上创建会在 main 返回时析构,而单例需要活到程序结束。 */
  static robot::Gimbal *gimbal = nullptr;
  if (gimbal == nullptr) {
    gimbal = new robot::Gimbal();
  }
  return gimbal;
}

/* Private function definitions ----------------------------------------------*/
