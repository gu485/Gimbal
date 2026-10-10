/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"
#include "comm_task.hpp"
#include "ins_imu.hpp"   // GetImu()
#include "ins_robot.hpp"  // GetGimbal()
#include "gimbal.hpp"     // robot::Gimbal 完整定义
#include "imu.hpp"       // Imu 完整定义(ins_imu.hpp 只有前向声明)
#include "main.h"        // Error_Handler
#include "tim.h"         // htim6

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* IMU 初始化(SPI 握手 + Mahony)。
   * 必须放在这里(定时器启动之前、while(1) 之前):BMI088.hpp:173 明确说明
   * imuInit() 内部会阻塞运行(HAL_Delay),不能在中断里调。
   * 失败则卡死,避免带着没握上手的 IMU 进入控制。 */
  if (GetImu()->init() != 0) {
    Error_Handler();
  }

  /* 通讯层初始化:启动 DT7 遥控器接收。
   * 不调的话 rc_lh() 永远是初值 0,云台会因为"没有输入"而完全不动,
   * 看起来像闭环没写对,其实是接收没启动。 */
  CommTaskInit();
}

void MainTask(void) {
  /* 1ms 控制周期入口,保持薄薄一层,不堆业务逻辑。
   * 顺序有意义:Gimbal::update() 内部读 IMU 的姿态角,
   * 所以 IMU 的 update() 必须在它之前跑过。
   * 而 IMU 的 update() 放在 CommTask() 里,所以调用顺序是
   * MainTask() → CommTask() 的写法会读到上一拍的 IMU 数据(延迟 1ms)。
   * 1ms 延迟对 1kHz 闭环影响很小,先这样;
   * 如果之后发现相位滞后导致振荡,把 IMU update 挪到 MainTask 开头。 */
  GetGimbal()->update();
}

/* Private function definitions ----------------------------------------------*/
/* htim6 周期中断回调:1ms tick 的唯一入口。
 *
 * 为什么要判 htim:这块板同时启动了 htim2 / htim4 / htim6 三个定时器中断
 * (见 Core/Src/main.c:115-119),HAL 会把三者都派发到同一个回调。
 * 不判的话会把 2ms/3ms 的 tick 误当成 1ms,控制周期就错了。 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim6) {
    MainTask();
    CommTask();
  }
}
