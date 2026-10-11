/**
 *******************************************************************************
 * @file      :ins_pid.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_INSTANCE_INS_PID_HPP_
#define GIMBAL_INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取 yaw 关节位置环 PID 单例。 */
hello_world::pid::Pid *GetPidMotorYaw();
/* 获取 yaw 关节「小陀螺模式」速度环 PID 单例(与上面的跟随/分离 PID 独立,单独调参)。 */
hello_world::pid::Pid *GetPidMotorYawSpin();
/* 获取 pitch 关节角度环 PID 单例。 */
hello_world::pid::Pid *GetPidMotorPitch();

#endif /* GIMBAL_INSTANCE_INS_PID_HPP_ */
