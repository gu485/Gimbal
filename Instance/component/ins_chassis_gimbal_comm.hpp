/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.hpp
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
#ifndef GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_
#define GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "gimbal_chassis_comm.hpp"   /* robot::kChassisToGimbalId / kGimbalToChassisId
                                      * 定义在这个共享协议头里,两块板都从这里取,
                                      * 不要在本文件另写一份——两份定义编译器互不检查,
                                      * 改了一边会导致双板通信静默失效。 */

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot {
class GimbalChassisComm;
}  // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板双板通讯对象（robot::GimbalChassisComm）单例。 */
robot::GimbalChassisComm *GetGimbalChassisComm();

#endif /* GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_ */
