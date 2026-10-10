/**
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
 * @brief     : 双板通讯单例
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"

robot::GimbalChassisComm *GetGimbalChassisComm() {
  /* 本板是 Gimbal,和底盘相反:
   *   tx_id = kGimbalToChassisId (0x211) —— 云台发、底盘收
   *   rx_id = kChassisToGimbalId (0x210) —— 底盘发、云台收
   * 这里最容易写反,写反的表现是:自己发的帧自己收不到(能收到但全是自己发的),
   * 或者完全收不到对方。 */
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Gimbal,
      robot::kGimbalToChassisId,    // tx_id:本板(云台)发送用这个 ID
      robot::kChassisToGimbalId);   // rx_id:本板(云台)接收用这个 ID
  return &comm;
}