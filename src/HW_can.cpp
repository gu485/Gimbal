/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"

#include "ins_motor.hpp"                        // GetMotorYaw()
#include "ins_chassis_gimbal_comm.hpp"          // kChassisToGimbalId
#include "gimbal_chassis_comm.hpp"              // robot::GimbalChassisComm 完整定义

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* === CAN 接收调试计数(Ozone 搜 g_can1_rx) === */
/* rx_cnt      : CAN1 收到的总帧数
 * c2g_cnt     : 命中双板下行帧(0x210)的次数 —— 底盘有没有在发
 * yaw_fb_cnt  : 命中云台电机反馈帧的次数 —— 电机有没有回话
 * last_id     : 最后收到的帧 ID,用来"到底是谁在路上" */
volatile uint32_t g_can1_rx_cnt     = 0;
volatile uint32_t g_can1_c2g_cnt    = 0;
volatile uint32_t g_can1_yaw_fb_cnt = 0;
volatile uint32_t g_can1_last_id    = 0;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      g_can1_rx_cnt++;
      g_can1_last_id = rx_header1.StdId;

      /* 按帧 ID 分发。云台板上只有两类帧走 CAN1:
       *   1. 双板下行帧 0x210(底盘发) → 解出摇杆
       *   2. 云台电机反馈帧 = GetRxId(2) → 解出角度/速度
       * 注意必须先判双板帧:0x210 和电机反馈 ID 不会撞,但顺序上
       * 先处理"协议帧"逻辑上更清晰,出问题时好定位。 */
      if (rx_header1.StdId == robot::kChassisToGimbalId) {
        g_can1_c2g_cnt++;
        auto* comm = GetGimbalChassisComm();
        if (comm != nullptr) {
          comm->decode(can1_rx_data);
        }
      } else if (rx_header1.StdId ==
                 hello_world::motor::DM_J4310::GetRxId(2)) {
        g_can1_yaw_fb_cnt++;
        /* 达妙同一个 ID 上有两类帧,靠 D[2] 区分:
         *   常规状态反馈帧: D[2] 是角度的任意字节
         *   寄存器应答帧  : D[2] 固定 0x33(读) / 0x55(写)
         * 寄存器应答是配置阶段的握手,不能当电机反馈解,
         * 否则会把配置返回值当成角度,angle() 直接跳成乱码。 */
        if (can1_rx_data[2] != 0x33 && can1_rx_data[2] != 0x55) {
          auto* yaw = GetMotorYaw();
          if (yaw != nullptr) {
            yaw->decode(can1_rx_data);
          }
        }
      }
      /* 其余 StdId 一律丢弃。总线上混着 M3508 等其他设备的帧,
       * 认错一个就会把别人的数据当成云台电机反馈。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* CAN2 上目前没有本板关心的帧(云台电机和双板通讯都在 CAN1),
       * 留空即可。以后要在 CAN2 上挂设备时,按 rx_header2.StdId 分发。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
