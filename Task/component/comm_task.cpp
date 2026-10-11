/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"
#include "ins_imu.hpp"   // GetImu()
#include "ins_motor.hpp" // GetMotorYaw()
#include "ins_robot.hpp" // GetGimbal()
#include "ins_chassis_gimbal_comm.hpp" // GetGimbalChassisComm() / robot::kGimbalToChassisId
#include "gimbal_chassis_comm.hpp"     // robot::GimbalChassisComm 完整定义
#include "gimbal.hpp"    // robot::Gimbal 完整定义
#include "imu.hpp"       // Imu 完整定义
#include "iwdg.h"        // hiwdg / HAL_IWDG_Refresh
#include "can.h"         // hcan1 / hcan2
#include "usart.h"       // huart3(DT7 走 USART3)
#include "HW_can.hpp"    // CAN_Send_Msg
#include "ins_comm.hpp"  // GetRemoteControl(DT7 单例)
#include "DT7.hpp"       // DT7::decode() 完整定义

/* === DT7 接收(USART3) === */
/* DBUS 协议固定 18 字节。静态缓冲而不是栈上:HAL 的 DMA 要在整个
 * 接收期间一直访问这块内存,函数返回后栈内存就没了。 */
static uint8_t g_dt7_rx_buf[18];

/* 接收帧计数。恒为 0 = 遥控器根本没接进来(接线/波特率/串口选错)。 */
volatile uint32_t g_dt7_rx_cnt  = 0;
volatile uint32_t g_dt7_decode_cnt = 0;
/* 解码后的通道值快照,每次 decode() 成功后刷新。
 * g_dbg_rc_lh 就是 Gimbal 里读的 rc_lh() 的镜像,摇杆不动它也不动
 * 就说明 DT7 链路没通。 */
volatile float   g_dbg_rc_lh = 0.0f;
volatile float   g_dbg_rc_lv = 0.0f;
volatile float   g_dbg_rc_rv = 0.0f;
volatile float   g_dbg_rc_rh = 0.0f;

/* === 云台控制调试镜像(Ozone 搜 g_dbg_gimbal) === */
volatile float   g_dbg_gimbal_target   = 0.0f;  /* 目标角速度 w_yaw rad/s */
volatile float   g_dbg_gimbal_measure  = 0.0f;  /* 实测角速度 w_current rad/s(IMU gyro z) */
volatile float   g_dbg_gimbal_error    = 0.0f;  /* 速度误差 rad/s */
volatile float   g_dbg_gimbal_torque   = 0.0f;  /* PID 输出力矩 N·m(±3 限幅) */
volatile bool    g_dbg_gimbal_ready    = false; /* IMU 就绪且已对齐目标 */
volatile uint8_t g_dbg_gimbal_mode     = 0;     /* 底盘模式 0=INIT 1=分离 2=跟随 3=小陀螺 4=急停 */
volatile bool    g_dbg_gimbal_spin     = false; /* 1=正在用小陀螺 PID */
volatile bool    g_dbg_gimbal_estop    = false; /* 急停锁存:1=已锁,云台输出全零 */
volatile uint32_t g_dbg_yaw_frame_cnt  = 0;
/* === pitch 电机通信调试(Ozone 搜 g_dbg_pitch) ===
 * frame_cnt : CAN2 发帧计数
 * enabled   : 电机已使能(收到过状态码=1 的反馈)
 * status    : 状态码 0=失能 1=使能 8~15=故障
 * angle/vel : 反馈角度 rad / 角速度 rad/s,手转电机应跟着变 */
volatile uint32_t g_dbg_pitch_frame_cnt = 0;
volatile bool     g_dbg_pitch_enabled   = false;
volatile uint8_t  g_dbg_pitch_status    = 0;
volatile float    g_dbg_pitch_angle     = 0.0f;
volatile float    g_dbg_pitch_vel       = 0.0f;
volatile float    g_dbg_pitch0          = 0.0f;  /* 零位(100 次平均) */
volatile float    g_dbg_pitch_target    = 0.0f;  /* 目标角度 rad */
volatile float    g_dbg_pitch_current   = 0.0f;  /* pitch1 − pitch0 */
volatile float    g_dbg_pitch_torque    = 0.0f;  /* PID 输出力矩 N·m */
volatile bool     g_dbg_pitch_calib     = false; /* 零位标定完成 */     /* CAN 帧计数,确认真在发 */


/* === 双板通讯调试 === */
/* g_dbg_g2c_frame_cnt : 云台→底盘 发帧计数,一直涨说明发送链路通
 * g_dbg_rc_lh_recv    : 从底盘收到的 rc_lh(反量化后),拨摇杆应该跟着变
 *                       恒为 0 = 底盘没在发,或者双板不通
 * g_dbg_gimbal_yaw_tx : 发给底盘的世界朝向 rad
 * g_dbg_peer_offline  : 1 = 判定底盘离线,云台已锁死输出 */
volatile uint32_t g_dbg_g2c_frame_cnt = 0;
volatile float   g_dbg_rc_lh_recv    = 0.0f;
volatile float   g_dbg_gimbal_yaw_tx = 0.0f;
volatile bool    g_dbg_peer_offline  = false;

/* === IMU 调试镜像(Ozone Symbols 搜 g_dbg_imu 即可全找到) === */
/* init_err:0=通,bit0=加速度计ID错(查PA4),bit1=陀螺仪ID错(查PB0),
 * bit2/3=自检失败,bit4/5=配置报错 */
volatile uint8_t g_dbg_imu_init_err   = 0;
volatile bool    g_dbg_imu_calib_done = false;   /* 上电 1 秒后变 true */
/* 姿态角(rad)。云台板水平放置时应全 ≈ 0;绕竖直轴转车,yaw 变化 */
volatile float   g_dbg_imu_roll  = 0.0f;
volatile float   g_dbg_imu_pitch = 0.0f;
volatile float   g_dbg_imu_yaw   = 0.0f;
/* 原始读数: acc 单位 g,静止 |acc| ≈ 9.8(验证芯片接线);
 * gyro 单位 rad/s,标定完成后静止 ≈ 0 */
volatile float   g_dbg_imu_acc[3]  = {0, 0, 0};
volatile float   g_dbg_imu_gyro[3] = {0, 0, 0};

/* === 定位用:心跳 + IMU 内部计数器 === */
/* g_dbg_tick 每进一次 CommTask 加 1。恒为 0 = 中断链路没通 / 程序死在 Error_Handler。
 * 一直涨但 calib_done 不变 = update() 在跑,问题在标定逻辑内部。
 * g_dbg_calib_cnt 是 Imu 私有的 calib_cnt_ 的镜像,直接看它涨到几,一眼定位。 */
volatile uint32_t g_dbg_tick       = 0;
volatile uint32_t g_dbg_calib_cnt  = 0;

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* CAN1:双板通讯(0x210/0x211) + yaw 电机(DM4310 ID=2)
   * 必须先配滤波器,再 Start,最后开通知——顺序不对会丢第一帧。 */
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* CAN2:pitch 电机(DM4310, ID=1) */
  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* DT7 遥控器接收启动:USART3 + 空闲中断 + DMA。
   * CubeMX 把 RX DMA 配成了 CIRCULAR:循环模式下缓冲区不会在帧边界重新对齐,
   * 上电时如果正好在一帧中间开始收,之后每一帧都会错位,解出来全是脏数据。
   * 改成 NORMAL:每帧结束(总线空闲)DMA 停下,回调里重启,天然对齐帧头。 */
  huart3.hdmarx->Init.Mode = DMA_NORMAL;
  HAL_DMA_Init(huart3.hdmarx);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dt7_rx_buf, sizeof(g_dt7_rx_buf));
  /* 关掉 DMA 半传输中断:DBUS 是定长 18 字节,不需要"收一半"就触发。 */
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void CommTask(void) {
  /* 每拍(1ms)先更新 IMU。
   * 放在最前面:后面的所有逻辑如果要读姿态角,读到的必须是"当前这一拍"的
   * 新鲜数据,不能是上一拍的残留。
   * IMU 内部前 1000 帧在标定零漂(约 1 秒),标定期间 update() 会自己 return,
   * 不会喂 Mahony,这段时间姿态角是 0,用 calib_done() 判断。 */
  GetImu()->update();
  g_dbg_tick++;

  /* 喂独立看门狗。
   * iwdg.c 配置:LSI 32kHz / Prescaler 32 ≈ 1kHz,Reload 400 → 超时约 400ms。
   * 每拍(1ms)喂一次;不喂的话每 400ms 硬件复位一次(g_dbg_tick 归零就是这个现象)。 */
  HAL_IWDG_Refresh(&hiwdg);

  /* 同步调试镜像。imu::Imu 的 getter 收的是 float* 而不是 volatile float*,
   * 所以先用普通局部变量接一下再赋值(volatile 是给 Ozone 看的,不影响值)。 */
  {
    float r = 0.0f, p = 0.0f, y = 0.0f;
    float a[3] = {0, 0, 0};
    float g_[3] = {0, 0, 0};
    auto* imu = GetImu();
    g_dbg_imu_init_err   = (uint8_t)imu->init_err();
    g_dbg_imu_calib_done = imu->calib_done();
    g_dbg_calib_cnt      = imu->calib_progress();
    imu->roll(&r);
    imu->pitch(&p);
    imu->yaw(&y);
    imu->acc(a);
    imu->gyro(g_);
    g_dbg_imu_roll  = r;
    g_dbg_imu_pitch = p;
    g_dbg_imu_yaw   = y;
    for (int i = 0; i < 3; i++) {
      g_dbg_imu_acc[i]  = a[i];
      g_dbg_imu_gyro[i] = g_[i];
    }
  }

  /* === 发云台 yaw 电机 ===
   * 达妙电机"一发一收":不发指令它就不会回反馈帧,所以每拍都要 encode + 发送。
   * enable 帧不用手写:encode() 内部 auto_enable 会处理。 */
  {
    auto* yaw = GetMotorYaw();
    if (yaw != nullptr) {
      uint8_t data[8] = {0};
      yaw->encode(data);
      CAN_Send_Msg(&hcan1, data, yaw->tx_id(), 8);
      g_dbg_yaw_frame_cnt++;
    }
  }

  /* === pitch 电机 (CAN2, ID=1) ===
   * 力矩由 Gimbal::UpdatePitch() 写入电机对象, 这里只负责每拍 encode + 发送。 */
  {
    auto* pitch = GetMotorPitch();
    if (pitch != nullptr) {
      uint8_t pdata[8] = {0};
      pitch->encode(pdata);
      CAN_Send_Msg(&hcan2, pdata, pitch->tx_id(), 8);
      g_dbg_pitch_frame_cnt++;
      g_dbg_pitch_enabled = pitch->is_enabled();
      g_dbg_pitch_status  = static_cast<uint8_t>(pitch->status_code());
      g_dbg_pitch_angle   = pitch->angle();
      g_dbg_pitch_vel     = pitch->vel();
    }
  }

  /* === 双板通讯:云台 → 底盘 (0x211) ===
   *
   * 发的是"云台当前的世界朝向"(来自 IMU)和 IMU 就绪标志。
   * 底盘拿这个 yaw 把摇杆平移指令从云台系旋转到世界系,以及算跟随模式的 w。
   *
   * 为什么要 imu_ready:IMU 标定完之前 euler_ 全是 0,发过去是个假值。
   * 底盘必须先判断这个标志再用 yaw,否则会拿假的 0 去算方向。 */
  {
    auto* comm = GetGimbalChassisComm();
    if (comm != nullptr) {
      auto* imu = GetImu();
      auto& gd = comm->gimbal_data();

      float psi = 0.0f;
      bool imu_ok = (imu != nullptr) && imu->calib_done();
      if (imu_ok) {
        imu->yaw(&psi);
      }
      gd.gimbal_yaw_world = psi;
      gd.imu_ready = imu_ok ? 1u : 0u;
      /* z 轴角速度(已扣零漂)→ 底盘板的云台角速度环作 w_current。
       * 标定未完成时发 0, 底盘靠 imu_ready 判断是否可用。 */
      float g3[3] = {0.0f, 0.0f, 0.0f};
      if (imu_ok) imu->gyro(g3);
      gd.gyro_z_q = robot::GimbalChassisComm::QuantizeGyro(g3[2]);

      uint8_t data[8] = {0};
      comm->encode(data);
      CAN_Send_Msg(&hcan1, data, robot::kGimbalToChassisId, 8);
      g_dbg_g2c_frame_cnt++;
    }
  }

  /* === 云台控制量调试镜像 === */
  {
    auto* gimbal = GetGimbal();
    g_dbg_gimbal_target  = gimbal->target_rate();
    g_dbg_gimbal_measure = gimbal->measure_rate();
    g_dbg_gimbal_error   = gimbal->error();
    g_dbg_gimbal_torque  = gimbal->torque_cmd();
    g_dbg_gimbal_ready   = gimbal->ready();
    g_dbg_gimbal_estop   = gimbal->estop();
    g_dbg_gimbal_spin    = gimbal->spin_mode();
    g_dbg_gimbal_mode    = gimbal->mode();
    g_dbg_pitch0         = gimbal->pitch0();
    g_dbg_pitch_target   = gimbal->pitch_target();
    g_dbg_pitch_current  = gimbal->pitch_current();
    g_dbg_pitch_torque   = gimbal->pitch_torque();
    g_dbg_pitch_calib    = gimbal->pitch_ready();

  }

  /* === 双板通讯调试镜像 === */
  {
    auto* comm = GetGimbalChassisComm();
    if (comm != nullptr) {
      g_dbg_rc_lh_recv =
          robot::GimbalChassisComm::DequantizeStick(comm->chassis_data().rc_lh_q);
      g_dbg_gimbal_yaw_tx = comm->gimbal_data().gimbal_yaw_world;
      g_dbg_peer_offline  = comm->isOffline();
    }
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief   UART 接收事件回调(空闲中断触发)
 * @note    HAL_UARTEx_ReceiveToIdle_DMA 在两种情况触发此回调:
 *          1. 收到完整帧(DMA 传输完成)
 *          2. 总线空闲(空闲中断触发,Size 是已收到的字节数)
 *          DBUS 用情况 2:1ms 一帧,空闲中断几乎每 1ms 触发一次。
 *          HAL 头文件里这个回调是 C 链接声明,必须 extern "C",
 *          否则链接时找不到符号,中断永远进不来。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) return;   // 只处理 DT7 用的 USART3

  g_dt7_rx_cnt++;

  /* DBUS 帧固定 18 字节,校验一下,防止半帧/脏数据被当完整帧解 */
  if (Size == sizeof(g_dt7_rx_buf)) {
    auto* rc = GetRemoteControl();
    if (rc != nullptr) {
      rc->decode(g_dt7_rx_buf);
      g_dt7_decode_cnt++;

      g_dbg_rc_lv = rc->rc_lv();
      g_dbg_rc_lh = rc->rc_lh();
      g_dbg_rc_rv = rc->rc_rv();
      g_dbg_rc_rh = rc->rc_rh();
    }
  }

  /* 重启下一轮接收。HAL 默认只触发一次,不重启就只有第一帧数据,
   * 之后 rc_lh() 会一直保持第一帧的值(或者初值 0)。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dt7_rx_buf, sizeof(g_dt7_rx_buf));
  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
