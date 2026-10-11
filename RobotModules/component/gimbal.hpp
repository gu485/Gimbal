/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     : 云台运动模块（yaw 关节 + pitch 关节）
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "motor_DaMiao.hpp"
#include "pid.hpp"

namespace robot
{

class Gimbal
{
 public:
  Gimbal(void) = default;

  /* 一次性初始化:绑定电机和 PID */
  void init(void);

  /* 每拍调用(1kHz):
   *   UpdateDataChassis → 按底盘模式分支算 yaw / pitch 力矩 → 统一下发(未就绪全零)。
   * 发送 CAN 帧的动作在 CommTask 里做,这里只写进电机对象。 */
  void update(void);

  /* 调试读数 */
  uint8_t mode(void) const { return mode_; }              /* 底盘模式编号 */
  float  target_rate(void) const { return w_yaw_; }       /* yaw 目标角速度 rad/s */
  float  measure_rate(void) const { return w_current_; }  /* IMU 实测角速度 rad/s */
  float  error(void) const { return error_; }             /* yaw 误差(速度环 rad/s / 位置环 rad) */
  float  torque_cmd(void) const { return torque_cmd_; }
  bool   ready(void) const { return ready_; }
  bool   estop(void) const { return gimbal_estop_; }
  bool   spin_mode(void) const;

  /* pitch 调试读数 */
  float  pitch_target(void) const { return pitch_target_; }    /* 目标角度 rad */
  float  pitch_current(void) const { return pitch_current_; }  /* pitch1 - pitch0 */
  float  pitch0(void) const { return pitch0_; }
  float  pitch_torque(void) const { return pitch_torque_; }
  bool   pitch_ready(void) const { return pitch_calib_done_; }

 private:
  /* 更新从底盘板传回来的数据: 急停 / 模式 / 摇杆 / 在线状态, 并判定 ready_ */
  void UpdateDataChassis(void);
  /* 三个模式各自选择控制环路并算出待输出力矩 */
  void RunSeparated(void);
  void RunFollow(void);
  void RunSpin(void);
  /* yaw 速度环: 目标角速度 w_yaw_(摇杆) vs IMU 陀螺仪 z -> 力矩 */
  void YawRateLoop(void);
  /* yaw 位置环(小陀螺): 进入时采 theta_init 并定死, 之后
   * 力矩 = 符号 x PID(0, theta_now - theta_init), 带斜率限制 */
  void YawThetaLoop(void);
  /* pitch 角度环: 零位标定 + 目标角度 vs 当前角度 -> 力矩 */
  void PitchAngleLoop(bool follow_stick);
  /* 模式切换时清掉旧模式残留的状态 */
  void OnModeChange(void);
  /* 未就绪时把控制状态复位 */
  void ResetControl(void);

  hello_world::motor::DM_J4310* yaw_motor_   = nullptr;
  hello_world::motor::DM_J4310* pitch_motor_ = nullptr;
  hello_world::pid::Pid*        yaw_pid_      = nullptr;   /* 分离/跟随速度环 */
  hello_world::pid::Pid*        yaw_pid_spin_ = nullptr;   /* 小陀螺位置环 */
  hello_world::pid::Pid*        pitch_pid_    = nullptr;

  /* --- 来自底盘的数据 --- */
  uint8_t mode_         = 0;       /* 底盘模式编号(kModeXxx) */
  bool    comm_ok_      = false;   /* 双板通讯在线 */
  bool    gimbal_estop_ = false;   /* 急停: 唯一来源是底盘下发 */
  float   stick_        = 0.0f;    /* yaw 摇杆(死区后) [-1,1] */
  float   pitch_stick_  = 0.0f;    /* pitch 摇杆(死区后) [-1,1] */

  /* --- yaw --- */
  float w_yaw_      = 0.0f;   /* 目标角速度(rad/s) */
  float w_current_  = 0.0f;   /* IMU 陀螺仪 z 轴实测角速度(rad/s) */
  float error_      = 0.0f;
  float torque_cmd_ = 0.0f;   /* yaw 力矩(N·m) */
  bool  inited_     = false;
  bool  ready_      = false;  /* 急停 / IMU 未标定 / 离线 / 非工作模式时为 false */

  /* 小陀螺位置环:进入时采 kSpinInitSamples 次 IMU yaw 取平均 -> theta_init(定死) */
  uint32_t spin_calib_cnt_ = 0;
  float    spin_calib_sum_ = 0.0f;   /* 相对第一个样本的回绕差之和 */
  float    spin_calib_ref_ = 0.0f;   /* 第一个样本,用于回绕均值 */
  bool     spin_init_done_ = false;
  float    theta_init_     = 0.0f;
  float    theta_now_      = 0.0f;

  /* --- pitch --- */
  uint32_t pitch_calib_cnt_  = 0;
  float    pitch_calib_sum_  = 0.0f;
  bool     pitch_calib_done_ = false;
  float    pitch0_           = 0.0f;   /* 进入角度模式时 100 次平均 */
  float    pitch_target_     = 0.0f;
  float    pitch_current_    = 0.0f;
  float    pitch_torque_     = 0.0f;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
