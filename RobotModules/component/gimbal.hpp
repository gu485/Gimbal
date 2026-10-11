/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     : 云台运动模块（yaw 关节，角速度环）
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

  /* 每拍调用(1kHz)。
   * 摇杆 → 目标角速度 w_yaw;PID(w_yaw, IMU 陀螺仪 z) → 力矩 → MIT 帧。
   * 发送动作在 CommTask 里做,这里只算控制量并写进电机对象。 */
  void update(void);

  /* 调试读数 */
  float  target_rate(void) const { return w_yaw_; }       /* 目标角速度 rad/s */
  float  measure_rate(void) const { return w_current_; }  /* IMU 实测角速度 rad/s */
  float  error(void) const { return error_; }             /* 速度误差 rad/s */
  float  torque_cmd(void) const { return torque_cmd_; }
  bool   ready(void) const { return ready_; }

  /* 急停状态,底盘经双板通讯下发 */
  bool   estop(void) const { return estop_; }
  /* 当前是否使用小陀螺 PID(底盘经双板通讯下发) */
  bool   spin_mode(void) const { return spin_mode_; }

  /* pitch 调试读数 */
  float  pitch_target(void) const { return pitch_target_; }    /* 目标角度 rad */
  float  pitch_current(void) const { return pitch_current_; }  /* pitch1 − pitch0 */
  float  pitch0(void) const { return pitch0_; }
  float  pitch_torque(void) const { return pitch_torque_; }
  bool   pitch_ready(void) const { return pitch_calib_done_; }

 private:
  /* 摇杆 → 目标角速度 w_yaw */
  void  UpdateTargetRate(void);
  /* pitch 角度环:零位标定 + 摇杆→目标角度 + PID→力矩 */
  void  UpdatePitch(void);
  /* w_yaw 与 IMU 角速度 → 速度环 PID → 力矩 */
  void  UpdateTorqueCmd(void);

  hello_world::motor::DM_J4310* yaw_motor_ = nullptr;
  hello_world::motor::DM_J4310* pitch_motor_ = nullptr;
  hello_world::pid::Pid*        pitch_pid_   = nullptr;
  hello_world::pid::Pid*        yaw_pid_   = nullptr;       /* 跟随/分离速度环 */
  hello_world::pid::Pid*        yaw_pid_spin_ = nullptr;    /* 小陀螺速度环(独立单例) */

  float w_yaw_       = 0.0f;   /* 目标角速度(rad/s),摇杆 × 系数 */
  float w_current_   = 0.0f;   /* IMU 陀螺仪 z 轴实测角速度(rad/s) */
  float error_       = 0.0f;
  float torque_cmd_  = 0.0f;   /* PID 输出力矩(N·m),填 MIT 帧 torq 字段 */
  float stick_       = 0.0f;
  bool  inited_      = false;
  bool  ready_       = false;  /* IMU 标定未完成 / 急停时为 false */
  bool  estop_       = false;
  /* pitch 角度环状态 */
  uint32_t pitch_calib_cnt_  = 0;
  float    pitch_calib_sum_  = 0.0f;
  bool     pitch_calib_done_ = false;
  float    pitch0_           = 0.0f;   /* 刚进入角度模式时 100 次平均 */
  float    pitch_target_     = 0.0f;
  float    pitch_current_    = 0.0f;
  float    pitch_torque_     = 0.0f;
  bool  spin_mode_   = false;  /* 小陀螺模式:位置环 */
  /* 小陀螺位置环:进入时采 kSpinInitSamples 次 IMU yaw 取平均 → theta_init(定死) */
  uint32_t spin_calib_cnt_  = 0;
  float    spin_calib_sum_  = 0.0f;   /* 相对第一个样本的回绕差之和 */
  float    spin_calib_ref_  = 0.0f;   /* 第一个样本,用于回绕均值 */
  bool     spin_init_done_  = false;
  float    theta_init_      = 0.0f;
  float    theta_now_       = 0.0f;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
