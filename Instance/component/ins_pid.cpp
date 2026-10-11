/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 各控制回路 PID 单例
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"
#include "pid.hpp"

hello_world::pid::Pid *GetPidMotorYaw() {
  /* yaw 关节角速度环:误差(rad/s) → 力矩(N·m),直接填 DM4310 的 MIT torq 字段。
   *
   * 量纲:kp [N·m / (rad/s)],ki [N·m / rad],力矩限幅 ±3.0(J4310 上限 7.0,留余量)。
   * 速度环的误差没有 ±π 回绕问题,可以放心加 ki 消除摩擦造成的稳态速度差。
   *
   * 调参方向(一次只改一个,先 kp 后 ki):
   *   转不动 / 响应慢 → kp 加大
   *   抖 / 嗡嗡       → kp 减小
   *   摇杆回中后还在缓慢转 → ki 加大(同时看 i_limit)
   *   积分过冲        → ki 减小或 i_limit 减小 */
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    p = new hello_world::pid::Pid(
        /*kp=*/        1.8f,
        /*ki=*/        0.05f,
        /*kd=*/        0.0f,
        /*i_limit=*/   1.0f,
        /*out_limit=*/ 3.0f,     /* N·m,力矩限幅 */
        /*dt=*/        0.001f,
        /*beta=*/      0.2f);
  }
  return p;
}

hello_world::pid::Pid *GetPidMotorYawSpin() {
  /* 小陀螺模式 yaw 位置环 PID,与 GetPidMotorYaw()(跟随/分离速度环)独立,单独调参。
   * 输入 PID(0, theta),theta = IMU yaw − 进入时的 theta_init(rad),输出力矩 N·m。
   * 量纲: kp [N·m/rad], kd [N·m/(rad/s)]。参数沿用旧位置环起步值,上机再调。
   * ki 必须为 0: 位置环积分会把残余误差一直累积。
   *   晃/超调 → kd 加;高频嗡嗡 → beta 减小或 kd 减小;软/转不动 → kp 加 */
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    p = new hello_world::pid::Pid(
        /*kp=*/        6.0f,
        /*ki=*/        0.0f,
        /*kd=*/        0.3f,
        /*i_limit=*/   0.0f,
        /*out_limit=*/ 3.0f,
        /*dt=*/        0.001f,
        /*beta=*/      0.2f);
  }
  return p;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  /* pitch 关节角度环 PID: 输入 (目标角度, 当前角度) rad, 输出力矩 N·m。
   * kp [N·m/rad], kd [N·m/(rad/s)]。起步值偏小, 上机后再调:
   *   软/到不了位 → kp 加;晃/超调 → kd 加;高频嗡嗡 → kd 减小或 beta 减小。
   * pitch 行程有限, 误差不会跨 ±π, 无需回绕。ki 先为 0。 */
  static hello_world::pid::Pid *p = nullptr;
  if (p == nullptr) {
    p = new hello_world::pid::Pid(
        /*kp=*/        4.0f,
        /*ki=*/        0.0f,
        /*kd=*/        0.1f,
        /*i_limit=*/   0.0f,
        /*out_limit=*/ 2.0f,     /* N·m,先给小一点 */
        /*dt=*/        0.001f,
        /*beta=*/      0.2f);
  }
  return p;
}
