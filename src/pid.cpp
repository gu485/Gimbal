/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : 位置式 PID 实现(云台板专用)
 * @history   :
 *  V2.1.0      2026-10-06      gujun     1. D 项除以 dt;改成先滤波后微分
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#include "pid.hpp"

namespace hello_world {
namespace pid {

Pid::Pid(float kp, float ki, float kd, float i_limit, float out_limit,
         float dt, float beta)
    : kp_(kp), ki_(ki), kd_(kd),
      i_limit_(i_limit), out_limit_(out_limit),
      dt_(dt > 1e-6f ? dt : 1e-6f),
      inv_dt_(1.0f / (dt > 1e-6f ? dt : 1e-6f)),
      beta_(beta > 1.0f ? 1.0f : (beta < 0.0f ? 0.0f : beta)) {}

float Pid::calc(float target, float measure) {
  /* 1. 当前误差 */
  error_ = target - measure;

  /* 2. P —— 直接用原始误差。
   * 不对 P 也滤波:那会让响应整体滞后,拖慢系统。 */
  float p_out = kp_ * error_;

  /* 3. I(积分累加,带限幅防饱和) */
  i_out_ += ki_ * error_ * dt_;
  if (i_out_ > i_limit_) {
    i_out_ = i_limit_;
  } else if (i_out_ < -i_limit_) {
    i_out_ = -i_limit_;
  }

  /* 4. 误差低通,再从低通结果取微分
   *
   * 为什么是"先滤波后微分":先差分再滤波的话,滤波器作用在
   * 已经放大了 1/dt 倍的信号上,同样的滤波系数会带来更大的相位滞后,
   * 等效阻尼达不到设计值,云台就会超调振荡。
   * 先滤波后微分,滤波器作用在原始量级的误差上,滞后最小。
   *
   * 为什么必须除以 dt:差分得到的是"每拍变化量"(rad/拍),
   * 除以 dt 才是"变化速度"(rad/s),这样 kd 才有物理量纲,
   * 能按 N·m per rad/s 的常规量级整定。不除的话系数小三个数量级,
   * 阻尼形同虚设,这就是"拨杆稍多就超调"的直接原因。 */
  if (!f_valid_) {
    /* 首拍:滤波器还没"跑起来",直接用原始误差初始化,
     * 否则差分会从 0 跳到实际误差,产生一个假的速度尖峰。 */
    error_f_ = error_;
    last_error_f_ = error_;
    f_valid_ = true;
  } else {
    error_f_ = beta_ * error_ + (1.0f - beta_) * error_f_;
  }

  float d_raw = (error_f_ - last_error_f_) * inv_dt_;   /* rad/s */
  last_error_f_ = error_f_;
  float d_out = kd_ * d_raw;

  /* 5. 总输出(带限幅) */
  float output = p_out + i_out_ + d_out;
  if (output > out_limit_) {
    output = out_limit_;
  } else if (output < -out_limit_) {
    output = -out_limit_;
  }

  return output;
}

void Pid::reset() {
  /* 清空运行状态,参数保持不变。
   * f_valid_ 必须置 false:滤波器状态不清的话,恢复后第一拍的
   * 差分会拿陈旧的 error_f_ 去算,产生一个不明来源的力矩尖峰。 */
  error_        = 0.0f;
  i_out_        = 0.0f;
  error_f_      = 0.0f;
  last_error_f_ = 0.0f;
  f_valid_      = false;
}

}  // namespace pid
}  // namespace hello_world