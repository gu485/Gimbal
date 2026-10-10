/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 位置式 PID 控制器(云台板专用:输出力矩,微分项带采样周期归一化)
 * @history   :
 *  V2.0.0      2026-10-06      gujun     1. 增加一阶滤波 D 项 + 出力矩限幅
 *  V2.1.0      2026-10-06      gujun     1. D 项除以采样周期,恢复物理量纲
 *                                      2. 改成"先滤波后微分",降低相位滞后
 *******************************************************************************
 * @attention :
 *  1. 位置式 PID(含积分限幅 + 输出限幅),输出为力矩 N·m
 *  2. ⚠ D 项必须除以采样周期 dt,否则量纲是"每拍变化量"而不是"变化速度"。
 *     1kHz 下不归一化会让 D 项系数小三个数量级,阻尼形同虚设,
 *     表现为拨杆稍多就超调、云台来回晃,而且怎么调 kd 都没用。
 *  3. 微分采用"先滤波后微分"(derivative on measurement):
 *     先对误差低通再取差分。相比"先差分再滤波",相位滞后小得多,
 *     同样滤波系数下阻尼效果更接近理论值。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
#ifndef GIMBAL_PID_HPP_
#define GIMBAL_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
  /**
   * @param       kp:        比例系数      [N·m / rad]
   * @param       ki:        积分系数      [N·m / (rad·s)]
   * @param       kd:        微分系数      [N·m / (rad/s)]  ← 已按 dt 归一化
   * @param       i_limit:   积分限幅(防积分饱和)
   * @param       out_limit: 输出限幅(力矩 N·m)
   * @param       dt:        采样周期(s)。控制循环 1kHz 就填 0.001f。
   *                         这个值不对,D 项量纲会整体错一个数量级。
   * @param       beta:      误差低通系数 (0,1]。越大越跟随原始信号、
   *                         但噪声越大。0.15~0.3 比较常用。
   *                         1.0 表示不滤波。
   */
  Pid(float kp, float ki, float kd, float i_limit, float out_limit,
      float dt = 0.001f, float beta = 0.2f);

  ~Pid() = default;

  /**
   * @brief       位置式 PID 计算
   * @param       target:  目标值
   * @param       measure: 实测值
   * @retval       本次输出(云台 yaw 用作力矩 N·m)
   */
  float calc(float target, float measure);

  /**
   * @brief       清空 PID 内部状态(含误差滤波缓存)
   * @note        切换目标/进急停时必须调用,否则残留的滤波状态
   *              会在恢复后第一拍产生一个假尖峰力矩。
   */
  void reset();

 private:
  float kp_        = 0.0f;
  float ki_        = 0.0f;
  float kd_        = 0.0f;
  float i_limit_   = 0.0f;
  float out_limit_ = 0.0f;
  float dt_        = 0.001f;
  float inv_dt_    = 1000.0f;
  float beta_      = 0.2f;

  float error_     = 0.0f;   /* 原始误差 */
  float i_out_     = 0.0f;
  /* 误差的低通结果。D 项从这里取差分,而不是从原始误差取。 */
  float error_f_   = 0.0f;
  float last_error_f_ = 0.0f;
  /* 首次 calc 时 error_f_ 还没初始化,直接用原始误差填充,
   * 否则第一拍的差分会是一个假的大跳变(从 0 到实际误差)。 */
  bool  f_valid_   = false;
};

}  // namespace pid
}  // namespace hello_world
#endif /* GIMBAL_PID_HPP_ */