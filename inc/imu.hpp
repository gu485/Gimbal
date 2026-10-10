/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装
 * @history   :
 *  V2.0.0    2026-10-06    gujun     1. 完成 BMI088 + Mahony 封装
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "ahrs_base.hpp"
#include "mahony.hpp"

namespace hello_world
{
namespace imu
{

class Imu
{
 public:
  Imu(void) = default;

  /* 单例入口 */
  static Imu* GetImu(void);

  /* 对外接口 */
  BMI088ErrState init(void);
  void        update(void);

  /* 查询接口 */
  bool         calib_done(void) const { return calib_done_; }
  BMI088ErrState  init_err(void)   const { return init_err_; }

  /* 标定进度:返回已累加的帧数,满 kCalibSamples 才算标完。
   * 仅供调试:能一眼看出 update() 到底有没有被反复调用。 */
  uint32_t      calib_progress(void) const { return calib_cnt_; }

  /* 姿态角 getter: 输出参数风格 */
  void roll (float* roll_out)  const { *roll_out  = euler_[0]; }
  void pitch(float* pitch_out) const { *pitch_out = euler_[1]; }
  void yaw  (float* yaw_out)   const { *yaw_out   = euler_[2]; }

  /* 原始数据 getter: 给调试镜像用(每帧覆写) */
  void acc (float acc_out[3])  const {
    for (int i = 0; i < 3; i++) acc_out[i]  = acc_data_[i];
  }
  void gyro(float gyro_out[3]) const {
    for (int i = 0; i < 3; i++) gyro_out[i] = gyro_data_[i];
  }

 private:
  BMI088           dev_;
  ahrs::Mahony*    ahrs_ = nullptr;

  float acc_data_[3]   = {0, 0, 0};   /* 单位 g */
  float gyro_data_[3]  = {0, 0, 0};   /* 单位 rad/s */
  float euler_[3]      = {0, 0, 0};   /* [roll, pitch, yaw] rad */

  float gyro_offset_[3] = {0, 0, 0};
  uint32_t calib_cnt_   = 0;
  bool     calib_done_  = false;

  BMI088ErrState init_err_ = kBMI088ErrStateNoErr;

  static constexpr uint32_t kCalibSamples = 1000;
  /* 标定期间单轴角速度超过此值(rad/s)视为非静止, 重新开始采样 */
  static constexpr float    kCalibMotionThres = 0.1f;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */