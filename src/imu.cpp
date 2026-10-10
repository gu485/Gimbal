/* ============================================================================
 * Imu 实现
 *
 * hpp 里已经内联实现了: yaw() / pitch() / roll() / calib_done() / init_err()
 * 因此本文件只实现两个非内联成员函数:
 *   - init()           SPI 握手 + Mahony 初始化
 *   - update()         每帧读数据 + 解算
 * ==========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"
#include "spi.h"
#include "gpio.h"

#include <cmath>

namespace hello_world
{
namespace imu
{

/* 静态单例指针(进程级只有一个 Imu) */
static Imu* s_imu = nullptr;

Imu* Imu::GetImu(void)
{
  if (s_imu == nullptr) {
    s_imu = new Imu();
  }
  return s_imu;
}

/* ==========================================================================*/
/* init() —— SPI 握手 + Mahony 初始化                                          */
/* ==========================================================================*/
BMI088ErrState Imu::init(void)
{
  /* ① 硬件配置: SPI + 两路 CS */
  static const BMI088HWConfig kHwConfig = {
    .hspi         = &hspi1,
    .acc_cs_port  = GPIOA,
    .acc_cs_pin   = GPIO_PIN_4,
    .gyro_cs_port = GPIOB,
    .gyro_cs_pin  = GPIO_PIN_0,
  };

  /* ② 旋转矩阵: 先用单位阵(假设芯片轴向和世界轴向对齐) */
  static const float kRotationMatrix[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
  };

  /* ③ 设备配置: 用默认(±3g, ±1000dps, 1kHz) */
  static const BMI088Config kDevConfig = {};

  /* ④ 实例化 BMI088 */
  dev_ = BMI088(kHwConfig, kRotationMatrix, kDevConfig);

  /* ⑤ 实例化 Mahony(必须 new,因为继承 MemMgr)
   * 注意:ahrs 是 hello_world 的子命名空间,当前代码已位于
   * hello_world::imu 内,所以写 ahrs::Mahony 而不是裸的 Mahony。 */
  ahrs_ = new ahrs::Mahony(1000.0f, 1.0f, 0.0f);

  /* ⑥ 真正读芯片寄存器(内部会 HAL_Delay),存错误码 + 返回 */
  init_err_ = dev_.imuInit(false);
  return init_err_;
}

/* ==========================================================================*/
/* update() —— 每帧:读数据 + 标定 + 解算                                          */
/* ==========================================================================*/
void Imu::update(void)
{
  /* ① 读 SPI 数据,直接写进成员变量 */
  dev_.getData(acc_data_, gyro_data_, nullptr);

  /* ② 阶段一: 零漂标定(前 1000 帧 = 1 秒) */
  if (!calib_done_) {
    /* 静止检测: 任一轴角速度超过阈值说明云台在动/被碰,
     * 此前累加的样本作废, 从头重新采。否则带着运动量算出的零漂会一直残留。 */
    if (fabsf(gyro_data_[0]) > kCalibMotionThres ||
        fabsf(gyro_data_[1]) > kCalibMotionThres ||
        fabsf(gyro_data_[2]) > kCalibMotionThres) {
      calib_cnt_ = 0;
      gyro_offset_[0] = gyro_offset_[1] = gyro_offset_[2] = 0.0f;
      return;
    }

    gyro_offset_[0] += gyro_data_[0];
    gyro_offset_[1] += gyro_data_[1];
    gyro_offset_[2] += gyro_data_[2];

    if (++calib_cnt_ >= kCalibSamples) {
      gyro_offset_[0] /= kCalibSamples;
      gyro_offset_[1] /= kCalibSamples;
      gyro_offset_[2] /= kCalibSamples;
      calib_done_ = true;
    }
    return;  // 标定期间不喂 Mahony
  }

  /* ③ 阶段二: 正常解算 */
  gyro_data_[0] -= gyro_offset_[0];
  gyro_data_[1] -= gyro_offset_[1];
  gyro_data_[2] -= gyro_offset_[2];

  ahrs_->update(acc_data_, gyro_data_);
  ahrs_->getEulerAngle(euler_);
}

}  // namespace imu
}  // namespace hello_world