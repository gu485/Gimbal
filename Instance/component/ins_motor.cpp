/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : 电机单例
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"

hello_world::motor::DM_J4310 *GetMotorYaw() {
  /* yaw 关节电机单例。
   * 参数直接从底盘板(Chassis (2)/ins_motor.cpp)搬过来,那些是实测调出来的:
   *   - id=2:达妙的 CAN 控制 ID
   *   - dir=kDirFwd:已实测确认方向
   *   - angle_range=kNegPiToPosPi:不加这个,驱动开机收到第一帧反馈时内部
   *     "过零计圈"逻辑会把默认值 0 rad 和真实角度一比,误判成转了一整圈,
   *     给圈数计数器凭空加 ±1,angle() 会一直带约 ±2π 的固定偏移(实测 -5.7)。
   *     用 kNegPiToPosPi 让 normAngle() 把角度折回 [-π, π) 抵消这个偏移。
   *   - angle_offset=-1.3f:机械零点补偿,云台转到正对底盘正前方时 angle() ≈ 0
   *   - enable_mode_switch=false:实测速度模式(setVelInput)不可靠,怀疑总线不稳定
   *     导致模式切换握手卡死。改用 MIT 模式,不需要任何模式切换。
   * auto_enable=true:encode() 内部没使能就自动发使能帧。 */
  static hello_world::motor::DM_J4310 *motor = nullptr;
  if (motor == nullptr) {
    hello_world::motor::OptionalParams opt;
    /* 方向必须和 IMU yaw 的正方向一致,否则闭环变成正反馈:
     * 拨左摇杆 → target 增大 → 电机往反方向转 → psi 减小
     * → error 越来越大 → 越转越猛,云台会一直转到撞机械限位。
     * 实测 kDirFwd 是反的,所以用 kDirRev。
     *
     * 改 dir 不影响 IMU 读数(那是加速度计/陀螺仪硬件直出),
     * 只影响电机的实际转向,正是这里要修的。 */
    opt.dir          = hello_world::motor::kDirRev;
    opt.input_type   = hello_world::motor::InputType::kTorq;
    opt.angle_range  = hello_world::motor::AngleRange::kNegPiToPosPi;
    opt.angle_offset = -1.3f;
    motor = new hello_world::motor::DM_J4310(
        /*id=*/2, opt, /*auto_enable=*/true, /*enable_mode_switch=*/false);
  }
  return motor;
}