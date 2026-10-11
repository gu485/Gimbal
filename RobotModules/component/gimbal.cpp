/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     : 云台运动模块实现（yaw 关节角速度环）
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include <cmath>

#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_chassis_gimbal_comm.hpp"  // GetGimbalChassisComm()
#include "gimbal_chassis_comm.hpp"      // robot::GimbalChassisComm 完整定义
#include "ins_imu.hpp"     // GetImu()
#include "imu.hpp"         // Imu 完整定义

/* Private constants ---------------------------------------------------------*/
namespace
{
/* 摇杆满杆对应的云台目标角速度(rad/s):w_yaw = ±kYawRateKx × 摇杆。
 * 负号 = 摇杆方向与云台转向互换(顺逆时针颠倒)。 */
constexpr float kYawRateKx = 2.0f;
/* IMU 陀螺仪 z 轴 → 云台 yaw 角速度的符号。
 * 实测推摇杆后云台往反方向转 / 越转越快 → 改成 -1。 */
constexpr float kYawRateSign = 1.0f;
/* 摇杆死区:DBUS 量化后中心附近有抖动,不滤会一直微动。 */
constexpr float kStickDeadzone = 0.05f;
/* 摇杆/拨杆信号异常的判据:DBUS 解出来应该在 [-1, 1],超出是脏数据。 */
constexpr float kRcValidRange = 1.2f;
/* === pitch 角度环 === */
constexpr uint32_t kPitchCalibSamples = 100;   /* 进入角度模式时的零位采样次数 */
/* 摇杆满杆对应的目标角度(rad): pitch_target = k_pitch × 摇杆。方向反了改负号。 */
constexpr float kPitchK = -1.0f;
/* 目标角度限幅(rad),机械行程未标定前的安全保护 */
constexpr float kPitchLimit = 1.0f;
constexpr float kPitchStickDeadzone = 0.05f;
/* 小陀螺进入时采集 IMU yaw 的次数(1kHz)。
 *
 * ⚠ 2026-10-10 从 1000 缩到 200:采集期间 torque_cmd_ 恒为 0,云台是
 *   完全无阻尼状态;而小陀螺下底盘正在以 ~1.5 rad/s 持续自转,会拖着
 *   云台一起转。采 1000 帧(1 秒)意味着云台被拖转 ~1.5 rad ≈ 86°,
 *   等 PID 上线要一次性拉回来,必然大幅超调。
 *   200 帧(0.2 秒)被拖转 ~0.3 rad,拉回量小一个量级。
 *   取平均仍然保留(压抖动),但采样窗口缩短。 */
constexpr uint32_t kSpinInitSamples = 200;
/* 小陀螺位置环:力矩符号。
 * 2026-10-10 实测 -1(原来 +1):力矩方向反了,云台偏离目标后越修越偏。
 *
 * 正负号取决于两件事的组合,只能实测:
 *   ① IMU yaw 的正方向 和 力矩正方向 在俯视坐标系下是否同向
 *   ② yaw 电机 opt.dir = kDirRev
 * 这两样叠加出一个总符号,和跟随/分离速度环的 kYawRateSign 是两套东西
 * (那个管的是角速度反馈方向,这个管的是位置环力矩方向),
 * 所以小陀螺单独开一个常量,不要共用。
 *
 * 改这个值之前先确认现象是"反向"而不是"发散":
 *   反向 = 云台被拉离目标方位,error 绝对值持续变大
 *   正向但 PID 太软 = error 变大但力度不够,只是收敛慢
 * 力矩正负和 out_limit 的关系:PID 内部算完力矩再落到底盘,
 * 所以这里改符号不会绕过 3.0 N·m 的限幅。 */
constexpr float kSpinTorqueSign = -1.0f;
/* 小陀螺位置环力矩斜率限制:每拍(N·m/拍)。
 *
 * 原本 PID 上线瞬间 torque_cmd_ 会从 0 跳到 kp·θ。theta 若为 0.3 rad、
 * kp=6.0,原始输出 1.8 N·m,被 out_limit 3.0 限幅后仍是个阶跃,
 * 表现为进模式时电机被猛地推一下。
 *
 * 斜率限制让力矩从当前值平滑爬向 PID 目标,消除这个阶跃。
 * 0.005 N·m/拍 × 1000 拍/s = 5.0 N·m/s,爬满量程(3 N·m)约需 0.6 秒,
 * 跟小陀螺自转速度同量级,不会因为限速而跟不上底盘的拖转。 */
constexpr float kSpinTorqueStep = 0.005f;
constexpr float kPi = 3.14159265f;
float WrapPi(float a) {
  while (a >  kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}
}  // namespace

namespace robot
{

bool Gimbal::spin_mode(void) const { return mode_ == kModeSpin; }

void Gimbal::init(void)
{
  if (inited_) return;

  yaw_motor_    = GetMotorYaw();
  pitch_motor_  = GetMotorPitch();
  yaw_pid_      = GetPidMotorYaw();
  yaw_pid_spin_ = GetPidMotorYawSpin();
  pitch_pid_    = GetPidMotorPitch();
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();

  inited_ = true;
  ready_  = false;
}

void Gimbal::OnModeChange(void)
{
  /* 换模式: 两个 yaw PID 都复位, 旧模式的积分 / 误差滤波不带进新模式。
   * 力矩也清零: 小陀螺的斜率限制是从 torque_cmd_ 往上爬的,
   * 不清的话退出小陀螺时残留的力矩会成为下次的起点。
   * pitch 不动: 分离 / 跟随 / 小陀螺之间切换时 pitch 的零位和目标应保持。 */
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();
  torque_cmd_ = 0.0f;
  w_yaw_      = 0.0f;
  error_      = 0.0f;
  spin_init_done_ = false;   /* 每次进入小陀螺都重新采 theta_init */
  spin_calib_cnt_ = 0;
  spin_calib_sum_ = 0.0f;
}

void Gimbal::ResetControl(void)
{
  /* 未就绪: 力矩 0, PID 复位避免积分累积, pitch 零位作废(恢复后重新标定) */
  torque_cmd_   = 0.0f;
  pitch_torque_ = 0.0f;
  error_        = 0.0f;
  w_yaw_        = 0.0f;
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();
  spin_init_done_ = false;
  spin_calib_cnt_ = 0;
  spin_calib_sum_ = 0.0f;
  pitch_calib_done_ = false;
  pitch_calib_cnt_  = 0;
  pitch_calib_sum_  = 0.0f;
  pitch_target_ = pitch_current_ = 0.0f;
  if (pitch_pid_ != nullptr) pitch_pid_->reset();
}

void Gimbal::UpdateDataChassis(void)
{
  auto* comm = GetGimbalChassisComm();
  auto* imu  = GetImu();

  comm_ok_ = (comm != nullptr) && !comm->isOffline();

  /* 急停唯一来源: 底盘经双板通讯下发的 estop 位。
   * 后面所有急停下的动作都只读 gimbal_estop_, 不在别处再赋值。 */
  gimbal_estop_ = (comm == nullptr) || (comm->chassis_data().estop != 0u);

  stick_ = 0.0f;
  pitch_stick_ = 0.0f;

  if (comm_ok_) {
    /* 底盘模式; 编号越界按 INIT 处理 */
    uint8_t m = comm->chassis_data().work_mode;
    if (m > kModeEmergency) m = kModeInit;
    if (m != mode_) {
      mode_ = m;
      OnModeChange();
    }

    /* 摇杆: 反量化 + 有效性 + 死区。云台使能位为 0 时忽略摇杆。 */
    if (comm->chassis_data().gimbal_enabled != 0u) {
      const float lh = GimbalChassisComm::DequantizeStick(comm->chassis_data().rc_lh_q);
      const float lv = GimbalChassisComm::DequantizeStick(comm->chassis_data().rc_lv_q);
      if (lh >= -kRcValidRange && lh <= kRcValidRange &&
          (lh <= -kStickDeadzone || lh >= kStickDeadzone)) {
        stick_ = lh;
      }
      if (lv >= -kRcValidRange && lv <= kRcValidRange &&
          (lv <= -kPitchStickDeadzone || lv >= kPitchStickDeadzone)) {
        pitch_stick_ = lv;
      }
    }
  }

  /* 就绪: 通讯在线 + 非急停 + IMU 标定完成 + 底盘处于三个工作模式之一 */
  const bool imu_ok = (imu != nullptr) && imu->calib_done();
  const bool mode_ok = (mode_ == kModeSeparated || mode_ == kModeFollow ||
                        mode_ == kModeSpin);
  ready_ = comm_ok_ && !gimbal_estop_ && imu_ok && mode_ok;
}

void Gimbal::YawRateLoop(void)
{
  /* ① 摇杆 x 系数 = 云台 yaw 目标角速度 w_yaw(rad/s) */
  w_yaw_ = kYawRateKx * stick_;

  /* 实测角速度: IMU 陀螺仪 z 轴(已扣开机零漂) */
  float g3[3] = {0.0f, 0.0f, 0.0f};
  GetImu()->gyro(g3);
  w_current_ = kYawRateSign * g3[2];

  /* ② PID(w_yaw, w_current) -> 力矩(N·m) */
  error_ = w_yaw_ - w_current_;
  torque_cmd_ = yaw_pid_->calc(w_yaw_, w_current_);
}

void Gimbal::PitchAngleLoop(bool follow_stick)
{
  pitch_torque_ = 0.0f;
  if (pitch_motor_ == nullptr || pitch_pid_ == nullptr) return;

  /* pitch 电机未使能: 力矩 0, 零位作废 */
  if (!pitch_motor_->is_enabled()) {
    pitch_calib_done_ = false;
    pitch_calib_cnt_  = 0;
    pitch_calib_sum_  = 0.0f;
    pitch_target_ = pitch_current_ = 0.0f;
    pitch_pid_->reset();
    return;
  }

  /* ① 刚进入角度模式: 采 100 次 pitch 电机反馈角度取平均 -> pitch0, 采集期间力矩 0 */
  if (!pitch_calib_done_) {
    pitch_calib_sum_ += pitch_motor_->angle();
    if (++pitch_calib_cnt_ >= kPitchCalibSamples) {
      pitch0_ = pitch_calib_sum_ / static_cast<float>(kPitchCalibSamples);
      pitch_calib_done_ = true;
      pitch_target_ = 0.0f;
      pitch_pid_->reset();
    }
    return;
  }

  /* ② 摇杆上下 x k_pitch = 目标角度(不是角速度)。
   * 小陀螺下左杆被征用成平移, follow_stick=false, 目标保持不变。 */
  if (follow_stick) {
    float t = kPitchK * pitch_stick_;
    if (t >  kPitchLimit) t =  kPitchLimit;
    if (t < -kPitchLimit) t = -kPitchLimit;
    pitch_target_ = t;
  }

  /* ③④ pitch1 = 反馈角度, pitch_current = pitch1 - pitch0 */
  const float pitch1 = pitch_motor_->angle();
  pitch_current_ = pitch1 - pitch0_;

  /* ⑤ PID(目标, 当前) -> 力矩 */
  pitch_torque_ = pitch_pid_->calc(pitch_target_, pitch_current_);
}

void Gimbal::RunSeparated(void)
{
  /* 分离: yaw 速度环(摇杆 -> 角速度), pitch 角度环(摇杆 -> 角度)。
   * 目前与跟随共用同一个 yaw 速度环 PID; 以后要分开调参, 在这里换成独立的 PID 单例。 */
  YawRateLoop();
  PitchAngleLoop(true);
}

void Gimbal::RunFollow(void)
{
  /* 跟随: yaw 速度环, pitch 角度环。底盘追云台的部分在底盘板, 这里与分离一致。 */
  YawRateLoop();
  PitchAngleLoop(true);
}

void Gimbal::YawThetaLoop(void)
{
  /* ① 进入时采 kSpinInitSamples 次 IMU yaw 取平均 -> theta_init(之后定死)
   * ② theta_now = IMU yaw;theta = theta_now - theta_init(回绕到 [-pi, pi))
   * ③ 力矩 = 符号 x PID(0, theta)(用 GetPidMotorYawSpin 单例), 再过斜率限制 */
  float psi = 0.0f;
  GetImu()->yaw(&psi);
  theta_now_ = psi;

  if (!spin_init_done_) {
    /* 角度求均值要处理回绕:以第一个样本为参考累加回绕差 */
    if (spin_calib_cnt_ == 0) spin_calib_ref_ = psi;
    spin_calib_sum_ += WrapPi(psi - spin_calib_ref_);
    if (++spin_calib_cnt_ >= kSpinInitSamples) {
      theta_init_ = WrapPi(spin_calib_ref_ +
                           spin_calib_sum_ / static_cast<float>(kSpinInitSamples));
      spin_init_done_ = true;
      yaw_pid_spin_->reset();
    }
    /* 采集期间力矩 0 */
    torque_cmd_ = 0.0f;
    error_ = 0.0f;
    return;
  }

  const float theta = WrapPi(theta_now_ - theta_init_);
  error_ = theta;

  /* PID 目标力矩 -> 斜率限制 -> 实际下发力矩。
   * 斜率限制只加在位置环上: 速度环靠 kp 直接响应, 加限速会让跟手变肉。 */
  const float torque_raw = kSpinTorqueSign * yaw_pid_spin_->calc(0.0f, theta);
  if (torque_raw > torque_cmd_ + kSpinTorqueStep) {
    torque_cmd_ += kSpinTorqueStep;
  } else if (torque_raw < torque_cmd_ - kSpinTorqueStep) {
    torque_cmd_ -= kSpinTorqueStep;
  } else {
    torque_cmd_ = torque_raw;
  }
}

void Gimbal::RunSpin(void)
{
  /* 小陀螺: yaw 位置环(锁定进入时的朝向), pitch 角度环(目标保持不变)。 */
  YawThetaLoop();
  PitchAngleLoop(false);
}

void Gimbal::update(void)
{
  if (!inited_) init();

  /* 电机或 PID 不可用时不能往下走,否则 setMitInput / calc 打在空指针上 */
  if (yaw_motor_ == nullptr || yaw_pid_ == nullptr || yaw_pid_spin_ == nullptr) {
    ready_ = false;
    return;
  }

  UpdateDataChassis();

  if (ready_) {
    switch (mode_) {
      case kModeSeparated: RunSeparated(); break;
      case kModeFollow:    RunFollow();    break;
      case kModeSpin:      RunSpin();      break;
      default:             ready_ = false; break;   /* 不会走到: ready_ 已含模式判断 */
    }
  }
  if (!ready_) {
    ResetControl();
  }

  if (!ready_) {
    /* 急停 / IMU 未就绪 / 离线: yaw 和 pitch 五个参数全 0, 电机不出力。
     * 仍要发帧(达妙不发指令就认为通信丢失)。 */
    yaw_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (pitch_motor_ != nullptr) {
      pitch_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    return;
  }

  /* MIT 帧只用力矩字段:pos / vel / kp / kd 全 0,电机只当力矩执行器 */
  yaw_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, torque_cmd_);
  if (pitch_motor_ != nullptr) {
    pitch_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, pitch_torque_);
  }
}

}  // namespace robot
