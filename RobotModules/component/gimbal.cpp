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

void Gimbal::init(void)
{
  if (inited_) return;

  yaw_motor_ = GetMotorYaw();
  yaw_pid_   = GetPidMotorYaw();
  pitch_motor_ = GetMotorPitch();
  pitch_pid_   = GetPidMotorPitch();
  yaw_pid_spin_ = GetPidMotorYawSpin();
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();

  inited_ = true;
  ready_  = false;
}

void Gimbal::UpdateTargetRate(void)
{
  /* 云台板没有接 DR16,摇杆和急停都从底盘经双板通讯转发过来。 */
  auto* comm = GetGimbalChassisComm();
  if (comm == nullptr) {
    estop_ = true;
    w_yaw_ = 0.0f;
    stick_ = 0.0f;
    return;
  }

  /* 底盘离线:目标角速度置 0,但不修改 estop_。
   * OfflineChecker 初始就是 offline,上电第一帧到来之前如果锁存,
   * 通信建立后永远解不开。 */
  if (comm->isOffline()) {
    w_yaw_ = 0.0f;
    stick_ = 0.0f;
    return;
  }

  /* 模式切换(跟随/分离 ↔ 小陀螺):换 PID 时两个都复位,避免旧积分带进新模式 */
  const bool spin_now = (comm->chassis_data().spin_mode != 0u);
  if (spin_now != spin_mode_) {
    spin_mode_ = spin_now;
    spin_init_done_ = false;   /* 每次进入/退出小陀螺都重新采 theta_init */
    spin_calib_cnt_ = 0;
    spin_calib_sum_ = 0.0f;
    if (yaw_pid_ != nullptr) yaw_pid_->reset();
    if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();
    /* 斜率限制是从 torque_cmd_ 往上/往下爬的,这里必须清零:
     * 不清的话退出小陀螺时残留的力矩会成为下次进入小陀螺的起点,
     * 相当于一进模式就带着一个来历不明的力矩。 */
    torque_cmd_ = 0.0f;
  }

  float raw = GimbalChassisComm::DequantizeStick(comm->chassis_data().rc_lh_q);

  /* 云台使能位:小陀螺下底盘置 0(左摇杆左右被征用成横移)。
   * 忽略摇杆,目标角速度 0,云台只靠速度环阻尼保持不转。 */
  if (comm->chassis_data().gimbal_enabled == 0u) {
    w_yaw_ = 0.0f;
    stick_ = 0.0f;
    return;
  }

  /* 信号有效性:摇杆值超出物理范围 = 脏数据 */
  if (raw < -kRcValidRange || raw > kRcValidRange) {
    estop_ = true;
    w_yaw_ = 0.0f;
    return;
  }

  /* 急停:底盘下发。通信在线且底盘没发急停时解除,避免初始离线永久锁死。 */
  estop_ = (comm->chassis_data().estop != 0u);
  if (estop_) {
    w_yaw_ = 0.0f;
    stick_ = 0.0f;
    return;
  }

  float stick = raw;
  if (stick > -kStickDeadzone && stick < kStickDeadzone) stick = 0.0f;
  stick_ = stick;

  /* ① 摇杆 × 系数 = 云台 yaw 目标角速度 w_yaw(rad/s) */
  w_yaw_ = kYawRateKx * stick;

}

void Gimbal::UpdateTorqueCmd(void)
{
  auto* imu = GetImu();
  /* 小陀螺用独立 PID,其余模式用跟随/分离 PID */
  hello_world::pid::Pid* pid = spin_mode_ ? yaw_pid_spin_ : yaw_pid_;

  if (estop_ || imu == nullptr || !imu->calib_done() || pid == nullptr) {
    /* 急停或 IMU 没标定完:力矩 0,PID 复位避免积分累积 */
    ready_ = false;
    torque_cmd_ = 0.0f;
    error_ = 0.0f;
    if (yaw_pid_ != nullptr) yaw_pid_->reset();
    if (yaw_pid_spin_ != nullptr) yaw_pid_spin_->reset();
    return;
  }

  /* === 小陀螺:yaw 位置环 ===
   * ① 进入时采 kSpinInitSamples 次 IMU yaw 取平均 → theta_init(之后定死)
   * ② theta_now = IMU yaw;theta = theta_now − theta_init(回绕到 [-π, π))
   * ③ 力矩 = PID(0, theta)(用 GetPidMotorYawSpin 单例),喂给 MIT 帧 */
  if (spin_mode_) {
    float psi = 0.0f;
    imu->yaw(&psi);
    theta_now_ = psi;

    if (!spin_init_done_) {
      /* 角度求均值要处理回绕:以第一个样本为参考累加回绕差 */
      if (spin_calib_cnt_ == 0) spin_calib_ref_ = psi;
      spin_calib_sum_ += WrapPi(psi - spin_calib_ref_);
      if (++spin_calib_cnt_ >= kSpinInitSamples) {
        theta_init_ = WrapPi(spin_calib_ref_ +
                             spin_calib_sum_ / static_cast<float>(kSpinInitSamples));
        spin_init_done_ = true;
        pid->reset();
      }
      /* 采集期间力矩 0 */
      torque_cmd_ = 0.0f;
      error_ = 0.0f;
      ready_ = true;
      return;
    }

    const float theta = WrapPi(theta_now_ - theta_init_);
    error_ = theta;

    /* PID 目标力矩 → 斜率限制 → 实际下发力矩。
     * 斜率限制只加在小陀螺位置环上:跟随/分离的速度环不加,
     * 那里本来就靠 kp 直接给响应,加限速反而会让跟手变肉。 */
    const float torque_raw = kSpinTorqueSign * pid->calc(0.0f, theta);
    if (torque_raw > torque_cmd_ + kSpinTorqueStep) {
      torque_cmd_ += kSpinTorqueStep;
    } else if (torque_raw < torque_cmd_ - kSpinTorqueStep) {
      torque_cmd_ -= kSpinTorqueStep;
    } else {
      torque_cmd_ = torque_raw;
    }
    ready_ = true;
    return;
  }

  /* 实测角速度:IMU 陀螺仪 z 轴(已扣开机零漂) */
  float g3[3] = {0.0f, 0.0f, 0.0f};
  imu->gyro(g3);
  w_current_ = kYawRateSign * g3[2];

  /* PID(w_yaw, w_current) → 力矩(N·m) */
  error_ = w_yaw_ - w_current_;
  torque_cmd_ = pid->calc(w_yaw_, w_current_);
  ready_ = true;
}

void Gimbal::UpdatePitch(void)
{
  if (pitch_motor_ == nullptr || pitch_pid_ == nullptr) return;

  auto* comm = GetGimbalChassisComm();
  /* 通信离线 / 急停 / 电机未使能: 力矩 0, 下次重新标定零位 */
  if (comm == nullptr || comm->isOffline() || estop_ ||
      !pitch_motor_->is_enabled()) {
    pitch_calib_done_ = false;
    pitch_calib_cnt_  = 0;
    pitch_calib_sum_  = 0.0f;
    pitch_target_ = pitch_current_ = pitch_torque_ = 0.0f;
    pitch_pid_->reset();
    pitch_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  /* ① 刚进入角度模式: 采 100 次 pitch 电机反馈角度取平均 → pitch0, 采集期间力矩 0 */
  if (!pitch_calib_done_) {
    pitch_calib_sum_ += pitch_motor_->angle();
    if (++pitch_calib_cnt_ >= kPitchCalibSamples) {
      pitch0_ = pitch_calib_sum_ / static_cast<float>(kPitchCalibSamples);
      pitch_calib_done_ = true;
      pitch_target_ = 0.0f;
      pitch_pid_->reset();
    }
    pitch_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  /* ② 摇杆上下 × k_pitch = 目标角度(不是角速度)。
   * 小陀螺下左杆被征用成平移,pitch 目标保持不变。 */
  if (!spin_mode_ && comm->chassis_data().gimbal_enabled != 0u) {
    float lv = GimbalChassisComm::DequantizeStick(comm->chassis_data().rc_lv_q);
    if (lv > -kPitchStickDeadzone && lv < kPitchStickDeadzone) lv = 0.0f;
    float t = kPitchK * lv;
    if (t >  kPitchLimit) t =  kPitchLimit;
    if (t < -kPitchLimit) t = -kPitchLimit;
    pitch_target_ = t;
  }

  /* ③④ pitch1 = 反馈角度, pitch_current = pitch1 − pitch0 */
  const float pitch1 = pitch_motor_->angle();
  pitch_current_ = pitch1 - pitch0_;

  /* ⑤ PID(目标, 当前) → 力矩 → MIT 帧 */
  pitch_torque_ = pitch_pid_->calc(pitch_target_, pitch_current_);
  pitch_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, pitch_torque_);
}

void Gimbal::update(void)
{
  if (!inited_) init();

  /* 电机不可用时不能往下走,否则 setMitInput 打在空指针上 */
  if (yaw_motor_ == nullptr) {
    ready_ = false;
    return;
  }

  UpdateTargetRate();
  UpdatePitch();
  UpdateTorqueCmd();

  if (!ready_) {
    /* 急停 / IMU 未就绪:五个参数全 0,电机不出力。
     * 仍要发帧(达妙不发指令就认为通信丢失)。 */
    yaw_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  /* MIT 帧只用力矩字段:pos / vel / kp / kd 全 0,电机只当力矩执行器 */
  yaw_motor_->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, torque_cmd_);
}

}  // namespace robot
