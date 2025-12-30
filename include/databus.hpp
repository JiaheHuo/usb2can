#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <algorithm>

// ====== DataBus: 单线程版本（无锁）======
// 约定：main loop 是唯一读写者，不需要 mutex

enum class CtrlMode : uint8_t {
  DISABLE = 0,
  MOTION  = 1,   // pos/vel + kp/kd （典型 PD）
  TORQUE  = 2    // 纯力矩
};

struct MotorCmd {
  CtrlMode mode = CtrlMode::DISABLE;
  bool enable   = false;

  // 目标量
  float q_des   = 0.f;   // rad
  float dq_des  = 0.f;   // rad/s
  float tau_ff  = 0.f;   // Nm

  // 增益
  float kp      = 0.f;
  float kd      = 0.f;

  // 备用：你们协议里可能还有“控制字/flags”，这里留扩展口
  uint32_t flags = 0;
};

struct JointCmd {
  // 目标量
  float q_joint_des   = 0.f;   // rad
  float dq_joint_des  = 0.f;   // rad/s
  float tau_joint_ff  = 0.f;   // Nm

  // 增益
  float kp_joint      = 0.f;
  float kd_joint      = 0.f;
};

struct MotorState {
  bool online   = false;
  float q       = 0.f;
  float dq      = 0.f;
  float tau     = 0.f;
  float temp    = 0.f;
  uint32_t age_ms = 0;
};
struct JointState {
  double q_joint;
  double dq_joint;
  double tau_joint;
};
struct ImuBusState {
  bool online = false;
  uint64_t seq = 0;

  // accel (m/s^2)
  float ax = 0.f, ay = 0.f, az = 0.f;
  // gyro (rad/s)
  float gx = 0.f, gy = 0.f, gz = 0.f;
  // euler (deg)
  float roll_deg = 0.f, pitch_deg = 0.f, yaw_deg = 0.f;

  uint32_t age_ms = 0;
};
struct SineProfile {
  // 对选定电机做 q_des = q0 + amp * sin(2*pi*f*t)
  float amp_rad = 0.3f;
  float freq_hz = 0.5f;
  float kp      = 30.f;
  float kd      = 2.0f;

  // sine 相对哪个基准（标0/记录当前角度）
  float q0      = 0.f;
  bool  armed   = false; // 是否已完成“记录 q0”
};

class DataBus {
public:
  explicit DataBus(int n_motor = 0) { resize(n_motor); }

  void resize(int n_motor) {
    motorCmds_.assign(n_motor, MotorCmd{});
    motorStates_.assign(n_motor, MotorState{});
    jointCmds_.assign(n_motor,JointCmd{});
    jointStates_.assign(n_motor,JointState{});
    sine_.assign(n_motor, SineProfile{});
    imu_ = ImuBusState{};
  }

  int size() const { return (int)motorCmds_.size(); }

  // -------- motorCmd/motorState 读写 --------
  MotorCmd&       motorCmd(int i)       { return motorCmds_.at(i); }
  const MotorCmd& motorCmd(int i) const { return motorCmds_.at(i); }

  MotorState&       motorState(int i)       { return motorStates_.at(i); }
  const MotorState& motorState(int i) const { return motorStates_.at(i); }

  // -------- jointCmd/jointState 读写 --------
  JointCmd&       jointCmd(int i)       { return jointCmds_.at(i); }
  const JointCmd& jointCmd(int i) const { return jointCmds_.at(i); }

  JointState&       jointState(int i)       { return jointStates_.at(i); }
  const JointState& jointState(int i) const { return jointStates_.at(i); }

  SineProfile&       sine(int i)       { return sine_.at(i); }
  const SineProfile& sine(int i) const { return sine_.at(i); }

  ImuBusState& imu() {return imu_;}
  const ImuBusState& imu() const { return imu_; }

  // -------- 常用操作：统一使能/失能 --------
  void disable_all() {
    for (auto &c : motorCmds_) { c = MotorCmd{}; }
  }

  // -------- 常用操作：把某电机“当前角度”记为 sine 的零点 --------
  void arm_sine_zero(int i) {
    sine_.at(i).q0 = motorStates_.at(i).q;
    sine_.at(i).armed = true;
  }

  // -------- 常用操作：生成 sine 指令（写入 databus.cmd）--------
  void write_sine_cmd(int i, double t_sec) {
    auto &sp = sine_.at(i);
    auto &c  = motorCmds_.at(i);

    if (!sp.armed) {
      // 没 arm 之前，默认保持 disable，避免跑飞
      c = MotorCmd{};
      return;
    }

    const double w = 2.0 * M_PI * (double)sp.freq_hz;
    const float q_des = sp.q0 + sp.amp_rad * std::sin(w * t_sec);

    c.enable = true;
    c.mode   = CtrlMode::MOTION;
    c.q_des  = q_des;
    c.dq_des = 0.f;
    c.tau_ff = 0.f;
    c.kp     = sp.kp;
    c.kd     = sp.kd;
  }

  // -------- 打印：按你们现有风格输出 --------
  void print_telemetry_line(const char* title = "Telemetry") const {
    std::printf("==== %s motors=%d ====\n", title, size());
    {
      const auto &u = imu_;
      std::printf(
          "IMU online=%d seq=%llu age=%ums | "
          "acc=% .3f % .3f % .3f  gyro=% .3f % .3f % .3f  rpy_deg=% .2f % .2f % .2f\n",
          (int)u.online,
          (unsigned long long)u.seq,
          u.age_ms,
          u.ax, u.ay, u.az,
          u.gx, u.gy, u.gz,
          u.roll_deg, u.pitch_deg, u.yaw_deg);
    }
    for (int i = 0; i < size(); ++i)
    {
      const auto &s = motorStates_[i];
      const auto &c = motorCmds_[i];
      std::printf(
          "M[%02d] online=%d q=% .4f dq=% .4f tau=% .4f temp=% .1f age=%ums | "
          "motorCmd(%s,en=%d) qd=% .4f vd=% .4f tq=% .4f kp=%.1f kd=%.1f\n",
          i,
          (int)s.online, s.q, s.dq, s.tau, s.temp, s.age_ms,
          mode_str(c.mode), (int)c.enable, c.q_des, c.dq_des, c.tau_ff, c.kp, c.kd);
    }
  }
private:
  static const char* mode_str(CtrlMode m) {
    switch(m){
      case CtrlMode::DISABLE: return "DISABLE";
      case CtrlMode::MOTION:  return "MOTION";
      case CtrlMode::TORQUE:  return "TORQUE";
      default: return "UNKNOWN";
    }
  }

  std::vector<MotorCmd> motorCmds_;
  std::vector<JointCmd> jointCmds_;

  std::vector<MotorState> motorStates_;
  std::vector<JointState> jointStates_;

  std::vector<SineProfile> sine_;
  ImuBusState imu_;
};
