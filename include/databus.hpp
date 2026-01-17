#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

// Thread-safe data bus that holds motor/joint/IMU state plus command targets.
// A background time thread updates the unified time stamps to keep all modules
// aligned without blocking their own worker loops.

enum class CtrlMode : uint8_t {
  DISABLE = 0,
  MOTION  = 1,
  TORQUE  = 2
};

struct MotorCmd {
  CtrlMode mode = CtrlMode::DISABLE;
  bool enable   = false;

  float q_des   = 0.f;   // rad
  float dq_des  = 0.f;   // rad/s
  float tau_ff  = 0.f;   // Nm

  float kp      = 0.f;
  float kd      = 0.f;

  uint32_t flags = 0;
  uint64_t stamp = 0;
};

struct JointCmd {
  float q_des      = 0.f;   // rad
  float dq_des     = 0.f;   // rad/s
  float tau_ff     = 0.f;   // Nm
  float kp         = 0.f;
  float kd         = 0.f;
  uint64_t stamp   = 0;
  bool enable      = false;
};

struct MotorState {
  bool online   = false;
  float q       = 0.f;
  float dq      = 0.f;
  float tau     = 0.f;
  float temp    = 0.f;
  uint32_t age_ms = 0;
  uint64_t stamp  = 0;
};

struct JointState {
  double q_joint   = 0.0;
  double dq_joint  = 0.0;
  double tau_joint = 0.0;
  uint32_t age_ms  = 0;
  uint64_t stamp   = 0;
};

struct ImuBusState {
  bool online = false;
  uint64_t seq = 0;

  float ax = 0.f, ay = 0.f, az = 0.f;
  float gx = 0.f, gy = 0.f, gz = 0.f;
  float roll_deg = 0.f, pitch_deg = 0.f, yaw_deg = 0.f;

  uint32_t age_ms = 0;
  uint64_t stamp  = 0;
};

struct BusTime {
  double system_sec = 0.0; // since databus start
  double policy_sec = 0.0; // since policy enter
  uint64_t tick     = 0;   // incremented by time thread
};

struct BusSnapshot {
  std::vector<MotorCmd>   motor_cmds;
  std::vector<MotorState> motor_states;
  std::vector<JointCmd>   joint_cmds;
  std::vector<JointState> joint_states;
  ImuBusState imu;
  BusTime time;
  std::vector<std::string> motor_names;
  std::vector<std::string> joint_names;
};

class DataBus {
public:
  explicit DataBus(std::size_t n_motor = 0);
  ~DataBus();

  void resize(std::size_t n_motor);
  void set_names(const std::vector<std::string>& motors,
                 const std::vector<std::string>& joints);

  std::size_t size() const;

  // Background time loop (provides a dedicated thread for this class).
  void start();
  void stop();

  // Policy timing hooks.
  void mark_policy_start();
  void mark_policy_stop();
  void set_policy_time(double policy_sec);

  BusTime time() const;

  // Write APIs
  void update_motor_state(std::size_t i, const MotorState& s);
  void update_motor_cmd(std::size_t i, const MotorCmd& c);
  void update_joint_state(std::size_t i, const JointState& s);
  void update_joint_cmd(std::size_t i, const JointCmd& c);
  void update_imu(const ImuBusState& imu);

  // Read APIs
  MotorState motor_state(std::size_t i) const;
  MotorCmd   motor_cmd(std::size_t i) const;
  JointState joint_state(std::size_t i) const;
  JointCmd   joint_cmd(std::size_t i) const;
  ImuBusState imu() const;

  std::vector<MotorCmd>   motor_cmds() const;
  std::vector<JointCmd>   joint_cmds() const;
  std::vector<MotorState> motor_states() const;
  std::vector<JointState> joint_states() const;

  BusSnapshot snapshot() const;

  void disable_all();

private:
  void time_thread_fn_();

  mutable std::shared_mutex mtx_;
  std::vector<MotorCmd> motor_cmds_;
  std::vector<JointCmd> joint_cmds_;

  std::vector<MotorState> motor_states_;
  std::vector<JointState> joint_states_;

  std::vector<std::string> motor_names_;
  std::vector<std::string> joint_names_;

  ImuBusState imu_{};
  BusTime time_{};

  std::atomic<bool> running_{false};
  std::atomic<bool> policy_running_{false};
  std::thread time_thread_;
  std::chrono::steady_clock::time_point start_tp_;
  std::chrono::steady_clock::time_point policy_tp_;
};

// ================= Implementation =================

inline DataBus::DataBus(std::size_t n_motor) { resize(n_motor); }

inline DataBus::~DataBus() { stop(); }

inline void DataBus::resize(std::size_t n_motor) {
  std::unique_lock lk(mtx_);
  motor_cmds_.assign(n_motor, MotorCmd{});
  motor_states_.assign(n_motor, MotorState{});
  joint_cmds_.assign(n_motor, JointCmd{});
  joint_states_.assign(n_motor, JointState{});
  motor_names_.assign(n_motor, "");
  joint_names_.assign(n_motor, "");
}

inline void DataBus::set_names(const std::vector<std::string>& motors,
                               const std::vector<std::string>& joints) {
  std::unique_lock lk(mtx_);
  motor_names_ = motors;
  joint_names_ = joints;
  if (joint_names_.size() != motor_names_.size()) {
    joint_names_.resize(motor_names_.size());
  }
}

inline std::size_t DataBus::size() const {
  std::shared_lock lk(mtx_);
  return motor_cmds_.size();
}

inline void DataBus::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) return;
  start_tp_ = std::chrono::steady_clock::now();
  policy_tp_ = start_tp_;
  time_thread_ = std::thread(&DataBus::time_thread_fn_, this);
}

inline void DataBus::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (time_thread_.joinable()) time_thread_.join();
}

inline void DataBus::mark_policy_start() {
  policy_running_.store(true, std::memory_order_relaxed);
  policy_tp_ = std::chrono::steady_clock::now();
}

inline void DataBus::mark_policy_stop() {
  policy_running_.store(false, std::memory_order_relaxed);
  std::unique_lock lk(mtx_);
  time_.policy_sec = 0.0;
}

inline void DataBus::set_policy_time(double policy_sec) {
  std::unique_lock lk(mtx_);
  time_.policy_sec = policy_sec;
}

inline BusTime DataBus::time() const {
  std::shared_lock lk(mtx_);
  return time_;
}

inline void DataBus::update_motor_state(std::size_t i, const MotorState& s) {
  std::unique_lock lk(mtx_);
  if (i >= motor_states_.size()) return;
  motor_states_[i] = s;
}

inline void DataBus::update_motor_cmd(std::size_t i, const MotorCmd& c) {
  std::unique_lock lk(mtx_);
  if (i >= motor_cmds_.size()) return;
  motor_cmds_[i] = c;
}

inline void DataBus::update_joint_state(std::size_t i, const JointState& s) {
  std::unique_lock lk(mtx_);
  if (i >= joint_states_.size()) return;
  joint_states_[i] = s;
}

inline void DataBus::update_joint_cmd(std::size_t i, const JointCmd& c) {
  std::unique_lock lk(mtx_);
  if (i >= joint_cmds_.size()) return;
  joint_cmds_[i] = c;
}

inline void DataBus::update_imu(const ImuBusState& imu) {
  std::unique_lock lk(mtx_);
  imu_ = imu;
}

inline MotorState DataBus::motor_state(std::size_t i) const {
  std::shared_lock lk(mtx_);
  if (i >= motor_states_.size()) return MotorState{};
  return motor_states_[i];
}

inline MotorCmd DataBus::motor_cmd(std::size_t i) const {
  std::shared_lock lk(mtx_);
  if (i >= motor_cmds_.size()) return MotorCmd{};
  return motor_cmds_[i];
}

inline JointState DataBus::joint_state(std::size_t i) const {
  std::shared_lock lk(mtx_);
  if (i >= joint_states_.size()) return JointState{};
  return joint_states_[i];
}

inline JointCmd DataBus::joint_cmd(std::size_t i) const {
  std::shared_lock lk(mtx_);
  if (i >= joint_cmds_.size()) return JointCmd{};
  return joint_cmds_[i];
}

inline ImuBusState DataBus::imu() const {
  std::shared_lock lk(mtx_);
  return imu_;
}

inline std::vector<MotorCmd> DataBus::motor_cmds() const {
  std::shared_lock lk(mtx_);
  return motor_cmds_;
}

inline std::vector<JointCmd> DataBus::joint_cmds() const {
  std::shared_lock lk(mtx_);
  return joint_cmds_;
}

inline std::vector<MotorState> DataBus::motor_states() const {
  std::shared_lock lk(mtx_);
  return motor_states_;
}

inline std::vector<JointState> DataBus::joint_states() const {
  std::shared_lock lk(mtx_);
  return joint_states_;
}

inline BusSnapshot DataBus::snapshot() const {
  std::shared_lock lk(mtx_);
  BusSnapshot s;
  s.motor_cmds = motor_cmds_;
  s.motor_states = motor_states_;
  s.joint_cmds = joint_cmds_;
  s.joint_states = joint_states_;
  s.imu = imu_;
  s.time = time_;
  s.motor_names = motor_names_;
  s.joint_names = joint_names_;
  return s;
}

inline void DataBus::disable_all() {
  std::unique_lock lk(mtx_);
  for (auto& c : motor_cmds_) c = MotorCmd{};
  for (auto& c : joint_cmds_) c = JointCmd{};
}

inline void DataBus::time_thread_fn_() {
  start_tp_ = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_relaxed)) {
    auto now = std::chrono::steady_clock::now();
    const double sys_sec = std::chrono::duration<double>(now - start_tp_).count();
    const double policy_sec =
        policy_running_.load(std::memory_order_relaxed)
            ? std::chrono::duration<double>(now - policy_tp_).count()
            : 0.0;

    {
      std::unique_lock lk(mtx_);
      time_.system_sec = sys_sec;
      if (policy_running_.load(std::memory_order_relaxed)) {
        time_.policy_sec = policy_sec;
      }
      time_.tick++;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
