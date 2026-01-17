#include "control_modules.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

MotorState read_motor_state(RobStrideMotor* m) {
  MotorState s{};
  auto pvtt = m->last_pvtt();
  if (!pvtt) {
    s.online = false;
    return s;
  }
  s.online = true;
  s.q = pvtt->p;
  s.dq = pvtt->v;
  s.tau = pvtt->t;
  s.temp = pvtt->temp;
  s.age_ms = 0;
  return s;
}

void apply_motor_cmd(RobStrideMotor* m, const MotorCmd& c) {
  if (!c.enable || c.mode == CtrlMode::DISABLE) {
    m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
    return;
  }
  if (c.mode == CtrlMode::MOTION) {
    m->send_motion_command(c.tau_ff, c.q_des, c.dq_des, c.kp, c.kd);
    return;
  }
  if (c.mode == CtrlMode::TORQUE) {
    m->send_motion_command(c.tau_ff, 0.f, 0.f, 0.f, 0.f);
    return;
  }
  m->send_motion_command(0.f, 0.f, 0.f, 0.f, 0.f);
}

} // namespace

// ================= LowLevelControl =================

LowLevelControl::LowLevelControl(DataBus& bus, Utility& util)
    : bus_(bus), util_(util) {}

LowLevelControl::~LowLevelControl() { stop(); }

bool LowLevelControl::start() {
  if (running_.load()) return true;
  if (!init_hardware_()) return false;
  running_.store(true);
  worker_ = std::thread(&LowLevelControl::loop_, this);
  return true;
}

void LowLevelControl::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lk(mtx_);
  if (router_) router_->stop();
  dev_objs_.clear();
  motors_.clear();
  active_.clear();
}

bool LowLevelControl::init_hardware_() {
  std::lock_guard<std::mutex> lk(mtx_);
  auto cfg = util_.config();

  bus_.resize(cfg.active_motors.size());
  bus_.set_names(cfg.motor_names, cfg.joint_names);

  const int n = static_cast<int>(cfg.active_motors.size());
  if (cfg.simulate) {
    sim_pos_rad_ = Eigen::VectorXd::Zero(n);
    sim_vel_rad_ = Eigen::VectorXd::Zero(n);
    sim_tau_nm_  = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      const auto& name = cfg.motor_names[i];
      const auto it = cfg.initial_motor_pos.find(name);
      if (it != cfg.initial_motor_pos.end()) sim_pos_rad_(i) = it->second;
      MotorState s{};
      s.online = true;
      s.q = static_cast<float>(sim_pos_rad_(i));
      s.dq = 0.f;
      s.tau = 0.f;
      bus_.update_motor_state(i, s);
    }
    return true;
  }

  std::vector<bool> need_dev(cfg.can.devices.size(), false);
  for (const auto& m : cfg.active_motors) {
    if (m.device_index >= 0 && m.device_index < static_cast<int>(need_dev.size())) {
      need_dev[m.device_index] = true;
    }
  }

  dev_ptrs_.assign(cfg.can.devices.size(), nullptr);
  dev_objs_.clear();
  for (int di = 0; di < static_cast<int>(cfg.can.devices.size()); ++di) {
    if (!need_dev[di]) continue;
    auto dev = std::make_unique<Usb2CanDevice>(cfg.can.devices[di].path);
    dev_ptrs_[di] = dev.get();
    std::cout << "[lowlevel] opened " << cfg.can.devices[di].name
              << " at " << cfg.can.devices[di].path << "\n";
    dev_objs_.push_back(std::move(dev));
  }

  motors_.clear();
  active_.clear();
  for (const auto& am : cfg.active_motors) {
    BusHandle bh;
    bh.dev_idx = am.device_index;
    bh.dev_name = am.device_name;
    bh.dev = (am.device_index < static_cast<int>(dev_ptrs_.size())) ? dev_ptrs_[am.device_index] : nullptr;
    bh.channel = am.channel;

    MotorKey mk;
    mk.dev_idx = am.device_index;
    mk.channel = am.channel;
    mk.motor_id = am.spec.id;
    mk.dev_name = am.device_name;
    mk.group = am.spec.group;
    mk.type_str = actuator_to_string(am.spec.type);
    mk.motor_name = am.spec.motorName;

    int sidx = stats_.register_motor(mk);
    motors_.push_back(std::make_unique<RobStrideMotor>(
        bh, cfg.can.master_id, am.spec.id, am.spec.type, am.spec.group, &stats_, sidx, am.spec.motorName));
  }
  for (auto& m : motors_) active_.push_back(m.get());

  std::vector<Usb2CanDevice*> opened;
  for (auto* p : dev_ptrs_) if (p) opened.push_back(p);
  router_ = std::make_unique<Usb2CanRouter>(opened);
  for (auto* m : active_) router_->attach_motor(m);
  router_->start();

  for (int i = 0; i < static_cast<int>(active_.size()); ++i) {
    MotorState s{};
    s.online = false;
    if (cfg.initial_motor_pos.count(cfg.motor_names[i])) {
      s.q = cfg.initial_motor_pos.at(cfg.motor_names[i]);
    }
    bus_.update_motor_state(i, s);
  }
  return true;
}

void LowLevelControl::loop_() {
  auto next = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_relaxed)) {
    auto cfg = util_.config();
    const double dt = 1.0 / std::max(1, cfg.loop_hz);
    if (cfg.simulate) {
      simulate_step_(dt);
    } else {
      read_states_();
      apply_commands_();
      stats_.print_periodic();
    }
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next);
  }
}

void LowLevelControl::simulate_step_(double dt) {
  std::lock_guard<std::mutex> lk(mtx_);
  const int n = static_cast<int>(sim_pos_rad_.size());
  for (int i = 0; i < n; ++i) {
    auto cmd = bus_.motor_cmd(i);

    double pos0 = sim_pos_rad_(i);
    double pos1 = pos0;
    double vel = 0.0;
    double tau = 0.0;

    if (cmd.enable) {
      if (cmd.mode == CtrlMode::MOTION) {
        const double alpha = std::min(1.0, dt / 0.05);
        pos1 = pos0 + alpha * ((double)cmd.q_des - pos0);
        vel = (pos1 - pos0) / std::max(1e-6, dt);
        tau = cmd.tau_ff;
      } else if (cmd.mode == CtrlMode::TORQUE) {
        tau = cmd.tau_ff;
      }
    }

    sim_pos_rad_(i) = pos1;
    sim_vel_rad_(i) = vel;
    sim_tau_nm_(i) = tau;

    MotorState s{};
    s.online = true;
    s.q = static_cast<float>(pos1);
    s.dq = static_cast<float>(vel);
    s.tau = static_cast<float>(tau);
    s.temp = 30.0f;
    s.age_ms = 0;
    bus_.update_motor_state(i, s);
  }
}

void LowLevelControl::read_states_() {
  std::lock_guard<std::mutex> lk(mtx_);
  for (int i = 0; i < static_cast<int>(active_.size()); ++i) {
    MotorState s = read_motor_state(active_[i]);
    bus_.update_motor_state(i, s);
  }
}

void LowLevelControl::apply_commands_() {
  std::lock_guard<std::mutex> lk(mtx_);
  auto cmds = bus_.motor_cmds();
  for (int i = 0; i < static_cast<int>(active_.size()) && i < static_cast<int>(cmds.size()); ++i) {
    apply_motor_cmd(active_[i], cmds[i]);
  }
}

// ================= ImuInterface =================

ImuInterface::ImuInterface(DataBus& bus, Utility& util)
    : bus_(bus), util_(util) {}

ImuInterface::~ImuInterface() { stop(); }

bool ImuInterface::start() {
  if (running_.load()) return true;
  running_.store(true);
  worker_ = std::thread(&ImuInterface::loop_, this);
  return true;
}

void ImuInterface::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (worker_.joinable()) worker_.join();
  runner_.stop();
}

void ImuInterface::loop_() {
  auto cfg = util_.config();
  if (cfg.imu.enable) {
    WitMotionImuRunner::Options opt;
    opt.port = cfg.imu.port;
    opt.baud = cfg.imu.baud;
    runner_.start(opt);
  }

  while (running_.load(std::memory_order_relaxed)) {
    ImuBusState s{};
    if (cfg.imu.enable) {
      ImuState imu;
      uint32_t age_ms = 0;
      if (runner_.get_latest(imu, &age_ms)) {
        s.online = true;
        s.seq = imu.seq;
        s.ax = static_cast<float>(imu.accel_mps2.x());
        s.ay = static_cast<float>(imu.accel_mps2.y());
        s.az = static_cast<float>(imu.accel_mps2.z());
        s.gx = static_cast<float>(imu.gyro_rps.x());
        s.gy = static_cast<float>(imu.gyro_rps.y());
        s.gz = static_cast<float>(imu.gyro_rps.z());
        s.roll_deg = static_cast<float>(imu.rpy_deg.x());
        s.pitch_deg = static_cast<float>(imu.rpy_deg.y());
        s.yaw_deg = static_cast<float>(imu.rpy_deg.z());
        s.age_ms = age_ms;
      }
    }
    bus_.update_imu(s);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    cfg = util_.config();
  }
}

// ================= Mapping =================

Mapping::Mapping(DataBus& bus, Utility& util)
    : bus_(bus), util_(util) {
  ankle_params_ = ankle::make_params_v2();
  ankle_params_.invert_motor1 = true;
  ankle_params_.invert_motor2 = false;
}

Mapping::~Mapping() { stop(); }

bool Mapping::start() {
  if (running_.load()) return true;
  build_indices_();
  running_.store(true);
  worker_ = std::thread(&Mapping::loop_, this);
  return true;
}

void Mapping::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (worker_.joinable()) worker_.join();
}

void Mapping::build_indices_() {
  std::lock_guard<std::mutex> lk(mtx_);
  left_ = AnkleIndices{};
  right_ = AnkleIndices{};
  auto idx = [&](const std::string& name) { return util_.motor_index(name); };
  left_.m1 = idx("L_ankle_L");
  left_.m2 = idx("L_ankle_R");
  left_.jp = idx("L_ankle_pitch");
  left_.jr = idx("L_ankle_roll");

  right_.m1 = idx("R_ankle_L");
  right_.m2 = idx("R_ankle_R");
  right_.jp = idx("R_ankle_pitch");
  right_.jr = idx("R_ankle_roll");
}

void Mapping::loop_() {
  auto next = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_relaxed)) {
    auto cfg = util_.config();
    auto snap = bus_.snapshot();
    update_joint_states_(snap, cfg);
    auto updated = bus_.snapshot(); // include freshly written joint states
    write_motor_cmds_(updated, cfg);

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1, cfg.loop_hz)));
    std::this_thread::sleep_until(next);
  }
}

void Mapping::update_joint_states_(const BusSnapshot& snap, const UtilityConfig& cfg) {
  for (std::size_t i = 0; i < snap.motor_states.size(); ++i) {
    JointState js{};
    js.q_joint = snap.motor_states[i].q;
    js.dq_joint = snap.motor_states[i].dq;
    js.tau_joint = snap.motor_states[i].tau;
    js.age_ms = snap.motor_states[i].age_ms;
    bus_.update_joint_state(i, js);
  }
  map_ankle_state_(left_, snap);
  map_ankle_state_(right_, snap);
}

void Mapping::map_ankle_state_(const AnkleIndices& idx, const BusSnapshot& snap) {
  if (!idx.valid()) return;
  if (idx.m1 >= static_cast<int>(snap.motor_states.size()) ||
      idx.m2 >= static_cast<int>(snap.motor_states.size()) ||
      idx.jp >= static_cast<int>(snap.joint_states.size()) ||
      idx.jr >= static_cast<int>(snap.joint_states.size())) {
    return;
  }
  const auto& s1 = snap.motor_states[idx.m1];
  const auto& s2 = snap.motor_states[idx.m2];

  double m1_rad = s1.q;
  double m2_rad = s2.q;

  auto fk = ankle::forwardKinematics(
      ankle_params_, ankle::rad2deg(m1_rad), ankle::rad2deg(m2_rad), 0.0, 0.0, 50, 1e-10);
  if (!fk.ok) return;

  JointState pitch{};
  JointState roll{};

  pitch.q_joint = ankle::deg2rad(fk.joint_deg.pitch_deg);
  roll.q_joint = ankle::deg2rad(fk.joint_deg.roll_deg);

  Eigen::Vector2d qdot = ankle::fkVelocity_joint(
      ankle_params_, pitch.q_joint, roll.q_joint, m1_rad, m2_rad, s1.dq, s2.dq);
  pitch.dq_joint = qdot(0);
  roll.dq_joint = qdot(1);

  pitch.age_ms = std::max(s1.age_ms, s2.age_ms);
  roll.age_ms = pitch.age_ms;

  bus_.update_joint_state(idx.jp, pitch);
  bus_.update_joint_state(idx.jr, roll);
}

void Mapping::write_motor_cmds_(const BusSnapshot& snap, const UtilityConfig& cfg) {
  const int n = static_cast<int>(snap.motor_cmds.size());
  std::vector<bool> is_ankle_motor(n, false);
  if (left_.valid()) {
    if (left_.m1 >= 0 && left_.m1 < n) is_ankle_motor[left_.m1] = true;
    if (left_.m2 >= 0 && left_.m2 < n) is_ankle_motor[left_.m2] = true;
  }
  if (right_.valid()) {
    if (right_.m1 >= 0 && right_.m1 < n) is_ankle_motor[right_.m1] = true;
    if (right_.m2 >= 0 && right_.m2 < n) is_ankle_motor[right_.m2] = true;
  }

  for (int i = 0; i < n; ++i) {
    if (is_ankle_motor[i]) continue;
    const JointCmd jc = snap.joint_cmds[i];
    MotorCmd mc{};
    if (jc.enable) {
      const float kp = jc.kp > 0.f ? jc.kp : cfg.running_gains.kp;
      const float kd = jc.kd > 0.f ? jc.kd : cfg.running_gains.kd;

      const std::string name = cfg.joint_names[i];
      float q_des = jc.q_des; 
      float dq_des = jc.dq_des;
      q_des = util_.clip_position(name, q_des);
      if(dq_des > cfg.limits.vel_max) dq_des =  cfg.limits.vel_max;
      if(dq_des <-cfg.limits.vel_max) dq_des = -cfg.limits.vel_max;

      mc.enable = true;
      mc.mode = CtrlMode::MOTION;
      mc.tau_ff = 0;
      mc.q_des = q_des;
      mc.dq_des = dq_des;
      mc.kp = kp;
      mc.kd = kd;
    }
    bus_.update_motor_cmd(i, mc);
  }

  map_ankle_torque_(left_, snap, cfg);
  map_ankle_torque_(right_, snap, cfg);
}

void Mapping::map_ankle_torque_(const AnkleIndices& idx, const BusSnapshot& snap,
                                const UtilityConfig& cfg) {
  if (!idx.valid()) return;
  if (idx.m1 >= static_cast<int>(snap.motor_states.size()) ||
      idx.m2 >= static_cast<int>(snap.motor_states.size()) ||
      idx.jp >= static_cast<int>(snap.joint_cmds.size()) ||
      idx.jr >= static_cast<int>(snap.joint_cmds.size())) {
    return;
  }

  const JointCmd pitch_cmd = snap.joint_cmds[idx.jp];
  const JointCmd roll_cmd = snap.joint_cmds[idx.jr];
  const JointState pitch_state = snap.joint_states[idx.jp];
  const JointState roll_state = snap.joint_states[idx.jr];

  if (!pitch_cmd.enable && !roll_cmd.enable) {
    bus_.update_motor_cmd(idx.m1, MotorCmd{});
    bus_.update_motor_cmd(idx.m2, MotorCmd{});
    return;
  }

  const float kp_p = pitch_cmd.kp > 0.f ? pitch_cmd.kp : cfg.running_gains.kp;
  const float kd_p = pitch_cmd.kd > 0.f ? pitch_cmd.kd : cfg.running_gains.kd;
  const float kp_r = roll_cmd.kp > 0.f ? roll_cmd.kp : cfg.running_gains.kp;
  const float kd_r = roll_cmd.kd > 0.f ? roll_cmd.kd : cfg.running_gains.kd;

  const float tau_pitch = kp_p * static_cast<float>(pitch_cmd.q_des - pitch_state.q_joint) +
                          kd_p * static_cast<float>(pitch_cmd.dq_des - pitch_state.dq_joint);
  const float tau_roll  = kp_r * static_cast<float>(roll_cmd.q_des - roll_state.q_joint) +
                          kd_r * static_cast<float>(roll_cmd.dq_des - roll_state.dq_joint);

  const double m1_rad = snap.motor_states[idx.m1].q;
  const double m2_rad = snap.motor_states[idx.m2].q;
  // 使用当前电机状态求 Jacobian 进行映射
  Eigen::Vector2d motor_tau = ankle::ikTorque_motor(
      ankle_params_, pitch_state.q_joint, roll_state.q_joint, m1_rad, m2_rad,
      tau_pitch, tau_roll);

  MotorCmd c1{};
  c1.enable = true;
  c1.mode = CtrlMode::TORQUE;
  c1.tau_ff = util_.clip_torque(static_cast<float>(motor_tau(0)));
  c1.kp = 0.f;
  c1.kd = 0.f;

  MotorCmd c2 = c1;
  c2.tau_ff = util_.clip_torque(static_cast<float>(motor_tau(1)));

  bus_.update_motor_cmd(idx.m1, c1);
  bus_.update_motor_cmd(idx.m2, c2);
}

// ================= RLLocomotion =================

RLLocomotion::RLLocomotion(DataBus& bus, Utility& util)
    : bus_(bus), util_(util) {}

RLLocomotion::~RLLocomotion() { stop(); }

bool RLLocomotion::start() {
  if (running_.load()) return true;
  auto cfg = util_.config();
  num_actions_ = static_cast<int>(cfg.joint_names.size());
  last_action_.assign(num_actions_, 0.f);

  frame_stack_ = std::max(1, cfg.rl.frame_stack);
  num_single_obs_ = 47;
  hist_obs_.clear();

  for(int i = 0; i<frame_stack_; ++i){
    hist_obs_.push_back(std::vector<float>(num_single_obs_,0.f));
  }

  if (!cfg.rl.policy_path.empty()) {
    load_policy_(cfg.rl.policy_path);
  }
  running_.store(true);
  worker_ = std::thread(&RLLocomotion::loop_, this);
  return true;
}

void RLLocomotion::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (worker_.joinable()) worker_.join();
}

void RLLocomotion::load_policy_(const std::string& path) {
  if (path.empty()) return;
  try {
    policy_ = torch::jit::load(path);
    policy_.eval();
    policy_ok_ = true;
    std::cout << "[rl] loaded policy: " << path << "\n";
  } catch (const c10::Error& e) {
    std::cerr << "[rl] failed to load policy: " << e.what() << "\n";
    policy_ok_ = false;
  }
}

std::vector<float> RLLocomotion::build_observation_(const BusSnapshot &snap,
                                                    const UtilityConfig &cfg)
{

  if (cfg.rl.frame_stack != frame_stack_ || 47 != num_single_obs_)
  {
    frame_stack_ = cfg.rl.frame_stack;
    num_single_obs_ = 47;
    for (int i = 0; i < frame_stack_; ++i)
    {
      hist_obs_.push_back(std::vector<float>(num_single_obs_, 0.f));
    }
  }

  auto single = build_single_observation_(snap, cfg);
  hist_obs_.push_back(std::move(single));
  while (static_cast<int>(hist_obs_.size()) > frame_stack_)
  {
    hist_obs_.pop_front();
  }

  std::vector<float> stacked_obs;
  stacked_obs.resize(static_cast<size_t>(num_single_obs_ * frame_stack_), 0.f);
  for (int i = 0; i < frame_stack_; ++i)
  {
    const auto& au = hist_obs_[i];
    std::copy(au.begin(),au.end(),stacked_obs.begin()+static_cast<size_t>(i * num_single_obs_));
    //把历史观测值从左到右，复制给stacked_obs的begin+i*47处
  }
  return stacked_obs;
}

std::vector<float> RLLocomotion::build_single_observation_(const BusSnapshot &snap,
                                                           const UtilityConfig &cfg)
{
  std::vector<float> obs;
  obs.reserve(static_cast<std::size_t>(num_single_obs_));

  const double period = static_cast<double>(cfg.rl.gait_period);
  const double phase = 2 * M_PI * snap.time.system_sec / period; // 这里是不是应该使用RL的time
  // Clock
  obs.push_back(static_cast<float>(std::sin(phase)));
  obs.push_back(static_cast<float>(std::cos(phase)));

  // Cmd
  obs.push_back(cfg.rl.cmd_vx * cfg.rl.obs_lin_vel_scale);
  obs.push_back(cfg.rl.cmd_vy * cfg.rl.obs_lin_vel_scale);
  obs.push_back(cfg.rl.cmd_dyaw * cfg.rl.obs_ang_vel_scale);

  // joint Pos
  const float pos_obs_scale = cfg.rl.obs_dof_pos_scale;
  for (std::size_t i = 0; i < snap.joint_states.size(); ++i)
  {
    const std::string name = cfg.joint_names[i];
    const float q0 = cfg.initial_joint_pos.at(name);
    obs.push_back((static_cast<float>(snap.joint_states[i].q_joint) - q0) * pos_obs_scale);
  }

  // joint Vel
  const float vel_obs_scale = cfg.rl.obs_dof_vel_scale;
  for (std::size_t i = 0; i < snap.joint_states.size(); ++i)
  {
    obs.push_back((static_cast<float>(snap.joint_states[i].dq_joint)) * vel_obs_scale);
  }

  // last action
  for (std::size_t i = 0; i < static_cast<size_t>(last_action_.size()); ++i)
  {
    obs.push_back(last_action_[i]);
  }

  // IMU gyro + rpy
  obs.push_back(snap.imu.gx);
  obs.push_back(snap.imu.gy);
  obs.push_back(snap.imu.gz);
  obs.push_back(snap.imu.roll_deg * static_cast<float>(kPi / 180.0));
  obs.push_back(snap.imu.pitch_deg * static_cast<float>(kPi / 180.0));
  obs.push_back(snap.imu.yaw_deg * static_cast<float>(kPi / 180.0));

  // obs clip
  const float obsClip = cfg.rl.clip_obs;
  for(auto& it : obs){
    if(it > obsClip){
      it = obsClip;
    }
    if(it < -obsClip){
      it = -obsClip;
    }
  }
  return obs;
}

std::vector<float> RLLocomotion::run_policy_(const std::vector<float>& obs) {
  std::vector<float> action(num_actions_, 0.f);
  if (!policy_ok_) return action;
  try {
    torch::NoGradGuard guard;
    auto input = torch::from_blob(const_cast<float*>(obs.data()),
                                  {1, static_cast<long>(obs.size())}).clone();
    auto out = policy_.forward({input}).toTensor();
    out = out.flatten();
    const int n = std::min<int>(action.size(), out.numel());
    auto acc = out.accessor<float,1>();
    for (int i = 0; i < n; ++i) {
      action[i] = acc[i];
    }
    // 该函数没有cfg的参数，暂时无法clip action，在loop_里面进行即可
  } catch (const c10::Error& e) {
    std::cerr << "[rl] inference error: " << e.what() << "\n";
  }
  return action;
}

void RLLocomotion::write_joint_targets_(const std::vector<float>& action,
                                        const UtilityConfig& cfg) {
  const int n = static_cast<int>(std::min<std::size_t>(action.size(), cfg.joint_names.size()));
  for (int i = 0; i < n; ++i) {
    JointCmd jc = bus_.joint_cmd(i);
    const std::string& name = cfg.joint_names[i];
    const float q0 = cfg.initial_joint_pos.count(name) ? cfg.initial_joint_pos.at(name) : 0.f;
    float target = q0 + action[i] * cfg.rl.action_scale;
    target = util_.clip_position(name, target);
    jc.q_des = target;
    jc.dq_des = 0.f;
    jc.tau_ff = 0.f;
    jc.kp = cfg.running_gains.kp;
    jc.kd = cfg.running_gains.kd;
    jc.enable = true;
    bus_.update_joint_cmd(i, jc);
  }
  last_action_.assign(action.begin(), action.begin() + n);
}

void RLLocomotion::loop_() {
  auto next = std::chrono::steady_clock::now();

  while (running_.load(std::memory_order_relaxed)) {
    auto cfg = util_.config();
    const double base_dt = 1.0 / std::max(1, cfg.loop_hz);
    const int decim = std::max(1, cfg.rl.decimation);
    const double period = base_dt * decim;
    auto snap = bus_.snapshot();
    auto obs = build_observation_(snap, cfg);
    auto action = run_policy_(obs);
    // clip action
    const float action_clip = cfg.rl.action_scale;
    for(auto &it:action) {
      if(it > action_clip) it = action_clip;
      if(it <-action_clip) it =-action_clip;
    }
    write_joint_targets_(action, cfg);
    if (!policy_started_.load(std::memory_order_relaxed)) {
      bus_.mark_policy_start();
      policy_started_.store(true, std::memory_order_relaxed);
    }
    bus_.set_policy_time(bus_.time().policy_sec); // keep time fresh for logging
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(period));
    std::this_thread::sleep_until(next);
  }
}
