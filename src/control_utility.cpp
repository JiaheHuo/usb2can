#include "control_utility.hpp"

#include <chrono>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "config.hpp"
#include "util.hpp"

namespace {

JointLimit make_limit(float lo, float hi) {
  JointLimit l;
  l.min = lo;
  l.max = hi;
  return l;
}

std::unordered_map<std::string, JointLimit> default_joint_limits() {
  // Mirrors the legacy limits used in GridRobot for safety.
  return {
      {"L_hip_pitch",   make_limit(-0.5f, 0.8f)},
      {"L_hip_roll",    make_limit(-0.3f, 0.3f)},
      {"L_hip_yaw",     make_limit(-0.9f, 0.9f)},
      {"L_knee",        make_limit(-1.58f, 0.05f)},
      {"L_ankle_pitch", make_limit(-0.5f, 0.1f)},
      {"L_ankle_roll",  make_limit(-0.52f, 0.52f)},
      {"R_hip_pitch",   make_limit(-0.8f, 0.5f)},
      {"R_hip_roll",    make_limit(-0.3f, 0.3f)},
      {"R_hip_yaw",     make_limit(-0.9f, 0.9f)},
      {"R_knee",        make_limit(-0.05f, 1.58f)},
      {"R_ankle_pitch", make_limit(-0.5f, 0.1f)},
      {"R_ankle_roll",  make_limit(-0.52f, 0.52f)},
  };
}

bool group_selected(const std::string& group, const std::vector<std::string>& only_groups) {
  if (only_groups.empty()) return true;
  for (const auto& g : only_groups) {
    if (g == group) return true;
  }
  return false;
}

std::vector<std::string> parse_only_groups(const YAML::Node& n) {
  std::vector<std::string> out;
  if (!n) return out;
  if (n.IsSequence()) {
    for (auto v : n) out.push_back(v.as<std::string>());
  } else if (n.IsScalar()) {
    out = split_csv(n.as<std::string>());
  }
  return out;
}

std::vector<float> parse_float_list(const YAML::Node& n) {
  std::vector<float> out;
  if (!n) return out;
  if (n.IsSequence()) {
    for (auto v : n) out.push_back(v.as<float>());
  } else if (n.IsScalar()) {
    out.push_back(n.as<float>());
  }
  return out;
}

void apply_gain_node(const YAML::Node& node, ControlGains& gains, bool is_kp) {
  if (!node) return;
  if (node.IsSequence()) {
    auto vals = parse_float_list(node);
    if (is_kp) gains.kp = std::move(vals);
    else gains.kd = std::move(vals);
    const auto& ref = is_kp ? gains.kp : gains.kd;
    if (!ref.empty()) {
      if (is_kp) gains.kp_default = ref.front();
      else gains.kd_default = ref.front();
    }
  } else if (node.IsScalar()) {
    const float v = node.as<float>();
    if (is_kp) gains.kp_default = v;
    else gains.kd_default = v;
  }
}

void apply_limit_vector(const YAML::Node& node, std::vector<float>& out_vec, float& fallback) {
  if (!node) return;
  if (node.IsSequence()) {
    out_vec = parse_float_list(node);
    if (!out_vec.empty()) fallback = out_vec.front();
  } else if (node.IsScalar()) {
    fallback = node.as<float>();
  }
}

void apply_joint_limits(const YAML::Node& node, MotorLimits& limits) {
  if (!node || !node.IsMap()) return;
  for (auto it = node.begin(); it != node.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    if (name == "default" || name == "min" || name == "max" || name == "joints") continue;
    JointLimit jl = limits.position;
    if ((*it)["min"]) jl.min = (*it)["min"].as<float>();
    if ((*it)["max"]) jl.max = (*it)["max"].as<float>();
    limits.per_joint[name] = jl;
  }
}

} // namespace

Utility::Utility(const std::string& config_path)
    : config_path_(config_path) {
  load_config_();
  try {
    last_write_time_ = std::filesystem::last_write_time(config_path_);
  } catch (...) {
    last_write_time_ = std::filesystem::file_time_type::min();
  }
}

Utility::~Utility() { stop(); }

void Utility::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) return;
  reload_thread_ = std::thread(&Utility::reload_loop_, this);
}

void Utility::stop() {
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false)) return;
  if (reload_thread_.joinable()) reload_thread_.join();
}

UtilityConfig Utility::config() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return cfg_;
}

float Utility::clip_torque(int motor_idx, float tau) const {
  std::lock_guard<std::mutex> lk(mtx_);
  const float hi = cfg_.limits.torque_max_at(motor_idx);
  const float lo = -hi;
  if (tau < lo) return lo;
  if (tau > hi) return hi;
  return tau;
}


float Utility::clip_position(const std::string& joint, float q_rad) const {
  auto lim = joint_limit(joint);
  if (q_rad < lim.min) return lim.min;
  if (q_rad > lim.max) return lim.max;
  return q_rad;
}

JointLimit Utility::joint_limit(const std::string& joint) const {
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = cfg_.limits.per_joint.find(joint);
  if (it != cfg_.limits.per_joint.end()) return it->second;
  return cfg_.limits.position;
}

int Utility::motor_index(const std::string& name) const {
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = cfg_.name_to_index.find(name);
  if (it == cfg_.name_to_index.end()) return -1;
  return it->second;
}

void Utility::reload_loop_() {
  while (running_.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    try {
      auto now = std::filesystem::last_write_time(config_path_);
      if (now != last_write_time_) {
        load_config_();
        last_write_time_ = now;
        std::cerr << "[utility] reloaded config: " << config_path_ << "\n";
      }
    } catch (const std::exception& e) {
      std::cerr << "[utility] reload failed: " << e.what() << "\n";
    }
  }
}

void Utility::load_config_() {
  UtilityConfig next;
  next.config_path = config_path_;

  YAML::Node root = YAML::LoadFile(config_path_);

  next.loop_hz = root["loop_hz"] ? root["loop_hz"].as<int>() : 1000;
  next.telemetry_hz = root["telemetry_hz"] ? root["telemetry_hz"].as<int>() : 10;
  next.simulate = root["simulate"] ? root["simulate"].as<bool>() : false;

  next.only_groups = parse_only_groups(root["only_groups"]);

  if (auto gains = root["gains"]) {
    if (gains["fixed"]) {
      apply_gain_node(gains["fixed"]["kp"], next.fixed_gains, true);
      apply_gain_node(gains["fixed"]["kd"], next.fixed_gains, false);
    }
    if (gains["running"]) {
      apply_gain_node(gains["running"]["kp"], next.running_gains, true);
      apply_gain_node(gains["running"]["kd"], next.running_gains, false);
    }
  }

  next.limits.per_joint = default_joint_limits();
  if (auto limits = root["limits"]) {
    apply_limit_vector(limits["torque_max"], next.limits.torque_max_vec, next.limits.torque_max);
    apply_limit_vector(limits["velocity_max"], next.limits.vel_max_vec, next.limits.vel_max);
    if (limits["position"]) {
      auto pos = limits["position"];
      if (pos["default"]) {
        if (pos["default"]["min"]) next.limits.position.min = pos["default"]["min"].as<float>();
        if (pos["default"]["max"]) next.limits.position.max = pos["default"]["max"].as<float>();
      } else {
        if (pos["min"]) next.limits.position.min = pos["min"].as<float>();
        if (pos["max"]) next.limits.position.max = pos["max"].as<float>();
      }
      if (pos["joints"]) apply_joint_limits(pos["joints"], next.limits);
      apply_joint_limits(pos, next.limits);
    }
    if (limits["joints"]) apply_joint_limits(limits["joints"], next.limits);
  }

  if (auto rl = root["rl"]) {
    if (rl["policy_path"]) next.rl.policy_path = rl["policy_path"].as<std::string>();
    else if (rl["policy"]) next.rl.policy_path = rl["policy"].as<std::string>();
    if (rl["frame_stack"]) next.rl.frame_stack = rl["frame_stack"].as<int>();
    if (rl["decimation"]) next.rl.decimation = rl["decimation"].as<int>();
    if (rl["action_scale"]) next.rl.action_scale = rl["action_scale"].as<float>();
    if (rl["clip_actions"]) next.rl.clip_actions = rl["clip_actions"].as<float>();
    if (rl["clip_obs"]) next.rl.clip_obs = rl["clip_obs"].as<float>();
    if (rl["gait_period"]) next.rl.gait_period = rl["gait_period"].as<float>();
    if (rl["cmd_vx"]) next.rl.cmd_vx = rl["cmd_vx"].as<float>();
    if (rl["cmd_vy"]) next.rl.cmd_vy = rl["cmd_vy"].as<float>();
    if (rl["cmd_dyaw"]) next.rl.cmd_dyaw = rl["cmd_dyaw"].as<float>();
    if (rl["obs_lin_vel_scale"]) next.rl.obs_lin_vel_scale = rl["obs_lin_vel_scale"].as<float>();
    if (rl["obs_ang_vel_scale"]) next.rl.obs_ang_vel_scale = rl["obs_ang_vel_scale"].as<float>();
    if (rl["obs_dof_pos_scale"]) next.rl.obs_dof_pos_scale = rl["obs_dof_pos_scale"].as<float>();
    if (rl["obs_dof_vel_scale"]) next.rl.obs_dof_vel_scale = rl["obs_dof_vel_scale"].as<float>();

  }

  if (auto imu = root["imu"]) {
    if (imu["enable"]) next.imu.enable = imu["enable"].as<bool>();
    if (imu["port"]) next.imu.port = imu["port"].as<std::string>();
    if (imu["baud"]) next.imu.baud = imu["baud"].as<int>();
  }

  // Parse motor layout from config YAML
  next.can = load_config_yaml(config_path_);

  auto add_motor = [&](const MotorSpec& ms, int dev_idx, const std::string& dev_name,
                       const std::string& dev_path, uint8_t channel) {
    ActiveMotorConfig am;
    am.spec = ms;
    am.device_name = dev_name;
    am.device_path = dev_path;
    am.channel = channel;
    am.device_index = dev_idx;
    const int idx = static_cast<int>(next.active_motors.size());
    next.active_motors.push_back(am);
    next.motor_names.push_back(ms.motorName);
    next.joint_names.push_back(ms.motorName);
    next.name_to_index[ms.motorName] = idx;
    next.initial_joint_pos[ms.motorName] = ms.default_joint_angle_rad;
    next.initial_motor_pos[ms.motorName] = ms.default_motor_angle_rad;
  };

  for (int di = 0; di < static_cast<int>(next.can.devices.size()); ++di) {
    const auto& dev = next.can.devices[di];
    for (const auto& bus : dev.buses) {
      for (const auto& ms : bus.motors) {
        if (!ms.enabled) continue;
        if (!group_selected(ms.group, next.only_groups)) continue;
        add_motor(ms, di, dev.name, dev.path, bus.channel);
      }
    }
  }

  auto rename_ankle = [&](const std::string& m1, const std::string& m2,
                          const std::string& jp, const std::string& jr) {
    auto it1 = next.name_to_index.find(m1);
    auto it2 = next.name_to_index.find(m2);
    if (it1 == next.name_to_index.end() || it2 == next.name_to_index.end()) return;
    next.joint_names[it1->second] = jp;
    next.joint_names[it2->second] = jr;
    next.name_to_index[jp] = it1->second;
    next.name_to_index[jr] = it2->second;
    next.initial_joint_pos[jp] = next.initial_joint_pos[m1];
    next.initial_joint_pos[jr] = next.initial_joint_pos[m2];
  };

  rename_ankle("L_ankle_L", "L_ankle_R", "L_ankle_pitch", "L_ankle_roll");
  rename_ankle("R_ankle_L", "R_ankle_R", "R_ankle_pitch", "R_ankle_roll");

  // Swap-in atomically
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cfg_ = std::move(next);
  }
}
