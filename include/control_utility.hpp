#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "config.hpp"

struct JointLimit {
  float min = -1e9f;
  float max = +1e9f;
};

struct MotorLimits {
  float torque_max = 40.0f;
  float vel_max    = 50.0f;
  std::vector<float> torque_max_vec; 
  std::vector<float> vel_max_vec;
  JointLimit position;
  std::unordered_map<std::string, JointLimit> per_joint;
  float torque_max_at(int i) const {
    if (i >= 0 && i < (int)torque_max_vec.size()) return torque_max_vec[i];
    return torque_max;
  }
  float vel_max_at(int i) const {
    if (i >= 0 && i < (int)vel_max_vec.size()) return vel_max_vec[i];
    return vel_max;
  }
};

struct ControlGains {
  float kp_default = 0.f;
  float kd_default = 0.f;
  std::vector<float> kp,kd;
  float kp_at(int i) const {
    if (i >= 0 && i < (int)kp.size()) return kp[i];
    return kp_default;
  }
  float kd_at(int i) const {
    if (i >= 0 && i < (int)kd.size()) return kd[i];
    return kd_default;
  }
};

struct ActiveMotorConfig {
  MotorSpec spec;
  std::string device_name;
  std::string device_path;
  uint8_t channel = 1;
  int device_index = 0;
};

struct RlConfig {
  std::string policy_path;
  int frame_stack = 15;
  int decimation = 10;
  float action_scale = 0.25f;
  float clip_actions = 18.0f;
  float clip_obs = 18.0f;
  float gait_period = 0.64f;
  float cmd_vx = 0.4f;
  float cmd_vy = 0.0f;
  float cmd_dyaw = 0.0f;
  float obs_lin_vel_scale = 2.0f;
  float obs_ang_vel_scale = 1.0f;
  float obs_dof_pos_scale = 1.0f;
  float obs_dof_vel_scale = 0.05f;
};

struct ImuConfig {
  bool enable = true;
  std::string port = "/dev/imu";
  int baud = 921600;
};

struct UtilityConfig {
  std::string config_path;
  std::vector<std::string> only_groups;

  int loop_hz = 1000;
  int telemetry_hz = 10;
  bool simulate = false;

  ControlGains fixed_gains;
  ControlGains running_gains;
  MotorLimits limits{};

  RlConfig rl{};
  ImuConfig imu{};

  AppConfig can;
  std::vector<ActiveMotorConfig> active_motors;

  std::vector<std::string> motor_names;
  std::vector<std::string> joint_names;
  std::unordered_map<std::string, int> name_to_index;

  std::unordered_map<std::string, float> initial_joint_pos;
  std::unordered_map<std::string, float> initial_motor_pos;
};

class Utility {
public:
  explicit Utility(const std::string& config_path);
  ~Utility();

  Utility(const Utility&) = delete;
  Utility& operator=(const Utility&) = delete;

  void start();
  void stop();

  UtilityConfig config() const;

  float clip_torque(float tau) const;
  float clip_torque(int motor_idx, float tau)const;
  float clip_position(const std::string& joint, float q_rad) const;
  JointLimit joint_limit(const std::string& joint) const;

  int motor_index(const std::string& name) const;

private:
  void load_config_();
  void reload_loop_();

  std::string config_path_;
  mutable std::mutex mtx_;
  UtilityConfig cfg_{};

  std::atomic<bool> running_{false};
  std::thread reload_thread_;
  std::filesystem::file_time_type last_write_time_{};
};
