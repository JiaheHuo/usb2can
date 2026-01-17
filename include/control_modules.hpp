#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <torch/script.h>

#include "ankle_kinematics.hpp"
#include "control_utility.hpp"
#include "databus.hpp"
#include "motor.hpp"
#include "router.hpp"
#include "stats.hpp"
#include "witmotion_imu.hpp"
#include <deque>

class LowLevelControl {
public:
  LowLevelControl(DataBus& bus, Utility& util);
  ~LowLevelControl();

  bool start();
  void stop();

private:
  void loop_();
  bool init_hardware_();
  void simulate_step_(double dt);
  void read_states_();
  void apply_commands_();

  DataBus& bus_;
  Utility& util_;

  std::mutex mtx_;
  std::atomic<bool> running_{false};
  std::thread worker_;

  Stats stats_;
  std::vector<std::unique_ptr<Usb2CanDevice>> dev_objs_;
  std::vector<Usb2CanDevice*> dev_ptrs_;
  std::unique_ptr<Usb2CanRouter> router_;
  std::vector<std::unique_ptr<RobStrideMotor>> motors_;
  std::vector<RobStrideMotor*> active_;

  // Simulated state when running without hardware
  Eigen::VectorXd sim_pos_rad_;
  Eigen::VectorXd sim_vel_rad_;
  Eigen::VectorXd sim_tau_nm_;
};

class ImuInterface {
public:
  ImuInterface(DataBus& bus, Utility& util);
  ~ImuInterface();

  bool start();
  void stop();

private:
  void loop_();

  DataBus& bus_;
  Utility& util_;

  std::mutex mtx_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  WitMotionImuRunner runner_;
};

class Mapping {
public:
  Mapping(DataBus& bus, Utility& util);
  ~Mapping();

  bool start();
  void stop();

private:
  struct AnkleIndices {
    int m1 = -1;
    int m2 = -1;
    int jp = -1;
    int jr = -1;
    bool valid() const { return m1 >= 0 && m2 >= 0 && jp >= 0 && jr >= 0; }
  };

  void loop_();
  void build_indices_();
  void update_joint_states_(const BusSnapshot& snap, const UtilityConfig& cfg);
  void write_motor_cmds_(const BusSnapshot& snap, const UtilityConfig& cfg);
  void map_ankle_state_(const AnkleIndices& idx, const BusSnapshot& snap);
  void map_ankle_torque_(const AnkleIndices& idx, const BusSnapshot& snap,
                         const UtilityConfig& cfg);

  DataBus& bus_;
  Utility& util_;
  ankle::AnkleParams ankle_params_;
  AnkleIndices left_;
  AnkleIndices right_;

  std::mutex mtx_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

class RLLocomotion {
public:
  RLLocomotion(DataBus& bus, Utility& util);
  ~RLLocomotion();

  bool start();
  void stop();

private:
  void loop_();
  void load_policy_(const std::string& path);
  std::vector<float> build_single_observation_(const BusSnapshot& snap, const UtilityConfig& cfg);
  std::vector<float> build_observation_(const BusSnapshot& snap, const UtilityConfig& cfg);
  std::vector<float> run_policy_(const std::vector<float>& obs);
  void write_joint_targets_(const std::vector<float>& action, const UtilityConfig& cfg);

  DataBus& bus_;
  Utility& util_;

  std::mutex mtx_;
  std::atomic<bool> running_{false};
  std::atomic<bool> policy_started_{false};
  std::thread worker_;

  torch::jit::script::Module policy_;
  bool policy_ok_ = false;
  int num_actions_ = 12;
  int num_single_obs_ = 0;
  int frame_stack_ = 0;
  std::deque<std::vector<float>> hist_obs_;

  std::vector<float> last_action_;
};
