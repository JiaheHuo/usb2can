#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "control_modules.hpp"
#include "control_utility.hpp"
#include "databus.hpp"

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run.store(false); }

int main(int argc, char** argv) {
  std::signal(SIGINT, on_sigint);

  std::string config_path = "config/config.yaml";
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto need = [&](const std::string& name) {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(1);
      }
      return std::string(argv[++i]);
    };
    if (k == "--config") config_path = need(k);
    if (k == "--help") {
      std::cout << "Usage: ./motor_test [--config config/config.yaml]\n";
      return 0;
    }
  }

  Utility util(config_path);
  util.start();
  auto cfg = util.config();

  DataBus bus(cfg.active_motors.size());
  bus.set_names(cfg.motor_names, cfg.joint_names);
  bus.start();

  // Seed joint commands with fixed gains at the configured defaults.
  for (std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
    JointCmd jc{};
    jc.enable = true;
    jc.q_des = cfg.initial_joint_pos.count(cfg.joint_names[i])
                   ? cfg.initial_joint_pos.at(cfg.joint_names[i])
                   : 0.f;
    jc.dq_des = 0.f;
    jc.kp = cfg.fixed_gains.kp;
    jc.kd = cfg.fixed_gains.kd;
    bus.update_joint_cmd(i, jc);
  }

  LowLevelControl low(bus, util);
  ImuInterface imu(bus, util);
  Mapping mapping(bus, util);
  RLLocomotion rl(bus, util);

  if (!low.start()) {
    std::cerr << "[main] failed to start low-level control\n";
    return 1;
  }
  imu.start();
  mapping.start();
  rl.start();

  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  rl.stop();
  mapping.stop();
  imu.stop();
  low.stop();
  bus.stop();
  util.stop();
  return 0;
}
