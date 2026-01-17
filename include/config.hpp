#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include "actuator.hpp"

struct MotorSpec {
  uint8_t id = 0;
  ActuatorType type = ActuatorType::RS04;
  bool enabled = true;
  std::string group;
  float kp = 20.f;   // 默认值
  float kd = 1.0f;
  std::string motorName;
  float default_motor_angle_rad = 0.f;
  float default_joint_angle_rad = 0.f;
};

struct BusSpec {
  uint8_t channel = 1; // 1 or 2
  std::vector<MotorSpec> motors;
};

struct DeviceSpec {
  std::string name;
  std::string path; // /dev/USB2CAN0
  std::vector<BusSpec> buses;
};

struct AppConfig {
  uint8_t master_id = 0xFF;
  std::vector<DeviceSpec> devices;
};

AppConfig load_config_yaml(const std::string& file);
