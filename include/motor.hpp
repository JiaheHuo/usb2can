#pragma once
#include <string>
#include <mutex>
#include <optional>
#include <array>
#include "usb2can_transport.hpp"
#include "rs_protocol.hpp"
#include "actuator.hpp"
#include "stats.hpp"
#include <condition_variable>
#include <chrono>

struct PVTT { float p=0, v=0, t=0, temp=0; };

struct BusHandle {
  int dev_idx = 0;
  std::string dev_name;
  Usb2CanDevice* dev = nullptr;
  uint8_t channel = 1; // 1 or 2
};

class RobStrideMotor {
public:
  RobStrideMotor(BusHandle bus, uint8_t master_id, uint8_t motor_id, ActuatorType type,
                 std::string group, Stats* stats, int stats_idx, std::string motorName);

  uint8_t motor_id() const { return motor_id_; }
  const BusHandle& bus() const { return bus_; }
  const std::string& group() const { return group_; }

  // --- basic commands ---
  void enable();
  void stop(uint8_t clear_error=0);
  void set_zero();
  void set_mode(uint8_t run_mode); // 写 0x7005
  void set_param_f32(uint16_t index, float value);
  void set_param_u8 (uint16_t index, uint8_t value);
  void get_param(uint16_t index);

  // --- motion control ---
  void send_motion_command(float torque, float pos_rad, float vel_rad_s, float kp, float kd);

  // --- rx hook (router调用) ---
  void on_frame(const Usb2CanFrame& f);

  // --- set zero_sta (0~2pi)-> (-pi~pi)

  void data_save(); // 0x16
  bool set_zero_sta(uint8_t v, bool persist=true, int timeout_ms=100);
  std::optional<uint8_t> get_zero_sta(int timeout_ms=100);

  std::optional<uint32_t> get_param_u32_blocking(uint16_t index, int timeout_ms=100);
  std::optional<float>    get_param_f32_blocking(uint16_t index, int timeout_ms=100);

  std::optional<PVTT> last_pvtt() const;
  std::string motorName_;

private:
  BusHandle bus_;
  uint8_t master_id_;
  uint8_t motor_id_;
  ActuatorType type_;
  std::string group_;
  

  Stats* stats_ = nullptr;
  int stats_idx_ = -1;

  mutable std::mutex mu_;
  PVTT pvtt_{};
  bool has_pvtt_ = false;

  // ---- param reply cache (Type17 reply) ----
  mutable std::condition_variable cv_param_;
  uint32_t last_param_seq_ = 0;
  bool     last_param_valid_ = false;
  uint16_t last_param_index_ = 0;
  uint32_t last_param_u32_   = 0;
};
