#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include <array>
#include <bitset>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <cstdint>

#include "usb2can_transport.hpp"
#include "motor.hpp"

// ========================
// 让 vector 可用：atomic 成员必须自定义 copy/move
// ========================
struct DevRxStat {
  std::array<std::atomic<uint64_t>,3> raw;
  std::array<std::atomic<uint64_t>,3> dispatched;
  std::array<std::atomic<uint64_t>,3> nomatch;
  std::array<std::atomic<uint64_t>,3> cycles;
  std::atomic<uint64_t> timeouts{0};

  DevRxStat() { reset(); }

  void reset() {
    for (int i=0;i<3;i++) {
      raw[i].store(0, std::memory_order_relaxed);
      dispatched[i].store(0, std::memory_order_relaxed);
      nomatch[i].store(0, std::memory_order_relaxed);
      cycles[i].store(0, std::memory_order_relaxed);
    }
    timeouts.store(0, std::memory_order_relaxed);
  }

  DevRxStat(const DevRxStat& o) { copy_from(o); }
  DevRxStat& operator=(const DevRxStat& o){
    if (this == &o) return *this;
    copy_from(o);
    return *this;
  }
  DevRxStat(DevRxStat&& o) noexcept : DevRxStat(o) {}
  DevRxStat& operator=(DevRxStat&& o) noexcept { return (*this = o); }

private:
  void copy_from(const DevRxStat& o) {
    for (int i=0;i<3;i++) {
      raw[i].store(o.raw[i].load(std::memory_order_relaxed), std::memory_order_relaxed);
      dispatched[i].store(o.dispatched[i].load(std::memory_order_relaxed), std::memory_order_relaxed);
      nomatch[i].store(o.nomatch[i].load(std::memory_order_relaxed), std::memory_order_relaxed);
      cycles[i].store(o.cycles[i].load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
    timeouts.store(o.timeouts.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }
};

struct BusSync {
  std::bitset<256> expected;
  std::bitset<256> got;
  uint16_t expected_cnt = 0;
  uint16_t got_cnt = 0;

  void finalize_expected() { expected_cnt = (uint16_t)expected.count(); }
  void reset_round() { got.reset(); got_cnt = 0; }

  void mark(uint8_t motor_id) {
    if (!expected.test(motor_id)) return;
    if (!got.test(motor_id)) {
      got.set(motor_id);
      ++got_cnt;
    }
  }
  bool round_complete() const { return expected_cnt > 0 && got_cnt == expected_cnt; }
};

struct DevSync {
  std::array<BusSync,3> bus; // 1/2 used
};

class Usb2CanRouter {
public:
  explicit Usb2CanRouter(const std::vector<Usb2CanDevice*>& devs);
  ~Usb2CanRouter();

  void attach_motor(RobStrideMotor* m);
  void start();
  void stop();

  // 由 main 每秒调用一次
  void print_periodic();

private:
  void rx_loop(int dev_idx);
  int  find_dev_index_(Usb2CanDevice* dev) const;

private:
  std::vector<Usb2CanDevice*> devs_;
  std::vector<std::thread> threads_;
  std::atomic<bool> running_{false};

  // [dev][ch][motor_id] -> RobStrideMotor*
  std::vector<std::array<std::array<RobStrideMotor*,256>,3>> table_;

  std::vector<DevSync> sync_;

  std::vector<DevRxStat> stat_;
  std::vector<DevRxStat> stat_last_;
  std::chrono::steady_clock::time_point last_print_{std::chrono::steady_clock::now()};
};
