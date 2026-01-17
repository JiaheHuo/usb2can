#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <deque>
#include <chrono>

struct MotorKey {
  int dev_idx = 0;
  uint8_t channel = 1;
  uint8_t motor_id = 0;
  std::string dev_name;
  std::string group;
  std::string type_str;
  std::string motor_name;
};

struct MotorCounters {
  std::atomic<uint64_t> tx_motion{0};
  std::atomic<uint64_t> tx_other{0};
  std::atomic<uint64_t> rx_status{0};
  std::atomic<uint64_t> rx_other{0};
  std::atomic<uint64_t> tx_motion_ok{0};
  std::atomic<uint64_t> tx_motion_fail{0};
};

class Stats {
public:
  int register_motor(const MotorKey& k);

  void inc_tx_motion(int idx);
  void inc_tx_other(int idx);
  void inc_rx_status(int idx);
  void inc_rx_other(int idx);
  void inc_tx_motion_ok  (int idx){ motors_[idx].cnt.tx_motion_ok  .fetch_add(1,std::memory_order_relaxed); }
  void inc_tx_motion_fail(int idx){ motors_[idx].cnt.tx_motion_fail.fetch_add(1,std::memory_order_relaxed); }


  // 每秒打印一次（由 main 调用）
  void print_periodic();

private:
  struct Entry {
    MotorKey key;
    MotorCounters cnt;

    uint64_t last_tx_motion      = 0;
    uint64_t last_tx_motion_ok   = 0;
    uint64_t last_tx_motion_fail = 0;

    uint64_t last_tx_other  = 0;
    uint64_t last_rx_status = 0;
    uint64_t last_rx_other  = 0;
  };

  std::deque<Entry> motors_;
  std::chrono::steady_clock::time_point last_print_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};
